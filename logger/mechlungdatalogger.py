"""mechlungdatalogger.py - runs MechanicalLungPlatform.ino from the PC and saves the data.

Usage (Windows):  python mechlungdatalogger.py COM3      (no port: lists the ports)
Needs:            pip install pyserial

Asks for room temperature, MSL pressure and a queue such as "4x20, s2x6, c600":
  1x10 to 4x10   breathing preset x complete breaths
  s1x6 to s4x6   steady flow x complete strokes
  c600           clock check: 600 s of rows at idle, measures the board's clock
For each item it returns the rig to centre, tares, runs the item, returns to
centre and saves:
  runs\\<date>_<hhmmss>_<item>\\samples.csv   the sketch's rows, plus pc_s (PC arrival time)
                                events.log    # lines, after the tick they followed
                                meta.json     conditions, # config, tare, timing, clock,
                                              per-breath or per-stroke figures
Prints one line per breath (or stroke) and a summary. Ctrl+C stops the actuator,
saves what it has and marks the run aborted.

The sketch only streams raw rows; every figure is worked out here. Steady flow
leak is taken from slopes over the middle of each stroke (see Strokes).
"""
import json
import math
import re
import statistics
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

BAUD = 230400          # ignored over native USB (NATIVE_USB 1 in the sketch); never 1200, which resets the board
ELEVATION_M = 27       # VU Footscray Park, Building D level 3: RL 26.1 m + about 1 m bench (VU design report)
ROW_TIMEOUT_S = 2      # rows normally arrive every 10 ms
TRIM = 0.10            # steady figures leave this fraction of the move time off each end
TRIM_INNER = 0.20      # second mean flow: if it differs from the TRIM one, the reversal is still settling
TARE_SETTLE_S = 2.0    # PLACEHOLDER: pot still for this long before the tare; set from pot noise at rest
TARE_STILL_DAC = 15.0  # DAC counts: pot noise at rest spans 9 to 11 (rig, 26 Oct 2026)
TARE_ROWS = 100        # rows averaged for the tare (1 s)
TARE_FLOW_MAX_SLM = 1.0  # good tares read 0.004 to 0.009 slm; more is not a zero
MIN_GOOD_FRAC = 0.9    # steady figures need good flow on this fraction of the window's rows
CLOCK_BLOCK_S = 1.0    # clock fit: earliest arrival in each block of this length
HERE = Path(__file__).parent
LAST = HERE / "last_conditions.json"

COLS = (["tick", "us", "breath", "mode", "cmd_dac", "pot_raw", "sfm_raw", "sfm_n", "sfm_span_us", "pot_dus"]
        + [f"p{i}_raw" for i in range(1, 8)] + [f"p{i}_dus" for i in range(1, 8)])
C = {name: k for k, name in enumerate(COLS)}
P_RAW = C["p1_raw"]
P_DUS = C["p1_dus"]
HEADER = ",".join(COLS) + ",pc_s"
MODE_IDLE = -1
MODE_SS = 4


def ask(prompt, default):
    text = input(f"{prompt} [{default}]: ").strip()
    return float(text) if text else default


def station_pressure(p_msl_hpa):
    return p_msl_hpa * (1 - 2.25577e-5 * ELEVATION_M) ** 5.25588


def parse_fields(line):
    """'# config a=1 b=2,3' -> {'a': 1.0, 'b': [2.0, 3.0]}"""
    out = {}
    for key, val in re.findall(r"(\w+)=(\S+)", line):
        nums = [float(v) for v in val.split(",")]
        out[key] = nums if len(nums) > 1 else nums[0]
    return out


def parse_row(text):
    """A row as a list of ints (None for an empty field), or None if it is not a whole row."""
    f = text.split(",")
    if len(f) != len(COLS):
        return None
    try:
        return [int(x) if x else None for x in f]
    except ValueError:
        return None


def nanmean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return statistics.mean(xs) if xs else math.nan


def slope(xs, ys):
    """Least squares slope of ys against xs (nan with fewer than 3 points)."""
    if len(xs) < 3:
        return math.nan
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else math.nan


class Rig:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self.buf = b""
        self.pending = deque()        # (text, PC time of arrival)
        self.recording = False
        self.rows = []                # (text, PC time, parsed) while recording
        self.events = []
        self.last_tick = None
        self.bad_lines = 0

    def send(self, key):
        self.ser.write(key.encode())

    def clear(self):
        self.ser.reset_input_buffer()
        self.buf = b""
        self.pending.clear()

    def _fill(self):
        # Read whatever has arrived in one call: pyserial's readline() reads one byte
        # per call, which cannot keep up with 100 rows/s on Windows. Each line gets the
        # time the read that completed it returned (the clock fit needs arrival times).
        chunk = self.ser.read(self.ser.in_waiting or 1)
        if not chunk:
            return
        t = time.perf_counter()
        self.buf += chunk
        *lines, self.buf = self.buf.split(b"\n")
        for ln in lines:
            self.pending.append((ln.decode("ascii", "replace").strip(), t))

    def get(self):
        """Next item: ("row", parsed, t), ("text", line, t), or None if nothing yet."""
        if not self.pending:
            self._fill()
        if not self.pending:
            return None
        text, t = self.pending.popleft()
        if not text:
            return None
        if text[0].isdigit():
            r = parse_row(text)
            if r is None:
                self.bad_lines += 1
                return None
            self.last_tick = r[C["tick"]]
            if self.recording:
                self.rows.append((text, t, r))
            return ("row", r, t)
        if text.startswith("#"):
            self.events.append(f"{'' if self.last_tick is None else self.last_tick},{text}")
            return ("text", text, t)
        return None

    def next_row(self, timeout_s=ROW_TIMEOUT_S):
        end = time.time() + timeout_s
        while time.time() < end:
            item = self.get()
            if item and item[0] == "row":
                return item[1]
        raise TimeoutError(f"no rows for {timeout_s} s")

    def config(self):
        """Ask for # config. Checks the line is whole (end=1) and from MechanicalLungPlatform."""
        for _ in range(3):
            self.send("c")
            end = time.time() + 3
            while time.time() < end:
                item = self.get()
                if item and item[0] == "text" and item[1].startswith("# config"):
                    cfg = parse_fields(item[1])
                    if cfg.get("end") != 1:
                        raise RuntimeError("# config line cut off (no end=1): raise LINE_LEN in the sketch")
                    if cfg.get("fmt") != 1:
                        raise RuntimeError("not MechanicalLungPlatform (# config fmt=1): wrong sketch?")
                    return cfg
        raise TimeoutError("no # config reply: is MechanicalLungPlatform running on this port?")

    def idle(self, timeout_s=60):
        """H, then wait until the rows show idle at centre."""
        self.send("h")
        end = time.time() + timeout_s
        n = 0
        while time.time() < end:
            r = self.next_row()
            n = n + 1 if r[C["mode"]] == MODE_IDLE else 0
            if n >= 5:
                return
        raise TimeoutError(f"not back at centre within {timeout_s} s")


class Conv:
    """Raw row values to physical units: flow (L/min at room conditions, + out of the
    cylinder, tared), pot volume (mL from centre) and pressure (Pa, tared)."""

    def __init__(self, cfg, tare, flow_conv):
        self.c, self.t, self.conv = cfg, tare, flow_conv

    def flow(self, r):
        sfm = r[C["sfm_raw"]]
        if sfm is None:
            return math.nan
        slm = (sfm - self.c["sfm_offset"]) / self.c["sfm_scale"]
        if abs(slm) > 200.0:           # outside the sensor's range: not a measurement
            return math.nan
        return self.c["flow_sign"] * (slm - self.t["flow_slm"]) * self.conv

    def pot_ml(self, pot):
        c = self.c
        fb = c["centre_dac"] + (pot - c["pot_raw_at_centre"]) * c["pot_dac_per_adc"]
        return c["v_tidal_ml"] * (fb - c["centre_dac"]) / c["stroke_amp_dac"]

    def pressure(self, r, k):
        raw, scale = r[P_RAW + k], self.c["sdp_scale"]
        if raw is None or not scale:
            return math.nan
        return raw / scale - self.t[f"p{k + 1}_Pa"]

    def sfm_short(self, r):
        return r[C["sfm_raw"]] is not None and r[C["sfm_n"]] < self.c["flow_avg"]


def tare(rig, cfg):
    """Pot still within TARE_STILL_DAC for TARE_SETTLE_S, then the mean of TARE_ROWS
    rows of flow (slm) and pressure (Pa). A flow tare outside +-TARE_FLOW_MAX_SLM
    stops the run: power cycle the rig and start again."""
    hz = cfg["sample_hz"]
    hist, ref, t0 = [], None, None
    end = time.time() + 30
    while True:
        if time.time() > end:
            raise TimeoutError("tare: the pot did not settle within 30 s")
        r = rig.next_row()
        if r[C["mode"]] != MODE_IDLE:
            raise RuntimeError("tare aborted, not idle")
        hist = (hist + [r[C["pot_raw"]]])[-Breaths.POT_MEDIAN:]
        fb = sorted(hist)[len(hist) // 2] * cfg["pot_dac_per_adc"]
        if ref is None or abs(fb - ref) > TARE_STILL_DAC:
            ref, t0 = fb, r[C["tick"]]      # pot moved: restart the settle time
        elif (r[C["tick"]] - t0) / hz >= TARE_SETTLE_S:
            break
    flow, press, pots = [], [[] for _ in range(7)], []
    for _ in range(TARE_ROWS):
        r = rig.next_row()
        if r[C["mode"]] != MODE_IDLE:
            raise RuntimeError("tare aborted, not idle")
        if r[C["sfm_raw"]] is not None:
            flow.append((r[C["sfm_raw"]] - cfg["sfm_offset"]) / cfg["sfm_scale"])
        for k in range(7):
            if r[P_RAW + k] is not None and cfg["sdp_scale"]:
                press[k].append(r[P_RAW + k] / cfg["sdp_scale"])
        hist = (hist + [r[C["pot_raw"]]])[-Breaths.POT_MEDIAN:]   # median: a pot spike is not a move
        pots.append(sorted(hist)[len(hist) // 2] * cfg["pot_dac_per_adc"])
    out = dict(flow_slm=statistics.mean(flow) if flow else math.nan)
    for k in range(7):
        out[f"p{k + 1}_Pa"] = statistics.mean(press[k]) if press[k] else math.nan
    out["pot_spread_dac"] = max(pots) - min(pots)
    out["flow_rows"] = len(flow)
    if math.isnan(out["flow_slm"]):
        raise RuntimeError("tare: no good flow reads")
    if abs(out["flow_slm"]) > TARE_FLOW_MAX_SLM:
        raise RuntimeError(f"tare rejected: flow {out['flow_slm']:.3f} slm is not a zero, power cycle the rig")
    return out


class Breaths:
    """Works out each breath's figures from the rows. The pot uses a POT_MEDIAN-point
    median: the rig's pot has single and paired spikes in motion (up to about 1700
    counts), and a 5-point median let some through to the peak pot volume (run 27 Oct
    11:10, breaths 1 and 4: push leak 181 and 171 mL against 150 and 158 mL with 11
    points). An N-point median rejects runs of up to (N - 1) / 2 spikes and runs
    (N - 1) / 2 samples behind a moving piston.

    Push leak = pot volume rise minus measured volume rise over the push. Return
    leak = the same for the fall from the peak to breath end. The measured volume
    starts each breath at the pot volume, so both measure from centre."""

    POT_MEDIAN = 11
    LAG = (POT_MEDIAN - 1) // 2

    def __init__(self, conv):
        self.cv = conv
        self.pot_hist = []
        self.prev_tick = None
        self.ml_hist = []              # flow volume of the last LAG - 1 rows (nan for a failed read)
        self.cur = None
        self.done = []

    def add(self, r):
        """Feed one row. Returns a finished breath's figures, or None."""
        tick, breath, mode = r[C["tick"]], r[C["breath"]], r[C["mode"]]
        self.pot_hist = (self.pot_hist + [r[C["pot_raw"]]])[-self.POT_MEDIAN:]
        vp = self.cv.pot_ml(sorted(self.pot_hist)[len(self.pot_hist) // 2])
        dt = 1 if self.prev_tick is None else tick - self.prev_tick
        self.prev_tick = tick
        flow = self.cv.flow(r)
        row_ml = flow * 1000 / 60 * dt / self.cv.c["sample_hz"]
        prev = list(self.ml_hist)
        self.ml_hist = (self.ml_hist + [row_ml])[-(self.LAG - 1):] if self.LAG > 1 else []
        finished = None
        if self.cur and (breath != self.cur["breath"] or mode < 0):
            if breath != self.cur["breath"] and mode >= 0:   # completed, not aborted
                finished = self.close()
            self.cur = None
        if mode < 0:
            return finished
        if self.cur is None:
            # The median runs LAG samples behind a moving piston, so the seed adds
            # the previous LAG - 1 rows' flow; this row's flow is added below. A failed
            # read in those rows makes the breath's flow bad, as one inside it does.
            seed_bad = any(math.isnan(x) for x in prev)
            seed = vp + (0.0 if seed_bad else sum(prev))
            self.cur = dict(breath=breath, seed=seed, vm=seed, pk_pot=-math.inf, pk_meas=-math.inf,
                            lost=0, failed=0, sfm_short=0, flow_bad=seed_bad)
        b = self.cur
        b["flow_bad"] |= math.isnan(flow)
        b["vm"] += 0 if b["flow_bad"] else row_ml
        b["lost"] += dt - 1
        b["failed"] += (r[C["sfm_raw"]] is None) + sum(1 for k in range(7) if r[P_RAW + k] is None)
        b["sfm_short"] += self.cv.sfm_short(r)
        b["vp"] = vp
        b["vm_hist"] = (b.get("vm_hist", []) + [b["vm"]])[-(self.LAG + 1):]   # vm at the time vp refers to
        b["pk_pot"] = max(b["pk_pot"], vp)
        b["pk_meas"] = max(b["pk_meas"], b["vm"])
        return finished

    def close(self):
        b = self.cur
        nan = math.nan
        push = nan if b["flow_bad"] else b["pk_pot"] - b["pk_meas"]
        # vp at breath end is LAG samples old (median), so compare it with vm LAG rows back.
        ret = nan if b["flow_bad"] else (b["pk_pot"] - b["vp"]) - (b["pk_meas"] - b["vm_hist"][0])
        net = nan if b["flow_bad"] else b["vm"] - b["seed"]
        out = dict(breath=b["breath"], push_leak_ml=push, return_leak_ml=ret,
                   net_ml=net, push_leak_pct=100 * push / b["pk_pot"] if b["pk_pot"] > 0 else nan,
                   lost_rows=b["lost"], failed_reads=b["failed"], sfm_short=b["sfm_short"])
        self.done.append(out)
        return out


class Strokes:
    """Steady flow (modes 4 to 7): figures for each stroke (one "breath" in the rows:
    the move plus the hold at its end, so it starts and ends at rest).

    Main figure, slope leak: over the middle of the move (TRIM of the move time off
    each end, clear of the reversals) a straight line is fitted to the pot volume
    against time; its slope is the piston's rate (mL/s). The measured rate is the
    mean flow over the same window (the slope of the measured volume, at constant
    flow). leak_rate = |pot rate| - |measured rate|, leak_pct = leak_rate / |pot
    rate|. Using every row in the window averages out pot noise, and a time offset
    between pot and flow shifts the line without changing its slope, so neither
    biases the leak. A failed flow read only drops that row from the mean; the
    figures need good flow on MIN_GOOD_FRAC of the window's rows.

    Also kept: the endpoint figures (pot and measured volume moved over the whole
    stroke, leak = the difference; any failed flow read makes them nan, as no gap
    in the volume is bridged), the mean flow over the TRIM_INNER window (if it
    differs from the TRIM one, the reversal is still settling inside the window)
    and the mean pressures over the TRIM window. The pot median runs LAG rows
    behind a moving piston, so pot is compared with the flow volume LAG rows back."""

    POT_MEDIAN = Breaths.POT_MEDIAN
    LAG = Breaths.LAG

    def __init__(self, conv, mode):
        self.cv, self.mode = conv, mode
        c = conv.c
        i = mode - MODE_SS
        self.hz = c["sample_hz"]
        self.q = c["ss_q_Lmin"][i]
        self.out_ticks = c["ss_out_ms"][i] * self.hz / 1000
        self.in_ticks = c["ss_in_ms"][i] * self.hz / 1000
        self.pot_hist = []
        self.prev_tick = None
        self.cum = 0.0                  # flow volume since the first row, nan rows count 0
        self.cum_hist = []              # cum of the last LAG + 1 rows
        self.nan_hist = []
        self.cur = None
        self.done = []

    def add(self, r):
        """Feed one row. Returns a finished stroke's figures, or None."""
        tick, breath, mode = r[C["tick"]], r[C["breath"]], r[C["mode"]]
        self.pot_hist = (self.pot_hist + [r[C["pot_raw"]]])[-self.POT_MEDIAN:]
        vp = self.cv.pot_ml(sorted(self.pot_hist)[len(self.pot_hist) // 2])
        dt = 1 if self.prev_tick is None else tick - self.prev_tick
        self.prev_tick = tick
        flow = self.cv.flow(r)
        bad = math.isnan(flow)
        self.cum += 0.0 if bad else flow * 1000 / 60 * dt / self.hz
        self.cum_hist = (self.cum_hist + [self.cum])[-(self.LAG + 1):]
        self.nan_hist = (self.nan_hist + [bad])[-(self.LAG + 1):]
        finished = None
        if self.cur and (breath != self.cur["breath"] or mode != self.mode):
            if breath != self.cur["breath"] and mode == self.mode:   # completed, not aborted
                finished = self.close()
            self.cur = None
        if mode != self.mode:
            return finished
        if self.cur is None:
            out = r[C["cmd_dac"]] < self.cv.c["centre_dac"]     # out strokes start at min
            self.cur = dict(breath=breath, t0=tick, vp0=vp, vm0=self.cum_hist[0], lost=0, failed=0,
                            sfm_short=0, flow_bad=any(self.nan_hist),
                            move=self.out_ticks if out else self.in_ticks,
                            ts=[], vps=[], win=0, q=[], q_inner=[], p=[[] for _ in range(7)])
        b = self.cur
        b["flow_bad"] |= bad
        b["lost"] += dt - 1
        b["failed"] += (r[C["sfm_raw"]] is None) + sum(1 for k in range(7) if r[P_RAW + k] is None)
        b["sfm_short"] += self.cv.sfm_short(r)
        b["vp"], b["vm"] = vp, self.cum_hist[0]
        t, move = tick - b["t0"], b["move"]
        if TRIM * move <= t <= (1 - TRIM) * move:
            b["ts"].append(t)
            b["vps"].append(vp)
            b["win"] += 1
            if not bad:
                b["q"].append(flow)
                if TRIM_INNER * move <= t <= (1 - TRIM_INNER) * move:
                    b["q_inner"].append(flow)
            for k in range(7):
                b["p"][k].append(self.cv.pressure(r, k))
        return finished

    def close(self):
        b = self.cur
        nan = math.nan
        pot = b["vp"] - b["vp0"]
        meas = nan if b["flow_bad"] else b["vm"] - b["vm0"]
        leak = abs(pot) - abs(meas)
        good = b["win"] > 0 and len(b["q"]) >= MIN_GOOD_FRAC * b["win"]
        sp = slope(b["ts"], b["vps"]) * self.hz                     # mL/s
        mean = statistics.mean(b["q"]) if good else nan             # L/min
        inner = statistics.mean(b["q_inner"]) if good and b["q_inner"] else nan
        sm = mean * 1000 / 60
        rate = abs(sp) - abs(sm)
        out = dict(breath=b["breath"], dir="out" if pot > 0 else "in", first=not self.done,
                   target_Lmin=self.q,
                   mean_flow_Lmin=mean, flow_pct=100 * abs(mean) / self.q,
                   mean_flow_inner_Lmin=inner, flow_pct_inner=100 * abs(inner) / self.q,
                   pot_rate_mLs=sp, meas_rate_mLs=sm, leak_rate_mLs=rate,
                   leak_pct=100 * rate / abs(sp) if sp else nan,
                   window_rows=b["win"], window_flow_rows=len(b["q"]),
                   pot_ml=pot, meas_ml=meas, endpoint_leak_ml=leak,
                   endpoint_leak_pct=100 * leak / abs(pot) if pot else nan,
                   p_mean_Pa=[nanmean(x) for x in b["p"]],
                   lost_rows=b["lost"], failed_reads=b["failed"], sfm_short=b["sfm_short"])
        self.done.append(out)
        return out


def clock_fit(rows, hz):
    """The board's clock against the PC's, from row arrival times. Rows reach the PC in
    bursts, late by a varying amount, but never early, so the earliest arrival (less
    the row's nominal tick time) in each CLOCK_BLOCK_S block traces the board's clock.
    A line through those points: slope = fractional clock error. + = the board runs
    slow (its 10 ms tick is longer): SFM volumes (sum of flow x 10 ms) and the commanded
    speeds both read low by this fraction, and leak figures high by it."""
    pts = {}
    for _, t, r in rows:
        tick = r[C["tick"]]
        block = int(tick // (hz * CLOCK_BLOCK_S))
        off = t - tick / hz
        if block not in pts or off < pts[block][1]:
            pts[block] = (tick / hz, off)
    if len(pts) < 10:
        return None
    xs, ys = zip(*pts.values())
    k = slope(xs, ys)
    mx = statistics.mean(xs)
    sxx = sum((x - mx) ** 2 for x in xs)
    my = statistics.mean(ys)
    res = [y - (my + k * (x - mx)) for x, y in zip(xs, ys)]
    se = math.sqrt(sum(e * e for e in res) / (len(xs) - 2) / sxx)
    return dict(ppm=k * 1e6, se_ppm=se * 1e6, span_s=max(xs) - min(xs), blocks=len(xs))


def clock_text(ck):
    if not ck:
        return "clock: run too short to measure"
    return (f"clock: board {ck['ppm']:+.0f} +/- {ck['se_ppm']:.0f} ppm over {ck['span_s']:.0f} s "
            f"(+ = slow: SFM volumes read {abs(ck['ppm']) / 1e4:.3f} % {'low' if ck['ppm'] > 0 else 'high'})")


def timing_stats(rows, cfg):
    """When each value in a row was read, from the sketch's micros() columns. The flow
    mean stands for the volume moved up to us + sfm_span_us / (2 (sfm_n - 1)); the
    *_vs_flow_ms figures are each read's offset from that time (+ = read later)."""
    if not rows:
        return {}
    avg = int(cfg["flow_avg"])
    gaps, pot, span, short = [], [], [], 0
    pd = [[] for _ in range(7)]
    prev = None
    for _, _, r in rows:
        if prev is not None and r[C["tick"]] - prev[C["tick"]] == 1:
            gaps.append((r[C["us"]] - prev[C["us"]]) % 2 ** 32)
        prev = r
        pot.append(r[C["pot_dus"]])
        for k in range(7):
            if r[P_DUS + k] is not None:
                pd[k].append(r[P_DUS + k])
        if r[C["sfm_raw"]] is not None:
            if r[C["sfm_n"]] == avg and avg > 1:
                span.append(r[C["sfm_span_us"]])
            elif r[C["sfm_n"]] < avg:
                short += 1

    def st(x):
        return dict(mean=statistics.mean(x), sd=statistics.pstdev(x), min=min(x), max=max(x)) if x else None

    span_mean = statistics.mean(span) if span else 0.0
    flow_us = span_mean / (2 * (avg - 1)) if avg > 1 else 0.0
    return dict(row_spacing_us=st(gaps), pot_dus=st(pot), sfm_span_us=st(span), sfm_short_rows=short,
                p_dus_mean=[statistics.mean(x) if x else None for x in pd],
                pot_vs_flow_ms=(statistics.mean(pot) - flow_us) / 1000,
                p_vs_flow_ms=[(statistics.mean(x) - flow_us) / 1000 if x else None for x in pd])


def timing_text(tm):
    if not tm or not tm["row_spacing_us"]:
        return "timing: no rows"
    g, s = tm["row_spacing_us"], tm["sfm_span_us"]
    span = f"{s['mean']:.0f} us (min {s['min']}, max {s['max']})" if s else "n/a"
    p = [x for x in tm["p_vs_flow_ms"] if x is not None]
    pr = f"{min(p):+.2f} to {max(p):+.2f} ms" if p else "n/a"
    return (f"timing: rows {g['mean']:.0f} +/- {g['sd']:.0f} us apart (max {g['max']}), "
            f"flow window {span}, short windows {tm['sfm_short_rows']}, "
            f"pot {tm['pot_vs_flow_ms']:+.2f} ms and p1..p7 {pr} from the flow time")


def run_item(rig, item, n, cond):
    """item: "1" to "4" (breathing preset), "s1" to "s4" (steady flow) or "c" (clock check, n s)."""
    clock_only = item == "c"
    steady = item.startswith("s")
    preset = 0 if clock_only else int(item[-1])
    mode = preset - 1 + (MODE_SS if steady else 0)
    what = "seconds" if clock_only else "strokes" if steady else "breaths"
    rig.rows, rig.events = [], []
    start = datetime.now()
    meta = dict(cond, item=f"{item}x{n}" if not clock_only else f"c{n}", preset=preset, steady=steady,
                clock_only=clock_only, **{f"{what}_requested": n},
                start=start.isoformat(timespec="seconds"), aborted=False)
    an = None
    try:
        rig.idle()
        meta["config"] = cfg = rig.config()
        hz = cfg["sample_hz"]
        if clock_only:
            rig.recording = True
            print(f"clock check: {n} s at idle")
            end = time.time() + n
            while time.time() < end:
                rig.next_row()
            rig.recording = False
        else:
            meta["tare"] = tare(rig, cfg)
            conv = Conv(cfg, meta["tare"], cond["flow_conv"])
            an = Strokes(conv, mode) if steady else Breaths(conv)
            rig.recording = True
            b0 = None
            rig.send(item)
            print(f"{'steady' if steady else 'preset'} {preset}: running {n} {what}")
            # Stop if rows stop, or the run takes far longer than it should. Steady
            # flow first ramps to min (up to 4095 DAC counts at 300 per s) and holds.
            if steady:
                i = preset - 1
                deadline = time.time() + 15 + cfg["ss_dwell_ms"] / 1000 + n * (
                    max(cfg["ss_out_ms"][i], cfg["ss_in_ms"][i]) + cfg["ss_dwell_ms"]) / 1000 + 30
            else:
                deadline = time.time() + n * cfg["period_ms"][preset - 1] / 1000 + 30
            while True:
                if time.time() > deadline:
                    raise TimeoutError(f"{n} {what} not complete by the expected time")
                r = rig.next_row()
                if b0 is None and r[C["mode"]] == mode:
                    b0 = r[C["breath"]]
                res = an.add(r)
                if res and steady:
                    k = res["breath"] - b0 + 1
                    print(f"  stroke {k:3d}/{n} {res['dir']:>3}: flow {res['flow_pct']:6.2f} % "
                          f"(20-80 % {res['flow_pct_inner']:6.2f} %) of {res['target_Lmin']:g} L/min, "
                          f"leak {res['leak_pct']:5.2f} % ({res['leak_rate_mLs']:5.2f} mL/s), "
                          f"endpoint {res['endpoint_leak_pct']:5.2f} %, lost rows {res['lost_rows']}, "
                          f"failed reads {res['failed_reads']}, short flow windows {res['sfm_short']}"
                          + (" (first, not in summary)" if res["first"] else ""))
                elif res:
                    k = res["breath"] - b0 + 1
                    print(f"  breath {k:3d}/{n}: push leak {res['push_leak_ml']:6.1f} mL "
                          f"({res['push_leak_pct']:5.2f} %), return leak {res['return_leak_ml']:6.1f} mL, "
                          f"lost rows {res['lost_rows']}, failed reads {res['failed_reads']}, "
                          f"short flow windows {res['sfm_short']}")
                if b0 is not None and r[C["breath"]] >= b0 + n:
                    break
            rig.recording = False
            rig.idle()
    except BaseException as e:          # any failure: stop the actuator and save first
        meta["aborted"] = True
        meta["error"] = repr(e)
        rig.recording = False
        try:                            # the port itself may be the failure
            rig.send("h")
        except Exception as e2:
            print(f"could not send H ({e2!r}): stop the actuator by hand")
        print(f"aborted ({e!r}): saving what was recorded")
    meta["end"] = datetime.now().isoformat(timespec="seconds")
    meta["rows"] = len(rig.rows)
    hz = meta.get("config", {}).get("sample_hz", 100)
    meta["timing"] = timing_stats(rig.rows, meta["config"]) if "config" in meta else {}
    meta["clock"] = clock_fit(rig.rows, hz)
    if an is not None:
        meta["strokes" if steady else "breaths"] = an.done
    print("  " + timing_text(meta["timing"]))
    print("  " + clock_text(meta["clock"]))
    save(start, meta["item"], meta, rig)
    return meta


def save(start, tag, meta, rig):
    folder = HERE / "runs" / f"{start:%Y-%m-%d_%H%M%S}_{tag}"
    folder.mkdir(parents=True, exist_ok=True)
    t0 = rig.rows[0][1] if rig.rows else 0.0
    (folder / "samples.csv").write_text(HEADER + "\n" + "".join(f"{text},{t - t0:.4f}\n" for text, t, _ in rig.rows))
    (folder / "events.log").write_text("tick,line\n" + "\n".join(rig.events) + "\n")
    (folder / "meta.json").write_text(json.dumps(meta, indent=2, default=str))
    print(f"saved {folder}")


def summary(meta):
    tail = f" (ABORTED: {meta.get('error', '')})" if meta["aborted"] else ""
    if meta["clock_only"]:
        return f"clock check {meta['item']}: " + clock_text(meta["clock"]) + tail
    if meta["steady"]:
        return steady_summary(meta, tail)
    all_b = meta.get("breaths", [])
    b = [x for x in all_b if not math.isnan(x["push_leak_ml"]) and x["lost_rows"] == 0]
    left = len(all_b) - len(b)
    if not b:
        return f"preset {meta['preset']}: no complete breaths with good flow and no lost rows{tail}"
    push = [x["push_leak_pct"] for x in b]
    ret = [x["return_leak_ml"] for x in b]
    sd = statistics.stdev(push) if len(push) > 1 else 0.0
    return (f"preset {meta['preset']}: {len(b)} breaths, push leak {statistics.mean(push):.2f} "
            f"+/- {sd:.2f} %, return leak {statistics.mean(ret):.1f} mL"
            + (f" ({left} left out: bad flow or lost rows)" if left else "") + tail)


def steady_summary(meta, tail):
    """Per direction, over the strokes with good flow and no lost rows, less the first
    stroke (it starts after the ramp to min, not after a stroke)."""
    name = f"steady {meta['preset']}"
    strokes = meta.get("strokes", [])
    lines = []
    for d in ("out", "in"):
        mine = [x for x in strokes if x["dir"] == d]
        s = [x for x in mine if not x["first"] and x["lost_rows"] == 0 and not math.isnan(x["leak_pct"])]
        if not s:
            continue

        def ms(key):
            v = [x[key] for x in s if not math.isnan(x[key])]
            if not v:
                return math.nan, math.nan
            return statistics.mean(v), (statistics.stdev(v) if len(v) > 1 else 0.0)

        pct, pct_sd = ms("flow_pct")
        leak, leak_sd = ms("leak_pct")
        p = " ".join(f"{nanmean([x['p_mean_Pa'][k] for x in s]):.0f}" for k in range(7))
        lines.append(f"{name} {d}: {len(s)} strokes, flow {pct:.2f} +/- {pct_sd:.2f} % "
                     f"(20-80 % {ms('flow_pct_inner')[0]:.2f} %) of {s[0]['target_Lmin']:g} L/min, "
                     f"leak {leak:.2f} +/- {leak_sd:.2f} % ({ms('leak_rate_mLs')[0]:.2f} mL/s), "
                     f"endpoint leak {ms('endpoint_leak_pct')[0]:.2f} %, p1..p7 {p} Pa"
                     + (f" ({len(mine) - len(s)} left out: first, bad flow or lost rows)" if len(mine) > len(s) else ""))
    if not lines:
        return f"{name}: no strokes with good flow and no lost rows (the first is not counted){tail}"
    return "\n".join(lines) + tail


def sensor_check(cfg):
    """Start-up checks from # config. False if the run cannot go ahead."""
    sdp = cfg["sdp_found"] if isinstance(cfg["sdp_found"], list) else [cfg["sdp_found"]]
    missing = [f"p{k + 1}" for k, ok in enumerate(sdp) if not ok]
    print(f"SFM3000 {'found' if cfg['sfm_found'] else 'NOT FOUND'}, SDP811 "
          + ("p1 to p7 found" if not missing else "not found: " + " ".join(missing))
          + f", timer {'running' if cfg['timer_ok'] else 'FAILED'}"
          + (f", Wire1 bus recovery: {cfg['wire1_pulses']:.0f}" if cfg["wire1_pulses"] else ""))
    if cfg["wire1_pulses"] < 0:
        print("Wire1 SDA held low: power cycle the rig")
    return bool(cfg["sfm_found"]) and bool(cfg["timer_ok"])


def main():
    if len(sys.argv) < 2:
        print("usage: python mechlungdatalogger.py <port>. Ports found:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")
        return
    last = json.loads(LAST.read_text()) if LAST.exists() else {"temp_c": 20.0, "p_msl_hpa": 1013.0}
    temp = ask("Room temperature (C)", last["temp_c"])
    p_msl = ask("MSL pressure (hPa)", last["p_msl_hpa"])
    LAST.write_text(json.dumps({"temp_c": temp, "p_msl_hpa": p_msl}))
    text = input("Queue, e.g. 4x20, s2x6, c600 (s = steady strokes, c = clock check seconds): ")
    queue = []
    for a, b, secs in re.findall(r"(s?[1-4])\s*x\s*(\d+)|c\s*(\d+)", text.lower()):
        queue.append(("c", int(secs)) if secs else (a, int(b)))
    if not queue:
        print("no valid queue items")
        return
    p_st = station_pressure(p_msl)
    cond = dict(temp_c=temp, p_msl_hpa=p_msl, elevation_m=ELEVATION_M, p_station_hpa=round(p_st, 2),
                flow_conv=((temp + 273.15) / 293.15) * (1013.0 / p_st), baud=BAUD)
    rig = Rig(sys.argv[1])
    time.sleep(1)
    rig.clear()
    if not sensor_check(rig.config()):
        print("stopping: fix the sensor or timer and reset the board")
        return
    results = []
    for item, n in queue:
        meta = run_item(rig, item, n, cond)
        results.append(meta)
        if meta["aborted"]:
            break
    print()
    for meta in results:
        print(summary(meta))


if __name__ == "__main__":
    main()

"""leaktest.py - runs leak tests on MechLung_LeakTest.ino (and its
MechLung_CorrectionFactorTest / MechLung_SteadyStateTest copies, and the
breakouts/B4_blocked_leak blocked-outlet test) and saves the data.

Blocked-outlet test (B4_blocked_leak, outlet plugged): queue items b1 to b5
(bN x moves, e.g. "b1x4, b2x4, b3x4, b4x4, b5x4"). Each move gives the leak from
the pot alone (piston rate at a steady pressure), the pressure it held, and the
SFM3000 flow (air leaving downstream of it: the SDP811s' flow-through and any
joint between the SFM3000 and the plug). With two or more speeds the
summary fits seal leak = k |p|^n per direction.

Usage (Windows):  python leaktest.py COM3      (no port: lists the ports)
Needs:            pip install pyserial
                  SERIAL_BAUD in the sketch = BAUD below (at least 230400)

Asks for room temperature, MSL pressure and a queue such as "4x20, 1x10, s2x6"
(preset x complete breaths; sN x complete strokes of steady flow N, needs
MechLung_SteadyStateTest). For each item it tares, turns log mode on,
runs the preset for N breaths (or strokes), returns to centre and saves:
  runs\\<date>_<hhmmss>_p<preset>x<N>\\samples.csv   raw log rows (_s<preset>x<N> for steady)
                                     events.log    # lines, after the tick they followed
                                     meta.json     conditions, constants, tare, per-breath figures
Prints one line per breath (or stroke). Ctrl+C stops the actuator, saves what it has and
marks the run aborted.
"""
import json
import math
import re
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

BAUD = 230400          # ignored over native USB (NATIVE_USB 1 in the sketch); never 1200, which resets the board
ELEVATION_M = 27       # VU Footscray Park, Building D level 3: RL 26.1 m + about 1 m bench (VU design report)
ROW_TIMEOUT_S = 2      # log rows normally arrive every 10 ms
HERE = Path(__file__).parent
LAST = HERE / "last_conditions.json"
HEADER = "tick,breath,mode,cmd_dac,pot_raw,sfm_raw," + ",".join(f"p{i}_raw" for i in range(1, 8))


def ask(prompt, default):
    text = input(f"{prompt} [{default}]: ").strip()
    return float(text) if text else default


def station_pressure(p_msl_hpa):
    return p_msl_hpa * (1 - 2.25577e-5 * ELEVATION_M) ** 5.25588


def slope(xs, ys):
    """Least squares slope of ys against xs (nan with fewer than 2 points)."""
    n = len(xs)
    if n < 2:
        return math.nan
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else math.nan


def parse_fields(line):
    """'# config a=1 b=2,3' -> {'a': 1.0, 'b': [2.0, 3.0]}"""
    out = {}
    for key, val in re.findall(r"(\w+)=(\S+)", line):
        nums = [float(v) for v in val.split(",")]
        out[key] = nums if len(nums) > 1 else nums[0]
    return out


class Rig:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.last_tick = None
        self.rows, self.events = [], []
        self.recording = False
        self.log_on = False
        self.buf = b""

    def resync(self):
        """Empty the input and drop the partial line the next read starts in: after
        a reset the first bytes can be the tail of a # line that starts with a digit."""
        self.ser.reset_input_buffer()
        self.buf = b""
        end = time.time() + 2
        while time.time() < end:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if b"\n" in chunk:
                self.buf = chunk.split(b"\n", 1)[1]
                return

    def send(self, key):
        self.ser.write(key.encode())

    def line(self):
        """Next line, or None. Rows are kept while recording; # lines are kept always."""
        # Read whatever has arrived in one call: pyserial's readline() reads one byte
        # per call, which cannot keep up with 100 rows/s on Windows.
        while b"\n" not in self.buf:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if not chunk:
                return None
            self.buf += chunk
        raw, self.buf = self.buf.split(b"\n", 1)
        raw = raw.decode("ascii", "replace").strip()
        if not raw:
            return None
        if raw[0].isdigit():
            if self.recording:
                self.rows.append(raw)
            self.last_tick = int(raw.split(",", 1)[0])
        elif raw.startswith("#"):
            self.events.append(f"{'' if self.last_tick is None else self.last_tick},{raw}")
        return raw

    def wait_for(self, prefix, timeout_s, on_row=None):
        end = time.time() + timeout_s
        while time.time() < end:
            ln = self.line()
            if ln is None:
                continue
            if on_row and ln[0].isdigit():
                on_row(ln)
            if ln.startswith(prefix):
                return ln
        raise TimeoutError(f"no '{prefix}' within {timeout_s} s")

    def idle_and_tare(self):
        """H until idle at centre, then H once more if needed so it tares. Returns the tare."""
        self.send("h")
        first = self.wait_for("# ", 10)
        while not first.startswith(("# centre", "# returning")):
            first = self.wait_for("# ", 10)
        if first.startswith("# returning"):
            self.wait_for("# centre", 30)
            self.send("h")
        tare = self.wait_for("# tare ", 30)
        if "aborted" in tare or "rejected" in tare:
            raise RuntimeError(tare)
        return parse_fields(tare)

    def log(self, on):
        if on == self.log_on:
            return None
        self.send("l")
        if on:
            cfg = self.wait_for("# ", 10)
            while not cfg.startswith(("# config", "# log mode needs")):
                cfg = self.wait_for("# ", 10)
            if cfg.startswith("# log mode needs"):
                raise RuntimeError(cfg)
            self.wait_for("# log on", 10)
        else:
            self.wait_for("# log off", 10)
        self.log_on = on
        return parse_fields(cfg) if on else None


class Breaths:
    """Works out each breath's figures from the raw rows, as the sketch does, except
    the pot uses a POT_MEDIAN-point median. The rig's pot has single and paired
    spikes in motion (up to about 1700 counts) that the sketch's 3-point median
    lets through, and a 5-point median let some through to the peak pot volume
    (run 27 Oct 11:10, breaths 1 and 4: push leak 181 and 171 mL against 150 and
    158 mL with 11 points). An N-point median rejects runs of up to (N - 1) / 2
    spikes and runs (N - 1) / 2 samples behind a moving piston."""

    POT_MEDIAN = 11
    LAG = (POT_MEDIAN - 1) // 2

    def __init__(self, cfg, tare, flow_conv):
        self.c, self.t, self.conv = cfg, tare, flow_conv
        self.pot_hist = []
        self.prev_tick = None
        self.ml_hist = []              # flow volume of the last LAG - 1 rows
        self.cur = None
        self.done = []

    def flow_lmin(self, sfm):
        if sfm == "":
            return math.nan
        slm = (int(sfm) - self.c["sfm_offset"]) / self.c["sfm_scale"]
        if abs(slm) > 200.0:           # outside the sensor's range: not a measurement
            return math.nan
        return self.c["flow_sign"] * (slm - self.t["flow_slm"]) * self.conv

    def add(self, row):
        """Feed one row. Returns a finished breath's figures, or None."""
        f = row.split(",")
        tick, breath, mode = int(f[0]), int(f[1]), int(f[2])
        self.pot_hist = (self.pot_hist + [int(f[4])])[-self.POT_MEDIAN:]
        pot = sorted(self.pot_hist)[len(self.pot_hist) // 2]
        c = self.c
        fb = c["centre_dac"] + (pot - c["pot_raw_at_centre"]) * c["pot_dac_per_adc"]
        vp = c["v_tidal_ml"] * (fb - c["centre_dac"]) / c["stroke_amp_dac"]
        dt = 1 if self.prev_tick is None else tick - self.prev_tick
        self.prev_tick = tick
        flow = self.flow_lmin(f[5])
        row_ml = flow * 1000 / 60 * dt / c["sample_hz"]
        prev_ml = sum(self.ml_hist)
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
            # the previous LAG - 1 rows' flow; this row's flow is added below.
            seed = vp + (prev_ml if not math.isnan(prev_ml) else 0.0)
            self.cur = dict(breath=breath, seed=seed, vm=seed, pk_pot=-math.inf, pk_meas=-math.inf,
                            lost=0, failed=0, flow_bad=False)
        b = self.cur
        b["flow_bad"] |= math.isnan(flow)
        b["vm"] += 0 if b["flow_bad"] else row_ml
        b["lost"] += dt - 1
        b["failed"] += sum(1 for x in f[5:] if x == "")
        b["vp"] = vp
        b["vm_hist"] = (b.get("vm_hist", []) + [b["vm"]])[-(self.LAG + 1):]   # vm at the time vp refers to
        b["pk_pot"] = max(b["pk_pot"], vp)
        b["pk_meas"] = max(b["pk_meas"], b["vm"])
        return finished

    def close(self):
        b = self.cur
        nan = math.nan
        # Push leak: rise of the pot volume minus rise of the measured volume (same seed).
        # Return leak: fall of each from its peak to the breath end. Exact even when
        # the piston is not back at centre at phase 0 (first breath from rest).
        push = nan if b["flow_bad"] else b["pk_pot"] - b["pk_meas"]
        # vp at breath end is LAG samples old (median), so compare it with vm LAG rows back.
        ret = nan if b["flow_bad"] else (b["pk_pot"] - b["vp"]) - (b["pk_meas"] - b["vm_hist"][0])
        net = nan if b["flow_bad"] else b["vm"] - b["seed"]
        out = dict(breath=b["breath"], push_leak_ml=push, return_leak_ml=ret,
                   net_ml=net, push_leak_pct=100 * push / b["pk_pot"] if b["pk_pot"] > 0 else nan,
                   lost_rows=b["lost"], failed_reads=b["failed"])
        self.done.append(out)
        return out


class Strokes:
    """Steady flow (MechLung_SteadyStateTest, modes 4 to 7): figures for each
    stroke (one "breath" in the log). mean_flow_Lmin is over the middle of the
    stroke, ss_trim of the stroke time left off each end. pot_ml and meas_ml are
    the volume moved by the piston (POT_MEDIAN-point median pot, as Breaths) and
    through the SFM3000 over the stroke, signed (+ = out of the cylinder). The
    median runs LAG rows behind a moving piston, so pot is compared with the
    flow volume LAG rows back."""

    POT_MEDIAN = Breaths.POT_MEDIAN
    LAG = Breaths.LAG

    def __init__(self, cfg, tare, flow_conv, mode):
        if "ss_out_ms" not in cfg and "ss_stroke_ms" not in cfg:
            raise RuntimeError("steady flow needs MechLung_SteadyStateTest (no ss_out_ms in # config)")
        self.c, self.mode = cfg, mode
        self.flow = Breaths(cfg, tare, flow_conv).flow_lmin
        i = mode - 4
        self.q = cfg["ss_q_Lmin"][i]
        # Out and in strokes differ once corrected; ss_stroke_ms: uncorrected sketch, both the same.
        out_ms = cfg["ss_out_ms"][i] if "ss_out_ms" in cfg else cfg["ss_stroke_ms"][i]
        in_ms = cfg["ss_in_ms"][i] if "ss_in_ms" in cfg else cfg["ss_stroke_ms"][i]
        self.out_ticks = out_ms * cfg["sample_hz"] / 1000
        self.in_ticks = in_ms * cfg["sample_hz"] / 1000
        self.pot_hist = []
        self.prev_tick = None
        self.cum = 0.0                  # flow volume since the first row, nan rows count 0
        self.cum_hist = []              # cum of the last LAG + 1 rows
        self.nan_hist = []
        self.cur = None
        self.done = []

    def add(self, row):
        """Feed one row. Returns a finished stroke's figures, or None."""
        f = row.split(",")
        tick, breath, mode = int(f[0]), int(f[1]), int(f[2])
        self.pot_hist = (self.pot_hist + [int(f[4])])[-self.POT_MEDIAN:]
        pot = sorted(self.pot_hist)[len(self.pot_hist) // 2]
        c = self.c
        fb = c["centre_dac"] + (pot - c["pot_raw_at_centre"]) * c["pot_dac_per_adc"]
        vp = c["v_tidal_ml"] * (fb - c["centre_dac"]) / c["stroke_amp_dac"]
        dt = 1 if self.prev_tick is None else tick - self.prev_tick
        self.prev_tick = tick
        flow = self.flow(f[5])
        bad = math.isnan(flow)
        self.cum += 0.0 if bad else flow * 1000 / 60 * dt / c["sample_hz"]
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
            out = int(f[3]) < c["centre_dac"]             # out strokes start at min
            self.cur = dict(breath=breath, t0=tick, vp0=vp, vm0=self.cum_hist[0], sum=0.0, n=0,
                            lost=0, failed=0, flow_bad=any(self.nan_hist),
                            ticks=self.out_ticks if out else self.in_ticks)
        b = self.cur
        b["flow_bad"] |= bad
        b["lost"] += dt - 1
        b["failed"] += sum(1 for x in f[5:] if x == "")
        b["vp"], b["vm"] = vp, self.cum_hist[0]
        t = tick - b["t0"]
        trim = c["ss_trim"] * b["ticks"]
        if trim <= t <= b["ticks"] - trim and not bad:
            b["sum"] += flow
            b["n"] += 1
        return finished

    def close(self):
        b = self.cur
        nan = math.nan
        pot = b["vp"] - b["vp0"]
        meas = nan if b["flow_bad"] else b["vm"] - b["vm0"]
        mean = nan if b["flow_bad"] or b["n"] == 0 else b["sum"] / b["n"]
        leak = abs(pot) - abs(meas)
        out = dict(breath=b["breath"], dir="out" if pot > 0 else "in", target_Lmin=self.q,
                   mean_flow_Lmin=mean, flow_pct=100 * abs(mean) / self.q,
                   pot_ml=pot, meas_ml=meas, leak_ml=leak,
                   leak_pct=100 * leak / abs(pot) if pot else nan,
                   lost_rows=b["lost"], failed_reads=b["failed"])
        self.done.append(out)
        return out


class Blocked:
    """Blocked-outlet test (B4_blocked_leak, modes 8 to 12): figures for each
    move (one "breath" in the log: the move plus the dwell after it).

    Outlet plugged, so at a steady pressure all the air the piston displaces
    leaves past the seal or a joint: leak = piston rate. Over the middle of the
    move (bl_trim of the move time off each end) a line is fitted to the pot
    volume (POT_MEDIAN-point median, as Breaths) against time; its slope is the
    piston rate. The commanded rate (cmd_dac slope) is given as a check of the
    tracking. The leak is converted to ambient volume: the displaced air is at
    cylinder pressure, so x (1 + p / P_station). No SFM3000 in it.

    p_mean_Pa: mean of the good p1..p7 over the window (all read cylinder
    pressure with the outlet plugged). p_slope_Pa_s: its drift over the window;
    the pressure should be level, and a drift of 1 Pa/s stores only about
    0.025 mL/s in 2.5 L. p_spread_Pa: largest difference between the sensors'
    window means (should be ~0 with no flow). p_rest_Pa: mean pressure over the
    last 0.5 s of the dwell (should be back near 0; if not, the tare or the
    seal holds pressure).

    downstream_Lmin: mean SFM3000 flow over the window, in the move's direction:
    air leaving between the SFM3000 and the plug. The SDP811s are flow-through
    sensors with one port open to air, so this is mainly the flow through the
    seven of them (Sensirion: about 33-43 sccm each at 100 Pa, 90-115 at
    500 Pa, wide device spread), plus any joint leak there.
    seal_Lmin = leak_Lmin - downstream_Lmin: past the seal and the joints
    upstream of the SFM3000."""

    POT_MEDIAN = Breaths.POT_MEDIAN
    REST_S = 0.5

    def __init__(self, cfg, tare, flow_conv, mode, p_station_hpa):
        if "bl_move_ms" not in cfg:
            raise RuntimeError("blocked items need B4_blocked_leak (no bl_move_ms in # config)")
        self.c, self.mode = cfg, mode
        self.flow = Breaths(cfg, tare, flow_conv).flow_lmin
        self.p_tare = [tare.get(f"p{i}_Pa", math.nan) for i in range(1, 8)]
        self.speed = cfg["bl_speed_mm_s"][mode - 8]
        hz = cfg["sample_hz"]
        self.move = cfg["bl_move_ms"] * hz / 1000
        self.cycle = (cfg["bl_move_ms"] + cfg["bl_dwell_ms"]) * hz / 1000
        self.p_st = p_station_hpa * 100.0
        self.pot_hist = []
        self.prev_tick = None
        self.cur = None
        self.done = []

    def vol(self, dac):
        c = self.c
        return c["v_tidal_ml"] * (dac - c["centre_dac"]) / c["stroke_amp_dac"]

    def add(self, row):
        """Feed one row. Returns a finished move's figures, or None."""
        f = row.split(",")
        tick, breath, mode = int(f[0]), int(f[1]), int(f[2])
        self.pot_hist = (self.pot_hist + [int(f[4])])[-self.POT_MEDIAN:]
        pot = sorted(self.pot_hist)[len(self.pot_hist) // 2]
        c = self.c
        vp = self.vol(c["centre_dac"] + (pot - c["pot_raw_at_centre"]) * c["pot_dac_per_adc"])
        vc = self.vol(int(f[3]))
        dt = 1 if self.prev_tick is None else tick - self.prev_tick
        self.prev_tick = tick
        flow = self.flow(f[5])
        p = [int(x) / c["sdp_scale"] - t if x != "" and c["sdp_scale"] else math.nan
             for x, t in zip(f[6:13], self.p_tare)]
        good = [x for x in p if not math.isnan(x)]
        pm = sum(good) / len(good) if good else math.nan
        finished = None
        if self.cur and (breath != self.cur["breath"] or mode != self.mode):
            if breath != self.cur["breath"] and mode == self.mode:   # completed, not aborted
                finished = self.close()
            self.cur = None
        if mode != self.mode:
            return finished
        if self.cur is None:
            self.cur = dict(breath=breath, t0=tick, out=int(f[3]) < c["centre_dac"],
                            ts=[], vps=[], vcs=[], pts=[], pms=[], flows=[],
                            psum=[0.0] * 7, pn=[0] * 7, rest=[], lost=0, failed=0)
        b = self.cur
        b["lost"] += dt - 1
        b["failed"] += sum(1 for x in f[5:] if x == "")
        t = tick - b["t0"]
        trim = c["bl_trim"] * self.move
        if trim <= t <= self.move - trim:
            b["ts"].append(t)
            b["vps"].append(vp)
            b["vcs"].append(vc)
            if not math.isnan(pm):
                b["pts"].append(t)
                b["pms"].append(pm)
            if not math.isnan(flow):
                b["flows"].append(flow)
            for i, x in enumerate(p):
                if not math.isnan(x):
                    b["psum"][i] += x
                    b["pn"][i] += 1
        elif self.cycle - self.REST_S * c["sample_hz"] <= t < self.cycle and not math.isnan(pm):
            b["rest"].append(pm)
        return finished

    def close(self):
        b = self.cur
        nan = math.nan
        hz = self.c["sample_hz"]
        sign = 1 if b["out"] else -1
        pot_rate = slope(b["ts"], b["vps"]) * hz                 # mL/s, + = out
        cmd_rate = slope(b["ts"], b["vcs"]) * hz
        p_mean = statistics.mean(b["pms"]) if b["pms"] else nan
        leak = abs(pot_rate) * 60 / 1000 * (1 + p_mean / self.p_st)   # L/min at ambient
        down = sign * statistics.mean(b["flows"]) if b["flows"] else nan
        means = [s / n for s, n in zip(b["psum"], b["pn"]) if n]
        expected = self.move * (1 - 2 * self.c["bl_trim"])
        out = dict(breath=b["breath"], dir="out" if b["out"] else "in", speed_mm_s=self.speed,
                   pot_rate_mLs=pot_rate, cmd_rate_mLs=cmd_rate,
                   track_pct=100 * pot_rate / cmd_rate if cmd_rate else nan,
                   leak_Lmin=leak, downstream_Lmin=down, seal_Lmin=leak - down,
                   p_mean_Pa=p_mean, p_slope_Pa_s=slope(b["pts"], b["pms"]) * hz,
                   p_spread_Pa=max(means) - min(means) if len(means) > 1 else nan,
                   p_rest_Pa=statistics.mean(b["rest"]) if b["rest"] else nan,
                   p_sensors_Pa=[s / n if n else nan for s, n in zip(b["psum"], b["pn"])],
                   window_rows=len(b["ts"]), flow_rows=len(b["flows"]),
                   ok=len(b["ts"]) >= 0.9 * expected and len(b["pms"]) >= 0.9 * expected,
                   lost_rows=b["lost"], failed_reads=b["failed"])
        self.done.append(out)
        return out


def run_item(rig, item, n, cond):
    """item: "1" to "4" (breathing preset), "s1" to "s4" (steady flow) or
    "b1" to "b5" (blocked outlet, B4_blocked_leak)."""
    kind = item[0] if item[0] in "sb" else "p"
    steady, blocked = kind == "s", kind == "b"
    preset = int(item[-1])
    mode = preset - 1 + {"p": 0, "s": 4, "b": 8}[kind]
    what = {"p": "breaths", "s": "strokes", "b": "moves"}[kind]
    rig.rows, rig.events = [], []
    start = datetime.now()
    meta = dict(cond, preset=preset, steady=steady, blocked=blocked, **{f"{what}_requested": n},
                start=start.isoformat(timespec="seconds"), aborted=False)
    an = None
    try:
        meta["tare"] = rig.idle_and_tare()
        meta["config"] = cfg = rig.log(True)
        if blocked:
            an = Blocked(cfg, meta["tare"], cond["flow_conv"], mode, cond["p_station_hpa"])
        elif steady:
            an = Strokes(cfg, meta["tare"], cond["flow_conv"], mode)
        else:
            an = Breaths(cfg, meta["tare"], cond["flow_conv"])
        rig.recording = True
        b0 = None
        rig.send(item)
        label = {"p": "preset", "s": "steady", "b": "blocked"}[kind]
        print(f"{label} {preset}: running {n} {what}"
              + (f" at {an.speed:g} mm/s" if blocked else ""))
        # Stop if rows stop arriving, or the breaths take far longer than they should.
        # Steady flow first ramps to min: up to 4095 DAC counts at 300 per s.
        # Blocked: ramps half a move to the start point at the test speed (at most
        # ramp_dac_per_s), then dwells.
        if blocked:
            span = cfg["bl_span_dac"][preset - 1]
            rate = min(cfg["ramp_dac_per_s"], span / (cfg["bl_move_ms"] / 1000))
            deadline = (time.time() + span / 2 / rate
                        + n * (cfg["bl_move_ms"] + cfg["bl_dwell_ms"]) / 1000 + 45)
        elif steady:
            deadline = time.time() + 15 + n * max(cfg["ss_out_ms"][preset - 1], cfg["ss_in_ms"][preset - 1]) / 1000 + 30
        else:
            deadline = time.time() + n * cfg["period_ms"][preset - 1] / 1000 + 30
        last_row = time.time()
        while True:
            if time.time() > deadline:
                raise TimeoutError(f"{n} {what} not complete by the expected time")
            if time.time() - last_row > ROW_TIMEOUT_S:
                raise TimeoutError(f"no log rows for {ROW_TIMEOUT_S} s")
            ln = rig.line()
            if ln and ln.startswith("# pressure limit"):
                raise RuntimeError(ln)
            if not ln or not ln[0].isdigit():
                continue
            last_row = time.time()
            f = ln.split(",")
            if b0 is None and int(f[2]) == mode:
                b0 = int(f[1])
            res = an.add(ln)
            if res and blocked:
                k = res["breath"] - b0 + 1
                print(f"  move {k:3d}/{n} {res['dir']:>3}: p {res['p_mean_Pa']:7.2f} Pa "
                      f"(drift {res['p_slope_Pa_s']:5.2f} Pa/s, rest {res['p_rest_Pa']:5.2f}), "
                      f"leak {res['leak_Lmin']:6.3f} L/min, downstream {res['downstream_Lmin']:6.3f}, "
                      f"seal {res['seal_Lmin']:6.3f}, tracking {res['track_pct']:6.2f} %, "
                      f"lost rows {res['lost_rows']}{'' if res['ok'] else ', NOT OK'}")
            elif res and steady:
                k = res["breath"] - b0 + 1
                print(f"  stroke {k:3d}/{n} {res['dir']:>3}: mean flow {res['mean_flow_Lmin']:7.2f} L/min "
                      f"({res['flow_pct']:6.2f} % of {res['target_Lmin']:g}), pot {res['pot_ml']:7.1f} mL, "
                      f"measured {res['meas_ml']:7.1f} mL, leak {res['leak_ml']:5.1f} mL ({res['leak_pct']:5.2f} %), "
                      f"lost rows {res['lost_rows']}, failed reads {res['failed_reads']}")
            elif res:
                k = res["breath"] - b0 + 1
                print(f"  breath {k:3d}/{n}: push leak {res['push_leak_ml']:6.1f} mL "
                      f"({res['push_leak_pct']:5.2f} %), return leak {res['return_leak_ml']:6.1f} mL, "
                      f"lost rows {res['lost_rows']}, failed reads {res['failed_reads']}")
            if b0 is not None and int(f[1]) >= b0 + n:
                break
        rig.send("h")
        rig.wait_for("# centre", 30)
        rig.recording = False
        rig.log(False)
    except BaseException as e:          # any failure: stop the actuator and save first
        meta["aborted"] = True
        meta["error"] = repr(e)
        rig.recording = False
        try:                            # the port itself may be the failure
            rig.send("h")
            if rig.log_on:
                rig.send("l")
                rig.log_on = False
        except Exception as e2:
            print(f"could not send H ({e2!r}): stop the actuator by hand")
        print(f"aborted ({e!r}): saving what was recorded")
    meta["end"] = datetime.now().isoformat(timespec="seconds")
    meta["rows"] = len(rig.rows)
    meta[what] = an.done if an else []
    save(start, kind + str(preset), n, meta, rig)
    return meta


def save(start, tag, n, meta, rig):
    folder = HERE / "runs" / f"{start:%Y-%m-%d_%H%M%S}_{tag}x{n}"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "samples.csv").write_text(HEADER + "\n" + "\n".join(rig.rows) + "\n")
    (folder / "events.log").write_text("tick,line\n" + "\n".join(rig.events) + "\n")
    (folder / "meta.json").write_text(json.dumps(meta, indent=2, default=str))
    print(f"saved {folder}")


def summary(meta):
    if meta.get("blocked"):
        return blocked_summary(meta)
    if meta.get("steady"):
        return steady_summary(meta)
    b = [x for x in meta["breaths"] if not math.isnan(x["push_leak_ml"])]
    if not b:
        return (f"preset {meta['preset']}: no complete breaths with good flow"
                + (f" (ABORTED: {meta.get('error', '')})" if meta["aborted"] else ""))
    push = [x["push_leak_pct"] for x in b]
    ret = [x["return_leak_ml"] for x in b]
    sd = statistics.stdev(push) if len(push) > 1 else 0.0
    lost = sum(x["lost_rows"] for x in meta["breaths"])
    return (f"preset {meta['preset']}: {len(b)} breaths, push leak {statistics.mean(push):.2f} "
            f"+/- {sd:.2f} %, return leak {statistics.mean(ret):.1f} mL, lost rows {lost}"
            + (" (ABORTED)" if meta["aborted"] else ""))


def steady_summary(meta):
    name = f"steady {meta['preset']}"
    tail = " (ABORTED)" if meta["aborted"] else ""
    lines = []
    for d in ("out", "in"):
        s = [x for x in meta["strokes"] if x["dir"] == d and not math.isnan(x["mean_flow_Lmin"])]
        if not s:
            continue
        pct = [x["flow_pct"] for x in s]
        sd = statistics.stdev(pct) if len(pct) > 1 else 0.0
        leak = statistics.mean(x["leak_pct"] for x in s)
        lines.append(f"{name} {d}: {len(s)} strokes, mean flow {statistics.mean(pct):.2f} +/- {sd:.2f} % "
                     f"of {s[0]['target_Lmin']:g} L/min, leak {leak:.2f} %")
    if not lines:
        return (f"{name}: no complete strokes with good flow"
                + (f" (ABORTED: {meta.get('error', '')})" if meta["aborted"] else ""))
    lost = sum(x["lost_rows"] for x in meta["strokes"])
    return "\n".join(lines) + f"\n{name}: lost rows {lost}{tail}"


def good_moves(meta):
    """Moves that count: complete window, no lost rows, a pressure and a leak."""
    return [x for x in meta.get("moves", []) if x["ok"] and x["lost_rows"] == 0
            and not math.isnan(x["leak_Lmin"]) and not math.isnan(x["p_mean_Pa"])]


def blocked_summary(meta):
    name = f"blocked {meta['preset']}"
    lines = []
    for d in ("out", "in"):
        s = [x for x in good_moves(meta) if x["dir"] == d]
        if not s:
            continue
        def ms(key):
            v = [x[key] for x in s if not math.isnan(x[key])]
            return (statistics.mean(v) if v else math.nan,
                    statistics.stdev(v) if len(v) > 1 else 0.0)
        leak, leak_sd = ms("leak_Lmin")
        lines.append(f"{name} {d}: {len(s)} moves at {s[0]['speed_mm_s']:g} mm/s, p {ms('p_mean_Pa')[0]:.2f} Pa, "
                     f"leak {leak:.3f} +/- {leak_sd:.3f} L/min, downstream {ms('downstream_Lmin')[0]:.3f} "
                     f"({ms('downstream_Lmin')[0] * 1000 / 7:.0f} sccm per SDP811), "
                     f"seal {ms('seal_Lmin')[0]:.3f} L/min, rest p {ms('p_rest_Pa')[0]:.2f} Pa, "
                     f"tracking {ms('track_pct')[0]:.2f} %")
    if not lines:
        return (f"{name}: no complete moves"
                + (f" (ABORTED: {meta.get('error', '')})" if meta["aborted"] else ""))
    return "\n".join(lines) + (" (ABORTED)" if meta["aborted"] else "")


def blocked_fit(results):
    """Seal leak = k |p|^n per direction, least squares in log-log over every
    good move of every blocked item (needs two or more speeds)."""
    out = []
    for d in ("out", "in"):
        pts = [(abs(x["p_mean_Pa"]), x["seal_Lmin"]) for m in results if m.get("blocked")
               for x in good_moves(m) if x["dir"] == d and x["seal_Lmin"] > 0 and abs(x["p_mean_Pa"]) > 0]
        speeds = {x["speed_mm_s"] for m in results if m.get("blocked") for x in good_moves(m) if x["dir"] == d}
        if len(speeds) < 2:
            continue
        lx = [math.log(p) for p, _ in pts]
        ly = [math.log(q) for _, q in pts]
        n = slope(lx, ly)
        k = math.exp(statistics.mean(ly) - n * statistics.mean(lx))
        out.append(f"blocked fit {d}: seal leak = {k:.4g} x |p|^{n:.3f} L/min (p in Pa), "
                   f"{len(pts)} moves, {min(p for p, _ in pts):.1f} to {max(p for p, _ in pts):.1f} Pa")
    return out


def main():
    if len(sys.argv) < 2:
        print("usage: python leaktest.py <port>. Ports found:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")
        return
    last = json.loads(LAST.read_text()) if LAST.exists() else {"temp_c": 20.0, "p_msl_hpa": 1013.0}
    temp = ask("Room temperature (C)", last["temp_c"])
    p_msl = ask("MSL pressure (hPa)", last["p_msl_hpa"])
    LAST.write_text(json.dumps({"temp_c": temp, "p_msl_hpa": p_msl}))
    queue = [(a.lower(), int(b)) for a, b in
             re.findall(r"([sSbB]?\d)\s*x\s*(\d+)",
                        input("Queue, e.g. 4x20, 1x10, s2x6 (s = steady strokes, b1-b5 = blocked moves): "))]
    bad = [a for a, _ in queue if not re.fullmatch(r"[1-4]|s[1-4]|b[1-5]", a)]
    if bad:
        print(f"not valid queue items: {', '.join(bad)} (1-4, s1-s4, b1-b5)")
        return
    if not queue:
        print("no valid queue items")
        return
    p_st = station_pressure(p_msl)
    cond = dict(temp_c=temp, p_msl_hpa=p_msl, elevation_m=ELEVATION_M, p_station_hpa=round(p_st, 2),
                flow_conv=((temp + 273.15) / 293.15) * (1013.0 / p_st), baud=BAUD)
    rig = Rig(sys.argv[1])
    time.sleep(3)                      # the board may reset when the port opens
    rig.resync()
    results = []
    for item, n in queue:
        meta = run_item(rig, item, n, cond)
        results.append(meta)
        if meta["aborted"]:
            break
    print()
    for meta in results:
        print(summary(meta))
    for ln in blocked_fit(results):
        print(ln)


if __name__ == "__main__":
    main()

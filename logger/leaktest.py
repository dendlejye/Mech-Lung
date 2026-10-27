"""leaktest.py - runs leak tests on MechLung_LeakTest.ino (and its
MechLung_CorrectionFactorTest / MechLung_SteadyStateTest copies) and saves the data.

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
        if "ss_stroke_ms" not in cfg:
            raise RuntimeError("steady flow needs MechLung_SteadyStateTest (no ss_stroke_ms in # config)")
        self.c, self.mode = cfg, mode
        self.flow = Breaths(cfg, tare, flow_conv).flow_lmin
        i = mode - 4
        self.q = cfg["ss_q_Lmin"][i]
        self.stroke_ticks = cfg["ss_stroke_ms"][i] * cfg["sample_hz"] / 1000
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
            self.cur = dict(breath=breath, t0=tick, vp0=vp, vm0=self.cum_hist[0], sum=0.0, n=0,
                            lost=0, failed=0, flow_bad=any(self.nan_hist))
        b = self.cur
        b["flow_bad"] |= bad
        b["lost"] += dt - 1
        b["failed"] += sum(1 for x in f[5:] if x == "")
        b["vp"], b["vm"] = vp, self.cum_hist[0]
        t = tick - b["t0"]
        trim = c["ss_trim"] * self.stroke_ticks
        if trim <= t <= self.stroke_ticks - trim and not bad:
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


def run_item(rig, item, n, cond):
    """item: "1" to "4" (breathing preset) or "s1" to "s4" (steady flow)."""
    steady = item.startswith("s")
    preset = int(item[-1])
    mode = preset - 1 + (4 if steady else 0)
    what = "strokes" if steady else "breaths"
    rig.rows, rig.events = [], []
    start = datetime.now()
    meta = dict(cond, preset=preset, steady=steady, **{f"{what}_requested": n},
                start=start.isoformat(timespec="seconds"), aborted=False)
    an = None
    try:
        meta["tare"] = rig.idle_and_tare()
        meta["config"] = cfg = rig.log(True)
        if steady:
            an = Strokes(cfg, meta["tare"], cond["flow_conv"], mode)
        else:
            an = Breaths(cfg, meta["tare"], cond["flow_conv"])
        rig.recording = True
        b0 = None
        rig.send(item)
        print(f"{'steady' if steady else 'preset'} {preset}: running {n} {what}")
        # Stop if rows stop arriving, or the breaths take far longer than they should.
        # Steady flow first ramps to min: up to 4095 DAC counts at 300 per s.
        if steady:
            deadline = time.time() + 15 + n * cfg["ss_stroke_ms"][preset - 1] / 1000 + 30
        else:
            deadline = time.time() + n * cfg["period_ms"][preset - 1] / 1000 + 30
        last_row = time.time()
        while True:
            if time.time() > deadline:
                raise TimeoutError(f"{n} breaths not complete by the expected time")
            if time.time() - last_row > ROW_TIMEOUT_S:
                raise TimeoutError(f"no log rows for {ROW_TIMEOUT_S} s")
            ln = rig.line()
            if not ln or not ln[0].isdigit():
                continue
            last_row = time.time()
            f = ln.split(",")
            if b0 is None and int(f[2]) == mode:
                b0 = int(f[1])
            res = an.add(ln)
            if res and steady:
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
    meta["strokes" if steady else "breaths"] = an.done if an else []
    save(start, ("s" if steady else "p") + str(preset), n, meta, rig)
    return meta


def save(start, tag, n, meta, rig):
    folder = HERE / "runs" / f"{start:%Y-%m-%d_%H%M%S}_{tag}x{n}"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "samples.csv").write_text(HEADER + "\n" + "\n".join(rig.rows) + "\n")
    (folder / "events.log").write_text("tick,line\n" + "\n".join(rig.events) + "\n")
    (folder / "meta.json").write_text(json.dumps(meta, indent=2, default=str))
    print(f"saved {folder}")


def summary(meta):
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
             re.findall(r"([sS]?[1-4])\s*x\s*(\d+)", input("Queue, e.g. 4x20, 1x10, s2x6 (s = steady strokes): "))]
    if not queue:
        print("no valid queue items")
        return
    p_st = station_pressure(p_msl)
    cond = dict(temp_c=temp, p_msl_hpa=p_msl, elevation_m=ELEVATION_M, p_station_hpa=round(p_st, 2),
                flow_conv=((temp + 273.15) / 293.15) * (1013.0 / p_st), baud=BAUD)
    rig = Rig(sys.argv[1])
    time.sleep(3)                      # the board may reset when the port opens
    rig.ser.reset_input_buffer()
    rig.buf = b""
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

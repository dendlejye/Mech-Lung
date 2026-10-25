"""leaktest.py - runs leak tests on MechLung_LeakTest.ino and saves the data.

Usage (Windows):  python leaktest.py COM3      (no port: lists the ports)
Needs:            pip install pyserial
                  SERIAL_BAUD in the sketch = BAUD below (at least 230400)

Asks for room temperature, MSL pressure and a queue such as "4x20, 1x10"
(preset x complete breaths). For each item it tares, turns log mode on,
runs the preset for N breaths, returns to centre and saves:
  runs\\<date>_<hhmmss>_p<preset>x<N>\\samples.csv   raw log rows
                                     events.log    # lines, after the tick they followed
                                     meta.json     conditions, constants, tare, per-breath figures
Prints one line per breath. Ctrl+C stops the actuator, saves what it has and
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

BAUD = 230400          # must match SERIAL_BAUD in the sketch
ELEVATION_M = 35       # VU Footscray Park, Building D level 3 (25 m ground + 10 m)
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

    def send(self, key):
        self.ser.write(key.encode())

    def line(self):
        """Next line, or None. Rows are kept while recording; # lines are kept always."""
        raw = self.ser.readline().decode("ascii", "replace").strip()
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
        if "aborted" in tare:
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
    """Works out each breath's figures from the raw rows, the same way the sketch does."""

    def __init__(self, cfg, tare, flow_conv):
        self.c, self.t, self.conv = cfg, tare, flow_conv
        self.pot_hist = []
        self.prev_tick = None
        self.cur = None
        self.done = []

    def flow_lmin(self, sfm):
        if sfm == "":
            return math.nan
        slm = max(-200.0, min(200.0, (int(sfm) - self.c["sfm_offset"]) / self.c["sfm_scale"]))
        return self.c["flow_sign"] * (slm - self.t["flow_slm"]) * self.conv

    def add(self, row):
        """Feed one row. Returns a finished breath's figures, or None."""
        f = row.split(",")
        tick, breath, mode = int(f[0]), int(f[1]), int(f[2])
        self.pot_hist = (self.pot_hist + [int(f[4])])[-3:]
        pot = sorted(self.pot_hist)[1] if len(self.pot_hist) == 3 else self.pot_hist[-1]
        c = self.c
        fb = c["centre_dac"] + (pot - c["pot_raw_at_centre"]) * c["pot_dac_per_adc"]
        vp = c["v_tidal_ml"] * (fb - c["centre_dac"]) / c["stroke_amp_dac"]
        dt = 1 if self.prev_tick is None else tick - self.prev_tick
        self.prev_tick = tick
        finished = None
        if self.cur and (breath != self.cur["breath"] or mode < 0):
            if breath != self.cur["breath"] and mode >= 0:   # completed, not aborted
                finished = self.close()
            self.cur = None
        if mode < 0:
            return finished
        if self.cur is None:
            self.cur = dict(breath=breath, seed=vp, vm=vp, pk_pot=-math.inf, pk_meas=-math.inf,
                            lost=0, failed=0, flow_bad=False)
        b = self.cur
        flow = self.flow_lmin(f[5])
        b["flow_bad"] |= math.isnan(flow)
        b["vm"] += 0 if b["flow_bad"] else flow * 1000 / 60 * dt / c["sample_hz"]
        b["lost"] += dt - 1
        b["failed"] += sum(1 for x in f[5:] if x == "")
        b["vp"] = vp
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
        ret = nan if b["flow_bad"] else (b["pk_pot"] - b["vp"]) - (b["pk_meas"] - b["vm"])
        net = nan if b["flow_bad"] else b["vm"] - b["seed"]
        out = dict(breath=b["breath"], push_leak_ml=push, return_leak_ml=ret,
                   net_ml=net, push_leak_pct=100 * push / b["pk_pot"] if b["pk_pot"] > 0 else nan,
                   lost_rows=b["lost"], failed_reads=b["failed"])
        self.done.append(out)
        return out


def run_item(rig, preset, n, cond):
    rig.rows, rig.events = [], []
    start = datetime.now()
    meta = dict(cond, preset=preset, breaths_requested=n, start=start.isoformat(timespec="seconds"),
                aborted=False)
    an = None
    try:
        meta["tare"] = rig.idle_and_tare()
        meta["config"] = cfg = rig.log(True)
        an = Breaths(cfg, meta["tare"], cond["flow_conv"])
        rig.recording = True
        b0 = None
        rig.send(str(preset))
        print(f"preset {preset}: running {n} breaths")
        # Stop if rows stop arriving, or the breaths take far longer than they should.
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
            if b0 is None and int(f[2]) == preset - 1:
                b0 = int(f[1])
            res = an.add(ln)
            if res:
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
    meta["breaths"] = an.done if an else []
    save(start, preset, n, meta, rig)
    return meta


def save(start, preset, n, meta, rig):
    folder = HERE / "runs" / f"{start:%Y-%m-%d_%H%M%S}_p{preset}x{n}"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "samples.csv").write_text(HEADER + "\n" + "\n".join(rig.rows) + "\n")
    (folder / "events.log").write_text("tick,line\n" + "\n".join(rig.events) + "\n")
    (folder / "meta.json").write_text(json.dumps(meta, indent=2, default=str))
    print(f"saved {folder}")


def summary(meta):
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
    queue = [(int(a), int(b)) for a, b in re.findall(r"([1-4])\s*x\s*(\d+)", input("Queue, e.g. 4x20, 1x10: "))]
    if not queue:
        print("no valid queue items")
        return
    p_st = station_pressure(p_msl)
    cond = dict(temp_c=temp, p_msl_hpa=p_msl, elevation_m=ELEVATION_M, p_station_hpa=round(p_st, 2),
                flow_conv=((temp + 273.15) / 293.15) * (1013.0 / p_st), baud=BAUD)
    rig = Rig(sys.argv[1])
    time.sleep(3)                      # the board may reset when the port opens
    rig.ser.reset_input_buffer()
    results = []
    for preset, n in queue:
        meta = run_item(rig, preset, n, cond)
        results.append(meta)
        if meta["aborted"]:
            break
    print()
    for meta in results:
        print(summary(meta))


if __name__ == "__main__":
    main()

# Mech-Lung

Arduino UNO R4 WiFi sketches for the piston-cylinder mechanical lung: a Jrk G2
driven actuator, an SFM3000 flow sensor and seven SDP811 pressure sensors.

## Layout

| Path | What it is |
|---|---|
| `MechLung_LeakTest/` | Merged sketch: actuator, flow and pressure at 100 Hz, per-breath volume and leak check. Plot mode for the Arduino IDE Serial Plotter; log mode (key L) streams raw 100 Hz rows for `leaktest.py`. |
| `logger/leaktest.py` | Runs leak tests from the PC (Windows, Python 3 + pyserial): asks T, P and a queue, drives the sketch, saves each run under `logger/runs/`. |
| `breakouts/B1_serial_throughput/` | Serial throughput test and `b1_reader.py` (PC gap counter). Sets `SERIAL_BAUD`. |
| `breakouts/B2_timer_dac/` | 1 kHz timer interrupt writing the DAC register directly. Disconnect the Jrk from A0 first. |
| `breakouts/B3_plotter/` | Arduino IDE 2 Serial Plotter window size and nan display. Sets `PLOT_DIV`. |

Each sketch's header comment says how to run it and what passes.

## Order

1. Run B1 (with `LINE_BYTES = 68`), B2 and B3. Copy the results into
   `MechLung_LeakTest.ino` (`SERIAL_BAUD`, at least 230400 for log mode, and
   `PLOT_DIV`) and set `BAUD` in `leaktest.py` to the same baud.
2. Set `TARE_SETTLE_MS` and `TARE_STILL_COUNTS` on the bench (marked PLACEHOLDER).
3. Set the Jrk travel limits, then run `MechLung_LeakTest`.
4. Leak tests: close the Arduino IDE (it holds the port), then
   `pip install pyserial` once and `python leaktest.py COM3`.

## Leak test run

`leaktest.py` asks for room temperature (C), MSL pressure (hPa, from BoM) and a
queue such as `4x20, 1x10` (preset x complete breaths). It converts MSL to
station pressure with `ELEVATION_M = 27` (VU Footscray Park, Building D level 3: RL 26.1 m plus bench).
Then, with no further input, for each item it: sends H until idle at centre and
tared, turns log mode on, starts the preset, counts N complete breaths, sends H,
waits for centre and turns log mode off. One console line per breath: push leak,
return leak, leak %, lost rows, failed reads. Ctrl+C stops the actuator, saves
and marks the run aborted.

Each item is saved in `logger/runs/<date>_<hhmmss>_p<preset>x<N>/`:

- `samples.csv`: `tick,breath,mode,cmd_dac,pot_raw,sfm_raw,p1_raw..p7_raw`, raw
  untared integers at 100 Hz; an empty field is a failed read, a tick gap a lost row.
- `events.log`: the # lines, each with the tick it followed.
- `meta.json`: conditions, the sketch's `# config` constants, the tare, and the
  per-breath figures.

Push leak = pot volume rise minus measured volume rise over the push. Return
leak = the same for the fall from the peak to breath end. Attach the run folders
here for analysis.

## Status

Not yet compiled for the R4 or run on hardware. Checked on a PC against a
simulated rig only.

## Volume traces

- `vol_pot_ml`: piston position from the pot, as volume from centre.
- `vol_measured_ml`: flow sensor integrated over the breath. It starts each breath
  at `vol_pot_ml`, not 0, so both traces measure from centre.

Piston lag (5 Hz filter plus Jrk and actuator) leaves the piston off centre when
the command reaches phase 0. Starting `vol_measured_ml` at 0 there made the leak
read high on every breath after the first, and more at higher flow (simulated:
37 mL read for a true 14 mL at 30 L/min with 150 ms lag). With the seed the
simulated leak reads 12 to 17 mL for a true 14 mL at all presets. The remaining
spread is about one pot step (1.4 mL).

Cost: `vol_measured_ml` includes one pot reading at breath start, so a pot error
at that sample (about 1.4 mL) goes into the leak figure. `net_ml` subtracts the
seed, so it is still flow-sensor only.

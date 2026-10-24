# Mech-Lung

Arduino UNO R4 WiFi sketches for the piston-cylinder mechanical lung: a Jrk G2
driven actuator, an SFM3000 flow sensor and seven SDP811 pressure sensors.

## Layout

| Path | What it is |
|---|---|
| `MechLung_NoLogger/` | Step 1 merged sketch: actuator, flow and pressure at 100 Hz, per-breath volume and leak check. Arduino IDE Serial Plotter / Monitor only. |
| `breakouts/B1_serial_throughput/` | Serial throughput test and `b1_reader.py` (PC gap counter). Sets `SERIAL_BAUD`. |
| `breakouts/B2_timer_dac/` | 1 kHz timer interrupt writing the DAC register directly. Disconnect the Jrk from A0 first. |
| `breakouts/B3_plotter/` | Arduino IDE 2 Serial Plotter window size and nan display. Sets `PLOT_DIV`. |

Each sketch's header comment says how to run it and what passes.

## Order

1. Run B1, B2 and B3, and copy their results into `MechLung_NoLogger.ino`
   (`SERIAL_BAUD`, `PLOT_DIV`).
2. Set `TARE_SETTLE_MS` and `TARE_STILL_COUNTS` on the bench (marked PLACEHOLDER).
3. Set the Jrk travel limits, then run `MechLung_NoLogger`.

## Status

Not yet compiled for the R4 or run on hardware. Known open issue: the per-breath
leak figure reads high on every breath after the first, because piston lag leaves
the piston off centre at phase 0 while `vol_measured_ml` resets to 0 there.
The first breath from idle is unaffected.

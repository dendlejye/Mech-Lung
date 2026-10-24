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

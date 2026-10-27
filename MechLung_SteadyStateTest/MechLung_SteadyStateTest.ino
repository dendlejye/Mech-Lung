/*
  MechLung_SteadyStateTest.ino

  MechLung_CorrectionFactorTest with the demo sine replaced by four steady
  flow modes (keys s1 to s4, same flows as presets 1 to 4). The piston moves
  at constant speed from min to max and back over the calibrated Jrk window,
  so the flow holds at the target for each whole stroke except the reversals.
  Steady modes have no flow correction yet. MechLung_CorrectionFactorTest
  stays as the backup. Same log format; leaktest.py runs all three sketches.

  Runs the actuator breathing profile, the SFM3000 flow sensor and seven
  SDP811 pressure sensors together, and checks the volume delivered each
  breath against the commanded 700 mL. Arduino IDE Serial Plotter / Monitor.

  Hardware (Arduino UNO R4 WiFi)
    A0     12-bit DAC output -> Jrk G2 analog input
    A1     12-bit ADC input  <- position potentiometer
    Wire   SFM3000 flow sensor, address 0x40 (4.7k pull-ups), 100 kHz
    Wire1  Qwiic -> PCA9548 mux 0x70, SDP811 0x26 on channels 0 to 6 (p1 to p7)

  How it runs
    A 1 kHz timer interrupt computes the profile, the filter and the ramps and
    writes the DAC, so sensor reads and Serial cannot delay the actuator.
    Every 10th interrupt is a 100 Hz sample tick: loop() reads the SFM3000,
    p1 to p7 and the pot, updates the volumes and sends at most one line.

  Keys (over the PC link, see NATIVE_USB; not case sensitive)
    1 2 3 4      breathe at a peak flow of 10, 15, 20 or 30 L/min
    s1 s2 s3 s4  steady flow of 10, 15, 20 or 30 L/min: ramp to min at
                 RAMP_RATE_DAC_PER_S, then constant-speed strokes min -> max
                 -> min until H. One stroke takes ss_stroke_ms (# config):
                 about 17.4, 11.6, 8.7 and 5.8 s for the 2.9 L stroke.
    max min      go to that end of the calibrated Jrk window and hold
    H            stop: return to centre at RAMP_RATE_DAC_PER_S. Tares if already idle.
    p            cycle plot set
    L            log mode on / off (for leaktest.py)
    A breathing key while breathing or in steady flow ramps to centre first,
    then starts. A steady key ramps straight to min from wherever the command
    is. While returning to centre start keys are refused; at max or min (or
    on the way there) send H first.

  Steady flow: min -> max pushes air out of the cylinder (+ flow), max -> min
  draws it in. The breath counter goes up at each end, so each stroke is one
  "breath" in the log, in one direction. The 5 Hz command filter rounds each
  reversal over about 64 ms.

  Plot sets (one row every PLOT_DIV sample ticks)
    1  flow_Lmin_x20, vol_cmd_ml, vol_pot_ml, vol_measured_ml
    2  leak_gap_ml (vol_pot - vol_measured), track_err_ml (vol_cmd - vol_pot)
    3  p1 to p7 in Pa
    4  command_dac, feedback_dac
    Flow and pressures are means over the row, volumes the latest value.
    Volume and gap channels print 0 outside breathing and steady flow. A failed
    read prints nan. vol_measured_ml starts each breath (or stroke) at
    vol_pot_ml, so both measure from centre even when piston lag leaves the
    piston off centre at phase 0. In steady flow the volumes run from about
    -1450 mL (min) to +1450 mL (max).

  Lines starting with # are comments: start-up checks, tare, mode changes,
  "# breath" once per breath, "# stroke" once per steady stroke, "# breath
  aborted", the first read failure of each sensor, and "# status" once per second.
  "# stroke" gives the mean flow over the middle of the stroke (SS_TRIM_FRAC
  left off each end) against the target, and the pot and measured volume moved.

  Log mode (key L, needs SERIAL_BAUD >= LOG_MIN_BAUD)
    L prints "# config ..." (the constants needed to convert the raw values)
    and "# log on", then one CSV row per 100 Hz sample tick:
      tick,breath,mode,cmd_dac,pot_raw,sfm_raw,p1_raw,...,p7_raw
    Raw integers as the sensors returned them, untared. A failed read is an
    empty field. mode: 0 to 3 breathing presets, 4 to 7 steady s1 to s4,
    negative idle / returning / moving to an end / holding. No plot rows,
    # status, # breath or # stroke lines: they are worked out from the rows.
    Short event lines are sent after a row only if they fit in the tick.
    L again: "# log off".

  Travel limits are set inside the Jrk, not here. Set them before the first run.
*/
#include <Wire.h>
#include <SensirionCore.h>   // Library Manager: "Sensirion Core"
#include "FspTimer.h"

// Actuator Calibration Constants Copy from Calibration Sketch
const int   JRK_CENTRE = 2050;
const int   JRK_HALF_RANGE = 2001;
const float DAC_CORR_OFFSET = 4.83f;
const float DAC_CORR_GAIN = 4159.28f;
const float POT_RAW_AT_CENTRE = 1168.41f;
const float POT_DAC_COUNTS_PER_ADC = 1.8156f;
// Record of this calibration run (26 Oct 2026):
// Jrk Input window (Input setup wizard): Minimum=48 Maximum=4051
// Jrk Feedback (Feedback setup wizard): forward=2292 reverse=88
// A1 at the ends: forward=2252.4 reverse=82.8
// Step 3 point 1: applied=648 Jrk Input=663 Jrk Feedback=427 A1=415.3
// Step 3 point 2: applied=2050 Jrk Input=2087 Jrk Feedback=1212 A1=1188.9
// Step 3 point 3: applied=3451 Jrk Input=3510 Jrk Feedback=1996 A1=1960.5
// A1 vs Jrk Feedback cross-check: worst difference 1.2 Jrk counts (0.06 % of span)
// Step 4 min: command=43 feedback=42.3 error=-0.7 counts (-0.02 % of span)
// Step 4 mid: command=2014 feedback=2016.9 error=2.9 counts (0.07 % of span)
// Step 4 max: command=3984 feedback=3990.8 error=6.8 counts (0.17 % of span)
// Verification: PASS

const int DAC_MAX = 4095;             // 12-bit DAC full scale

// Rig and breathing settings.
// ACTUATOR_STROKE_MM: distance between the min and max ends of the calibrated
// Jrk window, measured with the min / max commands. Remeasured on this rig: 221 mm
// (27 Oct 2026; was 230 mm, which moved the piston 4% short of the commanded volume
// and showed up as a 5% gap at every preset with no model fitted).
// If tidal volume or peak flow measure off on the rig, re-check it first.
const float ACTUATOR_STROKE_MM = 221.0f;
const float CYLINDER_DIAMETER_MM = 129.15f;   // bore of the cylinder, measured with callipers
const float V_TIDAL_ML = 700.0f;              // volume moved per breath
const float IE_EXP_PARTS = 2.0f;              // expiration : inspiration time ratio
const float IE_INSP_PARTS = 1.0f;
float preset_q_peak_Lmin[4] = {10.0f, 15.0f, 20.0f, 30.0f};  // keys 1 to 4
// Flow correction: scales each preset so the peak flow measured at the SFM3000
// meets the target, making up for the seal leak (target / measured, model
// connected, 221 mm stroke, 19.7 C, 1021.1 hPa MSL: uncorrected runs of 28 Oct
// 2026 08:47). The 27 Oct 15:23 gains overshot by 1 to 1.8 % after the actuator's
// tracking improved from about 15 to 6 mL short; re-check with an uncorrected run
// at the start of each session.
// The piston has to return to centre, so both strokes move the same volume:
//   GAIN_EXP scales the stroke amplitude, so expiratory peak flow x GAIN_EXP;
//   inspiration time is scaled by GAIN_EXP / GAIN_INSP, so inspiratory peak
//   flow x GAIN_INSP. The breath gets slightly longer (preset 1: +0.6 %, I:E
//   1:1.96) and moves 700 x GAIN_EXP mL, measured volume stays physical.
// Preset 1 expiration is corrected to peak flow; its volume stays about 5 %
// short (the push loss follows the oscillation, not the stroke shape).
// FLOW_GAIN_ON false: no correction (all gains 1).
const bool FLOW_GAIN_ON = true;
const float GAIN_EXP[4] = {1.008f, 1.003f, 1.005f, 1.014f};
const float GAIN_INSP[4] = {0.984f, 0.993f, 0.997f, 1.012f};

// Timing.
const int DAC_UPDATE_HZ = 1000;       // timer rate: DAC written every 1 ms
const int SAMPLE_HZ = 100;            // sensor sample rate, must divide DAC_UPDATE_HZ
const int TICKS_PER_SAMPLE = DAC_UPDATE_HZ / SAMPLE_HZ;
const uint8_t TIMER_PRIORITY = 4;     // above I2C, USB, serial (12) and millis (8)
const int PLOT_DIV = 40;              // sample ticks per plot row (2.5 rows/s)
const unsigned long LOG_MIN_BAUD = 230400; // a log row plus the sensor burst fits 10 ms
const unsigned long TICK_BUDGET_US = 1000000UL / SAMPLE_HZ - 1500;  // Serial time allowed per tick
// PC link. NATIVE_USB 1: the USB-C port is switched to the RA4M1's own USB at
// boot, bypassing the ESP32-S3 bridge, whose serial output stalls under
// continuous 100 Hz logging (B1 tests). The baud is then ignored and the
// board shows up as a new COM port. To upload: double-tap reset, then upload
// (retry once if it fails). NATIVE_USB 0: Serial through the bridge as before.
#define NATIVE_USB 1
#if NATIVE_USB
#define PC SerialUSB
const unsigned long SERIAL_BAUD = 12000000; // nominal USB full speed: only sets the line time budget
#else
#define PC Serial
const unsigned long SERIAL_BAUD = 115200;   // set from breakout sketch B1
#endif

// Steady flow (keys s1 to s4): same flows as the presets, no correction yet.
// The stroke runs over the calibrated Jrk window less SS_MARGIN_JRK at each end.
const int SS_MARGIN_JRK = 0;               // stay this far from the Jrk window ends
const float SS_TRIM_FRAC = 0.10f;          // # stroke mean flow leaves this much of the stroke time off each end

// Smoothing: two cascaded low-pass stages, delay about 1 / (PI * CMD_FILTER_HZ)
// (64 ms at 5 Hz). 0 = off.
const float CMD_FILTER_HZ = 5.0f;
// Speed of H and of the max / min moves, DAC counts per second.
const float RAMP_RATE_DAC_PER_S = 300.0f;
const float RAMP_STEP_DAC = RAMP_RATE_DAC_PER_S / DAC_UPDATE_HZ;

// Flow conversion, standard litres to L/min at ambient. Set T and P per
// session: room thermometer, local station pressure (not sea level).
const float AMBIENT_TEMP_C = 20.0f;
const float AMBIENT_PRESSURE_HPA = 1013.0f;
const float FLOW_SIGN = 1.0f;         // + = out of cylinder. Flip if the plot is mirrored.
const float SLM_TO_LMIN = ((AMBIENT_TEMP_C + 273.15f) / 293.15f) * (1013.0f / AMBIENT_PRESSURE_HPA);
const float SFM3000_OFFSET = 32000.0f;  // raw reading at zero flow (datasheet)
const float SFM3000_SCALE = 140.0f;     // raw counts per slm, air (datasheet)
const float SFM3000_MAX_SLM = 200.0f;   // sensor range: a word outside it is a failed read

// Tare: after the pot stays within TARE_STILL_COUNTS for TARE_SETTLE_MS,
// average TARE_SAMPLES of flow and of each pressure.
const int TARE_SAMPLES = 100;
const unsigned long TARE_SETTLE_MS = 2000;  // PLACEHOLDER: set on the bench from pot noise at rest
const float TARE_STILL_COUNTS = 15.0f;      // DAC counts: pot noise at rest spans 9 to 11 (rig, 26 Oct 2026)
// A flow tare outside +-TARE_FLOW_MAX_SLM is not a zero (good tares read 0.004 to
// 0.009 slm): the line reads "# tare rejected ...", the flow tare is nan, so flow
// stays nan until a good tare (H when idle), and leaktest.py stops.
const float TARE_FLOW_MAX_SLM = 1.0f;

// I2C.
const uint8_t SFM3000_ADDR = 0x40;
const uint8_t MUX_ADDR = 0x70;
const uint8_t SDP_ADDR = 0x26;
// Wire (SFM3000) at 100 kHz: at 400 kHz with 4 reads per tick the SFM3000 locked
// up (27 Oct 2026; after a board reset it NACKed its address until power cycled).
// This core only honours 100000, 400000 and 1000000.
const uint32_t WIRE_CLOCK_HZ = 100000;
const uint32_t WIRE1_CLOCK_HZ = 400000;
const unsigned int WIRE_TIMEOUT_US = 3000;   // a 3-byte read takes about 0.4 ms at 100 kHz
const unsigned int WIRE1_TIMEOUT_US = 1000;
// Wire1 bus recovery at boot: a reset during an SDP811 read can leave the sensor
// holding SDA low until power is removed (seen on the rig: mux_err every tick after
// a reset, cleared only by a power cycle). Up to 9 SCL pulses let it finish its
// byte and release SDA, then a STOP frees the bus. Pins from the UNO R4 WiFi variant.
const uint8_t WIRE1_SDA = 27;
const uint8_t WIRE1_SCL = 26;
const int SENSOR_FAIL_TICKS = 5;         // consecutive failures before a restart
// Flow averaging: FLOW_AVG_READS evenly spaced SFM3000 reads per tick, reported
// as their mean. The tick's own read is the last; the extra reads fall at
// multiples of FLOW_AVG_SPACING_US after the previous tick's read (taken in the
// idle time and between the SDP811 reads). N reads 10/N ms apart cancel ripple
// at 100 Hz and its harmonics up to (N - 1) x 100 Hz (4 reads: 100, 200 and
// 300 Hz), so the Jrk's 100 Hz update cannot alias into the volume. The mean
// is centred about 4 ms before the pot sample. Any failed read in the window
// makes the tick nan. A tick whose window missed a slot (loop late, sensor
// restarting) is still logged and counted in sfm_short in # status.
// 1: one point sample per tick.
// 4, not 10: at 10 reads per tick (1 ms apart) the SFM3000 locked up 3 s into
// preset 1 on 26 Oct 2026 (every read failed until a power cycle). 4 reads keep
// the filtering with 40% of that I2C traffic.
const int FLOW_AVG_READS = 4;            // 1, 2 or 4
const unsigned long FLOW_AVG_SPACING_US = 1000000UL / SAMPLE_HZ / FLOW_AVG_READS;
const unsigned long SFM_MIN_GAP_US = 600; // the SFM3000 NACKs a read with no new result (0.5 ms refresh)
// Frozen output: after a power glitch the SFM3000 can return one fixed word with a
// valid CRC instead of measuring (27 Oct 2026: 1367 for a whole run). Its noise
// moves the raw word every few ticks (longest identical run in rig data: 6), so
// SFM_FROZEN_TICKS identical tick reads in a row count as failed reads, and the
// usual failure path resends the start command. Words outside the sensor's
// +-200 slm range are also failed reads, not clamped values.
const int SFM_FROZEN_TICKS = 50;

// Modes held in current_preset (0 to 3 are the breathing presets, 4 to 7 steady s1 to s4).
const int MODE_SS = 4;         // first steady mode: s1 is MODE_SS + 0
const int MODE_IDLE = -1;
const int MODE_RETURNING = -3; // -2 was the demo sine
const int MODE_GOTO = -4;      // ramping to the max or min end
const int MODE_HOLD = -5;      // parked at max or min until H

float preset_t_exp_us[4];      // expiration time
float preset_t_insp_us[4];     // inspiration time
float preset_period_us[4];     // one full breath
float stroke_amp_dac = 0.0f;   // stroke amplitude in DAC counts for V_TIDAL_ML (volume scale)
float preset_amp_dac[4];       // stroke amplitude each preset commands (x GAIN_EXP)
int corrected_centre_dac = 0;
float filter_alpha = 1.0f;     // per-update filter coefficient (1 = no filter)
int ss_min_dac = 0, ss_max_dac = 0;   // steady stroke ends
unsigned long ss_stroke_ms[4];        // one steady stroke, min to max or back
float ss_stroke_ml = 0.0f;            // volume of one steady stroke

// Shared with the timer ISR. loop() changes these only with interrupts off.
volatile int current_preset = MODE_IDLE;
volatile int pending_mode = MODE_IDLE;   // mode to start once back at centre
volatile unsigned long phase_ms = 0;     // time into the breath or sweep
volatile uint32_t breath_count = 0;      // +1 at every breath start
volatile float cmd_f = 0.0f;             // command written, in DAC counts
volatile float filt1 = 0.0f, filt2 = 0.0f;
volatile int target_dac = 0;
volatile bool target_is_max = false;
volatile int isr_event = 0;              // 1 centre, 2 at max, 3 at min, 4 steady flow started
volatile uint32_t snap_tick = 0;         // snapshot, copied every sample tick
volatile float snap_cmd = 0.0f;
volatile int snap_mode = MODE_IDLE;
volatile uint32_t snap_breath = 0;

FspTimer dac_timer;

// Pot.
int pot_raw = 0;
float feedback_dac = 0;
int pot_history[3] = {0, 0, 0};
int pot_count = 0;

// Sensors: index 0 = SFM3000, 1 to 7 = p1 to p7.
const char *SENSOR_NAME[8] = {"flow", "p1", "p2", "p3", "p4", "p5", "p6", "p7"};
unsigned long read_err[8];
int fail_run[8];
bool fail_reported[8];
uint32_t last_restart[8];
bool sfm_discard = false;      // next good SFM3000 read is the invalid first one
long sfm_sum = 0;              // flow averaging: sum and count of raw words this tick
int sfm_n = 0;
bool sfm_bad = false;          // a read in this tick's window failed
unsigned long sfm_last_us = 0; // start of the last SFM3000 read
unsigned long sfm_tick_us = 0; // start of the last tick read: the extra reads are timed from it
int sfm_slot = FLOW_AVG_READS; // next extra read, 1 to FLOW_AVG_READS - 1 (FLOW_AVG_READS: none due)
unsigned long sfm_short = 0;   // ticks averaged over fewer than FLOW_AVG_READS reads
uint16_t sfm_prev_word = 0;    // last tick read's raw word, for the frozen-output check
int sfm_same = 0;              // tick reads in a row that returned sfm_prev_word
int sdp_blank[7];              // reads still blanked after an SDP811 restart
unsigned long mux_err = 0;
float tare[8];                 // flow in slm, pressures in Pa

// Tare state.
int tare_state = 0;            // 0 off, 1 waiting for the pot to settle, 2 averaging
uint32_t tare_t0 = 0;
float tare_ref = NAN, pot_lo = 0, pot_hi = 0;
float tare_sum[8], tare_n[8];

// Breath state.
bool in_breath = false;
int breath_preset = 0;         // preset of the breath in progress (for its summary)
bool flow_bad = false;
uint32_t last_breath = 0, breath_start = 0;
float vol_meas = 0, vol_seed = 0, pk_pot, pk_meas, pk_flow, t_pot, t_meas, p_pk[7];
float ss_cmd0 = 0, ss_last_cmd = 0, ss_last_vp = 0;   // steady stroke: command at start, last command and pot volume
float ss_sum_flow = 0;         // flow over the middle of the stroke
int ss_n = 0;

// Output state.
int plot_set = 1;
bool log_mode = false, log_requested = false;   // log mode starts once "# log on" is sent
uint16_t sfm_raw = 0;          // last raw SFM3000 word
int16_t sdp_raw[7];            // last raw SDP811 pressure words
int16_t sdp_scale = 0;         // last SDP811 scale factor (counts per Pa)
float sum_flow = 0, sum_p[7], n_sum = 0;
uint32_t last_tick = 0, last_row = 0, last_status = 0;
unsigned long dropped_ticks = 0, skipped_rows = 0, max_loop_us = 0;
char cmd_buf[4];               // collects the letters of "max" / "min" / "s1" to "s4"
int cmd_len = 0;
unsigned long cmd_ms = 0;

// ---- Serial writer: lines are queued, one is sent per sample tick ----
const int LINE_LEN = 480;              // the # config line is about 390 characters
const int QUEUE_LEN = 8;
struct Line : public Print {
  char s[LINE_LEN];
  int n;
  size_t write(uint8_t c) { if (n < LINE_LEN) s[n++] = c; return 1; }
};
Line queue_lines[QUEUE_LEN];
Line spare_line;               // takes the text of a line that did not fit
int q_head = 0, q_count = 0;

Line &newLine() {
  if (q_count == QUEUE_LEN) { skipped_rows++; spare_line.n = 0; return spare_line; }
  Line &L = queue_lines[(q_head + q_count++) % QUEUE_LEN];
  L.n = 0;
  return L;
}

void say(const char *text) { newLine().print(text); }

void sendOneLine() {
  if (q_count == 0) return;
  PC.write(queue_lines[q_head].s, queue_lines[q_head].n);
  PC.println();
  q_head = (q_head + 1) % QUEUE_LEN;
  q_count--;
}

// Log mode: one CSV row of raw values, sent at once. Empty field = failed read.
void sendLogRow(uint32_t tick, uint32_t breath, int m, float cmd, float raw[8]) {
  static Line row;
  row.n = 0;
  row.print(tick);
  row.print(',');
  row.print(breath);
  row.print(',');
  row.print(m);
  row.print(',');
  row.print((int)(cmd + 0.5f));
  row.print(',');
  row.print(pot_raw);
  row.print(',');
  if (!isnan(raw[0])) row.print(sfm_raw);
  for (int i = 0; i < 7; i++) {
    row.print(',');
    if (!isnan(raw[i + 1])) row.print(sdp_raw[i]);
  }
  PC.write(row.s, row.n);
  PC.println();
}

// Log mode: send the next queued line only if it fits in the rest of this tick.
void sendLineIfFits(unsigned long t0) {
  if (q_count == 0) return;
  unsigned long line_us = (queue_lines[q_head].n + 2) * 10000000UL / SERIAL_BAUD;
  if (micros() - t0 + line_us <= TICK_BUDGET_US) sendOneLine();
}

// ---- Actuator (from actuator_breathing.ino) ----

// Convert a desired value (Jrk counts) into the number to write to the DAC.
int applyDacCorrection(int desired) {
  return constrain((int)(((float)desired - DAC_CORR_OFFSET) * (float)DAC_MAX / DAC_CORR_GAIN + 0.5f), 0, DAC_MAX);
}

// Write a command in DAC counts. The DAC was opened by analogWrite() in setup().
void writeCommand(float v) {
  v = constrain(v, 0.0f, (float)DAC_MAX);
  cmd_f = v;
  R_DAC->DADR[0] = (uint16_t)(v + 0.5f);
}

// Send a profile value through the low-pass (if enabled) and write it.
void outputProfile(float x) {
  if (filter_alpha < 1.0f) {
    filt1 += filter_alpha * (x - filt1);
    filt2 += filter_alpha * (filt1 - filt2);
    writeCommand(filt2);
  } else {
    writeCommand(x);
  }
}

// Middle value of three numbers.
int median3(int a, int b, int c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

// Read the potentiometer. The median of the last three readings removes spikes.
void readFeedback() {
  pot_raw = analogRead(A1);
  pot_history[0] = pot_history[1];
  pot_history[1] = pot_history[2];
  pot_history[2] = pot_raw;
  if (pot_count < 3) pot_count++;
  int filtered = pot_raw;
  if (pot_count == 3) filtered = median3(pot_history[0], pot_history[1], pot_history[2]);
  feedback_dac = corrected_centre_dac + (filtered - POT_RAW_AT_CENTRE) * POT_DAC_COUNTS_PER_ADC;
}

// Swept volume per mm of stroke, in mL.
float computeKarea() {
  float r = CYLINDER_DIAMETER_MM / 2.0f;
  return (PI * r * r) / 1000.0f;
}

// Convert a distance in mm to Jrk input counts.
float mmToJrkCounts(float mm) {
  return mm * ((float)JRK_HALF_RANGE / (ACTUATOR_STROKE_MM / 2.0f));
}

// Work out stroke and timing for each peak flow rate.
void initPresets() {
  float k_area = computeKarea();
  float a_mm = V_TIDAL_ML / k_area;
  stroke_amp_dac = mmToJrkCounts(a_mm) * (float)DAC_MAX / DAC_CORR_GAIN;

  for (int i = 0; i < 4; i++) {
    float Q_mls = preset_q_peak_Lmin[i] * 1000.0f / 60.0f;
    float t_insp_s = (V_TIDAL_ML * PI) / (2.0f * Q_mls);
    float t_exp_s = t_insp_s * (IE_EXP_PARTS / IE_INSP_PARTS);
    float g_exp = FLOW_GAIN_ON ? GAIN_EXP[i] : 1.0f;
    float g_insp = FLOW_GAIN_ON ? GAIN_INSP[i] : 1.0f;
    t_insp_s *= g_exp / g_insp;             // inspiratory peak flow x g_insp
    preset_amp_dac[i] = stroke_amp_dac * g_exp;
    preset_t_exp_us[i] = t_exp_s * 1e6f;
    preset_t_insp_us[i] = t_insp_s * 1e6f;
    preset_period_us[i] = (t_exp_s + t_insp_s) * 1e6f;
  }

  // Steady flow: the stroke between the window ends, less SS_MARGIN_JRK each end.
  ss_min_dac = applyDacCorrection(JRK_CENTRE - JRK_HALF_RANGE + SS_MARGIN_JRK);
  ss_max_dac = applyDacCorrection(JRK_CENTRE + JRK_HALF_RANGE - SS_MARGIN_JRK);
  float ss_mm = ACTUATOR_STROKE_MM * (float)(JRK_HALF_RANGE - SS_MARGIN_JRK) / (float)JRK_HALF_RANGE;
  ss_stroke_ml = ss_mm * k_area;
  for (int i = 0; i < 4; i++) {
    float Q_mls = preset_q_peak_Lmin[i] * 1000.0f / 60.0f;
    ss_stroke_ms[i] = (unsigned long)(ss_stroke_ml / Q_mls * 1000.0f + 0.5f);
  }
}

// Breathing profile. phi is the time into the breath, in microseconds.
// The first part is expiration (0 to 1 on the stroke), the second is inspiration (back to 0).
void stepBreathing(float phi) {
  int i = current_preset;
  float pos_norm;
  if (phi < preset_t_exp_us[i]) {
    pos_norm = 0.5f * (1.0f - cosf(PI * phi / preset_t_exp_us[i]));
  } else {
    pos_norm = 0.5f * (1.0f + cosf(PI * (phi - preset_t_exp_us[i]) / preset_t_insp_us[i]));
  }
  outputProfile((float)corrected_centre_dac + preset_amp_dac[i] * pos_norm);
}

// Steady flow: constant speed min -> max over the first stroke, back over the
// second. phi_ms is the time into the cycle, in milliseconds.
void stepSteady(unsigned long phi_ms) {
  float T = (float)ss_stroke_ms[current_preset - MODE_SS];
  float span = (float)(ss_max_dac - ss_min_dac);
  float x = (float)phi_ms / T;
  float pos = (x <= 1.0f) ? x : 2.0f - x;
  outputProfile((float)ss_min_dac + span * pos);
}

// Start breathing or steady flow at phase 0. The filter starts from the command now written.
void startMode(int m) {
  current_preset = m;
  phase_ms = 0;
  filt1 = filt2 = cmd_f;
  if (m >= 0) breath_count++;
}

// Move the command toward the centre by at most RAMP_STEP_DAC per update.
void stepReturn() {
  float diff = (float)corrected_centre_dac - cmd_f;
  if (fabsf(diff) <= RAMP_STEP_DAC) {
    writeCommand((float)corrected_centre_dac);
    if (pending_mode != MODE_IDLE) {
      startMode(pending_mode);
      pending_mode = MODE_IDLE;
    } else {
      current_preset = MODE_IDLE;
      isr_event = 1;
    }
  } else {
    writeCommand(cmd_f + (diff > 0.0f ? RAMP_STEP_DAC : -RAMP_STEP_DAC));
  }
}

// Ramp toward target_dac at RAMP_STEP_DAC per update, then hold there, or
// start the steady flow waiting in pending_mode.
void stepGoto() {
  float diff = (float)target_dac - cmd_f;
  if (fabsf(diff) <= RAMP_STEP_DAC) {
    writeCommand((float)target_dac);
    if (pending_mode >= MODE_SS) {
      startMode(pending_mode);
      pending_mode = MODE_IDLE;
      isr_event = 4;
      return;
    }
    current_preset = MODE_HOLD;
    isr_event = target_is_max ? 2 : 3;
  } else {
    writeCommand(cmd_f + (diff > 0.0f ? RAMP_STEP_DAC : -RAMP_STEP_DAC));
  }
}

// Timer ISR, every 1 ms. No micros(), Serial, Wire or analogRead() here.
void onTick(timer_callback_args_t *) {
  static int sub_tick = 0;
  int m = current_preset;
  if (m == MODE_RETURNING) {
    stepReturn();
  } else if (m == MODE_GOTO) {
    stepGoto();
  } else if (m >= MODE_SS) {        // steady: a new "breath" at each end
    unsigned long stroke_ms = ss_stroke_ms[m - MODE_SS];
    stepSteady(phase_ms);
    if (++phase_ms >= 2 * stroke_ms) phase_ms = 0;
    if (phase_ms == 0 || phase_ms == stroke_ms) breath_count++;
  } else if (m >= 0) {
    stepBreathing(phase_ms * 1000.0f);
    if (++phase_ms >= (unsigned long)(preset_period_us[m] / 1000.0f + 0.5f)) {
      phase_ms = 0;
      breath_count++;
    }
  }
  if (++sub_tick >= TICKS_PER_SAMPLE) {
    sub_tick = 0;
    snap_tick++;
    snap_cmd = cmd_f;
    snap_mode = current_preset;
    snap_breath = breath_count;
  }
}

// ---- Keys (called with interrupts off) ----

void abortBreath() {
  if (in_breath) say("# breath aborted");
  in_breath = false;
}

void requestTare() {
  tare_state = 1;
  tare_ref = NAN;
}

void goToExtreme(bool is_max) {
  if (current_preset == MODE_RETURNING) {
    say("# still returning to centre, wait");
    return;
  }
  abortBreath();
  target_dac = applyDacCorrection(is_max ? (JRK_CENTRE + JRK_HALF_RANGE) : (JRK_CENTRE - JRK_HALF_RANGE));
  target_is_max = is_max;
  pending_mode = MODE_IDLE;
  current_preset = MODE_GOTO;
  say(is_max ? "# moving to max" : "# moving to min");
}

// The constants leaktest.py needs to convert the raw log values.
void printConfig() {
  Line &L = newLine();
  L.print("# config centre_dac=");
  L.print(corrected_centre_dac);
  L.print(" stroke_amp_dac=");
  L.print(stroke_amp_dac, 3);
  L.print(" gain_exp=");
  for (int i = 0; i < 4; i++) {
    if (i > 0) L.print(',');
    L.print(FLOW_GAIN_ON ? GAIN_EXP[i] : 1.0f, 3);
  }
  L.print(" gain_insp=");
  for (int i = 0; i < 4; i++) {
    if (i > 0) L.print(',');
    L.print(FLOW_GAIN_ON ? GAIN_INSP[i] : 1.0f, 3);
  }
  L.print(" v_tidal_ml=");
  L.print(V_TIDAL_ML, 1);
  L.print(" pot_raw_at_centre=");
  L.print(POT_RAW_AT_CENTRE, 2);
  L.print(" pot_dac_per_adc=");
  L.print(POT_DAC_COUNTS_PER_ADC, 4);
  L.print(" sfm_offset=");
  L.print(SFM3000_OFFSET, 0);
  L.print(" sfm_scale=");
  L.print(SFM3000_SCALE, 0);
  L.print(" sdp_scale=");
  L.print(sdp_scale);
  L.print(" flow_sign=");
  L.print(FLOW_SIGN, 0);
  L.print(" sample_hz=");
  L.print(SAMPLE_HZ);
  L.print(" flow_avg=");
  L.print(FLOW_AVG_READS);
  L.print(" period_ms=");
  for (int i = 0; i < 4; i++) {
    if (i > 0) L.print(',');
    L.print((unsigned long)(preset_period_us[i] / 1000.0f + 0.5f));
  }
  L.print(" ss_q_Lmin=");
  for (int i = 0; i < 4; i++) {
    if (i > 0) L.print(',');
    L.print(preset_q_peak_Lmin[i], 1);
  }
  L.print(" ss_stroke_ms=");
  for (int i = 0; i < 4; i++) {
    if (i > 0) L.print(',');
    L.print(ss_stroke_ms[i]);
  }
  L.print(" ss_stroke_ml=");
  L.print(ss_stroke_ml, 1);
  L.print(" ss_trim=");
  L.print(SS_TRIM_FRAC, 2);
}

// Keys s1 to s4 (i = 0 to 3): ramp straight to min from wherever the command
// is, then steady flow. stepGoto() starts it on arrival.
void startSteady(int i) {
  int m = current_preset;
  if (m == MODE_RETURNING) {
    say("# still returning to centre, wait");
    return;
  }
  if (m == MODE_GOTO || m == MODE_HOLD) {
    say("# send H first");
    return;
  }
  abortBreath();
  target_dac = ss_min_dac;
  target_is_max = false;
  pending_mode = MODE_SS + i;
  current_preset = MODE_GOTO;
  Line &L = newLine();
  L.print("# steady flow ");
  L.print(preset_q_peak_Lmin[i], 0);
  L.print(" L/min, moving to min first");
}

void handleKey(char key) {
  int m = current_preset;
  bool is_start = (key >= '1' && key <= '4');

  if (key == 'l') {
    if (log_mode || log_requested) {
      log_mode = log_requested = false;
      say("# log off");
    } else if (SERIAL_BAUD < LOG_MIN_BAUD) {
      say("# log mode needs SERIAL_BAUD >= 230400");
    } else {
      printConfig();
      say("# log on");
      log_requested = true;
    }
    return;
  }
  if (key == 'p') {
    if (!log_mode) plot_set = plot_set % 4 + 1;
    return;
  }
  if (key != 'h' && !is_start) return;

  if (m == MODE_RETURNING) {
    if (is_start) say("# still returning to centre, wait");
    else {                           // H cancels a start waiting for centre
      pending_mode = MODE_IDLE;
      say("# returning to centre");
    }
    return;
  }
  if (is_start && (m == MODE_GOTO || m == MODE_HOLD)) {
    say("# send H first");
    return;
  }
  abortBreath();

  if (key == 'h') {
    if (m == MODE_IDLE) {
      say("# centre");
      requestTare();
    } else {
      pending_mode = MODE_IDLE;      // cancels a steady start on its way to min
      current_preset = MODE_RETURNING;
      say("# returning to centre");
    }
    return;
  }

  int next = key - '1';
  if (m == MODE_IDLE) {
    startMode(next);
  } else {             // breathing or steady: ramp to centre, then start
    pending_mode = next;
    current_preset = MODE_RETURNING;
    say("# returning to centre first");
  }
  Line &L = newLine();
  L.print("# breathing, peak flow ");
  L.print(preset_q_peak_Lmin[next], 0);
  L.print(" L/min");
}

void readKeys() {
  if (cmd_len > 0 && millis() - cmd_ms > 1000) cmd_len = 0;   // "max"/"min"/"s1" buffer times out
  while (PC.available()) {
    char key = (char)tolower(PC.read());
    cmd_ms = millis();
    if (key == '\n' || key == '\r') {
      cmd_len = 0;
      continue;
    }
    noInterrupts();
    if (cmd_len > 0 || key == 'm' || key == 's') {
      cmd_buf[cmd_len++] = key;
      if (cmd_buf[0] == 's' && cmd_len == 2) {
        if (key >= '1' && key <= '4') startSteady(key - '1');
        else say("# unknown command");
        cmd_len = 0;
      } else if (cmd_len == 3) {
        cmd_buf[3] = '\0';
        if (strcmp(cmd_buf, "max") == 0) goToExtreme(true);
        else if (strcmp(cmd_buf, "min") == 0) goToExtreme(false);
        else say("# unknown command");
        cmd_len = 0;
      }
    } else {
      handleKey(key);
    }
    interrupts();
  }
}

// ---- Sensors ----
// I2C framing and CRC-8 from the Sensirion Core library (Library Manager:
// "Sensirion Core"). Its sensor drivers are not used: their reads add a 1 ms
// delay each (11 ms per tick here) and the SDP811 start a 10 ms delay.
// Functions below return Sensirion error codes (0 = OK, errorToString() for text).

// Send a 16-bit command with no arguments.
uint16_t sensirionCommand(TwoWire &bus, uint8_t addr, uint16_t cmd) {
  uint8_t buf[2];
  SensirionI2CTxFrame tx = SensirionI2CTxFrame::createWithUInt16Command(cmd, buf, 2);
  return SensirionI2CCommunication::sendFrame(addr, tx, bus);
}

// SFM3000 start continuous measurement (0x1000).
uint16_t sfmStart() { return sensirionCommand(Wire, SFM3000_ADDR, 0x1000); }

// One flow value in slm, or NAN on a failed read (short read or CRC, initial value 0x00).
float readFlow() {
  uint8_t buf[3];
  SensirionI2CRxFrame rx(buf, sizeof(buf));
  uint16_t raw;
  if (SensirionI2CCommunication::receiveFrame(SFM3000_ADDR, 3, rx, Wire, CRC31_00) || rx.getUInt16(raw)) return NAN;
  sfm_raw = raw;
  float slm = ((float)raw - SFM3000_OFFSET) / SFM3000_SCALE;
  if (fabsf(slm) > SFM3000_MAX_SLM) return NAN;   // outside the range: not a measurement
  return slm;
}

// Free a Wire1 bus left held by a sensor. Run before Wire1.begin(), which hands
// the pins back to the I2C peripheral. Lines are driven open drain: OUTPUT LOW to
// pull down, INPUT to release (the pull-ups take them high). Returns the number of
// SCL pulses needed (0: bus was free), or -1 if SDA is still held low.
int recoverWire1() {
  pinMode(WIRE1_SDA, INPUT);
  pinMode(WIRE1_SCL, INPUT);
  delayMicroseconds(5);
  int pulses = 0;
  while (digitalRead(WIRE1_SDA) == LOW && pulses < 9) {
    pinMode(WIRE1_SCL, OUTPUT);
    digitalWrite(WIRE1_SCL, LOW);
    delayMicroseconds(5);
    pinMode(WIRE1_SCL, INPUT);
    delayMicroseconds(5);
    pulses++;
  }
  // STOP: SDA low to high while SCL is high.
  pinMode(WIRE1_SDA, OUTPUT);
  digitalWrite(WIRE1_SDA, LOW);
  delayMicroseconds(5);
  pinMode(WIRE1_SDA, INPUT);
  delayMicroseconds(5);
  return digitalRead(WIRE1_SDA) == LOW ? -1 : pulses;
}

// Write the mux channel mask (1 << channel, or 0 for none). Returns true on success.
bool muxWrite(uint8_t mask) {
  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(mask);
  return Wire1.endTransmission() == 0;
}

// SDP811 start continuous measurement, differential pressure, no averaging (0x361E).
bool sdpStart() { return sensirionCommand(Wire1, SDP_ADDR, 0x361E) == 0; }

// SDP811 stop measurement (0x3FF9). The sensor may not answer if it is already idle.
void sdpStop() { sensirionCommand(Wire1, SDP_ADDR, 0x3FF9); }

// Read the SDP811 on mux channel ch (already selected), in Pa, or NAN on a failed read.
// 9 bytes: pressure, temperature and scale factor, each 2 bytes + CRC (initial value
// 0xFF). receiveFrame checks all three CRCs; the temperature is not used.
float readPressure(int ch) {
  uint8_t buf[9];
  SensirionI2CRxFrame rx(buf, sizeof(buf));
  if (SensirionI2CCommunication::receiveFrame(SDP_ADDR, 9, rx, Wire1, CRC31_ff)) {
    while (Wire1.available()) Wire1.read();   // empty the buffer after a short read
    return NAN;
  }
  int16_t raw_pressure, raw_temp, raw_scale;
  rx.getInt16(raw_pressure);
  rx.getInt16(raw_temp);
  rx.getInt16(raw_scale);
  if (raw_scale <= 0) return NAN;
  sdp_raw[ch] = raw_pressure;
  sdp_scale = raw_scale;
  return (float)raw_pressure / (float)raw_scale;
}

// Count a read result. After SENSOR_FAIL_TICKS failures in a row, resend the
// start command, at most once per second, and never on a tick whose bus failed.
void sensorResult(int s, bool ok, uint32_t tick, bool bus_ok) {
  if (ok) {
    fail_run[s] = 0;
    return;
  }
  read_err[s]++;
  if (!fail_reported[s]) {
    fail_reported[s] = true;
    Line &L = newLine();
    L.print("# first read failure: ");
    L.print(SENSOR_NAME[s]);
  }
  if (++fail_run[s] >= SENSOR_FAIL_TICKS && bus_ok && tick - last_restart[s] >= (uint32_t)SAMPLE_HZ) {
    last_restart[s] = tick;
    if (s == 0) {
      sfmStart();
      sfm_discard = true;           // first result after a start is not valid
    } else if (muxWrite(1 << (s - 1))) {
      sdpStop();
      delay(1);
      sdpStart();
      sdp_blank[s - 1] = 2;         // blank the reads at about 10 and 20 ms
    }
  }
}

// Read the SFM3000, then p1 to p7, into raw[] (slm, Pa). Failed reads are NAN.
// Flow averaging: take the next extra SFM3000 read once its slot is due
// (sfm_slot x FLOW_AVG_SPACING_US after the tick read). A late slot is read late,
// at least SFM_MIN_GAP_US after the read before it. Skipped while the sensor is
// failing or restarting, so a dead sensor cannot eat the tick.
void sfmSample() {
  if (sfm_slot >= FLOW_AVG_READS || sfm_discard || fail_run[0] > 0) return;
  unsigned long now = micros();
  if (now - sfm_tick_us < (unsigned long)sfm_slot * FLOW_AVG_SPACING_US) return;
  if (now - sfm_last_us < SFM_MIN_GAP_US) return;
  sfm_last_us = now;
  sfm_slot++;
  if (isnan(readFlow())) {
    sfm_bad = true;
  } else {
    sfm_sum += sfm_raw;
    sfm_n++;
  }
}

void readSensors(float raw[8], uint32_t tick) {
  // The SFM3000 NACKs a read with no new result, so leave SFM_MIN_GAP_US after
  // an averaging read.
  unsigned long since = micros() - sfm_last_us;
  if (FLOW_AVG_READS > 1 && since < SFM_MIN_GAP_US) delayMicroseconds(SFM_MIN_GAP_US - since);
  sfm_last_us = sfm_tick_us = micros();
  raw[0] = readFlow();
  bool ok = !isnan(raw[0]);
  if (ok) {                     // frozen-output check on the tick read's own word
    sfm_same = (sfm_raw == sfm_prev_word) ? sfm_same + 1 : 1;
    sfm_prev_word = sfm_raw;
    if (sfm_same >= SFM_FROZEN_TICKS) {
      ok = false;
      raw[0] = NAN;
      sfm_bad = true;
    }
  }
  if (ok && sfm_discard) {
    sfm_discard = false;
    raw[0] = NAN;
  }
  if (FLOW_AVG_READS > 1 && !isnan(raw[0])) {
    sfm_sum += sfm_raw;
    sfm_n++;
    if (sfm_n < FLOW_AVG_READS) sfm_short++;
    float mean = (float)sfm_sum / sfm_n;
    sfm_raw = (uint16_t)(mean + 0.5f);
    raw[0] = sfm_bad ? NAN : (mean - SFM3000_OFFSET) / SFM3000_SCALE;   // every read in the mean is in range
  }
  sfm_sum = 0;
  sfm_n = 0;
  sfm_bad = false;
  sfm_slot = 1;                 // next window starts from this read
  sensorResult(0, ok, tick, true);

  bool bus_ok = true;
  for (int ch = 0; ch < 7; ch++) {
    raw[ch + 1] = NAN;
    sfmSample();                           // flow averaging also covers the burst
    if (bus_ok && !muxWrite(1 << ch)) {   // a failed mux write skips the rest of Wire1
      bus_ok = false;
      mux_err++;
    }
    if (bus_ok) raw[ch + 1] = readPressure(ch);
    ok = !isnan(raw[ch + 1]);
    if (sdp_blank[ch] > 0) {
      sdp_blank[ch]--;
      raw[ch + 1] = NAN;
    }
    sensorResult(ch + 1, ok, tick, bus_ok);
  }
  if (bus_ok) muxWrite(0);   // no SDP811 stays connected between bursts
}

// ---- Tare ----

void updateTare(float raw[8], int m, uint32_t tick) {
  if (m != MODE_IDLE) {
    tare_state = 0;
    say("# tare aborted, not idle");
    return;
  }
  if (tare_state == 1) {
    if (isnan(tare_ref) || fabsf(feedback_dac - tare_ref) > TARE_STILL_COUNTS) {
      tare_ref = feedback_dac;     // pot moved: restart the settle time
      tare_t0 = tick;
    } else if ((tick - tare_t0) * 1000UL / SAMPLE_HZ >= TARE_SETTLE_MS) {
      tare_state = 2;
      tare_t0 = tick;
      pot_lo = pot_hi = feedback_dac;
      for (int i = 0; i < 8; i++) tare_sum[i] = tare_n[i] = 0;
    }
    return;
  }
  for (int i = 0; i < 8; i++) {
    if (!isnan(raw[i])) {
      tare_sum[i] += raw[i];
      tare_n[i]++;
    }
  }
  pot_lo = min(pot_lo, feedback_dac);
  pot_hi = max(pot_hi, feedback_dac);
  if (tick - tare_t0 >= (uint32_t)TARE_SAMPLES) {
    tare_state = 0;
    for (int i = 0; i < 8; i++) tare[i] = tare_sum[i] / tare_n[i];   // no good reads gives nan
    float flow_tare = tare[0];
    bool rejected = !isnan(flow_tare) && fabsf(flow_tare) > TARE_FLOW_MAX_SLM;
    if (rejected) tare[0] = NAN;   // flow stays nan until a good tare
    Line &L = newLine();
    L.print(rejected ? "# tare rejected" : "# tare");
    for (int i = 0; i < 8; i++) {
      L.print(i == 0 ? " flow_slm=" : " ");
      if (i > 0) { L.print(SENSOR_NAME[i]); L.print("_Pa="); }
      L.print(i == 0 ? flow_tare : tare[i], 3);
    }
    L.print(" pot_spread_dac=");
    L.print(pot_hi - pot_lo, 1);
  }
}

// ---- Breath summary ----

void printBreath() {
  static const char *NAMES[15] = {"track_err_ml", "leak_ml", "leak_pct", "t_pot_peak_s", "t_meas_peak_s",
                                  "shortfall_ml", "peak_flow_Lmin", "p1", "p2", "p3", "p4", "p5", "p6", "p7", "net_ml"};
  float leak = pk_pot - pk_meas;
  float v_cmd = V_TIDAL_ML * (FLOW_GAIN_ON ? GAIN_EXP[breath_preset] : 1.0f);
  float v[15] = {v_cmd - pk_pot, leak, 100.0f * leak / pk_pot, t_pot, t_meas,
                 V_TIDAL_ML - pk_meas, pk_flow, p_pk[0], p_pk[1], p_pk[2], p_pk[3], p_pk[4], p_pk[5], p_pk[6], vol_meas - vol_seed};
  Line &L = newLine();
  L.print("# breath");
  for (int i = 0; i < 15; i++) {
    L.print(' ');
    L.print(NAMES[i]);
    L.print('=');
    L.print(flow_bad ? NAN : v[i], 2);   // a failed flow read makes the whole summary nan
  }
}

// Steady stroke summary. out = min -> max (air out of the cylinder, + flow).
// mean_flow is over the middle of the stroke, signed; flow_pct compares its
// size with the target. pot_ml and meas_ml are the volume moved by the piston
// and through the SFM3000 over the whole stroke, signed.
void printStroke() {
  float q = preset_q_peak_Lmin[breath_preset - MODE_SS];
  float mean = ss_n > 0 ? ss_sum_flow / ss_n : NAN;
  float pot = ss_last_vp - vol_seed;
  float meas = vol_meas - vol_seed;
  bool out = ss_last_cmd > ss_cmd0;
  Line &L = newLine();
  L.print(out ? "# stroke dir=out" : "# stroke dir=in");
  L.print(" target_Lmin=");
  L.print(q, 1);
  L.print(" mean_flow_Lmin=");
  L.print(flow_bad ? NAN : mean, 2);
  L.print(" flow_pct=");
  L.print(flow_bad ? NAN : 100.0f * fabsf(mean) / q, 2);
  L.print(" pot_ml=");
  L.print(pot, 1);
  L.print(" meas_ml=");
  L.print(flow_bad ? NAN : meas, 1);
  L.print(" leak_ml=");
  L.print(flow_bad ? NAN : fabsf(pot) - fabsf(meas), 1);
}

#if NATIVE_USB
// Core hook, runs before setup(): switches the USB-C port to the RA4M1 and
// starts its USB. Defined here, after the struct definitions, because the
// Arduino IDE inserts its function prototypes above the first function.
void __USBStart();
void __maybe_start_usb() { __USBStart(); }
#endif

void setup() {
  PC.begin(SERIAL_BAUD);
  delay(2000);
  analogWriteResolution(12);
  analogReadResolution(12);
  corrected_centre_dac = applyDacCorrection(JRK_CENTRE);
  filter_alpha = (CMD_FILTER_HZ > 0.0f)
                   ? 1.0f - expf(-2.0f * PI * CMD_FILTER_HZ / DAC_UPDATE_HZ)
                   : 1.0f;
  initPresets();
  analogWrite(A0, corrected_centre_dac);   // opens and starts the DAC once, at centre
  cmd_f = filt1 = filt2 = corrected_centre_dac;
  PC.println("# MechLung_SteadyStateTest: 1 2 3 4 s1 s2 s3 s4 max min H p L");

  Wire.begin();
  Wire.setClock(WIRE_CLOCK_HZ);
  Wire.setWireTimeout(WIRE_TIMEOUT_US);
  int pulses = recoverWire1();
  if (pulses != 0) {
    PC.print("# Wire1 bus recovery: ");
    if (pulses < 0) PC.println("SDA still held low, power cycle the rig");
    else { PC.print(pulses); PC.println(" SCL pulses, bus freed"); }
  }
  Wire1.begin();
  Wire1.setClock(WIRE1_CLOCK_HZ);
  Wire1.setWireTimeout(WIRE1_TIMEOUT_US);

  uint16_t err = sfmStart();
  sfm_discard = true;
  if (err == 0) {
    PC.println("# SFM3000 found");
  } else {
    char msg[64];
    errorToString(err, msg, sizeof(msg));
    PC.print("# SFM3000 not found: ");
    PC.println(msg);
  }
  for (int ch = 0; ch < 7; ch++) {
    PC.print("# SDP811 p");
    PC.print(ch + 1);
    if (!muxWrite(1 << ch)) {
      PC.println(": multiplexer channel not found");
      continue;
    }
    delay(20);
    sdpStop();      // in case a previous run left the sensor measuring
    delay(1);
    PC.println(sdpStart() ? " found" : " not found");
    delay(25);
  }
  muxWrite(0);

  uint8_t timer_type;
  int8_t timer_ch = FspTimer::get_available_timer(timer_type);
  bool ok = timer_ch >= 0
            && dac_timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_ch, (float)DAC_UPDATE_HZ, 0.0f, onTick)
            && dac_timer.setup_overflow_irq(TIMER_PRIORITY)
            && dac_timer.open()
            && dac_timer.start();
  if (ok) {
    PC.print("# timer running, interrupt priority ");
    PC.println(NVIC_GetPriority(dac_timer.get_cfg()->cycle_end_irq));
  } else {
    PC.println("# TIMER SETUP FAILED: the actuator will not move");
  }
  requestTare();
}

void loop() {
  readKeys();

  noInterrupts();   // copy only
  uint32_t tick = snap_tick;
  float cmd = snap_cmd;
  int m = snap_mode;
  uint32_t breath = snap_breath;
  int ev = isr_event;
  isr_event = 0;
  interrupts();

  if (ev == 1) say("# centre");
  if (ev == 2) say("# at max, H returns to centre");
  if (ev == 3) say("# at min, H returns to centre");
  if (ev == 4) say("# at min, steady flow started");
  if (tick == last_tick) {
    sfmSample();   // flow averaging in the idle time
    return;
  }

  unsigned long t0 = micros();
  if (log_requested && q_count == 0) {   // "# log on" has gone out: rows start now
    log_requested = false;
    log_mode = true;
  }
  uint32_t dticks = tick - last_tick;
  dropped_ticks += dticks - 1;
  last_tick = tick;

  float raw[8];
  for (int i = 0; i < 8; i++) raw[i] = NAN;
  readSensors(raw, tick);
  readFeedback();
  if (tare_state) updateTare(raw, m, tick);

  float flow = FLOW_SIGN * (raw[0] - tare[0]) * SLM_TO_LMIN;
  float p[7];
  for (int i = 0; i < 7; i++) p[i] = raw[i + 1] - tare[i + 1];

  // Breath boundary: the ISR's breath counter changed. In steady flow each
  // stroke is a breath.
  if (m < 0) in_breath = false;
  if (breath != last_breath) {
    last_breath = breath;
    if (in_breath && !log_mode) {
      if (breath_preset >= MODE_SS) printStroke();
      else printBreath();
    }
    in_breath = (m >= 0);
    if (in_breath) breath_preset = m;
    breath_start = tick;
    flow_bad = false;
    vol_meas = 0;
    ss_cmd0 = cmd;
    ss_sum_flow = 0;
    ss_n = 0;
    pk_pot = pk_meas = pk_flow = -INFINITY;
    t_pot = t_meas = NAN;
    for (int i = 0; i < 7; i++) p_pk[i] = NAN;
  }

  float vc = 0, vp = 0, vm = 0;
  if (in_breath) {
    vc = V_TIDAL_ML * (cmd - corrected_centre_dac) / stroke_amp_dac;
    vp = V_TIDAL_ML * (feedback_dac - corrected_centre_dac) / stroke_amp_dac;
    // Breath start: seed from the pot so both traces share centre as zero.
    // While the piston moves, the 3-point median returns the previous sample's
    // pot reading, so vp is one sample old and this tick's flow is still added.
    if (tick == breath_start) vol_meas = vol_seed = vp;
    if (isnan(flow)) flow_bad = true;   // no gap is bridged
    vol_meas = flow_bad ? NAN : vol_meas + flow * 1000.0f / 60.0f * dticks / SAMPLE_HZ;
    vm = vol_meas;
    float t = (tick - breath_start) / (float)SAMPLE_HZ;
    if (vp > pk_pot) { pk_pot = vp; t_pot = t; }
    if (vm > pk_meas) { pk_meas = vm; t_meas = t; }
    if (flow > pk_flow) {
      pk_flow = flow;
      for (int i = 0; i < 7; i++) p_pk[i] = p[i];
    }
    if (m >= MODE_SS) {
      ss_last_cmd = cmd;
      ss_last_vp = vp;
      float T = ss_stroke_ms[m - MODE_SS] / 1000.0f;
      if (t >= SS_TRIM_FRAC * T && t <= (1.0f - SS_TRIM_FRAC) * T) {
        ss_sum_flow += flow;
        ss_n++;
      }
    }
  }

  if (log_mode) {
    sendLogRow(tick, breath, m, cmd, raw);
    sendLineIfFits(t0);
    return;
  }

  // Plot row every PLOT_DIV sample ticks.
  sum_flow += flow;
  for (int i = 0; i < 7; i++) sum_p[i] += p[i];
  n_sum++;
  if (tick - last_row >= (uint32_t)PLOT_DIV) {
    last_row = tick;
    Line &L = newLine();
    if (plot_set == 4) {
      L.print("command_dac:");
      L.print((int)(cmd + 0.5f));
      L.print(",feedback_dac:");
      L.print(feedback_dac, 1);
    } else if (plot_set == 1) {
      L.print("flow_Lmin_x20:");
      L.print(20.0f * sum_flow / n_sum, 1);
      L.print(",vol_cmd_ml:");
      L.print(vc, 1);
      L.print(",vol_pot_ml:");
      L.print(vp, 1);
      L.print(",vol_measured_ml:");
      L.print(vm, 1);
    } else if (plot_set == 2) {
      L.print("leak_gap_ml:");
      L.print(vp - vm, 1);
      L.print(",track_err_ml:");
      L.print(vc - vp, 1);
    } else {
      for (int i = 0; i < 7; i++) {
        if (i > 0) L.print(',');
        L.print(SENSOR_NAME[i + 1]);
        L.print(':');
        L.print(sum_p[i] / n_sum, 2);
      }
    }
    sum_flow = n_sum = 0;
    for (int i = 0; i < 7; i++) sum_p[i] = 0;
  }

  // Status line once per second.
  if (tick - last_status >= (uint32_t)SAMPLE_HZ) {
    last_status = tick;
    Line &L = newLine();
    L.print("# status tick=");
    L.print(tick);
    L.print(" dropped_ticks=");
    L.print(dropped_ticks);
    L.print(" skipped_rows=");
    L.print(skipped_rows);
    L.print(" read_err=");
    for (int i = 0; i < 8; i++) {
      if (i > 0) L.print(',');
      L.print(read_err[i]);
    }
    L.print(" mux_err=");
    L.print(mux_err);
    L.print(" sfm_short=");
    L.print(sfm_short);
    L.print(" max_loop_us=");
    L.print(max_loop_us);
  }

  sendOneLine();   // at most one line per sample tick
  unsigned long loop_us = micros() - t0;
  if (loop_us > max_loop_us) max_loop_us = loop_us;
}

/*
  MechanicalLungPlatform.ino

  Data platform for the piston-cylinder mechanical lung, driven from the PC by
  logger/mechlungdatalogger.py. Built from MechLung_SteadyStateTest: same
  breathing presets and steady flow modes with their flow corrections, but the
  sketch only streams raw data. Plot mode, the Serial Monitor messages and the
  in-sketch tare and volume figures are gone: the logger tares and works out
  every figure from the rows. The only text lines are the start-up checks and
  the # config reply.

  Hardware (Arduino UNO R4 WiFi)
    A0     12-bit DAC output -> Jrk G2 analog input
    A1     12-bit ADC input  <- position potentiometer
    Wire   SFM3000 flow sensor, address 0x40 (4.7k pull-ups), 100 kHz
    Wire1  Qwiic -> PCA9548 mux 0x70, SDP811 0x26 on channels 0 to 6 (p1 to p7)

  How it runs
    A 1 kHz timer interrupt computes the profile, the filter and the ramps and
    writes the DAC, so sensor reads and Serial cannot delay the actuator.
    Every 10th interrupt is a 100 Hz sample tick: loop() reads the SFM3000, the
    pot and p1 to p7, and sends one row.

  Keys (over the PC link, see NATIVE_USB; not case sensitive). Keys that do not
  apply in the current mode are ignored; the logger reads the mode column.
    1 2 3 4      breathe at a peak flow of 10, 15, 20 or 30 L/min
    s1 s2 s3 s4  steady flow of 10, 15, 20 or 30 L/min: ramp to min, hold
                 SS_DWELL_MS, then strokes min -> max -> min at constant speed,
                 holding SS_DWELL_MS at each end, until H
    max min      go to that end of the calibrated Jrk window and hold
    H            stop: return to centre at RAMP_RATE_DAC_PER_S
    c            send the # config line
    A breathing key while breathing or in steady flow ramps to centre first,
    then starts. A steady key ramps straight to min from wherever the command
    is. Start keys are ignored while returning to centre and at max or min
    (send H first).

  Steady flow: min -> max pushes air out of the cylinder (+ flow), max -> min
  draws it in. Each stroke is one "breath" in the rows: the move plus the hold
  at its end. The breath counter goes up as each move starts, so every stroke
  starts and ends with the piston at rest.

  Rows: one CSV row per 100 Hz sample tick, always streaming:
    tick,us,breath,mode,cmd_dac,pot_raw,sfm_raw,sfm_n,sfm_span_us,pot_dus,
    p1_raw,...,p7_raw,p1_dus,...,p7_dus
  tick         100 Hz sample tick (from the timer interrupt)
  us           micros() at the start of the tick's SFM3000 read (the row's time origin)
  breath       +1 at each breath start (steady flow: each stroke)
  mode         0 to 3 breathing presets, 4 to 7 steady s1 to s4, -1 idle,
               -3 returning to centre, -4 moving to an end, -5 holding there
  cmd_dac      command written to the DAC at the tick
  pot_raw      position pot, raw ADC
  sfm_raw      SFM3000 word, the mean of the sfm_n reads in the tick's window
  sfm_n        reads in that mean (FLOW_AVG_READS when the window was complete)
  sfm_span_us  first read of the window to the tick's read (ideal: 7500 for 4 reads)
  pot_dus      pot read start - us
  pN_raw       SDP811 pressure word (Pa = raw / sdp_scale)
  pN_dus       pN read start - us
  Sensor values are raw and untared. A failed read is an empty field (with an
  empty _dus if the read was not attempted). A tick gap is a lost row.

  The reads cannot be simultaneous (one ADC, two I2C buses read in turn), so
  each row carries its read times. The flow mean stands for the volume moved
  up to about us + sfm_span_us / (2 * (sfm_n - 1)) (1.25 ms for 4 reads); the
  pot is read straight after the tick's SFM3000 read, about 0.4 ms after us.

  # config (key c): the constants the logger needs to convert the rows, the
  start-up check results (sfm_found, sdp_found, timer_ok, wire1_pulses) and
  end=1 as the last field, so a cut-off line is detected.

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
// Jrk window, measured with the min / max commands (27 Oct 2026; was 230 mm).
const float ACTUATOR_STROKE_MM = 221.0f;
const float CYLINDER_DIAMETER_MM = 129.15f;   // bore of the cylinder, measured with callipers
const float V_TIDAL_ML = 700.0f;              // volume moved per breath
const float IE_EXP_PARTS = 2.0f;              // expiration : inspiration time ratio
const float IE_INSP_PARTS = 1.0f;
float preset_q_peak_Lmin[4] = {10.0f, 15.0f, 20.0f, 30.0f};  // keys 1 to 4, and s1 to s4
// Flow correction for the breathing presets (target / measured peak flow at the
// SFM3000, uncorrected runs of 28 Oct 2026 08:47, model connected).
//   GAIN_EXP scales the stroke amplitude, so expiratory peak flow x GAIN_EXP;
//   inspiration time is scaled by GAIN_EXP / GAIN_INSP, so inspiratory peak
//   flow x GAIN_INSP. FLOW_GAIN_ON false: no correction (all gains 1).
const bool FLOW_GAIN_ON = true;
const float GAIN_EXP[4] = {1.008f, 1.003f, 1.005f, 1.014f};
const float GAIN_INSP[4] = {0.984f, 0.993f, 0.997f, 1.012f};

// Timing.
const int DAC_UPDATE_HZ = 1000;       // timer rate: DAC written every 1 ms
const int SAMPLE_HZ = 100;            // sensor sample rate, must divide DAC_UPDATE_HZ
const int TICKS_PER_SAMPLE = DAC_UPDATE_HZ / SAMPLE_HZ;
const uint8_t TIMER_PRIORITY = 4;     // above I2C, USB, serial (12) and millis (8)
// PC link. NATIVE_USB 1: the USB-C port is switched to the RA4M1's own USB at
// boot, bypassing the ESP32-S3 bridge, whose serial output stalls under
// continuous 100 Hz logging (B1 tests). The board then shows up as a new COM
// port. To upload: double-tap reset, then upload (retry once if it fails).
// NATIVE_USB 0: Serial through the bridge; rows are about 125 bytes, so it
// needs SERIAL_BAUD >= 230400 and still stalls (B1).
#define NATIVE_USB 1
#if NATIVE_USB
#define PC SerialUSB
const unsigned long SERIAL_BAUD = 12000000; // ignored over native USB
#else
#define PC Serial
const unsigned long SERIAL_BAUD = 230400;
#endif

// Steady flow (keys s1 to s4): same flows as the presets, own correction below.
// The stroke runs over the calibrated Jrk window less SS_MARGIN_JRK at each end,
// so a Jrk overshoot at the reversal stays inside the window (20 Jrk counts
// is about 1.1 mm, 0.5 % of the stroke).
const int SS_MARGIN_JRK = 20;
// Hold at each end of a steady stroke and at min before the first: the piston
// and the pressures settle, so each stroke starts and ends at rest.
const unsigned long SS_DWELL_MS = 1000;
// Steady flow correction: scales the piston speed of each stroke so the mean
// flow at the SFM3000 over the middle of the stroke meets the target (target /
// measured, uncorrected runs of 28 Oct 2026 11:03, model connected, full window,
// no hold at the ends). Out (min -> max) takes the uncorrected stroke time /
// SS_GAIN_OUT, in (max -> min) / SS_GAIN_IN. Re-check with a corrected run: the
// margin and the holds change the stroke ends. SS_GAIN_ON false: no correction.
const bool SS_GAIN_ON = true;
const float SS_GAIN_OUT[4] = {1.0170f, 1.0263f, 1.0310f, 1.0446f};
const float SS_GAIN_IN[4] = {1.0129f, 1.0231f, 1.0280f, 1.0428f};

// Smoothing: two cascaded low-pass stages, delay about 1 / (PI * CMD_FILTER_HZ)
// (64 ms at 5 Hz). 0 = off.
const float CMD_FILTER_HZ = 5.0f;
// Speed of H and of the max / min moves, DAC counts per second.
const float RAMP_RATE_DAC_PER_S = 300.0f;
const float RAMP_STEP_DAC = RAMP_RATE_DAC_PER_S / DAC_UPDATE_HZ;

// SFM3000 conversion (for the range check; the logger converts the rows).
const float FLOW_SIGN = 1.0f;         // + = out of cylinder. Flip if the flow is mirrored.
const float SFM3000_OFFSET = 32000.0f;  // raw reading at zero flow (datasheet)
const float SFM3000_SCALE = 140.0f;     // raw counts per slm, air (datasheet)
const float SFM3000_MAX_SLM = 200.0f;   // sensor range: a word outside it is a failed read

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
// holding SDA low until power is removed (seen on the rig: mux errors every tick
// after a reset, cleared only by a power cycle). Up to 9 SCL pulses let it finish
// its byte and release SDA, then a STOP frees the bus. Pins from the UNO R4 WiFi variant.
const uint8_t WIRE1_SDA = 27;
const uint8_t WIRE1_SCL = 26;
const int SENSOR_FAIL_TICKS = 5;         // consecutive failures before a restart
// Flow averaging: FLOW_AVG_READS evenly spaced SFM3000 reads per tick, reported
// as their mean. The tick's own read is the last; the extra reads fall at
// multiples of FLOW_AVG_SPACING_US after the previous tick's read (taken in the
// idle time and between the SDP811 reads). N reads 10/N ms apart cancel ripple
// at 100 Hz and its harmonics up to (N - 1) x 100 Hz (4 reads: 100, 200 and
// 300 Hz), so the Jrk's 100 Hz update cannot alias into the volume. Any failed
// read in the window makes the tick's flow empty. sfm_n and sfm_span_us in each
// row show how many reads the window got and how far apart they were.
// 4, not 10: at 10 reads per tick (1 ms apart) the SFM3000 locked up 3 s into
// preset 1 on 26 Oct 2026 (every read failed until a power cycle).
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
const int MODE_GOTO = -4;      // ramping to the max or min end (and the hold before steady flow)
const int MODE_HOLD = -5;      // parked at max or min until H

float preset_t_exp_us[4];      // expiration time
float preset_t_insp_us[4];     // inspiration time
float preset_period_us[4];     // one full breath
float stroke_amp_dac = 0.0f;   // stroke amplitude in DAC counts for V_TIDAL_ML (volume scale)
float preset_amp_dac[4];       // stroke amplitude each preset commands (x GAIN_EXP)
int corrected_centre_dac = 0;
float filter_alpha = 1.0f;     // per-update filter coefficient (1 = no filter)
int ss_min_dac = 0, ss_max_dac = 0;   // steady stroke ends
unsigned long ss_out_ms[4];           // steady move, min to max (x 1 / SS_GAIN_OUT)
unsigned long ss_in_ms[4];            // max back to min (x 1 / SS_GAIN_IN)
float ss_stroke_ml = 0.0f;            // volume of one steady stroke

// Shared with the timer ISR. loop() changes these only with interrupts off.
volatile int current_preset = MODE_IDLE;
volatile int pending_mode = MODE_IDLE;   // mode to start once back at centre or at min
volatile unsigned long phase_ms = 0;     // time into the breath or steady cycle
volatile unsigned long settle_ms = 0;    // time held at min before steady flow starts
volatile uint32_t breath_count = 0;      // +1 at every breath (or stroke) start
volatile float cmd_f = 0.0f;             // command written, in DAC counts
volatile float filt1 = 0.0f, filt2 = 0.0f;
volatile int target_dac = 0;
volatile uint32_t snap_tick = 0;         // snapshot, copied every sample tick
volatile float snap_cmd = 0.0f;
volatile int snap_mode = MODE_IDLE;
volatile uint32_t snap_breath = 0;

FspTimer dac_timer;

// Start-up check results, reported in # config.
bool sfm_found = false;
bool sdp_found[7];
bool timer_ok = false;
int wire1_pulses = 0;

// Sensors: index 0 = SFM3000, 1 to 7 = p1 to p7.
int fail_run[8];
uint32_t last_restart[8];
bool sfm_discard = false;      // next good SFM3000 read is the invalid first one
long sfm_sum = 0;              // flow averaging: sum and count of good raw words in the window
int sfm_n = 0;
bool sfm_bad = false;          // a read in this tick's window failed
unsigned long sfm_last_us = 0; // start of the last SFM3000 read
unsigned long sfm_tick_us = 0; // start of the last tick read: the extra reads are timed from it
unsigned long sfm_first_us = 0;  // start of the window's first extra read
int sfm_extra = 0;             // extra reads tried in this window
int sfm_slot = FLOW_AVG_READS; // next extra read, 1 to FLOW_AVG_READS - 1 (FLOW_AVG_READS: none due)
uint16_t sfm_prev_word = 0;    // last tick read's raw word, for the frozen-output check
int sfm_same = 0;              // tick reads in a row that returned sfm_prev_word
int sdp_blank[7];              // reads still blanked after an SDP811 restart
int16_t sdp_scale = 0;         // last SDP811 scale factor (counts per Pa)

// This tick's sample, sent as one row.
uint16_t sfm_raw = 0;          // raw word of the last read, then the window mean
bool row_sfm_ok = false;
int row_sfm_n = 0;
unsigned long row_sfm_span_us = 0;
int pot_raw = 0;
unsigned long pot_dus = 0;
int16_t sdp_raw[7];
bool sdp_ok[7];
bool sdp_tried[7];
unsigned long sdp_dus[7];

// Output state.
uint32_t last_tick = 0;
bool config_requested = false;
char cmd_buf[4];               // collects the letters of "max" / "min" / "s1" to "s4"
int cmd_len = 0;
unsigned long cmd_ms = 0;

// A line built in memory and sent with one write.
const int LINE_LEN = 768;              // the # config line is about 620 characters
struct Line : public Print {
  char s[LINE_LEN];
  int n;
  size_t write(uint8_t c) { if (n < LINE_LEN) s[n++] = c; return 1; }
};
Line out_line;

// Sent only while a program has the port open (DTR set: the logger and the
// Arduino IDE Serial Monitor both set it), so no rows queue up with nobody reading.
void sendLine(Line &L) {
  if (!PC) return;
  PC.write(L.s, L.n);
  PC.println();
}

// One CSV row of raw values (format in the header comment).
void sendRow(uint32_t tick, uint32_t breath, int m, float cmd) {
  Line &L = out_line;
  L.n = 0;
  L.print(tick);
  L.print(',');
  L.print(sfm_tick_us);
  L.print(',');
  L.print(breath);
  L.print(',');
  L.print(m);
  L.print(',');
  L.print((int)(cmd + 0.5f));
  L.print(',');
  L.print(pot_raw);
  L.print(',');
  if (row_sfm_ok) L.print(sfm_raw);
  L.print(',');
  L.print(row_sfm_n);
  L.print(',');
  L.print(row_sfm_span_us);
  L.print(',');
  L.print(pot_dus);
  for (int i = 0; i < 7; i++) {
    L.print(',');
    if (sdp_ok[i]) L.print(sdp_raw[i]);
  }
  for (int i = 0; i < 7; i++) {
    L.print(',');
    if (sdp_tried[i]) L.print(sdp_dus[i]);
  }
  sendLine(L);
}

// ---- Actuator ----

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
    float t_ms = ss_stroke_ml / Q_mls * 1000.0f;
    ss_out_ms[i] = (unsigned long)(t_ms / (SS_GAIN_ON ? SS_GAIN_OUT[i] : 1.0f) + 0.5f);
    ss_in_ms[i] = (unsigned long)(t_ms / (SS_GAIN_ON ? SS_GAIN_IN[i] : 1.0f) + 0.5f);
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

// Steady flow cycle, phi_ms into it: move min -> max, hold at max, move
// max -> min, hold at min. Constant speed on each move.
void stepSteady(unsigned long phi_ms) {
  int i = current_preset - MODE_SS;
  unsigned long out_ms = ss_out_ms[i];
  unsigned long in_start = out_ms + SS_DWELL_MS;
  float pos;
  if (phi_ms < out_ms) pos = (float)phi_ms / out_ms;
  else if (phi_ms < in_start) pos = 1.0f;
  else if (phi_ms < in_start + ss_in_ms[i]) pos = 1.0f - (float)(phi_ms - in_start) / ss_in_ms[i];
  else pos = 0.0f;
  outputProfile((float)ss_min_dac + (float)(ss_max_dac - ss_min_dac) * pos);
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
    }
  } else {
    writeCommand(cmd_f + (diff > 0.0f ? RAMP_STEP_DAC : -RAMP_STEP_DAC));
  }
}

// Ramp toward target_dac at RAMP_STEP_DAC per update, then hold there. With a
// steady mode in pending_mode, hold SS_DWELL_MS at min first, then start it.
void stepGoto() {
  float diff = (float)target_dac - cmd_f;
  if (fabsf(diff) <= RAMP_STEP_DAC) {
    writeCommand((float)target_dac);
    if (pending_mode >= MODE_SS) {
      if (++settle_ms >= SS_DWELL_MS) {
        startMode(pending_mode);
        pending_mode = MODE_IDLE;
      }
      return;
    }
    current_preset = MODE_HOLD;
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
  } else if (m >= MODE_SS) {        // steady: a new "breath" as each move starts
    int i = m - MODE_SS;
    unsigned long in_start = ss_out_ms[i] + SS_DWELL_MS;
    stepSteady(phase_ms);
    if (++phase_ms >= in_start + ss_in_ms[i] + SS_DWELL_MS) phase_ms = 0;
    if (phase_ms == 0 || phase_ms == in_start) breath_count++;
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

void goToExtreme(bool is_max) {
  if (current_preset == MODE_RETURNING) return;
  target_dac = applyDacCorrection(is_max ? (JRK_CENTRE + JRK_HALF_RANGE) : (JRK_CENTRE - JRK_HALF_RANGE));
  pending_mode = MODE_IDLE;
  current_preset = MODE_GOTO;
}

// Keys s1 to s4 (i = 0 to 3): ramp straight to min from wherever the command
// is, hold, then steady flow. stepGoto() starts it.
void startSteady(int i) {
  int m = current_preset;
  if (m == MODE_RETURNING || m == MODE_GOTO || m == MODE_HOLD) return;
  target_dac = ss_min_dac;
  settle_ms = 0;
  pending_mode = MODE_SS + i;
  current_preset = MODE_GOTO;
}

// Keys 1 to 4 and H.
void handleKey(char key) {
  int m = current_preset;
  if (key == 'h') {
    pending_mode = MODE_IDLE;        // cancels a start waiting for centre or min
    if (m != MODE_IDLE) current_preset = MODE_RETURNING;
    return;
  }
  if (m == MODE_RETURNING || m == MODE_GOTO || m == MODE_HOLD) return;
  int next = key - '1';
  if (m == MODE_IDLE) {
    startMode(next);
  } else {             // breathing or steady: ramp to centre, then start
    pending_mode = next;
    current_preset = MODE_RETURNING;
  }
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
    if (cmd_len == 0 && key == 'c') {     // touches no ISR state
      config_requested = true;
      continue;
    }
    if (cmd_len > 0 || key == 'm' || key == 's') {
      cmd_buf[cmd_len++] = key;
      if (cmd_buf[0] == 's' && cmd_len == 2) {
        if (key >= '1' && key <= '4') {
          noInterrupts();
          startSteady(key - '1');
          interrupts();
        }
        cmd_len = 0;
      } else if (cmd_len == 3) {
        cmd_buf[3] = '\0';
        bool is_max = strcmp(cmd_buf, "max") == 0;
        if (is_max || strcmp(cmd_buf, "min") == 0) {
          noInterrupts();
          goToExtreme(is_max);
          interrupts();
        }
        cmd_len = 0;
      }
    } else if (key == 'h' || (key >= '1' && key <= '4')) {
      noInterrupts();
      handleKey(key);
      interrupts();
    }
  }
}

// The constants the logger needs to convert the rows, and the start-up checks.
void printList(Line &L, const char *name, const float *v, int digits) {
  L.print(name);
  for (int i = 0; i < 4; i++) {
    if (i > 0) L.print(',');
    L.print(v[i], digits);
  }
}

void printConfig() {
  float g_exp[4], g_insp[4], ss_out[4], ss_in[4], period[4], ss_go[4], ss_gi[4];
  for (int i = 0; i < 4; i++) {
    g_exp[i] = FLOW_GAIN_ON ? GAIN_EXP[i] : 1.0f;
    g_insp[i] = FLOW_GAIN_ON ? GAIN_INSP[i] : 1.0f;
    ss_go[i] = SS_GAIN_ON ? SS_GAIN_OUT[i] : 1.0f;
    ss_gi[i] = SS_GAIN_ON ? SS_GAIN_IN[i] : 1.0f;
    ss_out[i] = ss_out_ms[i];
    ss_in[i] = ss_in_ms[i];
    period[i] = (unsigned long)(preset_period_us[i] / 1000.0f + 0.5f);
  }
  Line &L = out_line;
  L.n = 0;
  L.print("# config fmt=1 centre_dac=");
  L.print(corrected_centre_dac);
  L.print(" stroke_amp_dac=");
  L.print(stroke_amp_dac, 3);
  printList(L, " gain_exp=", g_exp, 3);
  printList(L, " gain_insp=", g_insp, 3);
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
  L.print(" flow_avg_spacing_us=");
  L.print(FLOW_AVG_SPACING_US);
  printList(L, " period_ms=", period, 0);
  printList(L, " ss_q_Lmin=", preset_q_peak_Lmin, 1);
  printList(L, " ss_gain_out=", ss_go, 4);
  printList(L, " ss_gain_in=", ss_gi, 4);
  printList(L, " ss_out_ms=", ss_out, 0);
  printList(L, " ss_in_ms=", ss_in, 0);
  L.print(" ss_dwell_ms=");
  L.print(SS_DWELL_MS);
  L.print(" ss_margin_jrk=");
  L.print(SS_MARGIN_JRK);
  L.print(" ss_stroke_ml=");
  L.print(ss_stroke_ml, 1);
  L.print(" sfm_found=");
  L.print(sfm_found ? 1 : 0);
  L.print(" sdp_found=");
  for (int i = 0; i < 7; i++) {
    if (i > 0) L.print(',');
    L.print(sdp_found[i] ? 1 : 0);
  }
  L.print(" timer_ok=");
  L.print(timer_ok ? 1 : 0);
  L.print(" wire1_pulses=");
  L.print(wire1_pulses);
  L.print(" end=1");
  sendLine(L);
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

// One SFM3000 read into sfm_raw. False on a failed read (short read, CRC with
// initial value 0x00, or a word outside the sensor's range).
bool readFlow() {
  uint8_t buf[3];
  SensirionI2CRxFrame rx(buf, sizeof(buf));
  uint16_t raw;
  if (SensirionI2CCommunication::receiveFrame(SFM3000_ADDR, 3, rx, Wire, CRC31_00) || rx.getUInt16(raw)) return false;
  sfm_raw = raw;
  return fabsf(((float)raw - SFM3000_OFFSET) / SFM3000_SCALE) <= SFM3000_MAX_SLM;
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

// Read the SDP811 on mux channel ch (already selected) into sdp_raw[ch].
// 9 bytes: pressure, temperature and scale factor, each 2 bytes + CRC (initial
// value 0xFF). receiveFrame checks all three CRCs; the temperature is not used.
bool readPressure(int ch) {
  uint8_t buf[9];
  SensirionI2CRxFrame rx(buf, sizeof(buf));
  if (SensirionI2CCommunication::receiveFrame(SDP_ADDR, 9, rx, Wire1, CRC31_ff)) {
    while (Wire1.available()) Wire1.read();   // empty the buffer after a short read
    return false;
  }
  int16_t raw_pressure, raw_temp, raw_scale;
  rx.getInt16(raw_pressure);
  rx.getInt16(raw_temp);
  rx.getInt16(raw_scale);
  if (raw_scale <= 0) return false;
  sdp_raw[ch] = raw_pressure;
  sdp_scale = raw_scale;
  return true;
}

// Count a read result. After SENSOR_FAIL_TICKS failures in a row, resend the
// start command, at most once per second, and never on a tick whose bus failed.
void sensorResult(int s, bool ok, uint32_t tick, bool bus_ok) {
  if (ok) {
    fail_run[s] = 0;
    return;
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
  if (sfm_extra++ == 0) sfm_first_us = now;
  sfm_slot++;
  if (readFlow()) {
    sfm_sum += sfm_raw;
    sfm_n++;
  } else {
    sfm_bad = true;
  }
}

// Read the SFM3000 (the tick read closes the averaging window), the pot, then
// p1 to p7, recording when each read started.
void readSensors(uint32_t tick) {
  // The SFM3000 NACKs a read with no new result, so leave SFM_MIN_GAP_US after
  // an averaging read.
  unsigned long since = micros() - sfm_last_us;
  if (FLOW_AVG_READS > 1 && since < SFM_MIN_GAP_US) delayMicroseconds(SFM_MIN_GAP_US - since);
  sfm_last_us = sfm_tick_us = micros();
  bool ok = readFlow();
  if (ok) {                     // frozen-output check on the tick read's own word
    sfm_same = (sfm_raw == sfm_prev_word) ? sfm_same + 1 : 1;
    sfm_prev_word = sfm_raw;
    if (sfm_same >= SFM_FROZEN_TICKS) ok = false;
  }
  bool good = ok && !sfm_discard;
  if (ok) sfm_discard = false;
  if (good) {
    sfm_sum += sfm_raw;
    sfm_n++;
    sfm_raw = (uint16_t)((float)sfm_sum / sfm_n + 0.5f);
  }
  row_sfm_ok = good && !sfm_bad;            // any failed read in the window: no value
  row_sfm_n = sfm_n;
  row_sfm_span_us = sfm_extra > 0 ? sfm_tick_us - sfm_first_us : 0;
  sfm_extra = 0;
  sfm_sum = 0;
  sfm_n = 0;
  sfm_bad = false;
  sfm_slot = 1;                 // next window starts from this read
  sensorResult(0, ok, tick, true);

  // Pot straight after the tick read, close to the time the flow mean stands for.
  pot_dus = micros() - sfm_tick_us;
  pot_raw = analogRead(A1);

  bool bus_ok = true;
  for (int ch = 0; ch < 7; ch++) {
    sdp_ok[ch] = false;
    sdp_tried[ch] = false;
    sfmSample();                           // flow averaging also covers the burst
    if (bus_ok && !muxWrite(1 << ch)) bus_ok = false;   // a failed mux write skips the rest of Wire1
    bool read_ok = false;
    if (bus_ok) {
      sdp_tried[ch] = true;
      sdp_dus[ch] = micros() - sfm_tick_us;
      read_ok = readPressure(ch);
    }
    sdp_ok[ch] = read_ok;
    if (sdp_blank[ch] > 0) {
      sdp_blank[ch]--;
      sdp_ok[ch] = false;
    }
    sensorResult(ch + 1, read_ok, tick, bus_ok);
  }
  if (bus_ok) muxWrite(0);   // no SDP811 stays connected between bursts
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
  PC.println("# MechanicalLungPlatform: 1 2 3 4 s1 s2 s3 s4 max min H c");

  // Start-up checks. Lines sent before the PC opens the port are dropped (native
  // USB), so the results are also in # config.
  Wire.begin();
  Wire.setClock(WIRE_CLOCK_HZ);
  Wire.setWireTimeout(WIRE_TIMEOUT_US);
  wire1_pulses = recoverWire1();
  if (wire1_pulses != 0) {
    PC.print("# Wire1 bus recovery: ");
    if (wire1_pulses < 0) PC.println("SDA still held low, power cycle the rig");
    else { PC.print(wire1_pulses); PC.println(" SCL pulses, bus freed"); }
  }
  Wire1.begin();
  Wire1.setClock(WIRE1_CLOCK_HZ);
  Wire1.setWireTimeout(WIRE1_TIMEOUT_US);

  uint16_t err = sfmStart();
  sfm_discard = true;
  sfm_found = err == 0;
  if (sfm_found) {
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
    sdp_found[ch] = sdpStart();
    PC.println(sdp_found[ch] ? " found" : " not found");
    delay(25);
  }
  muxWrite(0);

  uint8_t timer_type;
  int8_t timer_ch = FspTimer::get_available_timer(timer_type);
  timer_ok = timer_ch >= 0
             && dac_timer.begin(TIMER_MODE_PERIODIC, timer_type, timer_ch, (float)DAC_UPDATE_HZ, 0.0f, onTick)
             && dac_timer.setup_overflow_irq(TIMER_PRIORITY)
             && dac_timer.open()
             && dac_timer.start();
  if (timer_ok) {
    PC.print("# timer running, interrupt priority ");
    PC.println(NVIC_GetPriority(dac_timer.get_cfg()->cycle_end_irq));
  } else {
    PC.println("# TIMER SETUP FAILED: the actuator will not move");
  }
}

void loop() {
  readKeys();

  noInterrupts();   // copy only
  uint32_t tick = snap_tick;
  float cmd = snap_cmd;
  int m = snap_mode;
  uint32_t breath = snap_breath;
  interrupts();

  if (tick == last_tick) {
    sfmSample();   // flow averaging in the idle time
    return;
  }
  last_tick = tick;
  readSensors(tick);
  sendRow(tick, breath, m, cmd);
  if (config_requested) {
    config_requested = false;
    printConfig();
  }
}

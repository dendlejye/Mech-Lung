/*
  MechLung_NoLogger.ino

  Runs the actuator breathing profile, the SFM3000 flow sensor and seven
  SDP811 pressure sensors together, and checks the volume delivered each
  breath against the commanded 700 mL. Arduino IDE Serial Plotter / Monitor.

  Hardware (Arduino UNO R4 WiFi)
    A0     12-bit DAC output -> Jrk G2 analog input
    A1     12-bit ADC input  <- position potentiometer
    Wire   SFM3000 flow sensor, address 0x40 (pull-ups 4-8k)
    Wire1  Qwiic -> PCA9548 mux 0x70, SDP811 0x26 on channels 0 to 6 (p1 to p7)

  How it runs
    A 1 kHz timer interrupt computes the profile, the filter and the ramps and
    writes the DAC, so sensor reads and Serial cannot delay the actuator.
    Every 10th interrupt is a 100 Hz sample tick: loop() reads the SFM3000,
    p1 to p7 and the pot, updates the volumes and sends at most one line.

  Keys (Serial at SERIAL_BAUD, not case sensitive)
    1 2 3 4  breathe at a peak flow of 10, 15, 20 or 30 L/min
    s        demo sine (actuator only, sensors off)
    max min  go to that end of the calibrated Jrk window and hold
    H        stop: return to centre at RAMP_RATE_DAC_PER_S. Tares if already idle.
    p        cycle plot set
    A start key while breathing or in the demo ramps to centre first, then
    starts. While returning to centre start keys are refused; at max or min
    send H first.

  Plot sets (one row every PLOT_DIV sample ticks)
    1  flow_Lmin_x20, vol_cmd_ml, vol_pot_ml, vol_measured_ml
    2  leak_gap_ml (vol_pot - vol_measured), track_err_ml (vol_cmd - vol_pot)
    3  p1 to p7 in Pa
    4  command_dac, feedback_dac (also shown during the demo sine)
    Flow and pressures are means over the row, volumes the latest value.
    Volume and gap channels print 0 outside breathing. A failed read prints nan.
    vol_measured_ml starts each breath at vol_pot_ml, so both measure from centre
    even when piston lag leaves the piston off centre at phase 0.

  Lines starting with # are comments: start-up checks, tare, mode changes,
  "# breath" once per breath, "# breath aborted", the first read failure of
  each sensor, and "# status" once per second.

  Travel limits are set inside the Jrk, not here. Set them before the first run.
*/
#include <Wire.h>
#include "FspTimer.h"

// Actuator Calibration Constants Copy from Calibration Sketch
const int   JRK_CENTRE = 2049;
const int   JRK_HALF_RANGE = 2002;
const float DAC_CORR_OFFSET = 3.90f;
const float DAC_CORR_GAIN = 4165.15f;
const float POT_RAW_AT_CENTRE = 1165.89f;
const float POT_DAC_COUNTS_PER_ADC = 1.8206f;
// Record of this calibration run:
// Jrk Input window (Input setup wizard): Minimum=47 Maximum=4051
// Jrk Feedback (Feedback setup wizard): forward=2294 reverse=88
// A1 at the ends: forward=2247.9 reverse=83.4
// Step 3 point 1: applied=648 Jrk Input=663 Jrk Feedback=428 A1=418.7
// Step 3 point 2: applied=2049 Jrk Input=2088 Jrk Feedback=1212 A1=1187.1
// Step 3 point 3: applied=3450 Jrk Input=3513 Jrk Feedback=1996 A1=1954.6
// A1 vs Jrk Feedback cross-check: worst difference 1.8 Jrk counts (0.08 % of span)
// Step 4 min: command=42 feedback=41.6 error=-0.4 counts (-0.01 % of span)
// Step 4 mid: command=2011 feedback=2013.3 error=2.3 counts (0.06 % of span)
// Step 4 max: command=3979 feedback=3979.9 error=0.9 counts (0.02 % of span)

const int DAC_MAX = 4095;             // 12-bit DAC full scale

// Rig and breathing settings.
// ACTUATOR_STROKE_MM: distance between the min and max ends of the calibrated
// Jrk window, measured with the min / max commands. Measured on this rig: 230 mm.
// If tidal volume or peak flow measure off on the rig, re-check it first.
const float ACTUATOR_STROKE_MM = 230.0f;
const float CYLINDER_DIAMETER_MM = 130.0f;    // bore of the cylinder
const float V_TIDAL_ML = 700.0f;              // volume moved per breath
const float IE_EXP_PARTS = 2.0f;              // expiration : inspiration time ratio
const float IE_INSP_PARTS = 1.0f;
float preset_q_peak_Lmin[4] = {10.0f, 15.0f, 20.0f, 30.0f};  // keys 1 to 4

// Timing.
const int DAC_UPDATE_HZ = 1000;       // timer rate: DAC written every 1 ms
const int SAMPLE_HZ = 100;            // sensor sample rate, must divide DAC_UPDATE_HZ
const int TICKS_PER_SAMPLE = DAC_UPDATE_HZ / SAMPLE_HZ;
const uint8_t TIMER_PRIORITY = 4;     // above I2C, USB, serial (12) and millis (8)
const int PLOT_DIV = 40;              // sample ticks per plot row (2.5 rows/s)
const unsigned long SERIAL_BAUD = 115200;  // set from breakout sketch B1
const float DEMO_SINE_PERIOD_US = 20.0e6f; // 20 s per demo sweep
const int DEMO_SINE_MARGIN_JRK = 0;        // stay this far from the Jrk window ends

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
const float SFM3000_MAX_SLM = 200.0f;   // sensor range, used to clamp output

// Tare: after the pot stays within TARE_STILL_COUNTS for TARE_SETTLE_MS,
// average TARE_SAMPLES of flow and of each pressure.
const int TARE_SAMPLES = 100;
const unsigned long TARE_SETTLE_MS = 2000;  // PLACEHOLDER: set on the bench from pot noise at rest
const float TARE_STILL_COUNTS = 5.0f;       // PLACEHOLDER: set on the bench, in DAC counts

// I2C.
const uint8_t SFM3000_ADDR = 0x40;
const uint8_t MUX_ADDR = 0x70;
const uint8_t SDP_ADDR = 0x26;
const uint32_t WIRE_CLOCK_HZ = 400000;   // never 200000: this core keeps the old rate
const uint32_t WIRE1_CLOCK_HZ = 400000;
const unsigned int I2C_TIMEOUT_US = 1000;
const int SENSOR_FAIL_TICKS = 5;         // consecutive failures before a restart

// Modes held in current_preset (0 to 3 are the breathing presets).
const int MODE_IDLE = -1;
const int MODE_DEMO = -2;
const int MODE_RETURNING = -3;
const int MODE_GOTO = -4;      // ramping to the max or min end
const int MODE_HOLD = -5;      // parked at max or min until H

float preset_t_exp_us[4];      // expiration time
float preset_t_insp_us[4];     // inspiration time
float preset_period_us[4];     // one full breath
float stroke_amp_dac = 0.0f;   // stroke amplitude in DAC counts
int corrected_centre_dac = 0;
float filter_alpha = 1.0f;     // per-update filter coefficient (1 = no filter)

// Shared with the timer ISR. loop() changes these only with interrupts off.
volatile int current_preset = MODE_IDLE;
volatile int pending_mode = MODE_IDLE;   // mode to start once back at centre
volatile unsigned long phase_ms = 0;     // time into the breath or sweep
volatile uint32_t breath_count = 0;      // +1 at every breath start
volatile float cmd_f = 0.0f;             // command written, in DAC counts
volatile float filt1 = 0.0f, filt2 = 0.0f;
volatile int target_dac = 0;
volatile bool target_is_max = false;
volatile int isr_event = 0;              // 1 centre, 2 at max, 3 at min
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
bool flow_bad = false;
uint32_t last_breath = 0, breath_start = 0;
float vol_meas = 0, vol_seed = 0, pk_pot, pk_meas, pk_flow, t_pot, t_meas, p_pk[7];

// Output state.
int plot_set = 1;
float sum_flow = 0, sum_p[7], n_sum = 0;
uint32_t last_tick = 0, last_row = 0, last_status = 0;
unsigned long dropped_ticks = 0, skipped_rows = 0, max_loop_us = 0;
char cmd_buf[4];               // collects the letters of "max" / "min"
int cmd_len = 0;
unsigned long cmd_ms = 0;

// ---- Serial writer: lines are queued, one is sent per sample tick ----
const int LINE_LEN = 256;
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
  Serial.write(queue_lines[q_head].s, queue_lines[q_head].n);
  Serial.println();
  q_head = (q_head + 1) % QUEUE_LEN;
  q_count--;
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
    preset_t_exp_us[i] = t_exp_s * 1e6f;
    preset_t_insp_us[i] = t_insp_s * 1e6f;
    preset_period_us[i] = (t_exp_s + t_insp_s) * 1e6f;
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
  outputProfile((float)corrected_centre_dac + stroke_amp_dac * pos_norm);
}

// Demo sine: sweeps the calibrated Jrk window end to end, less
// DEMO_SINE_MARGIN_JRK at each end. Starts at the middle, toward max first.
void stepDemoSine(float phi) {
  int min_dac = applyDacCorrection(JRK_CENTRE - JRK_HALF_RANGE + DEMO_SINE_MARGIN_JRK);
  int max_dac = applyDacCorrection(JRK_CENTRE + JRK_HALF_RANGE - DEMO_SINE_MARGIN_JRK);
  float mid_dac = 0.5f * (float)(min_dac + max_dac);
  float half_span_dac = 0.5f * (float)(max_dac - min_dac);
  outputProfile(mid_dac + half_span_dac * sinf(2.0f * PI * phi / DEMO_SINE_PERIOD_US));
}

// Start breathing or the demo at phase 0. The filter starts from the command now written.
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

// Ramp toward target_dac at RAMP_STEP_DAC per update, then hold there.
void stepGoto() {
  float diff = (float)target_dac - cmd_f;
  if (fabsf(diff) <= RAMP_STEP_DAC) {
    writeCommand((float)target_dac);
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
  } else if (m >= 0 || m == MODE_DEMO) {
    float period_us = (m == MODE_DEMO) ? DEMO_SINE_PERIOD_US : preset_period_us[m];
    if (m == MODE_DEMO) stepDemoSine(phase_ms * 1000.0f);
    else stepBreathing(phase_ms * 1000.0f);
    if (++phase_ms >= (unsigned long)(period_us / 1000.0f + 0.5f)) {
      phase_ms = 0;
      if (m >= 0) breath_count++;
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

void handleKey(char key) {
  int m = current_preset;
  bool is_start = (key >= '1' && key <= '4') || key == 's';

  if (key == 'p') {
    if (m != MODE_DEMO) plot_set = plot_set % 4 + 1;
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
      current_preset = MODE_RETURNING;
      say("# returning to centre");
    }
    return;
  }

  int next = (key == 's') ? MODE_DEMO : key - '1';
  if (m == MODE_IDLE) {
    startMode(next);
  } else {             // breathing or demo: ramp to centre, then start
    pending_mode = next;
    current_preset = MODE_RETURNING;
    say("# returning to centre first");
  }
  if (next == MODE_DEMO) {
    say("# demo sine");
  } else {
    Line &L = newLine();
    L.print("# breathing, peak flow ");
    L.print(preset_q_peak_Lmin[next], 0);
    L.print(" L/min");
  }
}

void readKeys() {
  if (cmd_len > 0 && millis() - cmd_ms > 1000) cmd_len = 0;   // "max"/"min" buffer times out
  while (Serial.available()) {
    char key = (char)tolower(Serial.read());
    cmd_ms = millis();
    if (key == '\n' || key == '\r') {
      cmd_len = 0;
      continue;
    }
    noInterrupts();
    if (cmd_len > 0 || key == 'm') {
      cmd_buf[cmd_len++] = key;
      if (cmd_len == 3) {
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

// ---- Sensors (from flow_sfm3000.ino and pressure_sdp811.ino) ----

// CRC-8, polynomial 0x31. Initial value 0x00 for the SFM3000, 0xFF for the SDP811.
uint8_t crc8(uint8_t init, uint8_t hi, uint8_t lo) {
  uint8_t crc = init;
  uint8_t data[2] = {hi, lo};
  for (int i = 0; i < 2; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else crc = crc << 1;
    }
  }
  return crc;
}
uint8_t crc8_sfm(uint8_t hi, uint8_t lo) { return crc8(0x00, hi, lo); }
uint8_t crc8_sdp(uint8_t hi, uint8_t lo) { return crc8(0xFF, hi, lo); }

// SFM3000 start continuous measurement (0x1000). Returns the endTransmission code.
uint8_t sfmStart() {
  Wire.beginTransmission(SFM3000_ADDR);
  Wire.write(0x10);
  Wire.write(0x00);
  return Wire.endTransmission();
}

// One flow value in slm, or NAN on a failed read.
float readFlow() {
  if (Wire.requestFrom((int)SFM3000_ADDR, 3) != 3) return NAN;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  uint8_t crc = Wire.read();
  if (crc8_sfm(hi, lo) != crc) return NAN;
  uint16_t raw = ((uint16_t)hi << 8) | lo;
  return constrain(((float)raw - SFM3000_OFFSET) / SFM3000_SCALE, -SFM3000_MAX_SLM, SFM3000_MAX_SLM);
}

// Write the mux channel mask (1 << channel, or 0 for none). Returns true on success.
bool muxWrite(uint8_t mask) {
  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(mask);
  return Wire1.endTransmission() == 0;
}

// SDP811 start continuous measurement, differential pressure, no averaging (0x361E).
bool sdpStart() {
  Wire1.beginTransmission(SDP_ADDR);
  Wire1.write(0x36);
  Wire1.write(0x1E);
  return Wire1.endTransmission() == 0;
}

// SDP811 stop measurement (0x3FF9). The sensor may not answer if it is already idle.
void sdpStop() {
  Wire1.beginTransmission(SDP_ADDR);
  Wire1.write(0x3F);
  Wire1.write(0xF9);
  Wire1.endTransmission();
}

// Read the SDP811 on the selected mux channel, in Pa, or NAN on a failed read.
// 9 bytes: pressure, temperature and scale factor, each 2 bytes + CRC.
float readPressure() {
  if (Wire1.requestFrom((uint8_t)SDP_ADDR, (uint8_t)9) < 9) {
    while (Wire1.available()) Wire1.read();   // empty the buffer after a short read
    return NAN;
  }
  uint8_t b[9];
  for (int i = 0; i < 9; i++) b[i] = Wire1.read();
  if (crc8_sdp(b[0], b[1]) != b[2]) return NAN;  // pressure
  if (crc8_sdp(b[3], b[4]) != b[5]) return NAN;  // temperature (checked, not used)
  if (crc8_sdp(b[6], b[7]) != b[8]) return NAN;  // scale factor
  int16_t raw_pressure = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  int16_t raw_scale = (int16_t)(((uint16_t)b[6] << 8) | b[7]);
  if (raw_scale <= 0) return NAN;
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
void readSensors(float raw[8], uint32_t tick) {
  raw[0] = readFlow();
  bool ok = !isnan(raw[0]);
  if (ok && sfm_discard) {
    sfm_discard = false;
    raw[0] = NAN;
  }
  sensorResult(0, ok, tick, true);

  bool bus_ok = true;
  for (int ch = 0; ch < 7; ch++) {
    raw[ch + 1] = NAN;
    if (bus_ok && !muxWrite(1 << ch)) {   // a failed mux write skips the rest of Wire1
      bus_ok = false;
      mux_err++;
    }
    if (bus_ok) raw[ch + 1] = readPressure();
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
    Line &L = newLine();
    L.print("# tare");
    for (int i = 0; i < 8; i++) {
      tare[i] = tare_sum[i] / tare_n[i];   // no good reads gives nan
      L.print(i == 0 ? " flow_slm=" : " ");
      if (i > 0) { L.print(SENSOR_NAME[i]); L.print("_Pa="); }
      L.print(tare[i], 3);
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
  float v[15] = {V_TIDAL_ML - pk_pot, leak, 100.0f * leak / pk_pot, t_pot, t_meas,
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

void setup() {
  Serial.begin(SERIAL_BAUD);
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
  Serial.println("# MechLung_NoLogger: 1 2 3 4 s max min H p");

  Wire.begin();
  Wire.setClock(WIRE_CLOCK_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US);
  Wire1.begin();
  Wire1.setClock(WIRE1_CLOCK_HZ);
  Wire1.setWireTimeout(I2C_TIMEOUT_US);

  uint8_t err = sfmStart();
  sfm_discard = true;
  if (err == 0) {
    Serial.println("# SFM3000 found");
  } else {
    Serial.print("# SFM3000 not found, I2C error code ");
    Serial.println(err);
  }
  for (int ch = 0; ch < 7; ch++) {
    Serial.print("# SDP811 p");
    Serial.print(ch + 1);
    if (!muxWrite(1 << ch)) {
      Serial.println(": multiplexer channel not found");
      continue;
    }
    delay(20);
    sdpStop();      // in case a previous run left the sensor measuring
    delay(1);
    Serial.println(sdpStart() ? " found" : " not found");
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
    Serial.print("# timer running, interrupt priority ");
    Serial.println(NVIC_GetPriority(dac_timer.get_cfg()->cycle_end_irq));
  } else {
    Serial.println("# TIMER SETUP FAILED: the actuator will not move");
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
  if (tick == last_tick) return;

  unsigned long t0 = micros();
  uint32_t dticks = tick - last_tick;
  dropped_ticks += dticks - 1;
  last_tick = tick;

  float raw[8];
  for (int i = 0; i < 8; i++) raw[i] = NAN;
  if (m != MODE_DEMO) readSensors(raw, tick);   // sensors off in the demo
  readFeedback();
  if (tare_state) updateTare(raw, m, tick);

  float flow = FLOW_SIGN * (raw[0] - tare[0]) * SLM_TO_LMIN;
  float p[7];
  for (int i = 0; i < 7; i++) p[i] = raw[i + 1] - tare[i + 1];

  // Breath boundary: the ISR's breath counter changed.
  if (m < 0) in_breath = false;
  if (breath != last_breath) {
    last_breath = breath;
    if (in_breath) printBreath();
    in_breath = (m >= 0);
    breath_start = tick;
    flow_bad = false;
    vol_meas = 0;
    pk_pot = pk_meas = pk_flow = -INFINITY;
    t_pot = t_meas = NAN;
    for (int i = 0; i < 7; i++) p_pk[i] = NAN;
  }

  float vc = 0, vp = 0, vm = 0;
  if (in_breath) {
    vc = V_TIDAL_ML * (cmd - corrected_centre_dac) / stroke_amp_dac;
    vp = V_TIDAL_ML * (feedback_dac - corrected_centre_dac) / stroke_amp_dac;
    if (tick == breath_start) vol_meas = vol_seed = vp;   // same zero as vol_pot_ml
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
  }

  // Plot row every PLOT_DIV sample ticks.
  if (m != MODE_DEMO) {
    sum_flow += flow;
    for (int i = 0; i < 7; i++) sum_p[i] += p[i];
    n_sum++;
  }
  if (tick - last_row >= (uint32_t)PLOT_DIV) {
    last_row = tick;
    Line &L = newLine();
    if (m == MODE_DEMO || plot_set == 4) {
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
    L.print(" max_loop_us=");
    L.print(max_loop_us);
  }

  sendOneLine();   // at most one line per sample tick
  unsigned long loop_us = micros() - t0;
  if (loop_us > max_loop_us) max_loop_us = loop_us;
}

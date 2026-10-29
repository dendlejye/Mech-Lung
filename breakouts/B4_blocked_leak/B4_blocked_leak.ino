/*
  B4_blocked_leak.ino

  Blocked-outlet leak test. With the outlet plugged downstream of every
  pressure tap, nothing flows through the line, so every SDP811 reads cylinder
  pressure. The piston moves at a slow constant speed until the pressure levels
  off; then all the air it displaces leaves past the seal (and any joint), so

      leak = piston area x piston speed (from the pot), at the measured pressure

  with no SFM3000 in the figure. The SFM3000 stays in line and reads the air
  leaving between it and the plug: mainly the flow through the seven SDP811s
  (flow-through sensors, one port open to air), plus any joint leak there.
  Repeat at several speeds, both directions, for leak against pressure.

  Forked from MechLung_SteadyStateTest: same sensors, tare, log rows and
  # config, so leaktest.py runs it (queue items b1 to b5). Breathing, steady
  flow, max / min and plot mode are removed.

  Setup
    Plug the airway model's open end (all SDP811 taps upstream of the plug).
    The pressure decays through the leak at rest, so the tare at centre is a
    zero; leaktest.py reports the pressure at the end of each dwell to check.

  Hardware (Arduino UNO R4 WiFi), as MechLung_SteadyStateTest
    A0     12-bit DAC output -> Jrk G2 analog input
    A1     12-bit ADC input  <- position potentiometer
    Wire   SFM3000 flow sensor, address 0x40 (4.7k pull-ups), 100 kHz
    Wire1  Qwiic -> PCA9548 mux 0x70, SDP811 0x26 on channels 0 to 6 (p1 to p7)

  Keys (over native USB; not case sensitive)
    b1 to b5  blocked test at BL_SPEED_MM_S (0.25, 0.5, 1, 2, 4 mm/s): ramp to
              the start point (centre less half the move) at the test speed,
              hold BL_DWELL_MS, then repeat until H:
                out move (min side -> max side, air out, + pressure) BL_MOVE_MS,
                dwell BL_DWELL_MS, in move back BL_MOVE_MS, dwell BL_DWELL_MS.
              Each move plus the dwell after it is one "breath" in the log; the
              counter goes up as each move starts. Mode 8 to 12 in the rows.
    H         return to centre at the last test speed (RAMP_RATE_DAC_PER_S
              at most). Tares if already idle.
    L         log mode on / off (for leaktest.py)
    A b key while a test runs ramps straight to the new start point.

  Pressure limit: if any tared SDP811 reads beyond +-BL_P_LIMIT_PA while the
  piston moves, the command stops where it is (mode -5) and "# pressure limit"
  is sent; leaktest.py stops the queue. Unplug the outlet, then H. The ramps
  (H and the move to the start point) run at the test speed for the same
  reason, so they build no more pressure than the test itself.

  Lines starting with # are comments: start-up checks, tare, mode changes, and
  (outside log mode) "# status" once per second with the mean pressure.

  Log mode (key L): "# config ..." and "# log on", then one CSV row per 100 Hz
  sample tick, as MechLung_SteadyStateTest:
    tick,breath,mode,cmd_dac,pot_raw,sfm_raw,p1_raw,...,p7_raw
  Raw, untared; a failed read is an empty field. mode: 8 to 12 blocked test
  b1 to b5, -1 idle, -3 returning, -4 moving to the start point (and its dwell),
  -5 held after a pressure limit.

  Travel limits are set inside the Jrk, not here.
*/
#include <Wire.h>
#include <SensirionCore.h>   // Library Manager: "Sensirion Core"
#include "FspTimer.h"

// Actuator Calibration Constants Copy from Calibration Sketch (26 Oct 2026, see
// MechLung_SteadyStateTest for the run record)
const int   JRK_CENTRE = 2050;
const int   JRK_HALF_RANGE = 2001;
const float DAC_CORR_OFFSET = 4.83f;
const float DAC_CORR_GAIN = 4159.28f;
const float POT_RAW_AT_CENTRE = 1168.41f;
const float POT_DAC_COUNTS_PER_ADC = 1.8156f;

const int DAC_MAX = 4095;             // 12-bit DAC full scale

// Rig geometry. The leak figure scales with both: 1 mm of stroke is 0.45 %,
// 0.1 mm of bore 0.15 %.
const float ACTUATOR_STROKE_MM = 221.0f;
const float CYLINDER_DIAMETER_MM = 129.15f;
const float V_TIDAL_ML = 700.0f;      // volume scale only: stroke_amp_dac in # config is this volume

// Blocked test. Leak at speed v: 13.1 mL/s per mm/s (0.79 L/min). Open runs
// leaked about 0.13 to 1.3 L/min, so 0.25 to 2 mm/s brackets them; 4 mm/s
// reaches past them if the pressure allows.
const int BL_N = 5;
const float BL_SPEED_MM_S[BL_N] = {0.25f, 0.5f, 1.0f, 2.0f, 4.0f};   // keys b1 to b5
const unsigned long BL_MOVE_MS = 20000;   // one move: 5 mm at 0.25 mm/s, 80 mm at 4 mm/s
const unsigned long BL_DWELL_MS = 3000;   // hold after each move: pressure back to zero
const float BL_TRIM_FRAC = 0.10f;         // leaktest.py leaves this much of the move off each end
const float BL_P_LIMIT_PA = 450.0f;       // SDP811-500Pa range +-500 Pa
// H and the move to the start point run at the test speed of the last b key,
// never faster than RAMP_RATE_DAC_PER_S (40 DAC counts/s, 2.2 mm/s; used for H
// before any b key). A fast move against the plug builds pressure.
const float RAMP_RATE_DAC_PER_S = 40.0f;

// Timing.
const int DAC_UPDATE_HZ = 1000;       // timer rate: DAC written every 1 ms
const int SAMPLE_HZ = 100;            // sensor sample rate, must divide DAC_UPDATE_HZ
const int TICKS_PER_SAMPLE = DAC_UPDATE_HZ / SAMPLE_HZ;
const uint8_t TIMER_PRIORITY = 4;     // above I2C, USB, serial (12) and millis (8)
const unsigned long LOG_MIN_BAUD = 230400;
const unsigned long TICK_BUDGET_US = 1000000UL / SAMPLE_HZ - 1500;  // Serial time allowed per tick
#define NATIVE_USB 1                  // see MechLung_SteadyStateTest. Upload: double-tap reset first.
#if NATIVE_USB
#define PC SerialUSB
const unsigned long SERIAL_BAUD = 12000000; // nominal USB full speed: only sets the line time budget
#else
#define PC Serial
const unsigned long SERIAL_BAUD = 115200;
#endif

// Smoothing of the move command: two cascaded low-pass stages (64 ms delay at 5 Hz).
const float CMD_FILTER_HZ = 5.0f;
const float RAMP_STEP_DAC = RAMP_RATE_DAC_PER_S / DAC_UPDATE_HZ;   // fastest ramp step

// Flow conversion for the # status line only; leaktest.py converts with the
// session's T and P.
const float AMBIENT_TEMP_C = 20.0f;
const float AMBIENT_PRESSURE_HPA = 1013.0f;
const float FLOW_SIGN = 1.0f;         // + = out of cylinder
const float SLM_TO_LMIN = ((AMBIENT_TEMP_C + 273.15f) / 293.15f) * (1013.0f / AMBIENT_PRESSURE_HPA);
const float SFM3000_OFFSET = 32000.0f;
const float SFM3000_SCALE = 140.0f;
const float SFM3000_MAX_SLM = 200.0f;

// Tare, as MechLung_SteadyStateTest.
const int TARE_SAMPLES = 100;
const unsigned long TARE_SETTLE_MS = 2000;
const float TARE_STILL_COUNTS = 15.0f;
const float TARE_FLOW_MAX_SLM = 1.0f;

// I2C, as MechLung_SteadyStateTest.
const uint8_t SFM3000_ADDR = 0x40;
const uint8_t MUX_ADDR = 0x70;
const uint8_t SDP_ADDR = 0x26;
const uint32_t WIRE_CLOCK_HZ = 100000;
const uint32_t WIRE1_CLOCK_HZ = 400000;
const unsigned int WIRE_TIMEOUT_US = 3000;
const unsigned int WIRE1_TIMEOUT_US = 1000;
const uint8_t WIRE1_SDA = 27;
const uint8_t WIRE1_SCL = 26;
const int SENSOR_FAIL_TICKS = 5;
const int FLOW_AVG_READS = 4;
const unsigned long FLOW_AVG_SPACING_US = 1000000UL / SAMPLE_HZ / FLOW_AVG_READS;
const unsigned long SFM_MIN_GAP_US = 600;
const int SFM_FROZEN_TICKS = 50;

// Modes held in current_preset.
const int MODE_BL = 8;         // first blocked mode: b1 is MODE_BL + 0
const int MODE_IDLE = -1;
const int MODE_RETURNING = -3;
const int MODE_GOTO = -4;      // ramping to the start point, then its dwell
const int MODE_HOLD = -5;      // stopped by the pressure limit until H

float stroke_amp_dac = 0.0f;   // DAC counts for V_TIDAL_ML (volume scale for leaktest.py)
float dac_per_mm = 0.0f;
float ml_per_mm = 0.0f;
float bl_span_dac[BL_N];       // one move
float bl_lo_dac[BL_N];         // start point (min side)
int corrected_centre_dac = 0;
float filter_alpha = 1.0f;

// Shared with the timer ISR. loop() changes these only with interrupts off.
volatile int current_preset = MODE_IDLE;
volatile int pending_mode = MODE_IDLE;   // blocked mode to start after the move to the start point
volatile unsigned long phase_ms = 0;     // time into the cycle (or the start dwell)
volatile uint32_t breath_count = 0;      // +1 at every move start
volatile float cmd_f = 0.0f;
volatile float filt1 = 0.0f, filt2 = 0.0f;
volatile float target_dac = 0.0f;
volatile float ramp_step = RAMP_STEP_DAC;   // DAC counts per update for H and the move to the start point
volatile bool at_target = false;
volatile int isr_event = 0;              // 1 centre, 4 test started
volatile uint32_t snap_tick = 0;
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
bool sfm_discard = false;
long sfm_sum = 0;
int sfm_n = 0;
bool sfm_bad = false;
unsigned long sfm_last_us = 0;
unsigned long sfm_tick_us = 0;
int sfm_slot = FLOW_AVG_READS;
unsigned long sfm_short = 0;
uint16_t sfm_prev_word = 0;
int sfm_same = 0;
int sdp_blank[7];
unsigned long mux_err = 0;
float tare[8];                 // flow in slm, pressures in Pa

// Tare state.
int tare_state = 0;
uint32_t tare_t0 = 0;
float tare_ref = NAN, pot_lo = 0, pot_hi = 0;
float tare_sum[8], tare_n[8];

// Output state.
bool log_mode = false, log_requested = false;
uint16_t sfm_raw = 0;
int16_t sdp_raw[7];
int16_t sdp_scale = 0;
float sum_flow = 0, sum_p = 0, n_flow = 0, n_p = 0;
uint32_t last_tick = 0, last_status = 0;
unsigned long dropped_ticks = 0, skipped_rows = 0, max_loop_us = 0;
char cmd_buf[3];               // collects "b1" to "b5"
int cmd_len = 0;
unsigned long cmd_ms = 0;

// ---- Serial writer: lines are queued, one is sent per sample tick ----
const int LINE_LEN = 560;
const int QUEUE_LEN = 8;
struct Line : public Print {
  char s[LINE_LEN];
  int n;
  size_t write(uint8_t c) { if (n < LINE_LEN) s[n++] = c; return 1; }
};
Line queue_lines[QUEUE_LEN];
Line spare_line;
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

void sendLineIfFits(unsigned long t0) {
  if (q_count == 0) return;
  unsigned long line_us = (queue_lines[q_head].n + 2) * 10000000UL / SERIAL_BAUD;
  if (micros() - t0 + line_us <= TICK_BUDGET_US) sendOneLine();
}

// ---- Actuator ----

int applyDacCorrection(int desired) {
  return constrain((int)(((float)desired - DAC_CORR_OFFSET) * (float)DAC_MAX / DAC_CORR_GAIN + 0.5f), 0, DAC_MAX);
}

void writeCommand(float v) {
  v = constrain(v, 0.0f, (float)DAC_MAX);
  cmd_f = v;
  R_DAC->DADR[0] = (uint16_t)(v + 0.5f);
}

void outputProfile(float x) {
  if (filter_alpha < 1.0f) {
    filt1 += filter_alpha * (x - filt1);
    filt2 += filter_alpha * (filt1 - filt2);
    writeCommand(filt2);
  } else {
    writeCommand(x);
  }
}

int median3(int a, int b, int c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

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

// Volume scale and the blocked moves. DAC counts per mm as MechLung_SteadyStateTest
// (mm -> Jrk counts -> DAC counts).
void initMoves() {
  float r = CYLINDER_DIAMETER_MM / 2.0f;
  ml_per_mm = PI * r * r / 1000.0f;
  dac_per_mm = ((float)JRK_HALF_RANGE / (ACTUATOR_STROKE_MM / 2.0f)) * (float)DAC_MAX / DAC_CORR_GAIN;
  stroke_amp_dac = V_TIDAL_ML / ml_per_mm * dac_per_mm;
  for (int i = 0; i < BL_N; i++) {
    bl_span_dac[i] = BL_SPEED_MM_S[i] * BL_MOVE_MS / 1000.0f * dac_per_mm;
    bl_lo_dac[i] = (float)corrected_centre_dac - bl_span_dac[i] / 2.0f;
  }
}

// One blocked cycle: out move, dwell, in move, dwell. phi_ms into the cycle.
void stepBlocked(unsigned long phi) {
  int i = current_preset - MODE_BL;
  const unsigned long M = BL_MOVE_MS, W = BL_DWELL_MS;
  float pos;
  if (phi < M) pos = (float)phi / M;
  else if (phi < M + W) pos = 1.0f;
  else if (phi < 2 * M + W) pos = 1.0f - (float)(phi - M - W) / M;
  else pos = 0.0f;
  outputProfile(bl_lo_dac[i] + bl_span_dac[i] * pos);
}

void startMode(int m) {
  current_preset = m;
  phase_ms = 0;
  filt1 = filt2 = cmd_f;
  breath_count++;
}

void stepReturn() {
  float diff = (float)corrected_centre_dac - cmd_f;
  if (fabsf(diff) <= ramp_step) {
    writeCommand((float)corrected_centre_dac);
    current_preset = MODE_IDLE;
    isr_event = 1;
  } else {
    writeCommand(cmd_f + (diff > 0.0f ? ramp_step : -ramp_step));
  }
}

// Ramp to the start point, dwell BL_DWELL_MS there, then start the test.
void stepGoto() {
  if (at_target) {
    if (++phase_ms >= BL_DWELL_MS) {
      startMode(pending_mode);
      pending_mode = MODE_IDLE;
      isr_event = 4;
    }
    return;
  }
  float diff = target_dac - cmd_f;
  if (fabsf(diff) <= ramp_step) {
    writeCommand(target_dac);
    at_target = true;
    phase_ms = 0;
  } else {
    writeCommand(cmd_f + (diff > 0.0f ? ramp_step : -ramp_step));
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
  } else if (m >= MODE_BL) {        // a new "breath" at each move start
    stepBlocked(phase_ms);
    if (++phase_ms >= 2 * (BL_MOVE_MS + BL_DWELL_MS)) phase_ms = 0;
    if (phase_ms == 0 || phase_ms == BL_MOVE_MS + BL_DWELL_MS) breath_count++;
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

void requestTare() {
  tare_state = 1;
  tare_ref = NAN;
}

void printList(Line &L, const char *name, const float *v, int n, int dp) {
  L.print(' ');
  L.print(name);
  L.print('=');
  for (int i = 0; i < n; i++) {
    if (i > 0) L.print(',');
    L.print(v[i], dp);
  }
}

// The constants leaktest.py needs to convert the raw log values.
void printConfig() {
  Line &L = newLine();
  L.print("# config centre_dac=");
  L.print(corrected_centre_dac);
  L.print(" stroke_amp_dac=");
  L.print(stroke_amp_dac, 3);
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
  L.print(" ml_per_mm=");
  L.print(ml_per_mm, 4);
  L.print(" dac_per_mm=");
  L.print(dac_per_mm, 4);
  printList(L, "bl_speed_mm_s", BL_SPEED_MM_S, BL_N, 3);
  printList(L, "bl_span_dac", bl_span_dac, BL_N, 2);
  L.print(" bl_move_ms=");
  L.print(BL_MOVE_MS);
  L.print(" bl_dwell_ms=");
  L.print(BL_DWELL_MS);
  L.print(" bl_trim=");
  L.print(BL_TRIM_FRAC, 2);
  L.print(" bl_p_limit_pa=");
  L.print(BL_P_LIMIT_PA, 0);
  L.print(" ramp_dac_per_s=");
  L.print(RAMP_RATE_DAC_PER_S, 1);
}

// Keys b1 to b5 (i = 0 to 4): ramp straight to the start point, dwell, start.
void startBlocked(int i) {
  int m = current_preset;
  if (m == MODE_RETURNING) {
    say("# still returning to centre, wait");
    return;
  }
  if (m == MODE_HOLD) {
    say("# send H first");
    return;
  }
  target_dac = bl_lo_dac[i];
  at_target = false;
  ramp_step = min(RAMP_STEP_DAC, bl_span_dac[i] / BL_MOVE_MS);   // test speed, per 1 ms update
  pending_mode = MODE_BL + i;
  current_preset = MODE_GOTO;
  Line &L = newLine();
  L.print("# blocked test b");
  L.print(i + 1);
  L.print(' ');
  L.print(BL_SPEED_MM_S[i], 2);
  L.print(" mm/s, moving to the start point");
}

void handleKey(char key) {
  int m = current_preset;
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
  if (key != 'h') return;
  if (m == MODE_RETURNING) {
    say("# returning to centre");
  } else if (m == MODE_IDLE) {
    say("# centre");
    requestTare();
  } else {
    pending_mode = MODE_IDLE;
    current_preset = MODE_RETURNING;
    say("# returning to centre");
  }
}

void readKeys() {
  if (cmd_len > 0 && millis() - cmd_ms > 1000) cmd_len = 0;
  while (PC.available()) {
    char key = (char)tolower(PC.read());
    cmd_ms = millis();
    if (key == '\n' || key == '\r') {
      cmd_len = 0;
      continue;
    }
    noInterrupts();
    if (cmd_len > 0 || key == 'b') {
      cmd_buf[cmd_len++] = key;
      if (cmd_len == 2) {
        if (key >= '1' && key < '1' + BL_N) startBlocked(key - '1');
        else say("# unknown command");
        cmd_len = 0;
      }
    } else {
      handleKey(key);
    }
    interrupts();
  }
}

// ---- Sensors (unchanged from MechLung_SteadyStateTest) ----

uint16_t sensirionCommand(TwoWire &bus, uint8_t addr, uint16_t cmd) {
  uint8_t buf[2];
  SensirionI2CTxFrame tx = SensirionI2CTxFrame::createWithUInt16Command(cmd, buf, 2);
  return SensirionI2CCommunication::sendFrame(addr, tx, bus);
}

uint16_t sfmStart() { return sensirionCommand(Wire, SFM3000_ADDR, 0x1000); }

float readFlow() {
  uint8_t buf[3];
  SensirionI2CRxFrame rx(buf, sizeof(buf));
  uint16_t raw;
  if (SensirionI2CCommunication::receiveFrame(SFM3000_ADDR, 3, rx, Wire, CRC31_00) || rx.getUInt16(raw)) return NAN;
  sfm_raw = raw;
  float slm = ((float)raw - SFM3000_OFFSET) / SFM3000_SCALE;
  if (fabsf(slm) > SFM3000_MAX_SLM) return NAN;
  return slm;
}

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
  pinMode(WIRE1_SDA, OUTPUT);
  digitalWrite(WIRE1_SDA, LOW);
  delayMicroseconds(5);
  pinMode(WIRE1_SDA, INPUT);
  delayMicroseconds(5);
  return digitalRead(WIRE1_SDA) == LOW ? -1 : pulses;
}

bool muxWrite(uint8_t mask) {
  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(mask);
  return Wire1.endTransmission() == 0;
}

bool sdpStart() { return sensirionCommand(Wire1, SDP_ADDR, 0x361E) == 0; }

void sdpStop() { sensirionCommand(Wire1, SDP_ADDR, 0x3FF9); }

float readPressure(int ch) {
  uint8_t buf[9];
  SensirionI2CRxFrame rx(buf, sizeof(buf));
  if (SensirionI2CCommunication::receiveFrame(SDP_ADDR, 9, rx, Wire1, CRC31_ff)) {
    while (Wire1.available()) Wire1.read();
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
      sfm_discard = true;
    } else if (muxWrite(1 << (s - 1))) {
      sdpStop();
      delay(1);
      sdpStart();
      sdp_blank[s - 1] = 2;
    }
  }
}

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
  unsigned long since = micros() - sfm_last_us;
  if (FLOW_AVG_READS > 1 && since < SFM_MIN_GAP_US) delayMicroseconds(SFM_MIN_GAP_US - since);
  sfm_last_us = sfm_tick_us = micros();
  raw[0] = readFlow();
  bool ok = !isnan(raw[0]);
  if (ok) {
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
    raw[0] = sfm_bad ? NAN : (mean - SFM3000_OFFSET) / SFM3000_SCALE;
  }
  sfm_sum = 0;
  sfm_n = 0;
  sfm_bad = false;
  sfm_slot = 1;
  sensorResult(0, ok, tick, true);

  bool bus_ok = true;
  for (int ch = 0; ch < 7; ch++) {
    raw[ch + 1] = NAN;
    sfmSample();
    if (bus_ok && !muxWrite(1 << ch)) {
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
  if (bus_ok) muxWrite(0);
}

// ---- Tare (unchanged) ----

void updateTare(float raw[8], int m, uint32_t tick) {
  if (m != MODE_IDLE) {
    tare_state = 0;
    say("# tare aborted, not idle");
    return;
  }
  if (tare_state == 1) {
    if (isnan(tare_ref) || fabsf(feedback_dac - tare_ref) > TARE_STILL_COUNTS) {
      tare_ref = feedback_dac;
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
    for (int i = 0; i < 8; i++) tare[i] = tare_sum[i] / tare_n[i];
    float flow_tare = tare[0];
    bool rejected = !isnan(flow_tare) && fabsf(flow_tare) > TARE_FLOW_MAX_SLM;
    if (rejected) tare[0] = NAN;
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

// ---- Pressure limit ----

// While the piston moves, any tared pressure past BL_P_LIMIT_PA stops the
// command where it is. The pressure then falls through the leak.
void checkPressureLimit(const float p[7], int m) {
  if (!(m >= MODE_BL || m == MODE_GOTO || m == MODE_RETURNING)) return;
  for (int i = 0; i < 7; i++) {
    if (isnan(p[i]) || fabsf(p[i]) <= BL_P_LIMIT_PA) continue;
    noInterrupts();
    current_preset = MODE_HOLD;
    pending_mode = MODE_IDLE;
    interrupts();
    Line &L = newLine();
    L.print("# pressure limit ");
    L.print(SENSOR_NAME[i + 1]);
    L.print('=');
    L.print(p[i], 1);
    L.print(" Pa: holding. Unplug the outlet, then H");
    return;
  }
}

#if NATIVE_USB
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
  initMoves();
  analogWrite(A0, corrected_centre_dac);
  cmd_f = filt1 = filt2 = corrected_centre_dac;
  PC.println("# B4_blocked_leak: b1 b2 b3 b4 b5 H L");

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
    sdpStop();
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
  if (ev == 4) say("# blocked test started");
  if (tick == last_tick) {
    sfmSample();
    return;
  }

  unsigned long t0 = micros();
  if (log_requested && q_count == 0) {
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
  checkPressureLimit(p, m);

  if (log_mode) {
    sendLogRow(tick, breath, m, cmd, raw);
    sendLineIfFits(t0);
    return;
  }

  if (!isnan(flow)) { sum_flow += flow; n_flow++; }
  for (int i = 0; i < 7; i++) {
    if (!isnan(p[i])) { sum_p += p[i]; n_p++; }
  }
  if (tick - last_status >= (uint32_t)SAMPLE_HZ) {
    last_status = tick;
    Line &L = newLine();
    L.print("# status mode=");
    L.print(m);
    L.print(" p_mean_Pa=");
    L.print(n_p > 0 ? sum_p / n_p : NAN, 2);
    L.print(" flow_Lmin=");
    L.print(n_flow > 0 ? sum_flow / n_flow : NAN, 3);
    L.print(" pot_mm=");
    L.print((feedback_dac - corrected_centre_dac) / dac_per_mm, 2);
    L.print(" dropped_ticks=");
    L.print(dropped_ticks);
    L.print(" read_err=");
    for (int i = 0; i < 8; i++) {
      if (i > 0) L.print(',');
      L.print(read_err[i]);
    }
    L.print(" mux_err=");
    L.print(mux_err);
    sum_flow = sum_p = n_flow = n_p = 0;
  }

  sendOneLine();
  unsigned long loop_us = micros() - t0;
  if (loop_us > max_loop_us) max_loop_us = loop_us;
}

/*
  B2_timer_dac.ino  (breakout B2, run before MechLung_LeakTest)

  Disconnect the Jrk input from A0 first, so the test waveform cannot move
  the piston.

  One analogWrite(A0, centre) in setup() opens the DAC. A 1 kHz FspTimer
  interrupt at priority 4 then writes a slow test sine straight to the DAC
  register and toggles pin D3. loop() prints continuously.

  How to run
    Board: Arduino UNO R4 WiFi. Upload. Serial Monitor at SERIAL_BAUD.
    Scope A0 and D3.
  Pass: every setup call prints ok, the priority reads 4, A0 follows a
  1 s sine of +/-500 counts around centre, and D3 toggles every 1 ms
  (2 ms period) while loop() prints.
  If it fails, the plan's fallback applies (DAC write moved to loop()).
*/
#include "FspTimer.h"

const unsigned long SERIAL_BAUD = 115200;  // the baud chosen in B1
const int CENTRE_DAC = 2011;               // applyDacCorrection(JRK_CENTRE) with the current constants
const int TOGGLE_PIN = 3;

FspTimer timer;
volatile unsigned long ticks = 0;

void onTick(timer_callback_args_t *) {
  ticks++;
  R_DAC->DADR[0] = (uint16_t)(CENTRE_DAC + 500.0f * sinf(2.0f * PI * (ticks % 1000) / 1000.0f));
  digitalWrite(TOGGLE_PIN, ticks & 1);
}

void report(const char *step, bool ok) {
  Serial.print("# ");
  Serial.print(step);
  Serial.println(ok ? " ok" : " FAILED");
}

void setup() {
  pinMode(TOGGLE_PIN, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  delay(2000);
  analogWriteResolution(12);
  analogWrite(A0, CENTRE_DAC);

  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  report("get_available_timer", ch >= 0);
  report("begin", timer.begin(TIMER_MODE_PERIODIC, type, ch, 1000.0f, 0.0f, onTick));
  report("setup_overflow_irq(4)", timer.setup_overflow_irq(4));
  report("open", timer.open());
  report("start", timer.start());
  Serial.print("# timer type ");
  Serial.print(type == GPT_TIMER ? "GPT" : "AGT");
  Serial.print(", interrupt priority ");
  Serial.println(NVIC_GetPriority(timer.get_cfg()->cycle_end_irq));
}

void loop() {
  Serial.print("ticks:");
  Serial.println(ticks);
}

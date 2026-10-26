/*
  B1_serial_throughput.ino  (breakout B1, run before MechLung_LeakTest)

  Prints fixed-length rows at 100 Hz, each with a sequence number, and
  toggles pin D2 high for the duration of each print. LINE_BYTES sets the
  length: 68 = a MechLung_LeakTest log row (log mode, the one that matters),
  203 = the longest plot-mode line (# breath).

  How to run, once per baud: 230400, 460800, 921600 (and 115200 for plot mode)
    1. Set SERIAL_BAUD below. Board: Arduino UNO R4 WiFi. Upload.
    2. Scope D2 (optional): the high time is the print time. Each row also
     carries the previous row's print time (us:), which the reader reports.
    3. Close the Arduino Serial Monitor, then run on the PC:
         python b1_reader.py <port> <baud>
       It counts sequence gaps for 10 minutes.
  Pass (LINE_BYTES = 68): no gaps, and print time + 3 to 4 ms sensor burst
  + 1.5 ms for a short event line under 10 ms. The fastest passing baud of
  at least 230400 becomes SERIAL_BAUD in MechLung_LeakTest.ino and BAUD in
  logger/leaktest.py. If none passes, log mode cannot run at 100 Hz.
*/
const unsigned long SERIAL_BAUD = 115200;
const int LINE_BYTES = 68;                 // row length including CR LF (68 log row, 203 # breath)
const unsigned long PERIOD_US = 10000;     // 100 Hz
const int PRINT_PIN = 2;

char row[LINE_BYTES - 1];                  // row text without CR LF
unsigned long seq = 0;
unsigned long next_us;
unsigned long print_us = 0;                // how long the previous row took to send

void setup() {
  pinMode(PRINT_PIN, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  delay(2000);
  memset(row, 'x', sizeof(row));
  next_us = micros();
}

void loop() {
  if ((long)(micros() - next_us) < 0) return;
  next_us += PERIOD_US;
  snprintf(row, 27, "seq:%010lu,us:%07lu,", seq++, print_us);  // fixed 26 characters, rest stays 'x'
  row[26] = 'x';
  digitalWrite(PRINT_PIN, HIGH);
  unsigned long t = micros();
  Serial.write(row, sizeof(row) - 1);
  Serial.println();
  print_us = micros() - t;
  digitalWrite(PRINT_PIN, LOW);
}

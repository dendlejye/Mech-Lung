/*
  B1_serial_throughput.ino  (breakout B1, run before MechLung_LeakTest)

  Prints fixed-length rows the size of the longest MechLung_LeakTest line
  (the # breath line, 203 bytes with CR LF) at 100 Hz, each with a sequence
  number, and toggles pin D2 high for the duration of each print.

  How to run, once per baud: 115200, 230400, 460800, 921600
    1. Set SERIAL_BAUD below. Board: Arduino UNO R4 WiFi. Upload.
    2. Scope D2: the high time is the print time.
    3. Close the Arduino Serial Monitor, then run on the PC:
         python b1_reader.py <port> <baud>
       It counts sequence gaps for 10 minutes.
  Pass: no gaps, and print time + 3 to 4 ms sensor burst under 10 ms.
  The fastest passing baud becomes SERIAL_BAUD in MechLung_LeakTest.ino.
  If none passes, that sketch needs the RAM transmit queue (plan step 8).
*/
const unsigned long SERIAL_BAUD = 115200;
const int LINE_BYTES = 203;                // row length including CR LF
const unsigned long PERIOD_US = 10000;     // 100 Hz
const int PRINT_PIN = 2;

char row[LINE_BYTES - 1];                  // row text without CR LF
unsigned long seq = 0;
unsigned long next_us;

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
  snprintf(row, 16, "seq:%010lu,", seq++);    // fixed 15 characters, rest stays 'x'
  row[15] = 'x';
  digitalWrite(PRINT_PIN, HIGH);
  Serial.write(row, sizeof(row) - 1);
  Serial.println();
  digitalWrite(PRINT_PIN, LOW);
}

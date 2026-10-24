/*
  B3_plotter.ino  (breakout B3, run before MechLung_NoLogger)

  Prints name:value rows at 2.5 rows per second (the PLOT_DIV = 40 rate):
    ramp     counts up by 1 each row
    with_nan same as ramp, but prints nan on every 10th row

  How to run
    Board: Arduino UNO R4 WiFi. Upload. Open the Arduino IDE 2 Serial Plotter
    at 115200 baud and let it run for over a minute.
  Check
    1. Points the window holds: the ramp's span on screen (last value minus
       first value + 1). 50 is expected (Arduino IDE issue 803).
    2. How nan draws on with_nan: a gap, a drop to zero, or skipped.
  If the window is not 50 points, set PLOT_DIV in MechLung_NoLogger.ino to
  1980 / points, rounded up.
*/
unsigned long row = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  delay(400);
  row++;
  Serial.print("ramp:");
  Serial.print(row);
  Serial.print(",with_nan:");
  if (row % 10 == 0) Serial.println(NAN);
  else Serial.println(row);
}

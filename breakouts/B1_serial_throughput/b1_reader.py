"""B1 reader: counts sequence gaps in B1_serial_throughput rows for 10 minutes.

Usage: python b1_reader.py <port> <baud>     (needs: pip install pyserial)
"""
import sys
import time
import serial

port, baud = sys.argv[1], int(sys.argv[2])
DURATION_S = 600

ser = serial.Serial(port, baud, timeout=1)
ser.reset_input_buffer()
ser.readline()                         # drop a partial first line
last = None
rows = gaps = lost = bad = 0
print_us = []
t0 = time.time()
next_report = t0 + 60
while time.time() - t0 < DURATION_S:
    if rows == 0 and bad == 0 and time.time() - t0 > 10:
        print(f"nothing received on {port} in 10 s: wrong port, B1 not uploaded, or the baud differs from the sketch")
        sys.exit(1)
    if time.time() > next_report:
        next_report += 60
        print(f"  {time.time() - t0:.0f} s: {rows} rows, {gaps} gaps, {bad} malformed"
              + (f", print time max {max(print_us) / 1000:.2f} ms" if print_us else ""))
    line = ser.readline().decode("ascii", "replace").strip()
    if not line.startswith("seq:"):
        bad += 1 if line else 0
        continue
    seq = int(line[4:14])
    row_len = len(line) + 2
    if line[15:18] == "us:":
        print_us.append(int(line[18:25]))
    if last is not None and seq != last + 1:
        gaps += 1
        lost += seq - last - 1
    last = seq
    rows += 1
elapsed = time.time() - t0
print(f"baud {baud}: {rows} rows in {elapsed:.0f} s ({rows / elapsed:.1f} rows/s), "
      f"{gaps} gaps, {lost} rows lost, {bad} malformed lines")
if print_us:
    print_us = print_us[1:] or print_us          # the first row carries 0
    print(f"print time per row (Arduino side): mean {sum(print_us) / len(print_us) / 1000:.2f} ms, "
          f"max {max(print_us) / 1000:.2f} ms, theory {row_len * 10 / baud * 1000:.2f} ms")
print("PASS" if gaps == 0 and bad == 0 and rows / elapsed > 99 else "FAIL")

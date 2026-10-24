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
t0 = time.time()
while time.time() - t0 < DURATION_S:
    line = ser.readline().decode("ascii", "replace").strip()
    if not line.startswith("seq:"):
        bad += 1 if line else 0
        continue
    seq = int(line[4:14])
    if last is not None and seq != last + 1:
        gaps += 1
        lost += seq - last - 1
    last = seq
    rows += 1
elapsed = time.time() - t0
print(f"baud {baud}: {rows} rows in {elapsed:.0f} s ({rows / elapsed:.1f} rows/s), "
      f"{gaps} gaps, {lost} rows lost, {bad} malformed lines")
print("PASS" if gaps == 0 and bad == 0 and rows / elapsed > 99 else "FAIL")

import serial
import csv
import time
"""
host_logger.py 

This script communicates with an Rpi Pico 2 over serial connection to
log thermal data for monitoring and regulating the front-end system.

Edit as needed. We'll likely not write to .csv

Configuration:
    - adjust the PORT variable to match host system's serial device.
    - modify PORT to control the polling rate (s). 

"""
PORT   = "/dev/tty.usbmodem2101"    # adjust with "ls /dev/tty.usbmodem*"
BAUD   = 115200
OUTCSV = "pico_pid_log.csv"
PERIOD = 5.0                        # s between requests, at PERIOD=2.0 output tends to trip over itself, sad.

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)                   

with open(OUTCSV, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["host_epoch", "pico_epoch",
                     "temp", "set_c", "drive"])

    while True:
        ser.write(b"REQ\n")
        line = ser.readline().decode().strip()
        if not line:
            continue
        try:
            pico_time, temp, set_c, drive = map(float, line.split(","))
        except ValueError:
            print("bad line:", line)
            continue

        host_time = time.time()
        writer.writerow([f"{host_time:.3f}", pico_time, temp, set_c, drive])
        f.flush()
        print(line)
        time.sleep(PERIOD)


#!/usr/bin/env python3
import serial
import time
import csv

#PORT = "/dev/ttyACM0"  # for running on Rpi5
PORT = "/dev/tty.usbmodem101" # for running on macOS, check port with ls /dev/tty.usbmodem*
BAUD = 115200          # typical MicroPython USB-serial baud (often ignored)
LOGFILE = "SNAPbaseline_data.csv"

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    
    with open(LOGFILE, "w", newline="") as f:
        csv_writer = csv.writer(f)
        
        # csv header, matching output of pico
        header = [
            "local_time_s",  # Pi's local time
            "elapsed_s",     # pico run time
            "pico_time_s",   # optional: the 'time.time()' from Pico
            "raw_adc",
            "vout_volts",
            "current_amps",
            "Rt_ohms",
            "temp_c"
        ]
        csv_writer.writerow(header)

        print("Polling the Pico for thermistor data. Ctrl+C to stop.")
        
        while True:
            # 1. sends the "REQ" command
            ser.write(b"REQ\n")
            
            # 2. read the response line from the Pico
            line = ser.readline().decode().strip()
            
            if not line:
                # if no response, skip. Could add retries, prob not needed.
                continue
            
            # example Pico output:
            #   "1234.56, 34567, 1.65000, 0.000165, 9999.99, 26.532"
            #  (elapsed_s, timestamp, raw_adc, vout, current, rt, temp_c)
            parts = line.split(",")
            if len(parts) != 7:
                print(f"Unexpected response: {line}")
                continue
            
            elapsed_s = parts[0]
            pico_time_s = parts[1]
            raw_adc = parts[2]
            vout_volts = parts[3]
            current_amps = parts[4]
            rt_ohms = parts[5]
            temp_c = parts[6]
            
            local_time_s = time.time()
            
            csv_writer.writerow([
                f"{local_time_s:.2f}",
                elapsed_s,
                pico_time_s,
                raw_adc,
                vout_volts,
                current_amps,
                rt_ohms,
                temp_c
            ])
            f.flush()
            
            print("Pico:", line)
            
            time.sleep(1) # sleeps for 1s...

if __name__ == "__main__":
    main()


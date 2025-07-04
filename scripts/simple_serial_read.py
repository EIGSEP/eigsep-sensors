import serial
import sys
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

if len(sys.argv) < 2 or sys.argv[1] not in ("REQ", "CAL"):
    print("Usage: python send_command.py [REQ|CAL]")
    sys.exit(1)

command = sys.argv[1] + "\n"
is_cal_mode = sys.argv[1] == "CAL"

def try_open_serial():
    while True:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=1)
            print("Connected to serial.")
            return ser
        except serial.SerialException:
            print("Waiting for Pico...")
            time.sleep(1)

while True:
    ser = try_open_serial()
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(2)  # Let Pico boot

        print(f"Sending '{command.strip()}' repeatedly. Press Ctrl+C to stop.")
        while True:
            if not is_cal_mode:
                ser.write(command.encode())
                ser.flush()

            time.sleep(0.1)

            while ser.in_waiting:
                line = ser.readline().decode(errors="ignore").strip()
                print(line)

                if is_cal_mode and "3,3" in line:
                    print("Calibration complete (3,3 detected). Stopping.")
                    ser.close()
                    sys.exit(0)

            if is_cal_mode:
                ser.write(command.encode())
                ser.flush()

            time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Serial error: {e}. Reconnecting...")
        time.sleep(1)  # Give USB stack time to recover
        continue
    except KeyboardInterrupt:
        print("\nExiting.")
        break

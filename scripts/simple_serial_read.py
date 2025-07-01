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

try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(2)  # Give Pico time to boot

        print(f"Sending '{command.strip()}' repeatedly. Press Ctrl+C to stop.")
        while True:
            if not is_cal_mode:
                ser.write(command.encode())
                ser.flush()

            time.sleep(0.1)  # Give Pico time to respond

            while ser.in_waiting:
                line = ser.readline().decode(errors="ignore").strip()
                print(line)

                # Stop CAL loop if line contains "3,3"
                if is_cal_mode and "3,3" in line:
                    print("Calibration complete (3,3 detected). Stopping.")
                    sys.exit(0)

            if is_cal_mode:
                ser.write(command.encode())
                ser.flush()

            time.sleep(0.1)

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nExiting.")

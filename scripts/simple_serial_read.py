import serial
import sys
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

if len(sys.argv) < 2 or sys.argv[1] not in ("REQ", "CAL"):
    print("Usage: python send_command.py [REQ|CAL]")
    sys.exit(1)

command = sys.argv[1] + "\n"

try:
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(2)  # Give Pico time to boot

        print(f"Sending '{command.strip()}' repeatedly. Press Ctrl+C to stop.")
        while True:
            ser.write(command.encode())
            ser.flush()
            time.sleep(0.1)  # Give Pico time to respond

            while ser.in_waiting:
                line = ser.readline()
                print(line.decode(errors="ignore").strip())

            time.sleep(0.1)

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nExiting.")

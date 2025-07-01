import serial
import sys
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

if len(sys.argv) < 2 or sys.argv[1] not in ("REQ", "CAL"):
    print("Usage: python send_command.py [REQ|CAL]")
    sys.exit(1)

command = sys.argv[1] + "\n"
time.sleep(2)  # Let the Pico finish booting

while True:
    try:
        with serial.Serial(PORT, BAUD, timeout=2) as ser:
            time.sleep(1)
            ser.reset_input_buffer()
            ser.write(command.encode())

            print(f"Sent: {command.strip()}")
            time.sleep(0.5)
            print("Response:")
            while True:
                line = ser.readline()
                if not line:
                    break
                print(line.decode().strip())

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        time.sleep(0.5)

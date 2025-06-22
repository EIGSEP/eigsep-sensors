from machine import I2C, Pin
import time
import struct
import sys

# === GRF-250 I2C Configuration ===
I2C_ADDR = 0x66       # GRF-250 default I2C address
CMD_READ  = 44        # Command ID for distance+strength+temp
LOOP_DELAY = 0.1      # seconds between reads

# Initialize I2C bus (Pico: GP1=SCL, GP0=SDA)
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)

# Wait until sensor present
print("Scanning for GRF-250...")
while I2C_ADDR not in i2c.scan():
    time.sleep(0.1)
print("Found GRF-250 at", hex(I2C_ADDR))

# Main loop: raw send of CMD_READ, then parse response
while True:
    line = sys.stdin.readline().strip()
    if line == "REQ":
        try:
            # send single-byte command
            i2c.writeto(I2C_ADDR, bytes([CMD_READ]))
            time.sleep(0.02)
            resp = i2c.readfrom(I2C_ADDR, 12)

            if len(resp) == 12:
                # unpack ints: distance raw (4 bytes), strength (4), temp raw (4)
                dist_raw, strength, temp_raw = struct.unpack('<iii', resp[0:12])
                distance_m = dist_raw / 10.0
                temperature_c = temp_raw / 100.0
                print(f"Distance: {distance_m:.2f} m, Strength: {strength}, Temp: {temperature_c:.2f} °C")
            else:
                # print raw if invalid
                print("Unexpected response:", [hex(b) for b in resp])

        except Exception as e:
            print("I2C error:", e)
    time.sleep(LOOP_DELAY)

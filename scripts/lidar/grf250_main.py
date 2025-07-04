from machine import I2C, Pin
import time
import struct
import sys

# === GRF-250 I2C Configuration ===
I2C_ADDR = 0x66       # GRF-250 default I2C address
CMD_OUTPUT_CONFIG = 27
CMD_READ  = 44        # Command ID for distance+strength+temp
CMD_UPDATE_RATE = 74
MAX_RATE = 50
output_flags = 0b01100010  # bits: 1 (dist raw), 2 (strength), 6 (temp)
LOOP_DELAY = 0.1      # seconds between reads

# Initialize I2C bus (Pico: GP1=SCL, GP0=SDA)
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)

# Wait until sensor present
print("Scanning for GRF-250...")
while I2C_ADDR not in i2c.scan():
    time.sleep(0.1)
print("Found GRF-250 at", hex(I2C_ADDR))

rate_bytes = struct.pack('<I', MAX_RATE)  # little-endian uint32
i2c.writeto(I2C_ADDR, bytes([CMD_UPDATE_RATE]) + rate_bytes)
time.sleep(0.05)

i2c.writeto(I2C_ADDR, bytes([CMD_UPDATE_RATE]))
time.sleep(0.02)
rate_resp = i2c.readfrom(I2C_ADDR, 4)
update_rate = struct.unpack('<I', rate_resp)[0]
#print(f"Configured update rate: {update_rate} Hz")

payload = struct.pack('<I', output_flags)
i2c.writeto(I2C_ADDR, bytes([CMD_OUTPUT_CONFIG]) + payload)

i2c.writeto(I2C_ADDR, bytes([CMD_OUTPUT_CONFIG]))
resp = i2c.readfrom(I2C_ADDR, 4)
output_config = struct.unpack('<I', resp)[0]
#print(f"Distance output config: {output_config:08b}")

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
                dist_raw, strength_dB, temp_raw = struct.unpack('<iii', resp[0:12])
                distance_m = dist_raw / 10.0
                temperature_c = temp_raw / 100.0
                print(f"{distance_m:.2f},{strength_dB},{temperature_c:.2f}")
            else:
                # print raw if invalid
                print("Unexpected response:", [hex(b) for b in resp])

        except Exception as e:
            print("I2C error:", e)
    time.sleep(LOOP_DELAY)

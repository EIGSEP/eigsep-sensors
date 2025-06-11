import time
import busio
import board
import adafruit_mma8451

# === Setup ===
i2c = busio.I2C(board.GP1, board.GP0)

# Wait until I2C is ready and any device is found
print("Waiting for any I2C device...")

while True:
    if i2c.try_lock():
        try:
            devices = i2c.scan()
            if devices:
                print("Found I2C device(s):", [hex(addr) for addr in devices])
                break
            else:
                print("No I2C devices found. Retrying...")
        finally:
            i2c.unlock()
    time.sleep(0.5)

# Initialize sensor (assuming it's the MMA8451)
sensor = adafruit_mma8451.MMA8451(i2c)

# === Main Loop ===
while True:
    try:
        x, y, z = sensor.acceleration
        orientation = sensor.orientation
        print(f"{x:.2f},{y:.2f},{z:.2f},{orientation}")
        time.sleep(1.0)
    except Exception as e:
        print("Sensor read error:", e)
        time.sleep(1.0)
        
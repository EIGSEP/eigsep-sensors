from machine import I2C, Pin
import time

TF_LUNA_ADDR = 0x10  # I2C address (default)

# Initialize I2C on GP0 (SDA), GP1 (SCL)
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100_000)

def read_tfluna():
    try:
        buf = i2c.readfrom(TF_LUNA_ADDR, 9)
        if buf[0] == 0x59 and buf[1] == 0x59:
            distance = buf[2] + (buf[3] << 8)
            strength = buf[4] + (buf[5] << 8)
            temp_raw = buf[6] + (buf[7] << 8)
            temperature = (temp_raw / 8.0) - 256
            return distance, strength, temperature
    except Exception as e:
        print("Error:", e)
    return None, None, None

# === Main loop ===
while True:
    distance, strength, temperature = read_tfluna()
    if distance is not None:
        print(f"{distance},{strength},{temperature:.2f}")
    time.sleep(0.1)

import time
from datetime import datetime
import numpy as np
import eigsep_sensors as eig

imu = eig.IMU_BNO085()
N_READINGS = 10
SLEEP_TIME = 0.1

all_readings = []
header = {
    "times": [],
    "fields": ["q", "a", "la", "g", "m", "grav", "steps", "stab"],
}

cnt = 0
while True:
    try:
        data = imu.read_imu()
    except Exception as e:
        print("Error:", e)
        data = None

    if data is None or "ERR" in data:
        print("No or invalid IMU data")
        time.sleep(SLEEP_TIME)
        continue

    all_readings.append(data)
    header["times"].append(time.time())
    print(f"[{cnt}] {data}")
    cnt += 1

    if cnt == N_READINGS:
        filename = f"imu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"
        np.savez(filename, data=all_readings, header=header)
        print(f"Saved {filename}")
        cnt = 0
        all_readings = []
        header["times"] = []

    time.sleep(SLEEP_TIME)

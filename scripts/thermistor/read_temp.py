"""
Read temperature from a thermistor and save readings to a file.
This script is run on a computer (like Raspberry Pi), connected to a
Raspberry Pi Pico via USB. The script 'main.py' must be running on the
Pico.
"""
from datetime import datetime
import time
import numpy as np

from eigsep_sensors import Thermistor

PORT = "/dev/ttyACM0"
TIMEOUT = 1
N_READINGS = 100  # readings per file
SLEEP_TIME = 60  # seconds between readings
all_readings = np.zeros(N_READINGS)
therm = Thermistor(PORT, timeout=TIMEOUT)

header = {
    "times": [],
    "Vcc": therm.Vcc,
    "R_fixed": therm.R_fixed,
    "max_adc_value": therm.max_adc_value,
    "sh_coeffs": therm.sh_coeffs,
}

cnt = 0
while True:
    try:
        temp = therm.read_temperature()
    except ValueError as e:
        print(f"Error reading temperature: {e}")
        temp = None
    if not temp:
        time.sleep(SLEEP_TIME)
        continue
    all_readings[cnt] = temp
    time = time.time()
    header["times"].append(time)
    if cnt == N_READINGS - 1:
        filename = f"temp_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"
        np.savez(filename, data=all_readings, header=header)
        print(f"Saved {filename}")
        cnt = 0

"""
Read temperature from a thermistor and save readings to a file.
This script is run on a computer (like Raspberry Pi), connected to a
Raspberry Pi Pico via USB. The script 'main.py' must be running on the
Pico.
"""

from datetime import datetime
import logging
import numpy as np
import time

from eigsep_sensors import Thermistor

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

logger.debug("Initializing thermistor class")
PORT = "/dev/ttyACM0"
TIMEOUT = 1
N_READINGS = 10  # readings per file
SLEEP_TIME = 1  # seconds between readings
all_readings = np.zeros(N_READINGS)
therm = Thermistor(PORT, timeout=TIMEOUT)

logger.info("Thermistor initialized")
logger.debug(
    f"Thermistor Vcc: {therm.Vcc}, R_fixed: {therm.R_fixed}, max_adc_value: {therm.max_adc_value}"
)

header = {
    "times": [],
    "Vcc": therm.Vcc,
    "R_fixed": therm.R_fixed,
    "max_adc_value": therm.max_adc_value,
    "sh_coeffs": therm.sh_coeff,
}

cnt = 0
logging.info("Starting temperature readings")
while True:
    try:
        temp = therm.read_temperature()
    except ValueError as e:
        err_msg = f"Error reading temperature: {e}"
        logger.error(err_msg)
        temp = None
    if temp is None:
        logger.debug("No temperature reading, waiting.")
        time.sleep(SLEEP_TIME)
        continue
    all_readings[cnt] = temp
    current_time = time.time()
    header["times"].append(current_time)
    logger.debug(
        f"Reading {cnt}: {temp} °C at {datetime.fromtimestamp(current_time)}"
    )
    if cnt == N_READINGS - 1:
        filename = f"temp_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"
        np.savez(filename, data=all_readings, header=header)
        logger.info(f"Saved {filename}")
        cnt = 0
    cnt += 1
    time.sleep(SLEEP_TIME)

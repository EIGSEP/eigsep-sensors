from machine import Pin, ADC
import time
import math

# --- Thermistor and ADC parameters ---
ADC_PIN = 26  # GP26 (ADC0)
ADC_VOLTAGE = 3.3  # Pico's ADC reference voltage 
SERIES_RESISTOR = 10_000  # Ohms, fixed resistor value in voltage divider

# --- Your Steinhart-Hart Coefficients (update these!) ---
A = 1.046316779e-03
B = 2.491609615e-04
C = 1.643686261e-08
# A = 1.072453312e-03
# B = 2.442333673e-04
# C = 3.7260e-08

adc = ADC(0)  # ADC(0) corresponds to GP26

def read_temp_celsius():
    raw = adc.read_u16()
    voltage = (raw / 65535) * ADC_VOLTAGE
    resistance = SERIES_RESISTOR * voltage / (ADC_VOLTAGE - voltage)

    if resistance <= 0:
        return float('nan')  # invalid reading

    ln_r = math.log(resistance)
    inv_T = A + B * ln_r + C * (ln_r ** 3)
    temp_k = 1.0 / inv_T
    temp_c = temp_k - 273.15
    return temp_c

while True:
    temp = read_temp_celsius()
    print(f"Temperature: {temp:.2f} °C")
    time.sleep(1)  # s

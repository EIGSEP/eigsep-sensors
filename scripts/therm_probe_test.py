from machine import Pin, ADC
import time
import math

# --- Thermistor and ADC parameters ---
ADC_PIN = 26  # GP26 (ADC0)
ADC_VOLTAGE = 3.3  # Pico's ADC reference voltage 
SERIES_RESISTOR = 10_000  # Ohms, fixed resistor value in voltage divider

# --- Your Steinhart-Hart Coefficients (update these!) ---
#A = 1.046316779e-03
#B = 2.491609615e-04
#C = 1.643686261e-08
A = 1.028671831e-03
B = 2.392041087e-04
C = 1.563817562e-07

# adc = ADC(0)
adc = ADC(Pin(26))

def steinhart_temp(r_ohms):
   ln_r = math.log(r_ohms) 
   inv_t = A + B * ln_r + C * (ln_r)**3
   temp_k = 1.0 / inv_t
   return (1.0 / inv_t) - 273.15 # ˚C

def read_temp_celsius():
   raw = adc.read_u16()
   voltage = (raw / 65535) * ADC_VOLTAGE # v_out
   denom = (ADC_VOLTAGE - voltage)
   resistance = SERIES_RESISTOR * (voltage / denom)
   print(f"Raw: {raw}, Voltage: {voltage:.3f} V, Resistance: {resistance:.1f} ohm ")
   if resistance <= 0:
       return float('nan')  # invalid reading

   return steinhart_temp(resistance)

# B = 3950  # Typical value
# T0 = 298.15  # Reference temp = 25°C in Kelvin
# R0 = 10000  # Resistance at 25°C

# def read_temp_celsius_bmodel():
#     raw = adc.read_u16()
#     voltage = (raw / 65535) * ADC_VOLTAGE
#     resistance = SERIES_RESISTOR * voltage / (ADC_VOLTAGE - voltage)
# # INVERT the formula if the thermistor is on the low side:
# #    resistance = SERIES_RESISTOR * (ADC_VOLTAGE - voltage) / voltage
#     print(f"Resistance: {resistance:.1f} Ω")

#     temp_k = 1 / (1/T0 + (1/B) * math.log(resistance / R0))
#     temp_c = temp_k - 273.15
#     return temp_c

while True:
   temp = read_temp_celsius()
    # temp = read_temp_celsius_bmodel()
   print(f"Temperature: {temp:.2f} °C")
   time.sleep(1)  # s


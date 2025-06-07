import sys
import time
import math
from machine import Pin, ADC

"""
Simple script runs on Rpi Pico.
Upon powering on, this script runs on the Pico then the Pico awaits a request from an Rpi (running rpi_poll.py).
"""
# ----- Steinhart–Hart Coefficients -----

A = 1.046316779e-03
B = 2.491609615e-04
C = 1.643686261e-08

R_FIXED = 10000       # 10 kΩ fixed resistor
VCC = 3.3             # V
adc = ADC(Pin(26))    # Using ADC on gpio pin 26

def measure_thermistor():
    """
    Reads the ADC and returns a tuple:
      (raw_adc, vout, current, rt, temp_c)
    for exactly one measurement.
    """
    # 1. Raw ADC to voltage
    raw_value = adc.read_u16()
    vout = (raw_value / 65535.0) * VCC

    # 2. current through the top resistor
    current = (VCC - vout) / R_FIXED  # amps

    # 3. thermistor resistance: top resistor is R_FIXED, bottom is thermistor
    #    R_therm = R_FIXED * (vout / (VCC - vout))
    denom = (VCC - vout)
    if abs(denom) < 1e-9:
        rt = float('inf')
    else:
        rt = R_FIXED * (vout / denom)

    # 4. converting resistance to temperature (°C) via Steinhart–Hart
    #    1/T(K) = A + B * ln(R) + C * [ln(R)]^3
    if rt <= 0:
        temp_c = float('nan')
    else:
        ln_r = math.log(rt)
        inv_t = A + B * ln_r + C * (ln_r ** 3)  # 1 / T(K)
        temp_k = 1.0 / inv_t
        temp_c = temp_k - 273.15

    return (raw_value, vout, current, rt, temp_c)

def main():
    """
    Waits for 'REQ' commands over USB/serial.
    When received, measure thermistor then print one CSV line.
    """
    print("Pico is ready. Send 'REQ' to get a measurement.")
    start_time = None # initializing but not starting yet...
    
    while True:

        line = sys.stdin.readline().strip()
        
        if not line:
            continue  # ignoring blank lines

        if line == "REQ":
            if start_time is None:
                start_time = time.time()	
            
            raw_adc, vout, current, rt, temp_c = measure_thermistor()
           
            timestamp = time.time() # seconds
            elapsed_s = timestamp - start_time
            response = f"{elapsed_s:.6f},{timestamp:.2f},{raw_adc},{vout:.5f},{current:.9f},{rt:.2f},{temp_c:.3f}"
             
            print(response)

        elif line == "END":
#            end_time = time.time()
#            elapsed = end_time - start_time
            print("Ending request loop.")
#            print(f"Runtime: {elapsed.2f} seconds")
            break
        
        # else: ignore unknown commands

# ----- Run main loop -----
if __name__ == "__main__":
    main()


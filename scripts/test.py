import sys
import time
import math
from machine import Pin, ADC

# ----- Steinhart–Hart Coefficients -----
# Example "Curve B" (0–50 °C) from a typical 10k thermistor table.
# If you have better or more accurate coefficients, put them here.
A = 1.046316779e-03
B = 2.491609615e-04
C = 1.643686261e-08

# ----- Configuration Constants -----
R_FIXED = 10000       # 10 kΩ fixed resistor
VCC = 3.3
adc = ADC(Pin(26))    # Use ADC on GP26

def measure_thermistor():
    """
    Reads the ADC and returns a tuple:
      (raw_adc, vout, current, rt, temp_c)
    for exactly one measurement. Now uses
    Steinhart–Hart for temperature calculation.
    """
    # 1) Raw ADC → voltage
    raw_value = adc.read_u16()
    vout = (raw_value / 65535.0) * VCC

    # 2) Current through the top resistor
    current = (VCC - vout) / R_FIXED  # in amps

    # 3) Thermistor resistance: top resistor is R_FIXED, bottom is thermistor
    #    R_therm = R_FIXED * (vout / (VCC - vout))
    denom = (VCC - vout)
    if abs(denom) < 1e-9:
        rt = float('inf')
    else:
        rt = R_FIXED * (vout / denom)

    # 4) Convert resistance → temperature (°C) via Steinhart–Hart
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
    Wait for 'REQ' commands over USB/serial.
    When received, measure thermistor, print one CSV line.
    """
    print("Pico is ready. Send 'REQ' to get a measurement.")
    start_time = None # init but don't start yet
    
    while True:

        # Read a line from stdin (blocks until something arrives)
        line = sys.stdin.readline().strip()
        
        if not line:
            continue  # blank line, ignore

        if line == "REQ":
            if start_time is None:
                start_time = time.time()	
            # Perform one measurement
            raw_adc, vout, current, rt, temp_c = measure_thermistor()
           
            timestamp = time.time() # now
            elapsed_s = timestamp - start_time
            response = f"{elapsed_s:.6f},{timestamp:.2f},{raw_adc},{vout:.5f},{current:.9f},{rt:.2f},{temp_c:.3f}"
            
            # Print back to the Pi or any terminal 
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



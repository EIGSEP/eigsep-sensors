"""
Simple script runs on Rpi Pico. The variable `adc_pins' defines the
pins used for reading thermistors. This must be a list.

Upon powering on, this script runs on the Pico then the Pico awaits a
request from an Rpi (running read_temp.py).
"""

import sys
from machine import Pin, ADC

adc_pins = [26]  # single thermistor
# adc_pins = [26, 27, 28]  # all three thermistors
adcs = {pin: ADC(Pin(pin)) for pin in adc_pins}


def main():
    """
    Waits for 'REQ' commands over USB/serial.
    When received, measure thermistor then print one CSV line.
    """
    while True:
        line = sys.stdin.readline().strip()
        if line == "REQ":
            out = ",".join(
                f"{pin}:{adc.read_u16()}" for pin, adc in adcs.items()
            )
            print(out)


# ----- Run main loop -----
if __name__ == "__main__":
    main()

"""
Simple script runs on Rpi Pico.
Upon powering on, this script runs on the Pico then the Pico awaits a
request from an Rpi (running rpi_poll.py).
"""

import sys
from machine import Pin, ADC


def main():
    """
    Waits for 'REQ' commands over USB/serial.
    When received, measure thermistor then print one CSV line.
    """
    adc = ADC(Pin(26))  # Using ADC on gpio pin 26
    while True:
        line = sys.stdin.readline().strip()
        if line == "REQ":
            raw_adc = adc.read_u16()  # read ADC value
            print(raw_adc)  # print raw ADC value


# ----- Run main loop -----
if __name__ == "__main__":
    main()

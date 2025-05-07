"""
config.py contains constants and tunable perameters.
"""

# --- pico 2 pinouts to H-bridge -----
PWM_PIN = 16  # GP 16 to ENA 
CURR_FWD = 18 # GP 18 to IN1
CURR_REV = 19 # GP 19 to IN2
ADC_CHAN = 0  # GP 26 to thermistor | may relocate...

# --- TEC & thermistor constants ---
V_SUPP = 12.0  # TEC supply voltage input into H-bridge
I_MAX = 3.5    # Amps, revisit after testing
ADC_V = 3.3    # V, pico adc reference
NTC_R = 10000 # 10k-ohm fixed resistor | may change
NTC_BETA = 3950 # NTC beta coeff, seems standard? 

# Steinhart-Hart coeffs
A = 
B = 
C = 

# PID control params
Kp = 
Ki = 
Kd = 
TARGET_TEMP  = 30.0 # C˚, default temp
"""
config.py contains constants and tunable parameters.
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
NTC_R = 10_000 # 10k-ohm fixed resistor | may change
NTC_BETA = 3950 # NTC beta coeff, seems standard? 

# Steinhart-Hart coeffs, can either calibrate the thermistor or use prev-calibration form pico/therm test
A = 1.046316779e-03
B = 2.491609615e-04
C = 1.643686261e-08

# PID control params, setting values based off simulations
Kp = 1.0
Ki = 0.0
Kd = 0.75
MIN_OUT = -100.0
MAX_OUT = 100.0
TARGET_TEMP  = 30.0 # C˚, default temp
TEMP_MAX = 35.0 # can change
SAMPLE_PER = 1 # s
LOG_EVERY_N = 5 # logs temp, duty cycle, and mode (cool or hot) every 5th iteration

# ---- experimental ----
MAX_STEP = 5 # (%) largest duty change allowed per control loop
DIR_HYST = 0.5 # (˚C)
MAX_DUTY = 60 # (%)  max ceiling for current
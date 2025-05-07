"""
General overview for control loop:
---------------------------------
1. read temperature from thermistor to pico2, rather therm. is connected to pico2 adc
2. compare to target setpoint
3. compute error, difference between current temp and target
4. PID to calc 
    • direction of the current (IN1 or IN2 on H-bridge)
    • pwm duty cycle to control power to the peltier
5. send control signal to H-bridge:
    • switch between heating and cooling, i.e. reverse current (heating) when required
    • modulate current with pwm to avoid overcorrection ? 

Thus there are 3 requirements (subject to change);
    • temp polling loop, 
    • PID control logic
    • H-bridge control logic with direction of current [1, 0] for cooling, [0, 1] for heating and PWM cycle

Notes:
------
H-bridge (XY-160D) ENA1 takes maximum of 10 KHz PWM

"""
import sys
import time
import math 
from machine import Pin, ADC
# import config

# ----- thermistor 
class TemperatureSensor: 
    """
    base class for any temp source.
    Return: float
    """
    def read_celsius(self): # float
        raise NotImplementedError # given read_celsius has not been implemented yet

class ThermistorSensor(TemperatureSensor):
    """
    adc_channel: GPIO pin for thermistor to ADC on pico.
    """
    def __init__(self, adc_channel):
        self.adc = ADC(adc_channel)

    def read_celsius(self):
        """
        reads raw adc value from pico, converts to output voltage. Temps are 
        determined via Steinhart-Hart eq.
        """
        raw_adc = adc.read_u16
        vout = (raw_adc / 65535.0) * ADC_V # V
        r_therm = NTC_R * (vout / (ADC_V - vout)) # ohmer-simpson
        inv_T = A + B*math.log(r_therm) + C * (math.log(r_therm)**3) # 1 / T Steinhart-Hart eq.
        temp_K = 1 / inv_T
        return temp_K - 273.15 # C

# pwm and H-bridge 
class rainbowBridge:
    """
    NEEDS: POWER, CURRENT DIRECTION, INIT PINS
    ------------------------------------------
    You will ride eternal, shiny, and chrome!
    XY-160D and [wm control.
    Maximum PWM freq. H-bridge can handle on 1-channel is 10 KHz
    """
    def __init__(self, pwm_pin, in1_pin, in2_pin, pwm_freq=8_000):
        self.pwm = PWM_PIN(Pin(pwm_pin))
        self.pwm.freq(pwm_freq) # pwm pin and freq 8KHz default
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin>OUT)
        # self.disable() # but do i need it?

    def _apply_power(self, duty_pct):
        '''duty_pct from 0 - 100'''
        d16 = int(duty_pct * 65535 / 100) # 16-bit, scales integer to bit range
        self.pwm.duty_u16(d16)
    
    def _apply_dir(self, cool): # boolean
        """determines direction of current flow"""
        self.in1.value(1 if cool else 0) # if cool is True, in1 is high, else low -- sets direction to cooling
        self.in2.value(0 if cool else 1)

    def disable(self):
        """safety first - effectively turns off power and stops current flow"""
        self._apply_power(0)
        self.in1.low()
        self.in2.low()

        
# PID
class PIDcontroller:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd
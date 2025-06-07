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
import sys # prob don't need
import math
from time import sleep, time
from machine import Pin, ADC, PWM
import config as cfg

# ----- thermistor 
class TemperatureSensor: 
    """
    base class for any temp source.
    Return: float
    """
    def read_celsius(self): # float
        raise NotImplementedError 

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
        raw_adc = self.adc.read_u16()
        vout = (raw_adc / 65535.0) * cfg.ADC_V # V, pico is 12-bit adc but micropython scales to 16-bit
        r_therm = cfg.NTC_R * (vout / (cfg.ADC_V - vout)) # ohm
        inv_T = cfg.A + cfg.B*math.log(r_therm) + cfg.C * (math.log(r_therm)**3) # 1 / T Steinhart-Hart eq.
        temp_K = 1 / inv_T
        return temp_K - 273.15 # C

# pwm and H-bridge 
class rainbowBridge:
    """
    XY-160D and pwm control.
    Maximum PWM freq. H-bridge can handle on 1-channel is 10 KHz
    """
    def __init__(self, pwm_pin, in1_pin, in2_pin, pwm_freq=8_000):
        self.pwm = PWM(Pin(pwm_pin)) 
        self.pwm.freq(pwm_freq)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self._last_dir = None # remember direction to prevenet shoot-through
        self._last_duty = 0   
        self.disable()        

    def _apply_power(self, duty_pct):
        '''duty_pct from 0 - 100'''
        duty_pct = min(max(duty_pct, 0), 100) # given if duty_pct > 100 the cast wraps at 65535 
        d16 = int(duty_pct * 65535 / 100) # 16-bit (because micropython scales to 16bit), scales integer to bit range
        self.pwm.duty_u16(d16)
    
    def _apply_dir(self, cool): 
        """determines direction of current flow"""
        self.in1.value(1 if cool else 0) # if cool is True, in1 is high, else low -- sets direction of current
        self.in2.value(0 if cool else 1) 
        self._last_dir = cool

    def drive(self, duty_pct, cool=True):
        """Applies PWM and direction
        • if direction changes while non-zero duty, ramp down to zero first to avoid shoot-through currents... 
        """
        # self._apply_dir(cool)
        # self._apply_power(duty_pct)

        #----------- test -----
        # Here we ramp down if direction flip is requested...
        if self._last_dir is not None and cool != self._last_dir and duty_pct > 0:
            self._apply_power(0)
            sleep(0.2) # sleep for 200ms

        # setting direction then power... 
        if cool != self._last_dir:
            self._apply_dir(cool)

        self._apply_power(duty_pct)
        # ---------------
        
        # ---- adding hysterisis ---

        # # clamping abs ceiling
        # duty_pct = min(duty_pct, cfg.MAX_DUTY)      

        # # rate-limit, caps how fast a command is allowed to change, example PWM duty_pct can only rise or fall by 5% per control loop. Intended to help prevent current spike
        # delta = duty_pct - self._last_duty
        # if   delta >  cfg.MAX_STEP: 
        #     duty_pct = self._last_duty + cfg.MAX_STEP
        # elif delta < -cfg.MAX_STEP: 
        #     duty_pct = self._last_duty - cfg.MAX_STEP

        # # ---  Safe direction flip  -----------------------
        # if self._last_dir is not None and cool != self._last_dir and duty_pct > 0:
        #     self._apply_power(0)        # ramp to zero first
        #     sleep(200)                  # 0.2 ms dead-time
        #     self._apply_dir(cool)
        # elif cool != self._last_dir:
        #     self._apply_dir(cool)       # no dead-time needed when duty already zero

        # # --- Apply new power level -------------------------
        # self._apply_power(duty_pct)
        self._last_dir  = cool
        self._last_duty = duty_pct
        
    def disable(self):
        """safety first - effectively turns off power and stops current flow"""
        self._apply_power(0)
        self.in1.low() # 0 
        self.in2.low() # 0
        self._last_dir  = None      
        # self._last_duty = 0     # uncomment for testing hysterisis
        
# PID
class PIDcontroller:
    """
    Parameters:
    ------------
    • kp, ki, kd : PID control gains
    • out_min, out_max : % duty range 
    """
    def __init__(self, kp, ki, kd, out_min=-100, out_max=100): # changed out_min = 0 to = -100 to allow for cooling...
        self.kp = kp 
        self.ki = ki 
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._integral = 0
        self._prev_err = 0

    def set_gains(self, kp=None, ki=None, kd=None):
        """adjustable gains for fine-tuning..."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def reset(self):
        self._integral = 0
        self._prev_err = 0
    
    def _clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    # def compute(self, setpoint, measured, dt):
    #     # output = kp * err + (ki * integral[err * dt]) + (kd * der[err / dt])
    #     err = setpoint - measured
    #     self._integral += err * dt
    #     deriv = (err - self._prev_err) / dt
    #     output = (self.kp * err + self.ki * self._integral + self.kd * deriv)
    #     self._prev_err = err
    #     return self._clamp(output, self.out_min, self.out_max) 

    def compute(self, setpoint, measured, dt):
        """Return a signed control value in out_min … out_max.

        Anti-wind-up (i.e. preventing thermal runaway):
            • integral term stops integrating while the output is saturated
            • integral is clamped so it can never drive output past the limits
        """
        # --- error terms -------
        err   = setpoint - measured
        deriv = (err - self._prev_err) / dt

        # --- integral with clamping -----
        self._integral += err * dt # wind-up happens when every sample err*dt is being added and never unwound...

        # limiting the integral term so kp*err + ki*integral cannot exceed bounds
        int_max = (self.out_max - self.kp * err - self.kd * deriv) / self.ki if self.ki else 0
        int_min = (self.out_min - self.kp * err - self.kd * deriv) / self.ki if self.ki else 0
        self._integral = self._clamp(self._integral, int_min, int_max)

        output = (self.kp * err +
                self.ki * self._integral +
                self.kd * deriv)

        # if still saturating, unwind integral to avoid wind-up
        if abs(output) >= self.out_max * 0.99:
            self._integral -= err * dt   # back out last addition

        self._prev_err = err
        return self._clamp(output, self.out_min, self.out_max)

class ControlManager:
    def __init__(self, cfg):
        self.sensor = ThermistorSensor(cfg.ADC_CHAN)
        self.pid = PIDcontroller(cfg.Kp, cfg.Ki, cfg.Kd, cfg.MIN_OUT, cfg.MAX_OUT) # added min/max_out to set duty range
        self.driver = rainbowBridge(cfg.PWM_PIN, cfg.CURR_FWD, cfg.CURR_REV)
        self.setpoint_c = cfg.TARGET_TEMP
        self.sample_dt = cfg.SAMPLE_PER
        self.loop_cnt = 0

    def safety_check(self, temp_c):
        if temp_c > cfg.TEMP_MAX:
            self.driver.disable()
            raise RuntimeError("Over-temp, shutting down.") 
        
    # def step(self):
    #     t_c = self.sensor.read_celsius()
    #     self.safety_check(t_c)
    #     duty = self.pid.compute(self.setpoint_c, t_c, self.sample_dt)
    #     cool = (t_c > self.setpoint_c)
    #     self.driver.drive(duty, cool)

    #     if self.loop_cnt % cfg.LOG_EVERY_N == 0:
    #         # log(f"T={t_c:.2f} °C, duty={duty:.1f} %, mode={'cool' if cool else 'heat'}")

    #         # ------ for testing -----
    #         log_line = f"{time():.1f},{t_c:.2f},{duty:.1f},{'cool' if cool else 'heat'}"
    #         print(log_line)
    #     self.loop_cnt += 1

    def step(self):
        t_c   = self.sensor.read_celsius()
        self.safety_check(t_c)
        err = self.setpoint_c - t_c

        # added tight dead-band, helps mitigate integral oversaturation (which was leading to thermal runaway, i think...)
        if abs(err) < 0.2:        # changed from 0.05, freeze band 
            self.driver.disable()
            self.pid.reset()      
            return

        # pid returns (-100, 100) (or whatever we set the duty limits to) though we take the abs.value of that for the PWM
        pwm_out = self.pid.compute(self.setpoint_c, t_c, self.sample_dt) # duty %
        cool  = pwm_out < 0        # sign = direction
        duty  = abs(pwm_out)       # magnitude for the PWM (0-100)

        if duty < 0.5:             
            self.driver.disable()
            self.pid.reset()       # anti-wind-up on OFF
        else:
            self.driver.drive(duty, cool)

        if self.loop_cnt % cfg.LOG_EVERY_N == 0:
            log_line = f"{time():.1f},{t_c:.2f},{pwm_out:+.1f},{'cool' if cool else 'heat'}"
            print(log_line)

        self.loop_cnt += 1

    def run_forrest(self):
        try:
            while True:
                self.step()
                sleep(self.sample_dt)
        except KeyboardInterrupt:
            print("okay stop running...")
            self.driver.disable() 

# ---- for testing ----
if __name__ == "__main__":
    manager = ControlManager(cfg)
    manager.run_forrest()

# --- for testing on Rpi Pico 2 although it sucks at multi-tasking so maybe not. --- 
# ampy --port /dev/ttyACM0 run pid_control.py > log.csv for plotting/fine-tuning gain params
#             /dev/tty.usbmodem2101 on mac

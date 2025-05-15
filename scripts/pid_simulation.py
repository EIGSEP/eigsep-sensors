# feed a step input to PID and check it settles within tolerance
import config
import matplotlib.pyplot as plt
# from pid_control import PIDcontroller # when testing on pico

"""
Feed a step input to PID and check it settles within acceptable tolerance.
"""
class PIDcontroller:
    """
    Parameters:
    -----------
    • kp, ki, kd : PID param control
    • out_min, out_max : % duty range
    """
    def __init__(self, kp, ki, kd, out_min=0, out_max=100):
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

    def compute(self, setpoint, measured, dt):
        # output = kp * err + (ki * integral[err * dt]) + (kd * der[err / dt])
        err = setpoint - measured
        self._integral += err * dt
        deriv = (err - self._prev_err) / dt
        output = (self.kp * err + self.ki * self._integral + self.kd * deriv)
        self._prev_err = err
        return self._clamp(output, self.out_min, self.out_max) 

# ------ simulates heating only ------
# pid = PIDcontroller(kp=3.0, ki=0.5, kd=8, out_min=0, out_max=100)
# temp = 25.0
# setpoint = 30.0
# for i in range(200):
#     duty = pid.compute(setpoint, temp, dt=0.2)
#     # crude plant: temp rises 0.02 °C per % duty
#     temp += duty * 0.02 * 0.2
#     if i % 10 == 0:
#         print(f"Step {i}: Temp = {temp:.2f}, Duty = {duty:.1f}")
# print(f"Final temp: {temp:.2f} °C")
# print(f"Error: {abs(temp - setpoint):.2f} °C")
# assert abs(temp - setpoint) < 0.2

# ----- "realistic" simulation -----

# --- sim params ----
setpoint = 30
temp = 25.0 # start above or below setpoint
ambient_temp = 32.0
dt = 0.2 # time step
gain = 0.02 # temp rise per % duty per second
cooling_rate = 0.005  # passive loss to environment, prob can do without - July be hot in Utah
steps = 600

pid = PIDcontroller(kp=3.75, ki=0.5, kd=20, out_min=-60, out_max=60)

temps = []
duties = []
times = []

for i in range(steps):
    error = setpoint - temp
    duty = pid.compute(setpoint, temp, dt)
    cool = (temp > setpoint)  # cooling mode if temp is too high

    if i % 20 == 0:
        print(f"Step {i}: Temp = {temp:.2f} °C, Duty = {duty:.1f}%, Mode = {'cool' if cool else 'heat'}")

    # if cool:
    #     temp -= duty * gain * dt
    # else:
    #     temp += duty * gain * dt

    temp += duty * gain * dt  

    # Passive heat exchange with ambient
    # temp -= (temp - ambient_temp) * cooling_rate * dt # for "realistic" ambient contribution...

    temps.append(temp)
    duties.append(duty)
    times.append(i * dt / 60) # mins

plt.figure(figsize=(12,6))
plt.plot(times, temps, label="Temperature (°C)")
plt.plot(times, [setpoint]*len(times), "--", label="Setpoint")
plt.plot(times, duties, label="Duty (%)", alpha=0.6)
plt.xlabel("Time (min)")
plt.ylabel("Value")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
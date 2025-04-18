import board
import adafruit_mma8451
from time import sleep

class Accelerometer:
    def __init__(self):
        self.i2c = board.I2C()
        sleep(0.5)
        self.sensor = adafruit_mma8451.MMA8451(self.i2c)
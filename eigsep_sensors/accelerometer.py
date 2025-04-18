import board
import adafruit_mma8451
from time import sleep

class Accelerometer:
    """
    A class to interface with the MMA8451 accelerometer sensor over I2C.

    This class initializes the I2C connection and the MMA8451 sensor. The sensor is 
    used to measure acceleration along the X, Y, and Z axes.

    Attributes:
        i2c (board.I2C): The I2C bus interface to communicate with the sensor.
        sensor (adafruit_mma8451.MMA8451): The MMA8451 accelerometer sensor object.

    Methods:
        __init__: Initializes the I2C connection and sensor object.
    """
    def __init__(self):
        """
        Initializes the I2C connection and MMA8451 accelerometer sensor.
        The sensor is ready to measure acceleration along the X, Y, and Z axes.
        """
        self.i2c = board.I2C()
        sleep(0.5)
        self.sensor = adafruit_mma8451.MMA8451(self.i2c)
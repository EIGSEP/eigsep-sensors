from abc import ABC, abstractmethod
import serial
from time import sleep
import board
import adafruit_mma8451

class IMU(ABC):

    def __init__(self):
        pass

    def _request_data(self):
        self.ser.write(b"REQ\n")
    
    def read(self):
        """Send REQ and return parsed IMU data as dict"""
        self._request_data()
        response = self.ser.readline().decode().strip()
        if not response:
            return None
        return self._parse_line(response)

class IMU_MMA8451(IMU):
    def __init__(self):
        super().__init__()
        """
        Initializes the I2C connection and MMA8451 accelerometer sensor.
        The sensor is ready to measure acceleration along the X, Y, and Z axes.
        """
        self.i2c = board.I2C()
        sleep(0.5)
        self.sensor = adafruit_mma8451.MMA8451(self.i2c)

class IMU_BNO085(IMU):
    def __init__(self, port="/dev/ttyACM0", timeout=1):
        super().__init__()
        self.ser = serial.Serial(port=port, baudrate=115200, timeout=timeout)

    def _parse_line(self, line):
        parts = line.strip().split(",")
        data = {}
        for part in parts:
            try:
                key, *values = part.split(":")
                if key in ["q", "a", "la", "g", "m", "grav"]:
                    data[key] = [float(v) for v in values]
                elif key == "steps":
                    data[key] = int(values[0])
                elif key == "stab":
                    data[key] = values[0]
            except Exception as e:
                data["ERR"] = str(e)
        return data

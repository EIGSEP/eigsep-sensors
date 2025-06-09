import serial
from time import sleep
import board
import adafruit_mma8451

class IMU:
    """
    Base class for IMU (Inertial Measurement Unit) sensors.

    Provides a generic interface for requesting and reading data from IMUs
    that communicate over a serial connection. Child classes should implement
    the `_parse_line` method if they use serial-based communication.
    """

    def _request_data(self):
        """
        Send a 'REQ' signal over serial to request sensor data.
        """
        self.ser.write(b"REQ\n")

    def read(self):
        """
        Request and read data from the IMU via serial.

        Returns:
            dict or None: Parsed data as a dictionary if successful,
                          otherwise None if no data is returned.
        """
        self._request_data()
        response = self.ser.readline().decode().strip()
        if not response:
            return None
        return self._parse_line(response)

class IMU_MMA8451(IMU):
    """
    IMU class for the Adafruit MMA8451 accelerometer sensor over I2C.

    Inherits from IMU, but uses I2C instead of serial. This class initializes
    the I2C bus and sets up the sensor for acceleration measurements.
    """

    def __init__(self):
        """
        Initialize the MMA8451 sensor over I2C.
        """
        super().__init__()
        self.i2c = board.I2C()
        sleep(0.5)
        self.sensor = adafruit_mma8451.MMA8451(self.i2c)

class IMU_BNO085(IMU):
    """
    IMU class for the BNO085 sensor over serial.

    Uses the base IMU class methods to send read requests and parse the
    sensor's response. Handles distance, orientation, and other motion data.
    """

    def __init__(self, port="/dev/ttyACM0", timeout=1):
        """
        Initialize the BNO085 sensor over a serial connection.

        Args:
            port (str): Serial port to which the sensor is connected.
            timeout (int or float): Read timeout in seconds.
        """
        super().__init__()
        self.ser = serial.Serial(port=port, baudrate=115200, timeout=timeout)

    def _parse_line(self, line):
        """
        Parse a comma-separated line from the sensor into structured data.

        Args:
            line (str): Raw string returned by the sensor.

        Returns:
            dict: Parsed data with keys like 'q', 'a', 'steps', etc.
        """
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

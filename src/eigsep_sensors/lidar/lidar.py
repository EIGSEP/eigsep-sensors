from abc import ABC, abstractmethod
import numpy as np
import smbus2
import serial
import time

class Lidar(ABC):
    """
    Base class for LiDAR sensors.

    Provides a generic interface for requesting and reading data from LiDARs.
    Subclasses must implement `_request_data` and `_parse_line` for communication and parsing.
    """

    def __init__(self, port, timeout=1):
        """
        Set up the serial connection and initialize the Lidar.

        Parameters
        ----------
        port : str
            The serial port to which the Lidar is connected.
        timeout : float
            The timeout for the serial connection in seconds.

        """
        self.ser = serial.Serial(port=port, baudrate=115200, timeout=timeout)

    def _request_data(self):
        """
        Send a data request to the LiDAR sensor.
        Should be implemented in subclasses depending on protocol (e.g., I2C or serial).
        """
        self.ser.write(b"REQ\n")

    def read(self):
        """
        Request and read data from the LiDAR.

        Returns:
            dict or None: Parsed data as a dictionary if successful,
                          otherwise None on failure.
        """
        self._request_data()
        response = self._read_raw()
        if not response:
            return None
        return self._parse_line(response)
    
    @abstractmethod
    def _parse_line(self, response):
        """
        Fill in subclass
        """
        pass

    def update_orientation(self, roll=None, pitch=None, yaw=None):
        """
        Update the IMU orientation used for transforming LiDAR readings.

        Parameters
        ----------
        roll : float or None
            Roll angle in degrees (rotation around X-axis).
        pitch : float or None
            Pitch angle in degrees (rotation around Y-axis).
        yaw : float or None
            Yaw angle in degrees (rotation around Z-axis).
        """
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class Lidar_TFLuna(Lidar):
    """
    LiDAR class for the TFLuna sensor using I2C.

    Reads distance, signal strength, and temperature.
    """

    def _parse_line(self, raw):
        """
        Convert (distance, strength, temperature) to structured dict.
        Adds Cartesian coordinates using yaw/pitch if available.

        Args:
            raw (tuple): (distance_cm, signal_strength, temperature_c)

        Returns:
            dict: Structured output with optional XY(Z) coordinates.
        """
        if raw is None:
            return None

        distance, strength, temperature = raw
        result = {
            "timestamp": time.time(),
            "distance_cm": distance,
            "signal_strength": strength,
            "temperature_c": temperature
        }

        # Store angles if available
        if self.yaw is not None:
            result["yaw_deg"] = self.yaw
        if self.pitch is not None:
            result["pitch_deg"] = self.pitch
        if self.roll is not None:
            result["roll_deg"] = self.roll

        # Calculate coordinates if orientation is provided
        if self.yaw is not None:
            yaw_rad = np.radians(self.yaw)

            if self.pitch is not None:
                pitch_rad = np.radians(self.pitch)
                # Full 3D coordinates
                x = distance * np.cos(pitch_rad) * np.cos(yaw_rad)
                y = distance * np.cos(pitch_rad) * np.sin(yaw_rad)
                z = distance * np.sin(pitch_rad)
                result.update({"x_cm": x, "y_cm": y, "z_cm": z})
            else:
                # 2D polar projection
                x = distance * np.cos(yaw_rad)
                y = distance * np.sin(yaw_rad)
                result.update({"x_cm": x, "y_cm": y})

        return result

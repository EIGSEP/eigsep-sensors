import serial
import time

class Lidar():
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

    def _parse_line(self, line):
        """
        Parse a comma-separated line from the Pico into structured LiDAR data.

        Args:
            line (str): String like '127,56,24.38' from the Pico

        Returns:
            dict or None: Parsed dictionary with distance, signal, and temperature.
        """
        data = {}
        try:
            parts = line.strip().split(",")
            if len(parts) != 3:
                return None
            data["distance"] = float(parts[0])
            data["strength"] = float(parts[1])
            data["temperature"] = float(parts[2])
            data["unix_time"] = time.time()
        except Exception as e:
            print("[GRF-250] Parse error:", e)
        return data

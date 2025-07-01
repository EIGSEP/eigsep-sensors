import serial
import time


class Lidar:
    """
    Base class for LiDAR sensors.

    Provides a generic interface for requesting and reading data from LiDARs.
    Subclasses must implement `_request_data` and `_parse_line` for communication and parsing.
    """

    def __init__(self, port, timeout=1, baudrate=115200, validate_data=True):
        """
        Set up the serial connection and initialize the Lidar.

        Parameters
        ----------
        port : str
            The serial port to which the Lidar is connected.
        timeout : float
            The timeout for the serial connection in seconds.
        baudrate : int
            The baudrate for serial communication (default: 115200).
        validate_data : bool
            Whether to validate parsed data ranges (default: True).

        Raises
        ------
        ValueError
            If port is empty or timeout/baudrate are invalid.
        serial.SerialException
            If serial connection cannot be established.
        """
        if not port or not isinstance(port, str):
            raise ValueError("Port must be a non-empty string")
        if timeout <= 0:
            raise ValueError("Timeout must be positive")
        if baudrate <= 0:
            raise ValueError("Baudrate must be positive")

        self.port = port
        self.timeout = timeout
        self.baudrate = baudrate
        self.validate_data = validate_data

        try:
            self.ser = serial.Serial(
                port=port, baudrate=baudrate, timeout=timeout
            )
        except serial.SerialException as e:
            raise serial.SerialException(f"Failed to connect to {port}: {e}")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()

    def close(self):
        """Close the serial connection."""
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()

    def _request_data(self):
        """
        Send a data request to the LiDAR sensor.
        Should be implemented in subclasses depending on protocol (e.g., I2C or serial).
        """
        self.ser.write(b"REQ\n")

    def _read_raw(self):
        """
        Read raw response from the serial connection.

        Returns:
            str or None: Raw response string if successful, None on failure.
        """
        try:
            return (
                self.ser.readline().decode("utf-8", errors="replace").strip()
            )
        except (serial.SerialException, UnicodeDecodeError) as e:
            print(f"[LIDAR] Read error: {e}")
            return None

    def read(self):
        """
        Request and read data from the LiDAR.

        Returns:
            dict or None: Parsed data as a dictionary if successful,
                          otherwise None on failure.
        """
        try:
            self._request_data()
            response = self._read_raw()
            if not response:
                return None
            return self._parse_line(response)
        except Exception as e:
            print(f"[LIDAR] Read operation failed: {e}")
            return None

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

            distance = float(parts[0])
            strength = float(parts[1])
            temperature = float(parts[2])

            # Validate data ranges if enabled
            if self.validate_data:
                if distance < 0 or distance > 1000:  # Reasonable max range
                    print(
                        f"[LIDAR] Warning: Distance {distance}m outside expected range"
                    )
                if (
                    temperature < -40 or temperature > 85
                ):  # Typical sensor range
                    print(
                        f"[LIDAR] Warning: Temperature {temperature}°C outside expected range"
                    )
                if strength < 0:  # Strength should be non-negative
                    print(
                        f"[LIDAR] Warning: Negative strength value {strength}"
                    )

            data["distance"] = distance
            data["strength"] = strength
            data["temperature"] = temperature
            data["unix_time"] = time.time()
        except Exception as e:
            print(f"[LIDAR] Parse error: {e}")
        return data

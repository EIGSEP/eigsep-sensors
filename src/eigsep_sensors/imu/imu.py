from abc import ABC, abstractmethod
import serial
import time
import numpy as np
import board
import adafruit_mma8451


class IMU(ABC):
    """
    Base class for IMU (Inertial Measurement Unit) sensors.

    Provides a generic interface for requesting and reading data from IMUs
    that communicate over a serial connection. Child classes should implement
    the `_parse_line` method if they use serial-based communication.
    """

    def __init__(self, port, timeout=1):
        """
        Initialize the IMU over a serial connection.

        Args:
            port (str): Serial port to which the sensor is connected.
            timeout (int or float): Read timeout in seconds.
        """
        if port is None:
            return
        else:
            self.ser = serial.Serial(
                port=port, baudrate=115200, timeout=timeout
            )

    def _request_imu(self):
        """
        Send a 'REQ' signal over serial to request sensor data.
        """
        self.ser.write(b"REQ\n")

    def read_imu(self):
        """
        Request and read data from the IMU via serial.

        Returns:
            dict or None: Parsed data as a dictionary if successful,
                          otherwise None if no data is returned.
        """
        self._request_imu()
        response = self.ser.readline().decode().strip()
        if not response:
            return None
        return self._parse_line(response)

    @abstractmethod
    def _parse_line(self, line):
        """
        Filled in subclasses.

        Args:
            line (str): Raw string returned by the sensor.

        Returns:
            dict: Parsed data with keys specific to the sensor.

        """

    def close(self):
        """
        Closes serial connection.
        """
        self.ser.close()

    @staticmethod
    def get_pitch_roll_from_gravity(gx, gy, gz):
        """
        Compute pitch and roll from the gravity vector.

        Args:
            gx, gy, gz (float): Gravity components

        Returns:
            tuple: (pitch_deg, roll_deg)
        """
        pitch = np.degrees(np.arcsin(-gx))
        roll = np.degrees(np.arctan2(gy, gz))
        return pitch, roll

    @staticmethod
    def angle_with_vertical(gz):
        """
        Compute angle from vertical using the Z-component of a unit
        gravity vector.

        Args:
            gz (float): Z-component of a normalized vector

        Returns:
            float: Angle from vertical in degrees
        """
        angle_rad = np.arccos(np.clip(gz, -1.0, 1.0))
        return np.degrees(angle_rad)


class IMU_MMA8451(IMU):
    """
    IMU class for the Adafruit MMA8451 accelerometer sensor over I2C.

    Inherits from IMU, but uses I2C instead of serial. This class initializes
    the I2C bus and sets up the sensor for acceleration measurements.

    Additional Arg:
    sensor (boolean): Optional arg if sensor is directly connected to Pi instead of to Pico.
    """

    def __init__(self, port, timeout=1, sensor=False):
        super().__init__(port=port, timeout=timeout)
        if sensor:
            i2c = board.I2C()
            self.sensor = adafruit_mma8451.MMA8451(i2c)

    def _parse_line(self, line):
        """
        Parse a comma-separated line from the sensor into structured data.

        Args:
            line (str): Raw string returned by the sensor.

        Returns:
            dict: Parsed data with keys x, y, z, orientation, and unix_time.

        Raises:
            ValueError: If the line does not contain exactly 4 parts.

        """
        parts = line.strip().split(",")
        if len(parts) != 4:
            raise ValueError(
                "Expected 4 parts in the line: x, y, z, orientation"
            )
        return {
            "x": float(parts[0]),
            "y": float(parts[1]),
            "z": float(parts[2]),
            "orientation": int(parts[3]),
            "unix_time": time.time(),
        }

    @staticmethod
    def calculate_orientation(x, y, z):
        """
        Calculate the orientation angles (theta and phi) based on 3D
        acceleration data.

        This function computes the azimuthal angle (theta) and polar
        angle (phi) from the given x, y, and z acceleration components
        using the arctangent function.

        Args:
            x (float): The acceleration along the X axis.
            y (float): The acceleration along the Y axis.
            z (float): The acceleration along the Z axis.

        Returns:
            tuple: A tuple containing the orientation angles:
                - theta (float): The azimuthal angle in degrees.
                - phi (float): The polar angle in degrees.
        """
        theta = np.arctan2(y, x)
        phi = np.arctan2(np.sqrt(x**2 + y**2), z)
        theta_deg = np.degrees(theta)
        phi_deg = np.degrees(phi)
        return theta_deg, phi_deg

    @staticmethod
    def get_orientation_unit_vector(x, y, z):
        """
        Normalize the gravity vector (x, y, z) from the accelerometer
        to get orientation.

        Args:
            x, y, z (float): Raw accelerometer readings.

        Returns:
            tuple: Unit vector showing device orientation with respect
            to gravity.

        Raises:
            ValueError: If the magnitude of the gravity vector is zero.

        """
        g_mag = np.sqrt(x**2 + y**2 + z**2)
        if g_mag == 0:
            raise ValueError("Zero-magnitude gravity vector.")

        return x / g_mag, y / g_mag, z / g_mag


class IMU_BNO085(IMU):
    """
    IMU class for the BNO085 sensor over serial.

    Uses the base IMU class methods to send read requests and parse the
    sensor's response. Handles distance, orientation, and other motion data.
    """

    def _parse_line(self, line):
        """
        Parse a comma-separated IMU serial line into structured components.

        Args:
            line (str): Raw serial line from Pico.

        Returns:
            dict: Data. Keys are
                'q': quaternion,
                'a': acceleration,
                'la': linear_acceleration,
                'g': gyro,
                'm': mag,
                'grav': gravity,
                'steps': steps,
                'stab': stab
            If the line cannot be parsed, the key will not be present.

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
                print(f"[IMU_BNO085] Parse error for {part}: {e}")
        data["unix_time"] = time.time()

        return data

    def _request_imu(self, cal=False):
        """
        Send a 'REQ' or 'CAL' signal over serial to request sensor data.

        Args:
            cal (bool): If True, sends 'CAL' command to calibrate the sensor.
                        If False, sends 'REQ' to request data.

        """
        if cal:
            self.ser.write(b"CAL\n")
        else:
            super()._request_imu()

    @staticmethod
    def quaternion_to_euler(q):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw).

        Args:
            q (list): Quaternion [x, y, z, w]

        Returns:
            tuple: (roll, pitch, yaw) in degrees
        """
        x, y, z, w = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1, 1))

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    @staticmethod
    def normalize_vector(v):
        """
        Normalize a 3D vector.

        Args:
            v (list): Vector [x, y, z]

        Returns:
            np.ndarray: Normalized vector [x_unit, y_unit, z_unit]
        """
        mag = np.linalg.norm(v)
        return np.array(v) / mag

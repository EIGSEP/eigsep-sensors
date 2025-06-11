from abc import ABC, abstractmethod
import serial
import time
import numpy as np

class IMU(ABC):
    """
    Base class for IMU (Inertial Measurement Unit) sensors.

    Provides a generic interface for requesting and reading data from IMUs
    that communicate over a serial connection. Child classes should implement
    the `_parse_line` method if they use serial-based communication.
    """

    def __init__(self, port, timeout=1):
        """
        Initialize the BNO085 sensor over a serial connection.

        Args:
            port (str): Serial port to which the sensor is connected.
            timeout (int or float): Read timeout in seconds.
        """
        self.ser = serial.Serial(port=port, baudrate=115200, timeout=timeout)

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
        """
        pass

    def close(self):
        """
        Closes serial connection.
        """
        self.ser.close()

    
class IMU_MMA8451(IMU):
    """
    IMU class for the Adafruit MMA8451 accelerometer sensor over I2C.

    Inherits from IMU, but uses I2C instead of serial. This class initializes
    the I2C bus and sets up the sensor for acceleration measurements.
    """

    def _parse_line(self, line):
        """
        Parse a comma-separated line from the sensor into structured data.

        Args:
            line (str): Raw string returned by the sensor.

        Returns:
            dict: Parsed data with keys like x, y, z, etc.
        """
        try:
            parts = line.strip().split(",")
            if len(parts) != 4:
                return None
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            orientation = int(parts[3])
            return x, y, z, orientation, time.time()
        except Exception as e:
            print("[MMA8451] Parse error:", e)
            return None
        
    def calculate_orientation(self, x, y, z):
        """
        Calculate the orientation angles (theta and phi) based on 3D acceleration data.

        This function computes the azimuthal angle (theta) and polar angle (phi)
        from the given x, y, and z acceleration components using the arctangent function.

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
    
    def get_orientation_unit_vector(self, x, y, z):
        """
        Normalize the gravity vector (x, y, z) from the accelerometer to get orientation.

        Args:
            x, y, z (float): Raw accelerometer readings.

        Returns:
            dict: Unit vector showing device orientation with respect to gravity.
        """
        g_mag = np.sqrt(x**2 + y**2 + z**2)
        if g_mag == 0:
            raise ValueError("Zero-magnitude gravity vector.")

        return x/g_mag, y/g_mag, z/g_mag
    
    def get_pitch_roll_from_unit_vector(self, gx, gy, gz):
        """
        Calculate pitch and roll angles (in degrees) from a normalized gravity vector.

        Args:
            gx, gy, gz (float): Components of the unit gravity vector.

        Returns:
            tuple: (pitch, roll)
        """
        pitch = np.arcsin(-gx) * (180.0 / np.pi)  # tilt forward/backward
        roll = np.arctan2(gy, gz) * (180.0 / np.pi)  # tilt left/right
        return pitch, roll
    
    def angle_with_vertical(self, gz):
        """
        Compute the angle between the gravity unit vector and the Z axis.

        Args:
            gz: Z component of unit gravity vector.

        Returns:
            float: Angle in degrees between gravity and Z-axis (i.e., device tilt).
        """
        dot_product = gz  # since Z-axis unit vector is (0, 0, 1)
        angle_rad = np.arccos(dot_product)
        return np.degrees(angle_rad)

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
            tuple: (quaternion, accel, lin_accel, gyro, mag, gravity, steps, stab)
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
                print(f"[IMU_BNO085] Parse error: {e}")
                return None, None, None, None, None, None, None, None  # Fail-safe fallback

        # Unpack in expected order
        return (
            data.get("q"),     # quaternion
            data.get("a"),     # accel
            data.get("la"),    # linear accel
            data.get("g"),     # gyro
            data.get("m"),     # magnetometer
            data.get("grav"),  # gravity
            data.get("steps"), # step count
            data.get("stab")   # stability
        )

    def quaternion_to_euler(self, q):
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

    def normalize_vector(self, v):
        """
        Normalize a 3D vector.

        Args:
            v (list): Vector [x, y, z]

        Returns:
            list: Normalized vector [x_unit, y_unit, z_unit]
        """
        mag = np.linalg.norm(v)
        if mag == 0:
            raise ValueError("Zero-magnitude vector")
        return [x / mag for x in v]

    def get_pitch_roll_from_gravity(self, gx, gy, gz):
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

    def angle_with_vertical(self, gz):
        """
        Compute angle from vertical using the Z-component of a unit gravity vector.

        Args:
            gz (float): Z-component of a normalized vector

        Returns:
            float: Angle from vertical in degrees
        """
        angle_rad = np.arccos(np.clip(gz, -1.0, 1.0))
        return np.degrees(angle_rad)

    def vector_magnitude(self, v):
        """
        Compute the magnitude of a 3D vector.

        Args:
            v (list): Vector [x, y, z]

        Returns:
            float: Magnitude
        """
        return np.linalg.norm(v)

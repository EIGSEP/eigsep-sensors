from abc import ABC, abstractmethod
import smbus2
import time

class Lidar(ABC):
    """
    Abstract base class for LiDAR devices.

    Subclasses must implement methods to read distance, signal strength,
    and temperature data, and optionally provide cleanup via `close()`.
    """

    @abstractmethod
    def read_all(self):
        """
        Read distance, signal strength, and temperature from the LiDAR sensor.

        Returns:
            tuple: A 3-element tuple containing:
                - distance_cm (int or None): Measured distance in centimeters.
                - signal_strength (int or None): Signal strength of the measurement.
                - temperature_celsius (float or None): Internal temperature in degrees Celsius.
        """
        pass

    @abstractmethod
    def close(self):
        """
        Close the LiDAR device connection, if applicable.

        Typically used to release I2C or serial resources.
        """
        pass

class Lidar_TFLuna(Lidar):
    """
    Driver for the TFLuna LiDAR sensor over I2C.

    Attributes:
        i2c_address (int): I2C address of the sensor.
        bus (smbus2.SMBus): I2C bus instance for communication.
    """

    def __init__(self, i2c_bus=1, i2c_address=0x10, bus=None):
        """
        Initialize the TFLuna sensor on the specified I2C bus and address.

        Args:
            i2c_bus (int): I2C bus number (default: 1).
            i2c_address (int): I2C address of the TFLuna sensor (default: 0x10).
            bus (smbus2.SMBus or None): Optional existing SMBus instance.
        """
        self.i2c_address = i2c_address
        self.bus = bus if bus is not None else smbus2.SMBus(i2c_bus)

    def read_word(self, reg_low, reg_high):
        """
        Read a 16-bit word from two consecutive 8-bit registers.

        Args:
            reg_low (int): Address of the lower byte register.
            reg_high (int): Address of the higher byte register.

        Returns:
            int: Combined 16-bit value.
        """
        low = self.bus.read_byte_data(self.i2c_address, reg_low)
        high = self.bus.read_byte_data(self.i2c_address, reg_high)
        return (high << 8) + low

    def read_all(self):
        """
        Read distance, signal strength, and temperature from the sensor.

        Returns:
            tuple: (distance_cm, signal_strength, temperature_celsius),
                   or (None, None, None) on error.
        """
        try:
            distance = self.read_word(0x00, 0x01)
            strength = self.read_word(0x02, 0x03)
            temp_raw = self.read_word(0x04, 0x05)
            temperature = temp_raw * 0.01
            return distance, strength, temperature
        except Exception as e:
            print(f"[TFLuna] Read error: {e}")
            return None, None, None

    def close(self):
        """
        Close the I2C connection to the TFLuna sensor.
        """
        self.bus.close()

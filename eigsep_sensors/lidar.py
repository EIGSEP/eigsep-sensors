from abc import ABC, abstractmethod
import smbus2
import time

class Lidar(ABC):
    @abstractmethod
    def read_all(self):
        """
        Read distance, strength, and temperature.
        Returns:
            tuple: (distance_cm, signal_strength, temperature_celsius)
        """
        pass

    def close(self):
        """Optional: Close any open connections."""
        pass

class TFLuna(Lidar):
    def __init__(self, i2c_bus=1, i2c_address=0x10):
        self.i2c_address = i2c_address
        self.bus = smbus2.SMBus(i2c_bus)

    def read_word(self, reg_low, reg_high):
        """Reads a 16-bit value from two 8-bit registers."""
        low = self.bus.read_byte_data(self.i2c_address, reg_low)
        high = self.bus.read_byte_data(self.i2c_address, reg_high)
        return (high << 8) + low

    def read_all(self):
        """Reads distance (cm), signal strength, and temperature (°C)."""
        try:
            distance = self.read_word(0x00, 0x01)
            strength = self.read_word(0x02, 0x03)
            temp_raw = self.read_word(0x04, 0x05)
            temperature = temp_raw * 0.01  # unit is 0.01°C
            return distance, strength, temperature
        except Exception as e:
            print(f"[TFLuna] Read error: {e}")
            return None, None, None

    def close(self):
        self.bus.close()

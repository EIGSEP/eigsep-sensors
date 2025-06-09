import pytest
from unittest.mock import MagicMock
import eigsep_sensors as eig

class TestLidar:
    def make_fake_bus(self):
        fake_bus = MagicMock()
        fake_bus.read_byte_data.side_effect = [
            0x34, 0x12,  # distance = 0x1234 = 4660
            0x78, 0x56,  # strength = 0x5678 = 22136
            0x10, 0x27   # temp_raw = 0x2710 = 100.00°C
        ]
        return fake_bus

    def test_read_all_returns_expected_values(self):
        fake_bus = self.make_fake_bus()
        lidar = eig.TFLuna(bus=fake_bus)

        distance, strength, temperature = lidar.read_all()

        assert distance == 0x1234
        assert strength == 0x5678
        assert temperature == 100.00

    def test_read_all_handles_exception(self):
        fake_bus = MagicMock()
        fake_bus.read_byte_data.side_effect = IOError("I2C failed")
        lidar = eig.TFLuna(bus=fake_bus)

        distance, strength, temperature = lidar.read_all()

        assert distance is None
        assert strength is None
        assert temperature is None

    def test_close_calls_bus_close(self):
        fake_bus = MagicMock()
        lidar = eig.TFLuna(bus=fake_bus)

        lidar.close()
        fake_bus.close.assert_called_once()
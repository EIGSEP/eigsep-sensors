import pytest
from unittest.mock import MagicMock, patch
import eigsep_sensors as eig

class TestLidar:
    @patch("eigsep_sensors.smbus2.SMBus")
    def test_read_all_returns_expected_values(self, mock_smbus):
        mock_bus = MagicMock()
        mock_bus.read_byte_data.side_effect = [
            0x34, 0x12,  # distance = 0x1234
            0x78, 0x56,  # strength = 0x5678
            0x10, 0x27   # temp_raw = 0x2710 = 100.00 °C
        ]
        mock_smbus.return_value = mock_bus

        lidar = eig.TFLuna()
        distance, strength, temp = lidar.read_all()

        assert distance == 0x1234
        assert strength == 0x5678
        assert temp == 100.00

    @patch("eigsep_sensors.smbus2.SMBus")
    def test_read_all_handles_exception(self, mock_smbus):
        mock_bus = MagicMock()
        mock_bus.read_byte_data.side_effect = IOError("I2C fail")
        mock_smbus.return_value = mock_bus

        lidar = eig.TFLuna()
        distance, strength, temp = lidar.read_all()

        assert distance is None
        assert strength is None
        assert temp is None

    @patch("eigsep_sensors.smbus2.SMBus")
    def test_close_calls_bus_close(self, mock_smbus):
        mock_bus = MagicMock()
        mock_smbus.return_value = mock_bus

        lidar = eig.TFLuna()
        lidar.close()

        mock_bus.close.assert_called_once()

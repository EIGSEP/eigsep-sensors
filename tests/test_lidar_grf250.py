import pytest
import serial
from unittest.mock import Mock, patch
from eigsep_sensors.lidar.lidar import Lidar


class TestGRF250Integration:
    """Integration tests specific to GRF-250 LIDAR sensor behavior."""
    
    @patch('serial.Serial')
    @patch('time.time', return_value=1625097600.0)
    def test_grf250_data_format(self, mock_time, mock_serial):
        """Test parsing GRF-250 specific data format."""
        mock_ser = Mock()
        # GRF-250 returns distance in mm/10, temperature in Celsius*100
        mock_ser.readline.return_value = b"12.30,856,24.50\n"
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        result = lidar.read()
        
        expected = {
            'distance': 12.30,  # meters
            'strength': 856.0,  # dB
            'temperature': 24.50,  # Celsius
            'unix_time': 1625097600.0
        }
        assert result == expected

    @patch('serial.Serial')
    def test_grf250_error_response(self, mock_serial):
        """Test handling of GRF-250 error responses."""
        mock_ser = Mock()
        mock_ser.readline.return_value = b"Unexpected response: ['0xff', '0xfe']\n"
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        result = lidar._parse_line("Unexpected response: ['0xff', '0xfe']")
        
        # Should return None on invalid format (not CSV)
        assert result is None

    @patch('serial.Serial')
    def test_grf250_i2c_error_response(self, mock_serial):
        """Test handling of I2C error from GRF-250."""
        mock_ser = Mock()
        mock_ser.readline.return_value = b"I2C error: [Errno 121] Remote I/O error\n"
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        result = lidar._parse_line("I2C error: [Errno 121] Remote I/O error")
        
        # Should return None on invalid format (not CSV)
        assert result is None

    @patch('serial.Serial')
    def test_grf250_range_limits(self, mock_serial):
        """Test GRF-250 range limits (0.05m - 250m)."""
        mock_serial.return_value = Mock()
        
        lidar = Lidar('/dev/ttyACM0')
        
        # Test minimum range
        with patch('time.time', return_value=1625097600.0):
            result_min = lidar._parse_line("0.05,856,24.5")
        assert result_min['distance'] == 0.05
        
        # Test maximum range
        with patch('time.time', return_value=1625097600.0):
            result_max = lidar._parse_line("250.00,856,24.5")
        assert result_max['distance'] == 250.00

    @patch('serial.Serial')
    def test_grf250_high_precision_distance(self, mock_serial):
        """Test GRF-250 high precision distance measurements."""
        mock_serial.return_value = Mock()
        
        lidar = Lidar('/dev/ttyACM0')
        
        with patch('time.time', return_value=1625097600.0):
            result = lidar._parse_line("123.456,856,24.5")
        
        assert result['distance'] == 123.456
        assert isinstance(result['distance'], float)

    @patch('serial.Serial')
    def test_grf250_negative_temperature(self, mock_serial):
        """Test GRF-250 handling negative temperatures."""
        mock_serial.return_value = Mock()
        
        lidar = Lidar('/dev/ttyACM0')
        
        with patch('time.time', return_value=1625097600.0):
            result = lidar._parse_line("10.50,856,-15.25")
        
        assert result['temperature'] == -15.25
        assert isinstance(result['temperature'], float)

    @patch('serial.Serial')
    def test_grf250_zero_values(self, mock_serial):
        """Test GRF-250 handling zero values."""
        mock_serial.return_value = Mock()
        
        lidar = Lidar('/dev/ttyACM0')
        
        with patch('time.time', return_value=1625097600.0):
            result = lidar._parse_line("0.00,0,0.00")
        
        expected = {
            'distance': 0.0,
            'strength': 0.0,
            'temperature': 0.0,
            'unix_time': 1625097600.0
        }
        assert result == expected

    @patch('serial.Serial')
    def test_grf250_continuous_reading_simulation(self, mock_serial):
        """Simulate continuous reading from GRF-250."""
        mock_ser = Mock()
        # Simulate sequence of readings
        readings = [
            b"10.25,856,24.5\n",
            b"10.30,857,24.6\n", 
            b"10.28,855,24.5\n"
        ]
        mock_ser.readline.side_effect = readings
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        
        results = []
        for _ in range(3):
            result = lidar.read()
            if result:
                results.append(result['distance'])
        
        expected_distances = [10.25, 10.30, 10.28]
        assert len(results) == 3
        assert results == expected_distances
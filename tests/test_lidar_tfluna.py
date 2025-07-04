import pytest
import serial
from unittest.mock import Mock, patch, MagicMock
from eigsep_sensors.lidar.lidar import Lidar


class TestLidar:
    """Test suite for the base Lidar class."""
    
    @patch('serial.Serial')
    def test_init_success(self, mock_serial):
        """Test successful initialization."""
        mock_ser = Mock()
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0', timeout=2)
        
        mock_serial.assert_called_once_with(port='/dev/ttyACM0', baudrate=115200, timeout=2)
        assert lidar.ser == mock_ser

    @patch('serial.Serial')
    def test_init_default_timeout(self, mock_serial):
        """Test initialization with default timeout."""
        mock_ser = Mock()
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        
        mock_serial.assert_called_once_with(port='/dev/ttyACM0', baudrate=115200, timeout=1)

    @patch('serial.Serial')
    def test_request_data(self, mock_serial):
        """Test data request sends correct command."""
        mock_ser = Mock()
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        lidar._request_data()
        
        mock_ser.write.assert_called_once_with(b"REQ\n")

    @patch('serial.Serial')
    def test_read_raw_success(self, mock_serial):
        """Test successful raw data reading."""
        mock_ser = Mock()
        mock_ser.readline.return_value = b"1.23,856,24.5\n"
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        result = lidar._read_raw()
        
        assert result == "1.23,856,24.5"

    @patch('serial.Serial')
    def test_read_raw_serial_exception(self, mock_serial):
        """Test raw reading with serial exception."""
        mock_ser = Mock()
        mock_ser.readline.side_effect = serial.SerialException("Connection lost")
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        
        with patch('builtins.print') as mock_print:
            result = lidar._read_raw()
            
        assert result is None
        mock_print.assert_called_once_with("[LIDAR] Read error: Connection lost")

    @patch('serial.Serial')
    def test_read_raw_unicode_error(self, mock_serial):
        """Test raw reading with unicode decode error."""
        mock_ser = Mock()
        mock_ser.readline.return_value = b'\xff\xfe invalid utf-8'
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        result = lidar._read_raw()
        
        # Should handle decode errors gracefully
        assert isinstance(result, str)

    @patch('serial.Serial')
    @patch('time.time', return_value=1625097600.0)
    def test_parse_line_success(self, mock_time, mock_serial):
        """Test successful line parsing."""
        mock_serial.return_value = Mock()
        
        lidar = Lidar('/dev/ttyACM0')
        result = lidar._parse_line("1.23,856,24.5")
        
        expected = {
            'distance': 1.23,
            'strength': 856.0,
            'temperature': 24.5,
            'unix_time': 1625097600.0
        }
        assert result == expected

    @patch('serial.Serial')
    def test_parse_line_invalid_format(self, mock_serial):
        """Test parsing with invalid number of fields."""
        mock_serial.return_value = Mock()
        
        lidar = Lidar('/dev/ttyACM0')
        result = lidar._parse_line("1.23,856")  # Missing temperature
        
        assert result is None

    @patch('serial.Serial')
    def test_parse_line_invalid_numbers(self, mock_serial):
        """Test parsing with invalid number formats."""
        mock_serial.return_value = Mock()
        
        lidar = Lidar('/dev/ttyACM0')
        
        with patch('builtins.print') as mock_print:
            result = lidar._parse_line("invalid,856,24.5")
            
        assert result == {}  # Returns empty dict on parse error
        mock_print.assert_called_once()
        assert "[LIDAR] Parse error:" in str(mock_print.call_args)

    @patch('serial.Serial')
    def test_read_success(self, mock_serial):
        """Test successful complete read operation."""
        mock_ser = Mock()
        mock_ser.readline.return_value = b"1.23,856,24.5\n"
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        
        with patch('time.time', return_value=1625097600.0):
            result = lidar.read()
        
        expected = {
            'distance': 1.23,
            'strength': 856.0,
            'temperature': 24.5,
            'unix_time': 1625097600.0
        }
        assert result == expected
        mock_ser.write.assert_called_once_with(b"REQ\n")

    @patch('serial.Serial')
    def test_read_no_response(self, mock_serial):
        """Test read with no response from sensor."""
        mock_ser = Mock()
        mock_ser.readline.return_value = b""
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        result = lidar.read()
        
        assert result is None

    @patch('serial.Serial')
    def test_read_exception(self, mock_serial):
        """Test read with exception during operation."""
        mock_ser = Mock()
        mock_ser.write.side_effect = serial.SerialException("Write failed")
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        
        with patch('builtins.print') as mock_print:
            result = lidar.read()
        
        assert result is None
        mock_print.assert_called_once()
        assert "[LIDAR] Read operation failed:" in str(mock_print.call_args)

    @patch('serial.Serial')
    def test_context_manager(self, mock_serial):
        """Test context manager functionality."""
        mock_ser = Mock()
        mock_ser.is_open = True
        mock_serial.return_value = mock_ser
        
        with Lidar('/dev/ttyACM0') as lidar:
            assert lidar.ser == mock_ser
        
        mock_ser.close.assert_called_once()

    @patch('serial.Serial')
    def test_close_method(self, mock_serial):
        """Test explicit close method."""
        mock_ser = Mock()
        mock_ser.is_open = True
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        lidar.close()
        
        mock_ser.close.assert_called_once()

    @patch('serial.Serial')
    def test_close_already_closed(self, mock_serial):
        """Test closing already closed connection."""
        mock_ser = Mock()
        mock_ser.is_open = False
        mock_serial.return_value = mock_ser
        
        lidar = Lidar('/dev/ttyACM0')
        lidar.close()
        
        mock_ser.close.assert_not_called()

    @patch('serial.Serial')
    def test_close_no_serial_attr(self, mock_serial):
        """Test close when serial attribute doesn't exist."""
        mock_serial.return_value = Mock()
        
        lidar = Lidar('/dev/ttyACM0')
        delattr(lidar, 'ser')  # Remove serial attribute
        
        # Should not raise exception
        lidar.close()
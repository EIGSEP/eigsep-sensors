import pytest
import threading
import time
from unittest.mock import Mock, patch, MagicMock
import serial

from eigsep_sensors.peltier.temperature_controller import TemperatureController


@pytest.fixture
def mock_serial():
    with patch(
        "eigsep_sensors.peltier.temperature_controller.serial.Serial"
    ) as mock:
        mock_instance = Mock()
        mock.return_value = mock_instance
        yield mock_instance


class TestTemperatureController:
    def test_init(self, mock_serial):
        with (
            patch("eigsep_sensors.peltier.temperature_controller.time.sleep"),
            patch(
                "eigsep_sensors.peltier.temperature_controller.serial.Serial"
            ) as mock_serial_class,
        ):
            mock_serial_class.return_value = mock_serial
            controller = TemperatureController(
                "/dev/ttyACM0", baudrate=9600, timeout=2
            )

            mock_serial_class.assert_called_once_with(
                "/dev/ttyACM0", baudrate=9600, timeout=2
            )
            assert controller.ser == mock_serial
            assert controller._monitor_thread is None
            assert not controller._stop_event.is_set()

    def test_write_command(self, mock_serial):
        mock_serial.readline.return_value = b"ACK\n"
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        result = controller._write_command("TEST")

        mock_serial.write.assert_called_with(b"TEST\r\n")
        mock_serial.readline.assert_called_once()
        assert result == "ACK"

    def test_start_control_loop(self, mock_serial):
        mock_serial.readline.return_value = b"RESUMED\n"
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        result = controller.start_control_loop()

        mock_serial.write.assert_called_with(b"RESUME\r\n")
        assert result == "RESUMED"

    def test_stop_control_loop(self, mock_serial):
        mock_serial.readline.return_value = b"STOPPED\n"
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        result = controller.stop_control_loop()

        mock_serial.write.assert_called_with(b"STOP\r\n")
        assert result == "STOPPED"

    def test_read_temperature_valid_response(self, mock_serial):
        mock_serial.readline.side_effect = [b"12345,\n", b"25.5,30.0,0.75,\n", b"32.0,35.0,0.5\n"]
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        timestamp, t1_now, t1_target, t1_drive, t2_now, t2_target, t2_drive = controller.read_temperature()

        mock_serial.write.assert_called_with(b"REQ\r\n")
        assert timestamp == 12345
        assert t1_now == 25.5
        assert t1_target == 30.0
        assert t1_drive == 0.75
        assert t2_now == 32.0
        assert t2_target == 35.0
        assert t2_drive == 0.5

    def test_read_temperature_invalid_response(self, mock_serial):
        mock_serial.readline.side_effect = [b"invalid,response\n", b"bad\n", b"format\n"]
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        with pytest.raises(ValueError, match="Unexpected response format"):
            controller.read_temperature()

    def test_start_monitoring_with_callback(self, mock_serial):
        mock_serial.readline.side_effect = [b"12345,\n", b"25.5,30.0,0.75,\n", b"32.0,35.0,0.5\n"] * 10
        callback = Mock()

        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        controller.start_monitoring(0.1, callback)
        assert controller._monitor_thread is not None
        assert controller._monitor_thread.is_alive()

        time.sleep(0.15)
        controller.stop_monitoring()

        callback.assert_called()
        args = callback.call_args[0][0]
        assert args == (12345, 25.5, 30.0, 0.75, 32.0, 35.0, 0.5)

    def test_start_monitoring_without_callback(self, mock_serial):
        mock_serial.readline.side_effect = [b"12345,\n", b"25.5,30.0,0.75,\n", b"32.0,35.0,0.5\n"] * 10

        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        with patch("builtins.print") as mock_print:
            controller.start_monitoring(0.1)
            time.sleep(0.15)
            controller.stop_monitoring()

            mock_print.assert_called()
            args = mock_print.call_args[0][0]
            assert args == (12345, 25.5, 30.0, 0.75, 32.0, 35.0, 0.5)

    def test_start_monitoring_already_running(self, mock_serial):
        mock_serial.readline.side_effect = [b"12345,\n", b"25.5,30.0,0.75,\n", b"32.0,35.0,0.5\n"] * 10

        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        controller.start_monitoring(0.1)

        with pytest.raises(RuntimeError, match="Monitoring already running"):
            controller.start_monitoring(0.1)

        controller.stop_monitoring()

    def test_monitoring_handles_exceptions(self, mock_serial):
        mock_serial.readline.side_effect = Exception("Serial error")

        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        with patch("builtins.print") as mock_print:
            controller.start_monitoring(0.1)
            time.sleep(0.15)
            controller.stop_monitoring()

            error_calls = [
                call
                for call in mock_print.call_args_list
                if "Error reading temperature" in str(call)
            ]
            assert len(error_calls) > 0

    def test_stop_monitoring_no_thread(self, mock_serial):
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        controller.stop_monitoring()

    def test_close(self, mock_serial):
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        controller.close()

        mock_serial.close.assert_called_once()

    def test_init_invalid_port(self):
        """Test initialization with invalid port raises ValueError."""
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            with pytest.raises(ValueError, match="Port must be a non-empty string"):
                TemperatureController("")
            
            with pytest.raises(ValueError, match="Port must be a non-empty string"):
                TemperatureController(None)

    def test_init_serial_exception(self):
        """Test initialization handles serial port errors."""
        with (
            patch("eigsep_sensors.peltier.temperature_controller.time.sleep"),
            patch(
                "eigsep_sensors.peltier.temperature_controller.serial.Serial"
            ) as mock_serial_class,
        ):
            mock_serial_class.side_effect = serial.SerialException("Port not found")
            
            with pytest.raises(RuntimeError, match="Failed to open serial port"):
                TemperatureController("/dev/ttyACM0")

    def test_write_command_timeout(self, mock_serial):
        """Test command timeout handling."""
        mock_serial.readline.return_value = b""  # Empty response = timeout
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        with pytest.raises(TimeoutError, match="No response to command"):
            controller._write_command("TEST")

    def test_write_command_serial_error(self, mock_serial):
        """Test serial communication error handling."""
        mock_serial.write.side_effect = serial.SerialException("Write failed")
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        with pytest.raises(RuntimeError, match="Serial communication failed"):
            controller._write_command("TEST")

    def test_read_temperature_timeout(self, mock_serial):
        """Test temperature read timeout handling."""
        mock_serial.readline.side_effect = [b"12345,\n", b"", b""]  # Timeout on line 2
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        with pytest.raises(TimeoutError, match="Timeout reading line 2"):
            controller.read_temperature()

    def test_read_temperature_invalid_timestamp(self, mock_serial):
        """Test handling of invalid timestamp."""
        mock_serial.readline.side_effect = [b"invalid,\n", b"25.5,30.0,0.75,\n", b"32.0,35.0,0.5\n"]
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        with pytest.raises(ValueError, match="Invalid timestamp"):
            controller.read_temperature()

    def test_read_temperature_invalid_numeric(self, mock_serial):
        """Test handling of invalid numeric values."""
        mock_serial.readline.side_effect = [b"12345,\n", b"25.5,invalid,0.75,\n", b"32.0,35.0,0.5\n"]
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")

        with pytest.raises(ValueError, match="Invalid numeric values"):
            controller.read_temperature()

    def test_close_with_monitoring_error(self, mock_serial):
        """Test close handles errors gracefully."""
        mock_serial.readline.side_effect = [b"12345,\n", b"25.5,30.0,0.75,\n", b"32.0,35.0,0.5\n"] * 10
        with patch("eigsep_sensors.peltier.temperature_controller.time.sleep"):
            controller = TemperatureController("/dev/ttyACM0")
        
        # Start monitoring
        controller.start_monitoring(0.1)
        
        # Make stop_monitoring raise an exception
        controller._monitor_thread = None  # This will cause an AttributeError
        
        # Close should still work without raising
        controller.close()
        mock_serial.close.assert_called_once()

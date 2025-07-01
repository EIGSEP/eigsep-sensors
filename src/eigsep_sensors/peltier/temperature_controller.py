import serial
import threading
import time


class TemperatureController:
    """
    A controller for the Raspberry Pi Pico temperature control firmware.
    """

    def __init__(self, port, baudrate=115200, timeout=1):
        """
        Initialize and open the serial port. Waits briefly for Pico reset.

        Args:
            port (str): Serial port (e.g., '/dev/ttyACM0' or 'COM3').
            baudrate (int): Baud rate (default 115200).
            timeout (float): Read timeout in seconds.

        Raises:
            ValueError: If port is not a valid string.
            RuntimeError: If serial port cannot be opened.
        """
        if not isinstance(port, str) or not port:
            raise ValueError("Port must be a non-empty string")

        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}")
        except Exception as e:
            raise RuntimeError(f"Unexpected error opening serial port: {e}")

        time.sleep(2)  # Allow Pico to reboot and print its banner
        self._monitor_thread = None
        self._stop_event = threading.Event()

    def _write_command(self, cmd):
        """Send a command and read back an ACK (if any).

        Raises:
            RuntimeError: If serial communication fails.
        """
        try:
            self.ser.write((cmd + "\r\n").encode())
            response = self.ser.readline()
            if not response:
                raise TimeoutError(f"No response to command: {cmd}")
            return response.decode("utf-8", errors="replace").strip()
        except serial.SerialException as e:
            raise RuntimeError(
                f"Serial communication failed for command '{cmd}': {e}"
            )
        except TimeoutError:
            raise  # Re-raise timeout errors as-is
        except Exception as e:
            raise RuntimeError(f"Unexpected error in command '{cmd}': {e}")

    def start_control_loop(self):
        """Resume the temperature control loop on the Pico."""
        ack = self._write_command("RESUME")
        return ack

    def stop_control_loop(self):
        """Stop (pause) the temperature control loop on the Pico."""
        ack = self._write_command("STOP")
        return ack

    def read_temperature(self):
        """
        Request a single temperature reading.

        Returns:
            tuple: (timestamp (int), T1_now (float), T1_target (float),
                   T1_drive (float), T2_now (float), T2_target (float),
                   T2_drive (float))

        Raises:
            ValueError: if the response cannot be parsed.
            RuntimeError: if serial communication fails.
            TimeoutError: if no response is received.
        """
        try:
            self.ser.write(b"REQ\r\n")
            # Read multi-line response: timestamp, peltier1 data, peltier2 data
            lines = []
            for i in range(3):
                line = self.ser.readline()
                if not line:
                    raise TimeoutError(
                        f"Timeout reading line {i+1} of temperature data"
                    )
                lines.append(line.decode("utf-8", errors="replace").strip())

            # Parse each line
            timestamp_str = lines[0].rstrip(",")
            peltier1_parts = lines[1].rstrip(",").split(",")
            peltier2_parts = lines[2].split(",")

            if len(peltier1_parts) != 3 or len(peltier2_parts) != 3:
                raise ValueError(
                    f"Unexpected response format: "
                    f"{lines[0]}|{lines[1]}|{lines[2]}"
                )

            try:
                timestamp = int(timestamp_str)
            except ValueError:
                raise ValueError(f"Invalid timestamp: '{timestamp_str}'")

            try:
                T1_now, T1_target, T1_drive = map(float, peltier1_parts)
                T2_now, T2_target, T2_drive = map(float, peltier2_parts)
            except ValueError as e:
                raise ValueError(f"Invalid numeric values in response: {e}")

            return (
                timestamp,
                T1_now,
                T1_target,
                T1_drive,
                T2_now,
                T2_target,
                T2_drive,
            )
        except serial.SerialException as e:
            raise RuntimeError(f"Serial communication error: {e}")
        except (ValueError, TimeoutError):
            raise  # Re-raise these as-is
        except Exception as e:
            raise RuntimeError(f"Unexpected error reading temperature: {e}")

    def start_monitoring(self, interval, callback=None):
        """
        Begin background monitoring of temperature at regular intervals.

        Args:
            interval (float): Seconds between readings.
            callback (callable, optional): Function to call with each reading.
                If None, readings are printed.
        """
        if self._monitor_thread and self._monitor_thread.is_alive():
            raise RuntimeError("Monitoring already running")
        self._stop_event.clear()

        def _monitor():
            while not self._stop_event.is_set():
                try:
                    reading = self.read_temperature()
                    if callback:
                        callback(reading)
                    else:
                        print(reading)
                except Exception as e:
                    print(f"Error reading temperature: {e}")
                time.sleep(interval)

        self._monitor_thread = threading.Thread(target=_monitor, daemon=True)
        self._monitor_thread.start()

    def stop_monitoring(self):
        """Stop background temperature monitoring."""
        if self._monitor_thread:
            self._stop_event.set()
            self._monitor_thread.join()
            self._monitor_thread = None

    def close(self):
        """Close the serial connection and clean up resources."""
        try:
            self.stop_monitoring()  # Ensure monitoring is stopped
        except Exception:
            pass  # Best effort

        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass  # Best effort on cleanup

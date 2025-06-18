import serial
import threading
import time


class TemperatureController:
    """
    A controller for the Raspberry Pi Pico temperature control firmware.
    """

    def __init__(self, port, baudrate=115200, timeout=1):
        """
        Initialize and open the serial port. Waits briefly for the Pico to reset.

        Args:
            port (str): Serial port (e.g., '/dev/ttyACM0' or 'COM3').
            baudrate (int): Baud rate (default 115200).
            timeout (float): Read timeout in seconds.
        """
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)  # Allow Pico to reboot and print its banner
        self._monitor_thread = None
        self._stop_event = threading.Event()

    def _write_command(self, cmd):
        """Send a command and read back an ACK (if any)."""
        self.ser.write((cmd + "\r\n").encode())
        return self.ser.readline().decode().strip()

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
            tuple: (timestamp (int), T_now (float), T_target (float), drive (float))

        Raises:
            ValueError: if the response cannot be parsed.
        """
        self.ser.write(b"REQ\r\n")
        line = self.ser.readline().decode().strip()
        parts = line.split(",")
        if len(parts) != 4:
            raise ValueError(f"Unexpected response: {line}")
        timestamp, T_now, T_target, drive = parts
        return int(timestamp), float(T_now), float(T_target), float(drive)

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
        """Close the serial connection."""
        self.ser.close()

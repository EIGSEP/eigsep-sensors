from mockserial import create_serial_connection

from ..thermistor import constants as const
from .. import Thermistor


class DummyThermistor(Thermistor):
    """
    A dummy thermistor class that simulates a thermistor with a mock
    serial connection.
    """

    def __init__(self, timeout=1, sh_coeff=const.sh_coeff):
        """
        Initialize the DummyThermistor with a mock serial connection.
        """
        self.ser, self.pico = create_serial_connection(timeout=timeout)
        super()._set_temp_constants(sh_coeff)
        self.raw_adc_response = None

    def pico_response(self):
        """
        Simulate a response from the Pico for the given command. This
        mimics the behavior of the Pico connected to the thermistor.
        If the command is "REQ", it sends the raw_adc value, as set by
        the attribute ``raw_adc_response''.

        """
        cmd = self.pico.readline().decode().strip()
        if cmd != "REQ" or self.raw_adc_response is None:
            return
        self.pico.write(f"{self.raw_adc_response}\n".encode())

    def _request_temperature(self):
        """
        Overrides the _request_temperature method to simulate a
        temperature reading from the thermistor.
        """
        super()._request_temperature()
        self.pico_response()

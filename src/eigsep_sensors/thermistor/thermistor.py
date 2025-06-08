import numpy as np
import serial

from . import constants as const


def steinhart_hart(r_therm, sh_coeff):
    """
    Convert resistance of a thermistor to temperature in Kelvin
    using the Steinhart-Hart equation.
    The equation is given by: 1/T = A + B * ln(R) + C * ln(R)^3

    Parameters
    ----------
    r_therm : float
        The resistance of the thermistor in ohms.
    sh_coeff : dict
        The coefficients for the Steinhart-Hart equation.
        Keys are "A", "B", "C", values are floats.

    Returns
    -------
    float
        The temperature in Kelvin.

    Raises
    ------
    ValueError
        If one or more of the coefficients in ``sh_coeff'' is negative
        or the temperature is non-positive.

    """
    if any(c < 0 for c in sh_coeff.values()):
        raise ValueError(f"Invalid Steinhart-Hart coefficients: {sh_coeff}")
    ln_r = np.log(r_therm)
    t_inv = (
        sh_coeff.get("A", 0)
        + sh_coeff.get("B", 0) * ln_r
        + sh_coeff.get("C", 0) * ln_r**3
    )
    if t_inv <= 0:
        raise ValueError(f"Invalid temperature inverse: {t_inv}")
    return 1 / t_inv


class Thermistor:
    """
    Class handling communication with one or more thermistors
    connected to a serial port. If multiple thermistors are connected,
    they are expected to have the same Steinhart-Hart coefficients.
    """

    def __init__(self, port, timeout=1, sh_coeff=const.sh_coeff):
        """
        Set up the serial connection and initialize the thermistor.

        Parameters
        ----------
        port : str
            The serial port to which the thermistor is connected.
        timeout : float
            The timeout for the serial connection in seconds.
        sh_coeff : dict
            The coefficients for the Steinhart-Hart equation. Keys are
            "A", "B", "C", values are floats.

        """
        # serial connection
        self.ser = serial.Serial(port=port, baudrate=115200, timeout=timeout)
        # temperature conversion
        self._set_temp_constants(sh_coeff)

    def _set_temp_constants(self, sh_coeff):
        self.Vcc = 3.3  # voltage supply to the thermistor
        self.R_fixed = 1e4  # fixed resistor value in ohms
        self._nbits = 16
        self.max_adc_value = 2**self._nbits - 1
        self.sh_coeff = sh_coeff

    def raw_to_temp(self, raw):
        """
        Convert a raw ADC value to a temperature in degrees Celsius
        using the Steinhart-Hart equation.

        Parameters
        ----------
        raw : int
            The raw ADC value from the thermistor. Expected to be
            positive, 16-bit integer.

        Returns
        -------
        float
            The temperature in degrees Celsius.

        Raises
        ------
        ValueError
            If the raw ADC value is without the expected range.

        """
        if raw <= 0 or raw > self.max_adc_value:
            raise ValueError(f"Invalid raw ADC value: {raw}")
        vout = raw / self.max_adc_value * self.Vcc  # output voltage
        # compute the resistance of the thermistor
        den = self.Vcc - vout
        if den < 1e-9:
            r_therm = np.inf  # infinite resistance if voltage is equal to Vcc
        else:
            r_therm = self.R_fixed * vout / den
        # steinhart-hart equation
        t_kelvin = steinhart_hart(r_therm, self.sh_coeff)
        return t_kelvin - 273.15

    def _request_temperature(self):
        self.ser.write(b"REQ\n")

    def read_temperature(self):
        """
        Non-blocking read of the temperature from the thermistor(s).

        Returns
        -------
        dict
            The temperature in degrees Celsius. The key corresponds to
            the ADC pin that the reading is from. None if no data is
            available.

        Raises
        ------
        ValueError
            If the response from the thermistor is invalid.

        Notes
        -----
        The method assumes that the Pico sends a single line of
        data in the format PIN:VALUE, where PIN is the ADC pin
        number and VALUE is the raw ADC value. Multiple readings
        are commas separated, e.g. "0:12345,1:67890".

        """
        self._request_temperature()
        response = self.ser.readline().decode().strip()  # raw adc value
        if not response:
            return None
        readings = response.split(",")
        temp = {}
        for reading in readings:
            pin, raw = reading.split(":")
            pin = int(pin)
            raw_value = int(raw)
            temp[pin] = self.raw_to_temp(raw_value)
        return temp

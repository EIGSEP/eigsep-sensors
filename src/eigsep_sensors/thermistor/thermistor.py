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
        If the resistance is non-positive or the temperature inverse
        is non-positive.

    """

    ln_r = np.log(r_therm)
    t_inv = (
        sh_coeff["A"]
        + sh_coeff["B"] * ln_r
        + sh_coeff["C"] * ln_r**3
    )
    if t_inv <= 0:
        raise ValueError(f"Invalid temperature inverse: {t_inv}")
    return 1 / t_inv


class Thermistor:
    """
    Class handling communitcation with a thermistor connected to a
    serial port.
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
            The raw ADC value from the thermistor.

        Returns
        -------
        float
            The temperature in degrees Celsius.

        """
        vout = raw / self.max_adc_value * self.Vcc  # output voltage
        # compute the resistance of the thermistor
        den = self.Vcc - vout
        if den < 1e-9:
            r_therm = np.inf  # infinite resistance if voltage is equal to Vcc
        else:
            r_therm = self.R_fixed * vout / den
        if r_therm <= 0:
            raise ValueError(f"Invalid resistance value: {r_therm} ohms")
        # Steinhart-Hart equation
        t_kelvin = steinhart_hart(r_therm, self.sh_coeff)
        return t_kelvin - 273.15

    def read_temperature(self):
        """
        Non-blocking read of the temperature from the thermistor.

        Returns
        -------
        float
            The temperature in degrees Celsius. None if no data is
            available.

        Raises
        ------
        ValueError
            If the response from the thermistor is invalid.

        """
        self.ser.write(b"REQ\n")
        raw = self.ser.readline().decode().strip()  # raw adc value
        try:
            raw_value = int(raw)
        except TypeError:
            return None  # no data available
        temp = self.raw_to_temp(raw_value)
        return temp

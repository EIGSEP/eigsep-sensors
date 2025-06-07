import numpy as np
import serial

from . import constants as const

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
        self.ser = serial.Serial(
            port=port, baudrate=115200, timeout=timeout
        )
        # temperature conversion
        self.Vcc = 3.3  # voltage supply to the thermistor
        self.R_fixed = 1e4  # fixed resistor value in ohms
        self._nbits = 16
        self.max_adc_value = 2 ** self._nbits - 1
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

        Raises
        ------
        ValueError
            If the raw value is such that the resistance or temperature
            in Kelvin become negative.

        """
        vout = raw / self.max_adc_value * self.Vcc  # output voltage
        # compute the resistance of the thermistor
        den = self.Vcc - vout
        if den < 1e-9:
            r_therm = np.inf  # infinite resistance if voltage is equal to Vcc
        else:
            r_therm = self.R_fixed * vout / den
        if r_therm <= 0:
            raise ValueError(
                f"Invalid resistance value: {r_therm} ohms"
            )
        # Steinhart-Hart equation
        ln_r = np.log(r_therm)
        t_inv = (
            self.sh_coeff["A"]
            + self.sh_coeff["B"] * ln_r
            + self.sh_coeff["C"] * ln_r ** 3
        )
        if t_inv <= 0:
            raise ValueError(f"Invalid temperature inverse: {t_inv}")
        t_kelvin = 1 / t_inv
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
        except ValueError:
            raise ValueError(f"Invalid raw value: {raw}")
        temp = self.raw_to_temp(raw_value)
        return temp

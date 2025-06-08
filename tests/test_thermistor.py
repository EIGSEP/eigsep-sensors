import numpy as np
import pytest

from eigsep_sensors.thermistor import constants as const
from eigsep_sensors.thermistor import thermistor as th
from eigsep_sensors.testing import DummyThermistor


@pytest.fixture
def therm():
    return DummyThermistor(timeout=1, sh_coeff=const.sh_coeff)


@pytest.mark.parametrize("logR", [0, 1, 5, 10, 20])
def test_steinhart_hart(logR):
    r_therm = np.exp(logR)
    coeffs = {"A": 1, "B": 1, "C": 1}
    temp = th.steinhart_hart(r_therm, coeffs)
    assert temp == pytest.approx(
        1 / (1 + logR + logR**3)
    )  # all coefficients are 1
    # one coeff at a time
    coeffs = {"A": 1}  # other coefficients are zero
    temp = th.steinhart_hart(r_therm, coeffs)
    assert temp == 1
    if logR > 0:
        coeffs = {"B": 1}
        temp = th.steinhart_hart(r_therm, coeffs)
        assert temp == pytest.approx(1 / logR)
        coeffs = {"C": 1}
        temp = th.steinhart_hart(r_therm, coeffs)
        assert temp == pytest.approx((1 / logR) ** 3)
    else:
        coeffs = {"B": 1}
        with pytest.raises(ValueError):  # t_inv cannot be zero
            th.steinhart_hart(r_therm, coeffs)
        coeffs = {"C": 1}
        with pytest.raises(ValueError):  # t_inv cannot be zero
            th.steinhart_hart(r_therm, coeffs)

    # invalid coefficients
    coeffs = {}  # all 0
    with pytest.raises(ValueError):
        th.steinhart_hart(r_therm, coeffs)
    coeffs = {"A": 1, "B": 1, "C": -1}  # c is negative
    with pytest.raises(ValueError):
        th.steinhart_hart(r_therm, coeffs)


def test_raw_to_temp(therm):
    # invalid raw
    with pytest.raises(ValueError):
        therm.raw_to_temp(-1)
    with pytest.raises(ValueError):
        therm.raw_to_temp(0)
    with pytest.raises(ValueError):
        therm.raw_to_temp(therm.max_adc_value + 1)
    # max adc value, gives 0 Kelvin so -273.15 celsius
    assert therm.raw_to_temp(therm.max_adc_value) == -273.15
    # valid raw values
    raw = therm.max_adc_value // 2
    temp = therm.raw_to_temp(raw)
    expected_rtherm = therm.R_fixed  # same resistance for both resistors
    log_rtherm = np.log(expected_rtherm)
    expected_temp = th.steinhart_hart(expected_rtherm, therm.sh_coeff)
    expected_celsius = expected_temp - 273.15
    assert temp == pytest.approx(expected_celsius, abs=1e-2)
    # the choice of coefficients matter
    therm._set_temp_constants({"A": 1})
    expected_temp = 1  # only A is non-zero
    expected_celsius = expected_temp - 273.15
    assert therm.raw_to_temp(raw) == pytest.approx(expected_celsius, abs=1e-2)
    therm._set_temp_constants({"B": 1})
    expected_temp = 1 / log_rtherm
    expected_celsius = expected_temp - 273.15
    assert therm.raw_to_temp(raw) == pytest.approx(expected_celsius, abs=1e-2)
    therm._set_temp_constants({"C": 1})
    expected_temp = (1 / log_rtherm) ** 3
    expected_celsius = expected_temp - 273.15
    assert therm.raw_to_temp(raw) == pytest.approx(expected_celsius, abs=1e-2)


def test_read_temperature(therm):
    # no response
    out = therm.read_temperature()
    assert out is None
    # valid response, format PIN:VALUE,
    pin = "26"
    value = therm.max_adc_value // 2
    therm.raw_adc_response = f"{pin}:{value}"
    out = therm.read_temperature()
    assert out is not None
    # calculate expected temperature like in test_raw_to_temp
    expected_rtherm = therm.R_fixed
    expected_temp = th.steinhart_hart(expected_rtherm, therm.sh_coeff)
    expected_celsius = expected_temp - 273.15
    expected_out = {int(pin): pytest.approx(expected_celsius, abs=1e-2)}
    assert out == expected_out
    # multiple pins
    pins = ["26", "27", "28"]
    values = [therm.max_adc_value // 2] * len(pins)
    therm.raw_adc_response = ",".join(f"{p}:{v}" for p, v in zip(pins, values))
    out = therm.read_temperature()
    assert out is not None
    expected_out = {
        int(pin): pytest.approx(expected_celsius, abs=1e-2) for pin in pins
    }
    assert out == expected_out

import numpy as np
import pytest
import time

from mockserial import create_serial_connection
from eigsep_sensors.imu import IMU_BNO085


class DummyIMU_BNO085(IMU_BNO085):
    def __init__(self, timeout=1):
        self.ser, self.pico = create_serial_connection(timeout=timeout)


@pytest.fixture
def imu():
    """
    Fixture to create a DummyIMU_BN0085 instance.
    """
    return DummyIMU_BNO085(timeout=1)


@pytest.fixture
def sample_values():
    """
    Typical values for the BNO085 IMU. See the bno085_code.py.
    """
    return {
        "q": np.array([0.707, 0.0, 0.707, 0.0]),  # quaternion (x, y, z, w)
        "gq": np.array([0.700, 0.0, 0.700, 0.100]), # game quaternion (x, y, z, w)
        "euler": np.array(IMU_BNO085.quaternion_to_euler(np.array([0.707, 0.0, 0.707, 0.0]))), # euler
        "geuler": np.array(IMU_BNO085.quaternion_to_euler(np.array([0.700, 0.0, 0.700, 0.100]))), # euler
        "a": np.array([0.0, 0.0, -9.81]),  # acceleration vector
        "la": np.array([0.0, 0.0, -9.81]),  # linear acceleration vector
        "g": np.array([0.0, 0.0, 0.0]),  # gyroscope vector
        "m": np.array([0.0, 0.0, 0.0]),  # magnetic field vector
        "grav": np.array([0.0, 0.0, -9.81]),  # gravity vector
        "steps": 0,  # steps
        "stab": "1",  # stability value
    }


def pack_values(values):
    """
    Helper function to format the sample values into a string
    that resembles the output from the BNO085 IMU.
    """
    qx, qy, qz, qw = values["q"]
    gqx, gqy, gqz, gqw = values["gq"]
    ax, ay, az = values["a"]
    la_x, la_y, la_z = values["la"]
    gx, gy, gz = values["g"]
    mx, my, mz = values["m"]
    gxv, gyv, gzv = values["grav"]
    steps = values["steps"]
    stability = values["stab"]

    return ",".join(
        [
            f"q:{qx:.3f}:{qy:.3f}:{qz:.3f}:{qw:.3f}",
            f"gq:{gqx:.3f}:{gqy:.3f}:{gqz:.3f}:{gqw:.3f}",
            f"a:{ax:.3f}:{ay:.3f}:{az:.3f}",
            f"la:{la_x:.3f}:{la_y:.3f}:{la_z:.3f}",
            f"g:{gx:.3f}:{gy:.3f}:{gz:.3f}",
            f"m:{mx:.3f}:{my:.3f}:{mz:.3f}",
            f"grav:{gxv:.3f}:{gyv:.3f}:{gzv:.3f}",
            f"steps:{steps}",
            f"stab:{stability}",
        ]
    )


def test_parse_line(imu, sample_values):
    """
    Test the _parse_line method of DummyIMU_BN0085.
    """
    read = imu._parse_line(pack_values(sample_values))
    # read adds timestamp
    t = read.pop("unix_time")
    now = time.time()
    assert now - t < 1, "Timestamps do not match expected time range"
    expected_keys = set(sample_values.keys())
    assert (
        set(read.keys()) == expected_keys
    ), "Parsed keys do not match expected keys"
    for key in read.keys():
        if key == "stab":
            # XXX think stability is a string, not an array
            assert read[key] == sample_values[key]
        else:
            np.testing.assert_array_almost_equal(
                read[key],
                sample_values[key],
                decimal=3,
                err_msg=f"Values for {key} do not match expected values",
            )

    # send an unexpected message, should ignore the key
    vals = pack_values(sample_values)
    vals += ",unexpected_key:123.456"
    read = imu._parse_line(vals)
    # should have all keys except 'unexpected_key'
    t = read.pop("unix_time")
    now = time.time()
    assert now - t < 1, "Timestamps do not match expected time range"
    expected_keys = set(sample_values.keys())
    assert (
        set(read.keys()) == expected_keys
    ), "Parsed keys do not match expected keys"
    for key in read.keys():
        if key == "stab":
            assert read[key] == sample_values[key]
        else:
            np.testing.assert_array_almost_equal(
                read[key],
                sample_values[key],
                decimal=3,
                err_msg=f"Values for {key} do not match expected values",
            )

    # add valid key with invalid value
    vals = pack_values(sample_values).split(",")
    vals[2] = "a:not:floating:point"  # invalid acceleration
    invalid_str = ",".join(vals)
    read = imu._parse_line(invalid_str)
    # should have all keys except 'a' for acceleration
    t = read.pop("unix_time")
    now = time.time()
    assert now - t < 1, "Timestamps do not match expected time range"
    expected_keys = set(sample_values.keys()) - {"a"}
    assert (
        set(read.keys()) == expected_keys
    ), "Invalid acceleration should be ignored"
    for key in read.keys():
        if key == "stab":
            assert read[key] == sample_values[key]
        else:
            np.testing.assert_array_almost_equal(
                read[key],
                sample_values[key],
                decimal=3,
                err_msg=f"Values for {key} do not match expected values",
            )


def test_request_imu(imu):
    """
    Test the request_imu method of DummyIMU_BN0085.
    """
    imu._request_imu()
    assert imu.pico.readline() == b"REQ\n"
    imu._request_imu(cal=True)
    assert imu.pico.readline() == b"CAL\n"


def test_read_imu(imu, sample_values):
    """
    Test the read_imu method of DummyIMU_BN0085.
    """
    imu.pico.write(pack_values(sample_values).encode())
    read = imu.read_imu()
    t = read.pop("unix_time")
    now = time.time()
    assert now - t < 1, "Timestamps do not match expected time range"
    expected_keys = set(sample_values.keys())
    assert (
        set(read.keys()) == expected_keys
    ), "Read keys do not match expected keys"
    for key in read.keys():
        if key == "stab":
            # stability is a string, not an array
            assert (
                read[key] == sample_values[key]
            ), f"Stability value for {key} does not match expected value"
        else:
            # all other values should be arrays
            np.testing.assert_array_almost_equal(
                read[key],
                sample_values[key],
                decimal=3,
                err_msg=f"Values for {key} do not match expected values",
            )

    # read again, with no data available
    read = imu.read_imu()
    assert read is None, "Read should return None when no data is available"

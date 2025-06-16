"""
Tests for convenience functions in the IMU class.
"""

import numpy as np

from eigsep_sensors.imu.imu import IMU


def test_get_pitch_roll_from_gravity():
    g = np.array([0.0, 0.0, -9.81])
    pitch, roll = IMU.get_pitch_roll_from_gravity(g)
    assert np.isclose(pitch, 0.0, atol=1e-5)
    assert np.isclose(roll, 0.0, atol=1e-5)

    # tilted 45 degrees around the x-axis
    g = np.array([0.0, 9.81, -9.81]) / np.sqrt(2)
    pitch, roll = IMU.get_pitch_roll_from_gravity(g)
    assert np.isclose(pitch, np.pi / 4, atol=1e-5)
    assert np.isclose(roll, 0.0, atol=1e-5)

    # tilted 45 degrees around the y-axis
    g = np.array([9.81, 0.0, -9.81]) / np.sqrt(2)
    pitch, roll = IMU.get_pitch_roll_from_gravity(g)
    assert np.isclose(pitch, 0.0, atol=1e-5)
    assert np.isclose(roll, np.pi / 4, atol=1e-5)


def test_angle_with_vertical():
    raise NotImplementedError("This test is not implemented yet.")


def test_calculate_orientation():
    raise NotImplementedError("This test is not implemented yet.")


def test_get_orientation_unit_vector():
    raise NotImplementedError("This test is not implemented yet.")


def test_quaternion_to_euler():
    raise NotImplementedError("This test is not implemented yet.")


def test_normalize_vector():
    raise NotImplementedError("This test is not implemented yet.")

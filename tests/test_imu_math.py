"""
Tests for convenience functions in the IMU class.
"""

import numpy as np
import pytest
from eigsep_sensors.imu import IMU, IMU_MMA8451, IMU_BNO085


def test_get_pitch_roll_from_gravity():
    # Straight down
    g = np.array([0.0, 0.0, -9.81])
    pitch, roll = IMU.get_pitch_roll_from_gravity(g[0], g[1], g[2])
    assert np.isclose(pitch, 0.0, atol=1e-5)
    assert np.isclose(roll, 0.0, atol=1e-5)

    # Tilted 45 degrees around Y-axis (pitch)
    g = np.array([9.81 * np.sin(np.radians(45)), 0.0, -9.81 * np.cos(np.radians(45))])
    pitch, roll = IMU.get_pitch_roll_from_gravity(g[0], g[1], g[2])
    assert np.isclose(pitch, -45.0, atol=1e-2)
    assert np.isclose(roll, 0.0, atol=1e-5)

    # Tilted 45 degrees around X-axis (roll)
    g = np.array([0.0, 9.81 * np.sin(np.radians(45)), -9.81 * np.cos(np.radians(45))])
    pitch, roll = IMU.get_pitch_roll_from_gravity(g[0], g[1], g[2])
    assert np.isclose(pitch, 0.0, atol=1e-2)
    assert np.isclose(roll, -45.0, atol=1e-2)

    # Case where roll > 90 -> should wrap to negative
    g = np.array([0.0, 9.81 * np.sin(np.radians(95)), 9.81 * np.cos(np.radians(95))])
    pitch, roll = IMU.get_pitch_roll_from_gravity(g[0], g[1], g[2])
    print(pitch, roll)
    assert np.isclose(pitch, 0.0, atol=1e-2)
    assert np.isclose(roll, -85.0, atol=1e-2)

    # Case where roll < -90 -> should wrap to positive
    g = np.array([0.0, -9.81 * np.sin(np.radians(95)), -9.81 * np.cos(np.radians(95))])
    pitch, roll = IMU.get_pitch_roll_from_gravity(g[0], g[1], g[2])
    print(pitch, roll)
    assert np.isclose(pitch, 0.0, atol=1e-2)
    assert np.isclose(roll, 85.0, atol=1e-2)


def test_angle_with_vertical():
    assert np.isclose(IMU.angle_with_vertical(1.0), 0.0)
    assert np.isclose(IMU.angle_with_vertical(0.0), 90.0)
    assert np.isclose(IMU.angle_with_vertical(-1.0), 180.0)


def test_calculate_orientation():
    theta, phi = IMU_MMA8451.calculate_orientation(1.0, 1.0, 1.0)
    assert isinstance(theta, float)
    assert isinstance(phi, float)
    assert 0 <= abs(theta) <= 180
    assert 0 <= phi <= 180


def test_get_orientation_unit_vector():
    ux, uy, uz = IMU_MMA8451.get_orientation_unit_vector(3.0, 4.0, 0.0)
    mag = np.sqrt(ux**2 + uy**2 + uz**2)
    assert np.isclose(mag, 1.0)
    assert np.isclose(ux, 0.6)
    assert np.isclose(uy, 0.8)
    assert np.isclose(uz, 0.0)
    with pytest.raises(ValueError, match="Zero-magnitude gravity vector."):
        IMU_MMA8451.get_orientation_unit_vector(0.0, 0.0, 0.0)


def test_quaternion_to_euler():
    # Identity quaternion (no rotation)
    q = [0.0, 0.0, 0.0, 1.0]
    roll, pitch, yaw = IMU_BNO085.quaternion_to_euler(q)
    assert np.isclose(roll, 0.0, atol=1e-5)
    assert np.isclose(pitch, 0.0, atol=1e-5)
    assert np.isclose(yaw, 0.0, atol=1e-5)


def test_normalize_vector():
    v = [3.0, 4.0, 0.0]
    v_unit = IMU_BNO085.normalize_vector(v)
    assert np.allclose(v_unit, [0.6, 0.8, 0.0])
    assert np.isclose(np.linalg.norm(v_unit), 1.0)

import time
import numpy as np
import math
import eigsep_sensors as eig
import pytest

class Test_Accelerometer:
    def test_calc_orientation(self):
        """
        Test `calculate_orientation` to ensure correct computation of azimuthal (theta)
        and polar (phi) angles from given x, y, z values.

        Uses a known input where:
        - theta = arctan2(1, 1) = 45°
        - phi   = arctan2(sqrt(2), sqrt(2)) = 45°
        """
        x, y, z = 1.0, 1.0, np.sqrt(2)
        theta, phi = eig.calculate_orientation(x, y, z)

        assert math.isclose(theta, 45.0, abs_tol=0.01)
        assert math.isclose(phi, 45.0, abs_tol=0.01)

    def test_get_orientation_unit_vector(self):
        """
        Test `get_orientation_unit_vector` to ensure the returned vector is normalized
        and directionally correct.

        Input vector (3, 0, 4) has magnitude 5, so expected unit vector is (0.6, 0.0, 0.8).
        """
        x, y, z = 3.0, 0.0, 4.0
        unit = eig.get_orientation_unit_vector(x, y, z)

        assert math.isclose(unit["x_unit"], 0.6, abs_tol=0.01)
        assert math.isclose(unit["y_unit"], 0.0, abs_tol=0.01)
        assert math.isclose(unit["z_unit"], 0.8, abs_tol=0.01)

    def test_get_pitch_roll_from_unit_vector(self):
        """
        Test `get_pitch_roll_from_unit_vector` to verify pitch and roll calculation
        from a unit gravity vector.

        Case:
        - gx = 0 → pitch = 0°
        - gy = 1, gz = 0 → roll = arctan2(1, 0) = 90°
        """
        gx, gy, gz = 0.0, 1.0, 0.0
        pitch, roll = eig.get_pitch_roll_from_unit_vector(gx, gy, gz)

        assert math.isclose(pitch, 0.0, abs_tol=0.01)
        assert math.isclose(roll, 90.0, abs_tol=0.01)

def test_angle_with_vertical(self):
    """
    Test `angle_with_vertical` for expected angles between the Z-axis and
    a given z-component of the unit vector.

    Cases:
    - z_unit = 1.0 → aligned with Z → 0°
    - z_unit = 0.0 → perpendicular to Z → 90°
    - z_unit = -1.0 → opposite to Z → 180°
    """
    # For z_unit = 1.0 (aligned with Z-axis)
    g_unit = {'z_unit': 1.0, 'x_unit': 0.0, 'y_unit': 0.0}
    assert math.isclose(eig.angle_with_vertical(g_unit), 0.0, abs_tol=0.01)

    # For z_unit = 0.0 (perpendicular to Z-axis)
    g_unit = {'z_unit': 0.0, 'x_unit': 1.0, 'y_unit': 0.0}
    assert math.isclose(eig.angle_with_vertical(g_unit), 90.0, abs_tol=0.01)

    # For z_unit = -1.0 (opposite to Z-axis)
    g_unit = {'z_unit': -1.0, 'x_unit': 0.0, 'y_unit': 0.0}
    assert math.isclose(eig.angle_with_vertical(g_unit), 180.0, abs_tol=0.01)

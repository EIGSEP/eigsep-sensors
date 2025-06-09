import numpy as np

def calculate_orientation(x, y, z):
    """
    Calculate the orientation angles (theta and phi) based on 3D acceleration data.

    This function computes the azimuthal angle (theta) and polar angle (phi)
    from the given x, y, and z acceleration components using the arctangent function.

    Args:
        x (float): The acceleration along the X axis.
        y (float): The acceleration along the Y axis.
        z (float): The acceleration along the Z axis.

    Returns:
        tuple: A tuple containing the orientation angles:
            - theta (float): The azimuthal angle in degrees.
            - phi (float): The polar angle in degrees.
    """
    theta = np.arctan2(y, x)
    phi = np.arctan2(np.sqrt(x**2 + y**2), z)
    theta_deg = np.degrees(theta)
    phi_deg = np.degrees(phi)
    return theta_deg, phi_deg


def get_orientation_unit_vector(x, y, z):
    """
    Normalize the gravity vector (x, y, z) from the accelerometer to get orientation.

    Args:
        x, y, z (float): Raw accelerometer readings.

    Returns:
        dict: Unit vector showing device orientation with respect to gravity.
    """
    g_mag = np.sqrt(x**2 + y**2 + z**2)
    if g_mag == 0:
        raise ValueError("Zero-magnitude gravity vector.")

    return {"x_unit": x / g_mag, "y_unit": y / g_mag, "z_unit": z / g_mag}


def get_pitch_roll_from_unit_vector(gx, gy, gz):
    """
    Calculate pitch and roll angles (in degrees) from a normalized gravity vector.

    Args:
        gx, gy, gz (float): Components of the unit gravity vector.

    Returns:
        tuple: (pitch, roll)
    """
    pitch = np.arcsin(-gx) * (180.0 / np.pi)  # tilt forward/backward
    roll = np.arctan2(gy, gz) * (180.0 / np.pi)  # tilt left/right
    return pitch, roll


def angle_with_vertical(g_unit):
    """
    Compute the angle between the gravity unit vector and the Z axis.

    Args:
        g_unit (dict): Unit gravity vector (from get_orientation_unit_vector).

    Returns:
        float: Angle in degrees between gravity and Z-axis (i.e., device tilt).
    """
    dot_product = g_unit["z_unit"]  # since Z-axis unit vector is (0, 0, 1)
    angle_rad = np.arccos(dot_product)
    return np.degrees(angle_rad)

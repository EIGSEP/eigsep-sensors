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

def transform_data_based_on_orientation(measured_data, theta):
    """
    Transform measured data based on the orientation angle.

    This function scales the measured data by the cosine of the azimuthal angle (theta),
    which adjusts the data based on the orientation of the sensor.

    Args:
        measured_data (float): The raw measured data to be transformed.
        theta (float): The azimuthal angle in degrees used for the transformation.

    Returns:
        float: The transformed data after applying the cosine of the theta angle.
    """
    transformed_data = measured_data * np.cos(np.radians(theta))
    return transformed_data

from eigsep_sensors import accelerometer
import pytest
import numpy as np
#import json
#import os

class Test_Accelerometer:
    def test_calc_orientation(self):
        # XXX do something with accelerator.calculate_orientation
        pass
        #def calculate_orientation(x, y, z):
        #    theta = np.arctan2(y, x)
        #    phi = np.arctan2(np.sqrt(x**2 + y**2), z)
        #    theta_deg = np.degrees(theta)
        #    phi_deg = np.degrees(phi)
        #    return theta_deg, phi_deg

    def test_transform_data_based_on_orientation(self):
        # XXX do something with accelerator.transform_data_based_on_orientation
        pass
        #def transform_data_based_on_orientation(measured_data, theta):
        #    transformed_data = measured_data * np.cos(np.radians(theta))
        #    return transformed_data



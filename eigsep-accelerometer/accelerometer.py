import time
import numpy as np
#import json
#import os
import board
import adafruit_mma8451

i2c = board.I2C()
sensor = adafruit_mma8451.MMA8451(i2c)

def calculate_orientation(x, y, z):
    theta = np.arctan2(y, x)
    phi = np.arctan2(np.sqrt(x**2 + y**2), z)
    theta_deg = np.degrees(theta)
    phi_deg = np.degrees(phi)
    return theta_deg, phi_deg

def transform_data_based_on_orientation(measured_data, theta):
    transformed_data = measured_data * np.cos(np.radians(theta))
    return transformed_data

stored_data = []
while True:
    x, y, z = sensor.acceleration
    theta, phi = calculate_orientation(x, y, z)
    measured_data = 100
    transformed_data = transform_data_based_on_orientation(measured_data, theta)
    data_entry = {
        "time": time.time(),
        "x": x,
        "y": y,
        "z": z,
        "theta_deg": theta,
        "phi_deg": phi,
        "transformed_data": transformed_data
    }
    stored_data.append(data_entry)
    print(f"Time: {data_entry['time']:.2f}")
    print(f"Accelerometer X: {x:.2f} Y: {y:.2f} Z: {z:.2f}")
    print(f"Azimuthal Angle (θ): {theta:.2f}°")
    print(f"Polar Angle (φ): {phi:.2f}°")
    print(f"Transformed Data: {transformed_data:.2f}")
    print("-" * 50)
    time.sleep(0.25)

#if not os.path.exists('data'):
#    os.makedirs('data')
#with open('data/accelerometer_data.json', 'w') as f:
#    json.dump(stored_data, f, indent=4)

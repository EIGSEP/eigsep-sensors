import time
import numpy as np
#import json
#import os
import board
import adafruit_mma8451
import eigsep_sensors as eig

accelerometer = eig.Accelerometer()

stored_data = []
while True:
    try:
        x, y, z = accelerometer.sensor.acceleration
        theta, phi = eig.calculate_orientation(x, y, z)
        measured_data = 100
        transformed_data = eig.transform_data_based_on_orientation(measured_data, theta)
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
    except Exception as e:
        print(e)
        time.sleep(0.25)

#if not os.path.exists('data'):
#    os.makedirs('data')
#with open('data/accelerometer_data.json', 'w') as f:
#    json.dump(stored_data, f, indent=4)

import time
# import json
# import os
import eigsep_sensors as eig

# === Setup ===

accelerometer = eig.IMU_MMA8451()
stored_data = []

# === Main Loop ===

try:
    while True:
        try:
            x, y, z = accelerometer.sensor.acceleration

            # Normalize gravity vector
            unit_vec = accelerometer.get_orientation_unit_vector(x, y, z)
            gx, gy, gz = (
                unit_vec["x_unit"],
                unit_vec["y_unit"],
                unit_vec["z_unit"],
            )

            # Compute orientation angles
            pitch, roll = accelerometer.get_pitch_roll_from_unit_vector(gx, gy, gz)
            tilt_angle = accelerometer.angle_with_vertical(gz)
            theta, phi = accelerometer.calculate_orientation(x, y, z)

            data_entry = {
                "time": time.time(),
                "x": x,
                "y": y,
                "z": z,
                "x_unit": gx,
                "y_unit": gy,
                "z_unit": gz,
                "pitch_deg": pitch,
                "roll_deg": roll,
                "tilt_from_vertical_deg": tilt_angle,
                "theta_deg": theta,
                "phi_deg": phi,
            }

            stored_data.append(data_entry)

            print(f"Time: {data_entry['time']:.2f}")
            print(f"Accelerometer X: {x:.2f} Y: {y:.2f} Z: {z:.2f}")
            print(
                f"Pitch: {pitch:.2f}° | Roll: {roll:.2f}° | Tilt from Vertical: {tilt_angle:.2f}°"
            )
            print(
                f"Azimuthal Angle (θ): {theta:.2f}° | Polar Angle (φ): {phi:.2f}°"
            )
            print("-" * 50)
            time.sleep(1.0)

        except IOError as e:
            print("Sensor read error:", e)
            time.sleep(1.0)

except KeyboardInterrupt:
    print("\nProgram interrupted by user. Exiting...")

# if not os.path.exists('data'):
#    os.makedirs('data')
# with open('data/accelerometer_data.json', 'w') as f:
#    json.dump(stored_data, f, indent=4)

# TO USE: Copy as "code.py" on to a Raspberry Pi Pico with CircuitPython.
import sys
import time
import board
import busio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_STABILITY_CLASSIFIER,
)

# Setup I2C
i2c = busio.I2C(board.GP1, board.GP0, frequency=800000)
while not i2c.try_lock:
    time.sleep(0.1)

imu = BNO08X_I2C(i2c)

# Enable all desired IMU features
imu.enable_feature(BNO_REPORT_ACCELEROMETER)
imu.enable_feature(BNO_REPORT_GYROSCOPE)
imu.enable_feature(BNO_REPORT_MAGNETOMETER)
imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
imu.enable_feature(BNO_REPORT_GRAVITY)
imu.enable_feature(BNO_REPORT_STEP_COUNTER)
imu.enable_feature(BNO_REPORT_STABILITY_CLASSIFIER)

def calibrate_imu():
    print("Starting calibration...")
    imu.begin_calibration()
    time.sleep(1)

    # Enable minimal necessary features for calibration feedback

    calibration_good_at = False
    wanted_mag_status = 2
    wanted_accel_status = 2
    wanted_gyro_status = 2
    while True:
        time.sleep(0.1)

        # Print live magnetic data
        mx, my, mz = imu.magnetic
        print("Magnetometer: X=%0.3f Y=%0.3f Z=%0.3f" % (mx, my, mz))

        # Print game rotation quaternion
        q_i, q_j, q_k, q_real = imu.quaternion
        print("Quat: I=%0.3f J=%0.3f K=%0.3f R=%0.3f" % (q_i, q_j, q_k, q_real))

        # Check calibration status (magnetometer)
        mag_status, accel_status, gyro_status = imu.calibration_status
        print("Mag Calibration: %s (%d)" % (
            ["Unreliable", "Low", "Medium", "High"][mag_status], mag_status))
        print("Accel Calibration: %s (%d)" % (
            ["Unreliable", "Low", "Medium", "High"][accel_status], accel_status))
        print("Gyro Calibration: %s (%d)" % (
            ["Unreliable", "Low", "Medium", "High"][gyro_status], gyro_status))

        if not calibration_good_at:
            if mag_status >= wanted_mag_status:
                wanted_mag_status += 1
                calibration_good_at = True
            if accel_status >= wanted_accel_status:
                wanted_accel_status += 1
                calibration_good_at = True
            if gyro_status >= wanted_gyro_status:
                wanted_gyro_status += 1
                calibration_good_at = True
        if calibration_good_at:
            # No user input on microcontrollers usually, so save automatically
            imu.save_calibration_data()
            print("Calibration saved.")
            calibration_good_at = False
        if mag_status >= 3 and accel_status >= 3 and gyro_status >= 3:
            break

        print("--------------------------------------------------")

def read_and_format_imu_data():
    """
    IMU Output Format for BNO085:

    Each output line is a comma-separated string containing labeled sensor readings.
    Each key is followed by its corresponding values, separated by colons.

    Key Format and Data Types:
    --------------------------

    q     : Rotation vector quaternion [qx:qy:qz:qw]       -> List[float] (length 4)
            Absolute orientation expressed as a unit quaternion.
            Provides accurate rotation without gimbal lock. Used for 3D tracking, AR/VR, and heading.

    gq    : Game rotation vector quaternion [gqx:gqy:gqz:gqw]  -> List[float] (length 4)
            Relative orientation (no magnetometer). Stable, drift-limited rotation tracking.
            Ideal for fast, smooth motion in AR/VR or gaming. Not referenced to magnetic north.

    a     : Acceleration [ax:ay:az]                       -> List[float] (length 3)
            Raw accelerometer output in m/s^2 including both gravity and linear motion.

    la    : Linear acceleration [la_x:la_y:la_z]           -> List[float] (length 3)
            Gravity-compensated acceleration in m/s^2.
            Represents only movement-related forces - ideal for gesture or motion tracking.

    g     : Gyroscope / angular velocity [gx:gy:gz]        -> List[float] (length 3)
            Measures rate of rotation around each axis in rad/s.

    m     : Magnetometer / magnetic field [mx:my:mz]       -> List[float] (length 3)
            Magnetic field strength in microtesla (mu*T). Useful for compass heading or field mapping.

    grav  : Gravity vector [gxv:gyv:gzv]                   -> List[float] (length 3)
            Isolated gravitational force in m/s^2 - shows Earth's gravity direction and tilt.
            Complementary to `linear_acceleration`.

    steps : Step counter                                   -> int  
            Number of walking or running steps detected since the sensor was initialized.
            Step detection is based on repetitive motion patterns and resets to zero on power-up.

    stab  : Stability classification                       -> str or None
            Sensor's assessment of current motion state.

            Possible values:
            - "Unknown"     - Unable to determine stability
            - "On Table"    - Stationary on a flat surface
            - "Stationary"  - Minimal movement, but duration threshold not met
            - "Stable"      - Movement below threshold for long enough
            - "In motion"   - Device is moving

    Example Output Line:
    --------------------
    q:0.123:0.456:0.789:0.012,a:0.01:0.02:0.03,la:0.00:0.00:0.01,
    g:0.12:0.13:0.14,m:0.23:0.24:0.25,grav:0.00:0.00:1.00,
    steps:42,stab:Stable
    """

    try:
        qx, qy, qz, qw = imu.quaternion
        ax, ay, az = imu.acceleration
        la_x, la_y, la_z = imu.linear_acceleration
        gx, gy, gz = imu.gyro
        mx, my, mz = imu.magnetic
        gxv, gyv, gzv = imu.gravity
        steps = imu.steps
        stability = imu.stability_classification

        return ",".join(
            [
                f"q:{qx:.3f}:{qy:.3f}:{qz:.3f}:{qw:.3f}",
                f"a:{ax:.3f}:{ay:.3f}:{az:.3f}",
                f"la:{la_x:.3f}:{la_y:.3f}:{la_z:.3f}",
                f"g:{gx:.3f}:{gy:.3f}:{gz:.3f}",
                f"m:{mx:.3f}:{my:.3f}:{mz:.3f}",
                f"grav:{gxv:.3f}:{gyv:.3f}:{gzv:.3f}",
                f"steps:{steps}",
                f"stab:{stability}",
            ]
        )
    except Exception as e:
        return f"ERR:{str(e)}"

# === Main loop ===
while True:
    line = sys.stdin.readline().strip()
    if line == "CAL":
        calibrate_imu()
    if line == "REQ":
        print(read_and_format_imu_data())
    time.sleep(0.5)

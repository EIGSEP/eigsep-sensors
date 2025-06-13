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
i2c = busio.I2C(board.GP1, board.GP0)
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


def read_and_format_imu_data():
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
    if line == "REQ":
        print(read_and_format_imu_data())
    time.sleep(0.5)

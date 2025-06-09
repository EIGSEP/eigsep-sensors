__author__ = "EIGSEP Team"
__version__ = "0.0.1"

from .imu_mma8451_orientation import calculate_orientation, get_orientation_unit_vector, get_pitch_roll_from_unit_vector, angle_with_vertical
from .imu import IMU_MMA8451, IMU_BNO085

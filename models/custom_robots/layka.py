from models.custom_robots.mobile_robot import MobileRobot
from models.differential_drive_dynamics import DifferentialDriveDynamics


class Layka(MobileRobot):
    # Layka Properties
    LAYKA_WHEEL_RADIUS = 0.021  # meters
    LAYKA_WHEEL_BASE_LENGTH = 0.0885  # meters
    LAYKA_WHEEL_TICKS_PER_REV = 2765
    LAYKA_MAX_WHEEL_DRIVE_RATE = 10.0  # rad/s

    # Layka outline
    LAYKA_OUTLINE = [
        [0.000, 0.070],
        [0.050, 0.050],
        [0.070, 0.000],
        [0.050, -0.050],
        [0.000, -0.070],
        [-0.050, -0.050],
        [-0.070, 0.000],
        [-0.050, 0.050],
    ]

    LAYKA_SENSOR_MIN_RANGE = 0.01
    LAYKA_SENSOR_MAX_RANGE = 0.25

    LAYKA_TRANS_VEL_LIMIT = 0.3148  # m/s
    LAYKA_ANG_VEL_LIMIT = 2.2763  # rad/s

    LAYKA_SENSOR_POSES = [
        [0.070, 0.000, 0],        # Right-middle, facing forward
        [0.050, 0.050, 45],       # Front-right, diagonal
        [0.000, 0.070, 90],       # Front-center, facing left
        [-0.050, 0.050, 135],     # Front-left, diagonal
        [-0.070, 0.000, 180],     # Left-middle, facing backward
        [-0.050, -0.050, -135],   # Back-left, diagonal
        [0.000, -0.070, -90],     # Back-center, facing right
        [0.050, -0.050, -45],     # Back-right, diagonal
    ]


    def __init__(self):
        super().__init__(
            wheel_radius = self.LAYKA_WHEEL_RADIUS, 
            wheel_base_length = self.LAYKA_WHEEL_BASE_LENGTH, 
            max_wheel_drive_rate = self.LAYKA_MAX_WHEEL_DRIVE_RATE, 
            wheel_ticks_per_rev = self.LAYKA_WHEEL_TICKS_PER_REV, 
            bottom_plate = self.LAYKA_OUTLINE, 
            sensor_poses = self.LAYKA_SENSOR_POSES, 
            sensor_min_range = self.LAYKA_SENSOR_MIN_RANGE, 
            sensor_max_range = self.LAYKA_SENSOR_MAX_RANGE, 
            sensor_fov = 40, 
        )
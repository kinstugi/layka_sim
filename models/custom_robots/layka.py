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
        [-0.050, 0.050],  # Left front diagonal
        [0.000, 0.070],   # Front
        [0.050, 0.050],   # Right front diagonal
        [0.070, 0.000],   # Right side
        [0.050, -0.050],  # Right rear diagonal
        [0.000, -0.070],  # Rear
        [-0.050, -0.050], # Left rear diagonal
        [-0.070, 0.000],  # Left side
    ]


    LAYKA_SENSOR_MIN_RANGE = 0.01
    LAYKA_SENSOR_MAX_RANGE = 0.25

    LAYKA_TRANS_VEL_LIMIT = 0.3148  # m/s
    LAYKA_ANG_VEL_LIMIT = 2.2763  # rad/s

    LAYKA_SENSOR_POSES = [
        # [-0.050, 0.050, 135],  # Similar to [-0.038, 0.048, 128] (Front-left outward)
        # [0.000, 0.070, 90],    # Similar to [0.019, 0.064, 75] (Front-left inward)
        # [0.050, 0.050, 45],    # Similar to [0.050, 0.050, 42] (Front-left shallow inward)
        # [0.070, 0.000, 0],     # Similar to [0.070, 0.017, 13] (Front-center)
        # [0.050, -0.050, 315],  # Similar to [0.070, -0.017, -13] (Front-right shallow inward)
        # [0.000, -0.070, 270],  # Similar to [0.019, -0.064, -75] (Front-right inward)
        # [-0.050, -0.050, 225], # Similar to [-0.038, -0.048, -128] (Front-right outward)
        # [-0.070, 0.000, 180],  # Similar to [-0.048, 0.000, 180] (Rear-center)
        
        [-0.038, 0.048, 128],  # x, y, theta (in degrees)
        [0.019, 0.064, 75],
        [0.050, 0.050, 42],
        [0.070, 0.017, 13],
        [0.070, -0.017, -13],
        [0.050, -0.050, -42],
        [0.019, -0.064, -75],
        [-0.038, -0.048, -128],
        [-0.048, 0.000, 180],
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
            sensor_fov = 20, 
        )
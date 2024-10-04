from models.custom_robots.mobile_robot import MobileRobot
from models.differential_drive_dynamics import DifferentialDriveDynamics


class Layka(MobileRobot):
    def __init__(self) -> None:
        WHEEL_RADIUS = 0.021  # meters
        WHEEL_BASE_LENGTH = 0.0885  # meters
        WHEEL_TICKS_PER_REV = 2765
        MAX_WHEEL_DRIVE_RATE = 15.0  # rad/s
        ROBOT_OUTLINE = [
            [0.000, 0.070],
            [0.050, 0.050],
            [0.070, 0.000],
            [0.050, -0.050],
            [0.000, -0.070],
            [-0.050, -0.050],
            [-0.070, 0.000],
            [-0.050, 0.050],
        ]

        SENSOR_MIN_RANGE = 0.02
        SENSOR_MAX_RANGE = 0.2

        SENSOR_POSES = [
            # , 225 degrees[-0.050, 0.05, 135],  
            # [0.00, 0.070, 90],
            # [0.050, 0.050, 45],
            # [0.070, 0.00, 0],
            # # [0.070, -0.017, -13],
            # [0.050, -0.050, -45],
            # [0.00, -0.070, -90],
            # [-0.05, -0.05, -135],
            # [-0.07, 0.000, 180], #Rear-left diagonal

            [-0.038, 0.048, 135],  
            [0.019, 0.064, 90],
            [0.050, 0.050, 45],
            [0.070, 0.0, 0],
            [0.070, -0.017, -0],
            [0.050, -0.050, -45],
            [0.019, -0.064, -90],
            [-0.038, -0.048, -135],
            [-0.048, 0.000, 180], #Rear-left diagonal, 225 degrees
        ]

        super().__init__(
            wheel_radius= WHEEL_RADIUS,
            wheel_base_length=WHEEL_BASE_LENGTH,
            wheel_ticks_per_rev=WHEEL_TICKS_PER_REV,
            max_wheel_drive_rate=MAX_WHEEL_DRIVE_RATE,
            sensor_min_range= SENSOR_MIN_RANGE,
            sensor_max_range= SENSOR_MAX_RANGE,
            sensor_fov= 40,
            bottom_plate=ROBOT_OUTLINE,
            sensor_poses= SENSOR_POSES
        )
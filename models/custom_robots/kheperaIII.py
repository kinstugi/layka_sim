from models.custom_robots.mobile_robot import MobileRobot

class KheperaIII(MobileRobot):
    K3_WHEEL_RADIUS = 0.021  # meters
    K3_WHEEL_BASE_LENGTH = 0.0885  # meters
    K3_WHEEL_TICKS_PER_REV = 2765
    K3_MAX_WHEEL_DRIVE_RATE = 15.0  # rad/s
    K3_BOTTOM_PLATE = [
        [-0.024, 0.064],
        [0.033, 0.064],
        [0.057, 0.043],
        [0.074, 0.010],
        [0.074, -0.010],
        [0.057, -0.043],
        [0.033, -0.064],
        [-0.025, -0.064],
        [-0.042, -0.043],
        [-0.048, -0.010],
        [-0.048, 0.010],
        [-0.042, 0.043],
    ]
    K3_SENSOR_MIN_RANGE = 0.02
    K3_SENSOR_MAX_RANGE = 0.2
    K3_SENSOR_POSES = [
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
            wheel_radius=self.K3_WHEEL_RADIUS,
            wheel_base_length=self.K3_WHEEL_BASE_LENGTH,
            max_wheel_drive_rate=self.K3_MAX_WHEEL_DRIVE_RATE,
            wheel_ticks_per_rev=self.K3_WHEEL_TICKS_PER_REV,
            bottom_plate=self.K3_BOTTOM_PLATE,
            sensor_poses=self.K3_SENSOR_POSES,
            sensor_min_range=self.K3_SENSOR_MIN_RANGE,
            sensor_max_range=self.K3_SENSOR_MAX_RANGE,
            sensor_fov=20
        )

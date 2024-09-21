from math import radians
from typing import List
from models.differential_drive_dynamics import DifferentialDriveDynamics
from models.polygon import Polygon
from models.pose import Pose
from models.proximity_sensor import ProximitySensor
from models.wheel_encoder import WheelEncoder
from robot_control.robot_supervisor_interface import RobotSupervisorInterface
from robot_control.supervisor import Supervisor

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
    [0.070, 0.000, 0],
    [0.050, 0.050, 45],
    [0.000, 0.070, 90],
    [-0.050, 0.050, 135],
    [-0.070, 0.000, 180],
    [-0.050, -0.050, 225],
    [0.000, -0.070, 270],
    [0.050, -0.050, 315],
]

class Layka:
    def __init__(self) -> None:
        self.geometry = Polygon(LAYKA_OUTLINE)
        self.global_geometry = Polygon(LAYKA_OUTLINE)

        self.wheel_radius = LAYKA_WHEEL_RADIUS
        self.wheel_base_length = LAYKA_WHEEL_BASE_LENGTH

        self.pose = Pose(0.0, 0.0, 0.0)

        self.left_wheeel_encoder = WheelEncoder(LAYKA_WHEEL_TICKS_PER_REV)
        self.right_wheel_encoder = WheelEncoder(LAYKA_WHEEL_TICKS_PER_REV)
        self.wheel_encoders = [self.left_wheeel_encoder, self.right_wheel_encoder]

        self.ir_sensors: List[ProximitySensor] = []    
        for _p1, _p2, _p3 in LAYKA_SENSOR_POSES:
            ir_pose = Pose(_p1, _p2, radians(_p3))
            self.ir_sensors.append(
                ProximitySensor(self, ir_pose, LAYKA_SENSOR_MIN_RANGE, LAYKA_SENSOR_MAX_RANGE, radians(40))
            )
        
        self.dynamics = DifferentialDriveDynamics(self.wheel_radius, self.wheel_base_length)

        self.supervisor = Supervisor(
            RobotSupervisorInterface(self),
            LAYKA_WHEEL_RADIUS,
            LAYKA_WHEEL_BASE_LENGTH,
            LAYKA_WHEEL_TICKS_PER_REV,
            LAYKA_SENSOR_POSES,
            LAYKA_SENSOR_MAX_RANGE
        )

        self.left_wheel_drive_rate = 0.0
        self.right_wheel_drive_rate = 0.0
    

    def step_motion(self, dt) -> None:
        v_l, v_r = self.left_wheel_drive_rate, self.right_wheel_drive_rate

        self.dynamics.apply_dynamics(v_l, v_r, dt, self.pose, self.wheel_encoders)

        self.global_geometry = self.geometry.get_transformation_to_pose(self.pose)

        for ir_sensor in self.ir_sensors:
            ir_sensor.update_position()
    

    def set_wheel_drive_rates(self, v_l: float, v_r: float) -> None:
        v_l = min(LAYKA_MAX_WHEEL_DRIVE_RATE, v_l)
        v_r = min(LAYKA_MAX_WHEEL_DRIVE_RATE, v_r)

        self.left_wheel_drive_rate = v_l
        self.right_wheel_drive_rate = v_r        
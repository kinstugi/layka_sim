from math import radians
from models.differential_drive_dynamics import DifferentialDriveDynamics
from models.polygon import Polygon
from models.pose import Pose
from models.proximity_sensor import ProximitySensor
from models.wheel_encoder import WheelEncoder
from robot_control.robot_supervisor_interface import RobotSupervisorInterface
from robot_control.supervisor import Supervisor

class MobileRobot:
    def __init__(self, 
                wheel_radius, 
                wheel_base_length, 
                max_wheel_drive_rate, 
                wheel_ticks_per_rev, 
                bottom_plate, 
                sensor_poses, 
                sensor_min_range, 
                sensor_max_range, 
                sensor_fov, 
                dynamics_class=DifferentialDriveDynamics, 
                supervisor=None,
                initial_pose = [0.0, 0.0, 0.0]
            ):
        """
        Initializes a mobile robot with basic properties. 
        This class is meant to be inherited and customized for specific robot models.
        
        Parameters:
        - wheel_radius: Radius of the robot's wheels.
        - wheel_base_length: Distance between the two wheels (wheel axle length).
        - max_wheel_drive_rate: Maximum angular velocity of the wheels.
        - wheel_ticks_per_rev: Number of encoder ticks per wheel revolution.
        - bottom_plate: List of coordinates defining the robot's shape.
        - sensor_poses: List of positions and orientations of sensors (x, y, theta).
        - sensor_min_range: Minimum sensing range for sensors.
        - sensor_max_range: Maximum sensing range for sensors.
        - sensor_fov: Field of view of the proximity sensors.
        - dynamics_class: Class to handle the robot's motion dynamics (default: DifferentialDriveDynamics).
        - supervisor_class: Supervisor class to manage high-level control (default: Supervisor).
        """
        # geometry
        self.geometry = Polygon(bottom_plate)
        self.global_geometry = Polygon(bottom_plate)

        # wheel arrangement
        self.wheel_radius = wheel_radius
        self.wheel_base_length = wheel_base_length
        self.max_wheel_drive_rate = max_wheel_drive_rate

        # pose
        self.pose = Pose(initial_pose[0], initial_pose[1], initial_pose[2])

        # wheel encoders
        self.left_wheel_encoder = WheelEncoder(wheel_ticks_per_rev)
        self.right_wheel_encoder = WheelEncoder(wheel_ticks_per_rev)
        self.wheel_encoders = [self.left_wheel_encoder, self.right_wheel_encoder]

        # sensors
        self.ir_sensors = []
        for _pose in sensor_poses:
            sensor_pose = Pose(_pose[0], _pose[1], radians(_pose[2]))
            self.ir_sensors.append(
                ProximitySensor(
                    self, sensor_pose, sensor_min_range, sensor_max_range, radians(sensor_fov)
                )
            )

        # dynamics
        self.dynamics = dynamics_class(self.wheel_radius, self.wheel_base_length)

        # supervisor
        self.supervisor = Supervisor(
            RobotSupervisorInterface(self),  # interface to interact with robot
            self.wheel_radius,
            self.wheel_base_length,
            wheel_ticks_per_rev,
            sensor_poses,
            sensor_max_range,
            initial_pose_args= initial_pose
        )
        
        if supervisor:
            self.supervisor = supervisor

        # initial wheel drive rates
        self.left_wheel_drive_rate = 0.0
        self.right_wheel_drive_rate = 0.0

    def step_motion(self, dt):
        """
        Updates the robot's motion over a time step `dt`.
        """
        v_l = self.left_wheel_drive_rate
        v_r = self.right_wheel_drive_rate

        # Apply dynamics
        self.dynamics.apply_dynamics(v_l, v_r, dt, self.pose, self.wheel_encoders)

        # Update global geometry
        self.global_geometry = self.geometry.get_transformation_to_pose(self.pose)

        # Update sensor positions
        for sensor in self.ir_sensors:
            sensor.update_position()

    def set_wheel_drive_rates(self, v_l, v_r):
        """
        Sets the angular velocities of the wheels, with maximum rate limit.
        """
        v_l = min(self.max_wheel_drive_rate, v_l)
        v_r = min(self.max_wheel_drive_rate, v_r)

        self.left_wheel_drive_rate = v_l
        self.right_wheel_drive_rate = v_r
    
    def set_initial_pose(self, x: float, y: float, theta: float):
        """
        Sets the initial pose of the robot.

        Args:
            x (float): Initial x-coordinate.
            y (float): Initial y-coordinate.
            theta (float): Initial orientation (in radians).
        """
        self.pose = Pose(x, y, theta)

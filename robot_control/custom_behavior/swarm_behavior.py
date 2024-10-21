from math import pi, log, sin, cos, radians
from models.pose import Pose
from robot_control.controllers.depart_swarm_controller import DepartSwarmController
from robot_control.controllers.search_robots_controller import SearchRobotsController
from robot_control.controllers.wait_swarm_controller import WaitSwarmController
from robot_control.custom_behavior.behavior_controller_interface import BehaviorControllerInterface
from robot_control.custom_behavior.behavior_state_machine import BehaviorStateMachine


class SwarmBehavior:
    def __init__(
            self,
            robot_interface,  # the interface through which this supervisor will interact with the robot
            wheel_radius,  # the radius of a drive wheel on the robot
            wheel_base_length,  # the robot's wheel base
            wheel_encoder_ticks_per_rev,  # the number of wheel encoder ticks per revolution of a drive wheel
            sensor_placements,  # placement pose of the sensors on the robot body
            sensor_range,  # max detection range of the sensors
            initial_pose_args=[0.0, 0.0, 0.0],
        ) -> None:
        
        self.time = 0.0

        self.robot = robot_interface

        self.proximity_sensor_placements = [
            Pose(rawpose[0], rawpose[1], radians(rawpose[2]))
            for rawpose in sensor_placements
        ]

        self.proximity_sensor_max_range = sensor_range

        self.robot_wheel_radius = wheel_radius
        self.robot_wheel_base_length = wheel_base_length
        self.wheel_encoder_ticks_per_revolution = wheel_encoder_ticks_per_rev
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

        # controllers
        controller_interface = BehaviorControllerInterface(self)
        self.search_robots_controller = SearchRobotsController(controller_interface)
        self.wait_swarm_controller = WaitSwarmController(controller_interface)
        self.depart_swarm_controller = DepartSwarmController(controller_interface)

        # state machine
        self.state_machine = BehaviorStateMachine(self)

        #state
        self.proximity_sensor_distances = [0.0, 0.0] * len(sensor_placements)
        self.estimated_pose = Pose(*initial_pose_args)
        self.current_controller = self.search_robots_controller
from math import pi, log, sin, cos, radians
from models.pose import Pose
from robot_control.supervisor_controller_interface import SupervisorControllerInterface
from robot_control.supervisor_state_machine import SupervisorStateMachine
from robot_control.controllers.avoid_obstacles_controller import AvoidObstaclesController
from robot_control.controllers.follow_wall_controller import FollowWallController
from robot_control.controllers.go_to_angle_controller import GoToAngleController
from robot_control.controllers.go_to_goal_controller import GoToGoalController
from robot_control.controllers.gtg_and_ao_controller import GTGAndAOController


class LaykaBehavior:
    def __init__(
            self,
            robot_interface,  # the interface through which this supervisor will interact with the robot
            wheel_radius,  # the radius of a drive wheel on the robot
            wheel_base_length,  # the robot's wheel base
            wheel_encoder_ticks_per_rev,  # the number of wheel encoder ticks per revolution of a drive wheel
            sensor_placements,  # placement pose of the sensors on the robot body
            sensor_range,  # max detection range of the sensors
            goal=[0.0, 0.0],  # the goal to which this supervisor will guide the robot
            initial_pose_args=[0.0, 0.0, 0.0],
        ) -> None:
        pass
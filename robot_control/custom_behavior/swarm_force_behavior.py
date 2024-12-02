from math import cos, log, radians, sin
from models.pose import Pose
from utils.constants import ANGULAR_GAIN, LINEAR_GAIN
from utils.linalg2_util import scale, add
from utils.math_util import cartesian_to_polar


# control parameters
K3_TRANS_VEL_LIMIT = 0.3148  # m/s
K3_ANG_VEL_LIMIT = 2.2763  # rad/s

class SwarmForceBehavior:
    def __init__(
        self,
        robot_interface,  # the interface through which this supervisor will interact with the robot
        wheel_radius,  # the radius of a drive wheel on the robot
        wheel_base_length,  # the robot's wheel base
        wheel_encoder_ticks_per_rev,  # the number of wheel encoder ticks per revolution of a drive wheel
        sensor_placements,  # placement pose of the sensors on the robot body
        sensor_range,  # max detection range of the sensors
        initial_pose_args=[0.0, 0.0, 0.0],
        target_orientation = 0.0
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

        self.proximity_sensor_distances = [0.0, 0.0] * len(sensor_placements)
        self.robot_detection_sensor_array = [False] * len(sensor_placements) # this will tell us which sensor is detecting another robot
        self.estimated_pose = Pose(*initial_pose_args)

        #control bounds
        self.v_max = K3_TRANS_VEL_LIMIT
        self.omega_max = K3_ANG_VEL_LIMIT

        # CONTROL OUTPUTS - UNICYCLE
        self.v_output = 0.0
        self.omega_output = 0.0 

    def step(self, dt):
        self.time += dt
        self.execute()

    def execute(self):
        f_vector = self.calculate_f_vector()
        # convert f_vector to v and omega send to robot
        r, omega = self.f_to_velocities(f_vector)
        self._send_robot_commands(r, omega)

    
    def calculate_f_vector(self):
        # ... implement the calculation of f based on sensor data and desired behaviors
        # - Use the robot's sensors to gather information about its neighbors and the environment.
        # - Calculate the proximal, alignment, and goal-seeking components of the f vector.
        # - Combine these components using the weights a, b, and c.
        
        a, b, c, d = 0.7, 0.1, 0.1, 0.9 # right now because keeping the proximity distance is most important it gets the most weight

        proximal_vector = scale(self.calculate_proximal_vector(), a)
        alignment_vector = scale(self.calculate_alignment_vector(), b)
        goal_vector = scale(self.calculate_goal_vector(), c)
        
        # obstacle_avoidance_vector = self.calculate_obstacle_avoidance_vector()
        f_vector = add(proximal_vector, add(alignment_vector, goal_vector))
        return f_vector

    def f_to_velocities(self, f_vector: list) -> list:
        _orientation = self.robot.robot.pose.theta
        v = f_vector[0] * cos(_orientation) + f_vector[1] * sin(_orientation)
        omega = f_vector[1] * cos(_orientation) - f_vector[0] * sin(_orientation)
    
        return v * LINEAR_GAIN, omega * ANGULAR_GAIN


    def calculate_proximal_vector(self)->tuple[float]:
        proximal_x, proximal_y = 0, 0

        strength_of_repulsion = 1.5
        proximal_distance = 0.26

        for neighbor_pose in self.robot.read_robot_neighbors_pose():
            r, _ = cartesian_to_polar([neighbor_pose.x, neighbor_pose.y])
            attr_repul = -8 * strength_of_repulsion * (2 * pow(proximal_distance, 4) / pow(r, 5) - pow(proximal_distance, 2) / pow(r, 3))

            proximal_x += attr_repul * neighbor_pose.x
            proximal_y += attr_repul * neighbor_pose.y

        return proximal_x, proximal_y
    
    def calculate_alignment_vector(self)->tuple[float]:
        alignment_x, alignment_y = 0, 0
        return alignment_x, alignment_y

    def calculate_goal_vector(self)->tuple[float]:
        current_heading = self.estimated_pose.theta
        goal_distance = 100

        goal_x = self.estimated_pose.x + goal_distance * cos(current_heading)
        goal_y = self.estimated_pose.y + goal_distance * sin(current_heading)

        dx = goal_x - self.estimated_pose.x
        dy = goal_y - self.estimated_pose.y
        return dx, dy

    def calculate_obstacle_avoidance_vector(self)->tuple[float]:
        pass

    def _update_proximity_sensor_distances(self):
        self.proximity_sensor_distances = [
            0.02 - (log(readval / 3960.0)) / 30.0
            for readval in self.robot.read_proximity_sensors()
        ]
        self.robot_detection_sensor_array = [
            v for v in self.robot.read_robot_detection_array()
        ]

    def calculate_individual_vector(self):
        pass

    # generate and send the correct commands to the robot
    def _send_robot_commands(self, v_output = 0, omega_output = 0):
        # limit the speeds:
        v = max(min(v_output, self.v_max), -self.v_max)
        omega = max(min(omega_output, self.omega_max), -self.omega_max)

        # send the drive commands to the robot
        v_l, v_r = self._uni_to_diff(v, omega)
        self.robot.set_wheel_drive_rates(v_l, v_r)
    
    def _uni_to_diff(self, v, omega):
        # v = translational velocity (m/s)
        # omega = angular velocity (rad/s)

        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        v_l = ((2.0 * v) - (omega * L)) / (2.0 * R)
        v_r = ((2.0 * v) + (omega * L)) / (2.0 * R)

        return v_l, v_r
    
    def _diff_to_uni(self, v_l, v_r):
        # v_l = left-wheel angular velocity (rad/s)
        # v_r = right-wheel angular velocity (rad/s)

        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        v = (R / 2.0) * (v_r + v_l)
        omega = (R / L) * (v_r - v_l)

        return v, omega

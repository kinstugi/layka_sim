from math import cos, log, radians, sin, pi
from models.pose import Pose
from utils.constants import ANGULAR_GAIN, LINEAR_GAIN
from utils.linalg2_util import scale, add
from utils.math_util import cartesian_to_polar

#remember to plot the proximal vector

# control parameters
K3_TRANS_VEL_LIMIT = 0.3148  # m/s
K3_ANG_VEL_LIMIT = 2.2763  # rad/s

# Distance thresholds for obstacle detection
D_CAUTION = 0.20  # meters from obstacle
D_DANGER = 0.15   # meters from obstacle

class SwarmForceBehavior:
    SEARCH = "search"
    WAIT = "wait"
    LEAVE = "leave"
    AVOID_OBSTACLE = "avoid obstacle"

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
        self.current_state = SwarmForceBehavior.SEARCH

    def step(self, dt):
        self.time += dt
        self.execute()

    def execute(self):
        self._update_state()
        # f_vector = self.calculate_f_vector()
        # # convert f_vector to v and omega send to robot
        # r, omega = self.f_to_velocities(f_vector)
        # self._send_robot_commands(r, omega)
        if self.current_state == SwarmForceBehavior.AVOID_OBSTACLE:
            self.execute_avoid_obstacle()
        elif self.current_state == SwarmForceBehavior.WAIT:
            self.execute_wait_in_swarm()
        elif self.current_state == SwarmForceBehavior.SEARCH:
            self.execute_search()

    def execute_avoid_obstacle(self):
        # print("..... avoiding obstacle")
        self.turn_to_avoid_obstacle()
        self.current_state = SwarmForceBehavior.SEARCH

    def execute_wait_in_swarm(self):
        # print(".... waiting in swarm")
        if not self.detect_robots_nearby():
            print("bots not detected")
        f_vector = self.calculate_f_vector()
        r, omega = self.f_to_velocities(f_vector)
        # i have to put a condition here for robot to leave swarm and resume search
        print(r, omega, "<<<<<<<<<<<<<<<<")
        self._send_robot_commands(r, omega)

    def execute_search(self):
        # print(".... search mode")
        if self.detect_robots_nearby():
            self.current_state = SwarmForceBehavior.WAIT
        elif self.detect_obstacle():
            self.current_state = SwarmForceBehavior.AVOID_OBSTACLE
        else:
            self.move_forward()
    
    def calculate_f_vector(self):
        # ... implement the calculation of f based on sensor data and desired behaviors
        # - Use the robot's sensors to gather information about its neighbors and the environment.
        # - Calculate the proximal, alignment, and goal-seeking components of the f vector.
        # - Combine these components using the weights a, b, and c.
        
        a, b, c, d = 0.7, 0.1, 0.1, 0.3 # right now because keeping the proximity distance is most important it gets the most weight

        proximal_vector = scale(self.calculate_proximal_vector(), a)
        alignment_vector = scale(self.calculate_alignment_vector(), b)
        goal_vector = scale(self.calculate_goal_vector(), c)
        noise_vector = scale(self.calculate_noise_vector(), d)
        # print(proximal_vector, goal_vector, alignment_vector, noise_vector)
        # obstacle_avoidance_vector = self.calculate_obstacle_avoidance_vector()
        f_vector = add(proximal_vector, add(alignment_vector, add(goal_vector, noise_vector)))
        return f_vector

    def f_to_velocities(self, f_vector: list) -> list:
        _orientation = self.robot.robot.pose.theta
        v = f_vector[0] * cos(_orientation) + f_vector[1] * sin(_orientation)
        omega = f_vector[1] * cos(_orientation) - f_vector[0] * sin(_orientation)
    
        return v * LINEAR_GAIN, omega * ANGULAR_GAIN


    def calculate_proximal_vector(self)->tuple[float]:
        proximal_x, proximal_y = 0, 0

        strength_of_repulsion = 1.5
        proximal_distance = 0.07

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
        return 0, 0

    def calculate_noise_vector(self):
        import random
        # Generate a small random vector for noise
        random_angle = random.uniform(0, 2 * pi)
        random_magnitude = random.uniform(0, 10)  # Adjust magnitude range as needed
        return [0, 0]
        return [random_magnitude * cos(random_angle), random_magnitude * sin(random_angle)]


    def calculate_individual_vectors(self):
        p_vec = self.calculate_proximal_vector()
        h_vec = self.calculate_alignment_vector()
        g_vec = self.calculate_goal_vector()
        return p_vec, h_vec, g_vec, [0, 0]
    
    def move_forward(self):
        self._send_robot_commands(0.15, 0)
    
    def turn_to_avoid_obstacle(self, angle = None):
        if not angle:
            angle = pi / 4
        self._send_robot_commands(0, angle)

    def detect_obstacle(self):
        return any(d < D_DANGER for d in self._forward_sensor_distances())

    def detect_robots_nearby(self):
        return any(self.robot_detection_sensor_array)

    def _update_state(self):
        # update estimated robot state from sensor readings
        self._update_proximity_sensor_distances()
        self._update_odometry()

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

    def _forward_sensor_distances(self):
        # Assumes the forward sensors are from indices 1 to 7
        return self.proximity_sensor_distances[1:7]

   # update the estimated position of the robot using it's wheel encoder readings
    def _update_odometry(self):
        R = self.robot_wheel_radius
        N = float(self.wheel_encoder_ticks_per_revolution)

        # read the wheel encoder values
        ticks_left, ticks_right = self.robot.read_wheel_encoders()

        # get the difference in ticks since the last iteration
        d_ticks_left = ticks_left - self.prev_ticks_left
        d_ticks_right = ticks_right - self.prev_ticks_right

        # estimate the wheel movements
        d_left_wheel = 2 * pi * R * (d_ticks_left / N)
        d_right_wheel = 2 * pi * R * (d_ticks_right / N)
        d_center = 0.5 * (d_left_wheel + d_right_wheel)

        # calculate the new pose
        prev_x, prev_y, prev_theta = self.estimated_pose.sunpack()
        new_x = prev_x + (d_center * cos(prev_theta))
        new_y = prev_y + (d_center * sin(prev_theta))
        new_theta = prev_theta + (
            (d_right_wheel - d_left_wheel) / self.robot_wheel_base_length
        )

        # update the pose estimate with the new values
        self.estimated_pose.supdate(new_x, new_y, new_theta)

        # save the current tick count for the next iteration
        self.prev_ticks_left = ticks_left
        self.prev_ticks_right = ticks_right

    def _update_proximity_sensor_distances(self):
        self.proximity_sensor_distances = [
            0.02 - (log(readval / 3960.0)) / 30.0
            for readval in self.robot.read_proximity_sensors()
        ]
        self.robot_detection_sensor_array = [
            v for v in self.robot.read_robot_detection_array()
        ]
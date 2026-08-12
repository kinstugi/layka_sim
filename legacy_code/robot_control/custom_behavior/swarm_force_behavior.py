from math import cos, log, radians, sin, pi, sqrt
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

        # Lennard-Jones parameters
        self.epsilon = 1.0  # Depth of the potential well
        self.sigma = 0.40  # Distance at which the potential is zero
        
        # Search behavior parameters
        self.search_turn_timer = 0.0
        self.search_turn_duration = 2.0  # seconds to turn when searching
        self.search_turn_angle = pi / 3  # angle to turn during search
        self.last_search_turn_time = 0.0
        self.search_wander_interval = 5.0  # seconds between random turns

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
        # Turn away from obstacle
        self.turn_to_avoid_obstacle()
        # Check if obstacle is cleared, if so return to search
        if not self.detect_obstacle():
            self.current_state = SwarmForceBehavior.SEARCH

    def execute_wait_in_swarm(self):
        # Check if robots are still nearby (within sensor range)
        if not self.detect_robots_nearby():
            # No robots detected, return to search mode
            self.current_state = SwarmForceBehavior.SEARCH
            return
        
        # Check for obstacles (walls) - avoid them even in swarm mode
        if self.detect_obstacle():
            self.current_state = SwarmForceBehavior.AVOID_OBSTACLE
            return
        
        # Apply Lennard-Jones forces for attraction/repulsion
        f_vector = self.calculate_f_vector()
        r, omega = self.f_to_velocities(f_vector)
        self._send_robot_commands(r, omega)

    def execute_search(self):
        # Check if robots are detected - switch to swarm mode
        if self.detect_robots_nearby():
            self.current_state = SwarmForceBehavior.WAIT
            return
        
        # Check for obstacles (walls) - avoid them
        if self.detect_obstacle():
            self.current_state = SwarmForceBehavior.AVOID_OBSTACLE
            return
        
        # Active search behavior: wander around looking for robots
        self.execute_search_wander()
    
    def calculate_f_vector(self):
        # ... implement the calculation of f based on sensor data and desired behaviors
        # - Use the robot's sensors to gather information about its neighbors and the environment.
        # - Calculate the proximal, alignment, and goal-seeking components of the f vector.
        # - Combine these components using the weights a, b, and c.
        
        a, b, c, d = 1, 0.1, 0.1, 0.3 # right now because keeping the proximity distance is most important it gets the most weight

        proximal_vector = scale(self.calculate_proximal_vector(), a)
        alignment_vector = scale(self.calculate_alignment_vector(), b)
        goal_vector = scale(self.calculate_goal_vector(), c)
        noise_vector = scale(self.calculate_noise_vector(), d)
        
        f_vector = add(proximal_vector, add(alignment_vector, add(goal_vector, noise_vector)))
        return f_vector

    def f_to_velocities(self, f_vector: list) -> list:
        _orientation = self.robot.robot.pose.theta
        v = f_vector[0] * cos(_orientation) + f_vector[1] * sin(_orientation)
        omega = f_vector[1] * cos(_orientation) - f_vector[0] * sin(_orientation)
    
        return v * LINEAR_GAIN, omega * ANGULAR_GAIN


    def calculate_proximal_vector(self)->tuple[float]:
        proximal_x, proximal_y = 0, 0

        for neighbor_pose, r in self.robot.read_robot_neighbors_pose():
            # Calculate vector from self to neighbor
            dx = neighbor_pose.x - self.estimated_pose.x
            dy = neighbor_pose.y - self.estimated_pose.y
            r = sqrt(dx**2 + dy**2)

            force_magnitude = 0.0
            if r > 0.01:  # Avoid division by zero
                # Standard Lennard-Jones force formula:
                # F(r) = 24 * epsilon * (2*(sigma/r)^12 - (sigma/r)^6) / r
                # When r < sigma: force is positive (repulsive)
                # When r > sigma: force is negative (attractive)
                force_magnitude = 24 * self.epsilon * ((2 * (self.sigma / r)**12) - ((self.sigma / r)**6)) / r
                
                # Normalize direction vector (points from self to neighbor)
                dx_normalized = dx / r
                dy_normalized = dy / r
                
                # Apply force: flip sign because dx/dy point toward neighbor
                # When force_magnitude > 0 (repulsive): -force pushes AWAY from neighbor
                # When force_magnitude < 0 (attractive): -force pulls TOWARD neighbor
                proximal_x += -force_magnitude * dx_normalized
                proximal_y += -force_magnitude * dy_normalized

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
    
    def execute_search_wander(self):
        """
        Active search behavior: moves forward with occasional random turns
        to explore the environment looking for other robots.
        """
        # Randomly turn occasionally to explore
        if self.time - self.last_search_turn_time > self.search_wander_interval:
            # Random turn direction
            import random
            turn_direction = random.choice([-1, 1])
            self._send_robot_commands(0.1, turn_direction * self.search_turn_angle)
            self.last_search_turn_time = self.time
        else:
            # Move forward while searching
            self.move_forward()
    
    def turn_to_avoid_obstacle(self, angle = None):
        """
        Turn away from obstacle. Calculates best direction to turn based on
        sensor readings to avoid the obstacle.
        """
        if not angle:
            # Find the direction with the most clearance
            forward_distances = self._forward_sensor_distances()
            # Get sensor indices for forward sensors (1-7)
            sensor_indices = list(range(1, min(8, len(self.proximity_sensor_distances))))
            
            # Find sensors that are NOT detecting robots (only walls/obstacles)
            non_robot_distances = []
            non_robot_indices = []
            for i, dist in enumerate(forward_distances):
                sensor_idx = sensor_indices[i] if i < len(sensor_indices) else i + 1
                if sensor_idx < len(self.robot_detection_sensor_array):
                    # Only consider obstacles (not robots)
                    if not self.robot_detection_sensor_array[sensor_idx]:
                        non_robot_distances.append(dist)
                        non_robot_indices.append(sensor_idx)
            
            if non_robot_distances:
                # Find the sensor with maximum distance (safest direction)
                max_dist_idx = non_robot_distances.index(max(non_robot_distances))
                best_sensor_idx = non_robot_indices[max_dist_idx]
                
                # Calculate turn angle based on sensor placement
                if best_sensor_idx < len(self.proximity_sensor_placements):
                    sensor_placement = self.proximity_sensor_placements[best_sensor_idx]
                    # Turn toward the sensor with most clearance
                    angle = sensor_placement.theta
                else:
                    # Default turn if we can't determine best direction
                    angle = pi / 4
            else:
                # Default turn if no obstacle sensors found
                angle = pi / 4
        
        self._send_robot_commands(0, angle)

    def detect_obstacle(self):
        """
        Detect obstacles (walls) but NOT robots.
        Only returns True if sensors detect something that is NOT a robot.
        """
        forward_distances = self._forward_sensor_distances()
        sensor_indices = list(range(1, min(8, len(self.proximity_sensor_distances))))
        
        # Check if any forward sensor detects an obstacle (not a robot)
        for i, dist in enumerate(forward_distances):
            if dist < D_DANGER:
                sensor_idx = sensor_indices[i] if i < len(sensor_indices) else i + 1
                # Only consider it an obstacle if it's NOT detecting a robot
                if sensor_idx < len(self.robot_detection_sensor_array):
                    if not self.robot_detection_sensor_array[sensor_idx]:
                        return True
        
        return False

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
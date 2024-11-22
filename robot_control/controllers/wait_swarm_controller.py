from math import atan2, sin, cos
from utils import math_util

class WaitSwarmController:
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.k_p_angular = 5.0  # Proportional gain for angular velocity control
        self.k_p_linear = 1.0   # Proportional gain for linear velocity control
        self.forward_speed = 0.1  # Base forward speed when distance is sufficient
        self.min_distance = 0.3  # Desired minimum distance between robots (in meters)

    def update_heading(self):
        """
        Update the robot's target heading based on the average heading of nearby robots.
        """
        swarm_orientations = self.supervisor.supervisor.robot_detection_sensor_array

        if any(swarm_orientations):  # Check if any sensors detect robots
            swarm_headings = [
                self.supervisor.proximity_sensor_placements()[i].theta
                for i, detected in enumerate(swarm_orientations) if detected
            ]
            avg_heading = atan2(
                sum(sin(heading) for heading in swarm_headings),
                sum(cos(heading) for heading in swarm_headings),
            )
            self.supervisor.supervisor.target_orientation = avg_heading

    def calculate_distance_correction(self):
        """
        Calculate velocity correction to maintain the minimum distance from other robots.
        """
        # Get distances from proximity sensors detecting robots
        proximity_distances = self.supervisor.proximity_sensor_distances()

        # Filter only the sensors detecting other robots
        robot_distances = [
            distance
            for distance, detected in zip(proximity_distances, self.supervisor.supervisor.robot_detection_sensor_array)
            if detected
        ]

        # If no robots detected, return the base forward speed
        if not robot_distances:
            return self.forward_speed

        # Find the closest robot's distance
        closest_distance = min(robot_distances)

        # If the closest robot is within the minimum distance, reduce forward speed
        if closest_distance < self.min_distance:
            speed_correction = -self.k_p_linear * (self.min_distance - closest_distance)
        else:
            speed_correction = self.forward_speed

        # Ensure the velocity is non-negative
        return max(speed_correction, 0.0)

    def execute(self):
        """
        Align with the swarm heading, maintain distance, and move forward.
        """
        # Update the swarm heading
        self.update_heading()

        # Get the target orientation from the supervisor
        theta_d = self.supervisor.get_target_orientation()

        # Get the current robot orientation
        theta = self.supervisor.estimated_pose().theta

        # Compute the heading error
        e = math_util.normalize_angle(theta_d - theta)

        # Compute angular velocity to align with swarm heading
        omega = self.k_p_angular * e

        # Compute forward velocity based on proximity correction
        v = self.calculate_distance_correction()

        # Set the control outputs
        self.supervisor.set_outputs(v, omega)

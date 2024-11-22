from math import atan2, sin, cos
# from robot_control.custom_behavior.behavior_controller_interface import BehaviorControllerInterface
from utils import math_util

class WaitSwarmController:
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.k_p_angular = 5.0  # Proportional gain for angular velocity control
        self.k_p_linear = 1.0   # Proportional gain for linear velocity control
        self.min_distance = 0.3  # Desired minimum distance between robots (in meters)
    
    def update_heading(self):
        pass

    def calculate_adjustment(self):
        """
        Calculate linear and angular velocity to maintain the minimum distance from other robots.
        """
        # Get distances and sensor angles of robots detected by proximity sensors
        proximity_distances = self.supervisor.proximity_sensor_distances()
        sensor_placements = self.supervisor.proximity_sensor_placements()

        # Filter only sensors detecting other robots
        robot_detections = [
            (distance, placement)
            for distance, placement, detected in zip(
                proximity_distances, sensor_placements, self.supervisor.supervisor.robot_detection_sensor_array
            )
            if detected
        ]

        # If no nearby robots are detected, stop
        if not robot_detections:
            return 0.0, 0.0  # No adjustment needed

        # Compute the net repulsive force to adjust position
        x_force = 0.0
        y_force = 0.0
        for distance, placement in robot_detections:
            if distance < self.min_distance:
                # Compute repulsion vector (pointing away from the detected robot)
                repulsion_strength = self.min_distance - distance
                x_force += repulsion_strength * cos(placement.theta)
                y_force += repulsion_strength * sin(placement.theta)

        # Convert the resultant force into linear and angular velocity
        target_angle = atan2(y_force, x_force)
        angular_velocity = self.k_p_angular * math_util.normalize_angle(target_angle)
        linear_velocity = self.k_p_linear * (x_force**2 + y_force**2)**0.5

        # Ensure the linear velocity does not exceed the robot's limits
        linear_velocity = max(0.0, min(linear_velocity, self.supervisor.v_max()))

        return linear_velocity, angular_velocity

    def execute(self):
        """
        Adjust position to maintain the desired distance without moving forward.
        """
        # Compute adjustment velocities
        v, omega = self.calculate_adjustment()

        # Set the control outputs
        self.supervisor.set_outputs(v, omega)

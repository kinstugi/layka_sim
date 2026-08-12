import utils.linalg2_util as linalg

VECTOR_LEN = 0.75  # Default length of the vectors for visualization

class SwarmForceBehaviorView:
    """
    Visualizes the swarm force supervisor in the simulation.

    Attributes:
        viewer (Viewer): The graphical viewer object.
        supervisor (Supervisor): The swarm supervisor being represented.
        robot_geometry (Geometry): The geometry of the robot for rendering.
        robot_estimated_traverse_path (list[Vector2]): A list of positions representing the robot's estimated traversed path.

    Methods:
        draw_supervisor_to_frame(): Draws the supervisor's vectors and robot trajectory to the current frame.
        _draw_force_vector(): Draws a single vector originating from the robot's position.
        _draw_trajectory(): Draws the robot's estimated traverse path.
    """

    def __init__(self, viewer, supervisor, robot_geometry):
        self.viewer = viewer
        self.supervisor = supervisor
        self.robot_geometry = robot_geometry
        self.robot_estimated_traverse_path = []  # To track robot's path

    def draw_supervisor_to_frame(self):
        # Update and draw robot trajectory
        self.robot_estimated_traverse_path.append(
            self.supervisor.estimated_pose.vposition()
        )
        self._draw_trajectory()

        # Draw vectors if visualization is enabled
        if self.viewer.show_invisibles:
            # Retrieve vectors from the supervisor
            f_vector = self.supervisor.calculate_f_vector()
            proximal_vector, alignment_vector, goal_vector, obstacle_avoidance_vector = (
                self.supervisor.calculate_individual_vectors()
            )

            # Draw individual force vectors
            self._draw_force_vector(proximal_vector, color="blue", alpha=0.5)
            self._draw_force_vector(alignment_vector, color="green", alpha=0.5)
            self._draw_force_vector(goal_vector, color="yellow", alpha=0.5)
            self._draw_force_vector(obstacle_avoidance_vector, color="orange", alpha=0.5)

            # Draw the resultant force vector
            self._draw_force_vector(f_vector, color="red", linewidth=0.01, alpha=0.8)

    def _draw_force_vector(self, vector, color="black", linewidth=0.005, alpha=1.0):
        """
        Draws a single force vector originating from the robot's position.

        Args:
            vector (Vector2): The force vector to draw.
            color (str): The color of the vector.
            linewidth (float): The thickness of the line.
            alpha (float): The transparency of the vector.
        """
        # Get robot position and heading
        robot_pos = self.supervisor.estimated_pose.vposition()

        # Scale the vector to standard length for visualization
        scaled_vector = linalg.scale(linalg.unit(vector), VECTOR_LEN)
        vector_line = [[0.0, 0.0], scaled_vector]

        # Translate the vector to the robot's position
        vector_line = linalg.translate_vectors(vector_line, robot_pos)

        # Draw the vector on the viewer
        self.viewer.current_frame.add_lines(
            [vector_line], color=color, linewidth=linewidth, alpha=alpha
        )

    def _draw_trajectory(self):
        """
        Draws the robot's estimated traverse path.
        """
        if len(self.robot_estimated_traverse_path) > 1:
            self.viewer.current_frame.add_lines(
                [self.robot_estimated_traverse_path],
                linewidth=0.005,
                color="red",
                alpha=0.5,
            )

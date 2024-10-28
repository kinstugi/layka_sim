import utils.linalg2_util as linalg

VECTOR_LEN = 0.75  # length of heading vector


class ForwardControllerView:
    def __init__(self, viewer, supervisor):
        self.viewer = viewer
        self.supervisor = supervisor

    # Draw a representation of the forward controller's heading to the frame
    def draw_forward_controller_to_frame(self):
        # Get the robot's current position and heading
        robot_pos, robot_theta = self.supervisor.estimated_pose.vunpack()

        # Create a forward vector in the robot's current heading direction
        forward_heading_vector = linalg.scale(
            linalg.unit([1.0, 0.0]), VECTOR_LEN
        )
        # Rotate the forward vector to match the robot's heading
        vector_line = [[0.0, 0.0], forward_heading_vector]
        vector_line = linalg.rotate_and_translate_vectors(
            vector_line, robot_theta, robot_pos
        )

        # Draw the forward heading vector on the viewer frame
        self.viewer.current_frame.add_lines(
            [vector_line], linewidth=0.02, color="blue", alpha=1.0
        )

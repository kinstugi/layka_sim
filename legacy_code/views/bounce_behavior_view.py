from views.controllers.forward_controller_view import ForwardControllerView
from robot_control.control_state import ControlState


class BounceBehaviorView:
    """
    Represents the visual representation of a bounce supervisor in the simulation.

    Attributes:
        viewer (Viewer): The graphical viewer object.
        supervisor (Supervisor): The supervisor object being represented.
        supervisor_state_machine (StateMachine): The supervisor's state machine.
        forward_controller_view (ForwardControllerView): The view for the forward (bounce) controller.
        robot_geometry (Geometry): The geometry of the robot.
        robot_estimated_traverse_path (list[Vector2]): A list of positions representing the robot's estimated traversed path.

    Methods:
        draw_supervisor_to_frame(): Draws the supervisor, its current state, and the bounce direction to the current frame.
        _draw_robot_state_estimate_to_frame(): Draws the supervisor's estimated robot state to the current frame.
        _draw_current_controller_to_frame(): Draws the current controller's state to the current frame.
    """

    def __init__(self, viewer, supervisor, robot_geometry):
        self.viewer = viewer
        self.supervisor = supervisor
        self.supervisor_state_machine = supervisor.state_machine

        # forward controller view for bounce behavior
        self.forward_controller_view = ForwardControllerView(viewer, supervisor)

        # additional information for rendering
        self.robot_geometry = robot_geometry  # robot geometry
        self.robot_estimated_traverse_path = []  # path taken by robot's internal image

    # draw a representation of the supervisor's internal state to the frame
    def draw_supervisor_to_frame(self):
        # update the estimated robot traverse path
        self.robot_estimated_traverse_path.append(
            self.supervisor.estimated_pose.vposition()
        )

        # draw the supervisor-generated data to frame if indicated
        if self.viewer.show_invisibles:
            self._draw_robot_state_estimate_to_frame()
            self._draw_current_controller_to_frame()

    def _draw_robot_state_estimate_to_frame(self):
        # draw the estimated position of the robot
        vertices = self.robot_geometry.get_transformation_to_pose(
            self.supervisor.estimated_pose
        ).vertices[:]
        vertices.append(vertices[0])  # close the drawn polygon
        self.viewer.current_frame.add_lines(
            [vertices], color="black", linewidth=0.0075, alpha=0.5
        )

        # draw the estimated traverse path of the robot
        self.viewer.current_frame.add_lines(
            [self.robot_estimated_traverse_path],
            linewidth=0.005,
            color="red",
            alpha=0.5,
        )

    # draw the current controller's state to the frame
    def _draw_current_controller_to_frame(self):
        current_state = self.supervisor_state_machine.current_state
        if current_state == ControlState.FORWARD:
            self.forward_controller_view.draw_forward_controller_to_frame()

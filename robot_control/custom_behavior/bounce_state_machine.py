from robot_control.control_state import ControlState
from utils import linalg2_util as linalg

# Distance thresholds for obstacle detection
D_CAUTION = 0.15  # meters from obstacle
D_DANGER = 0.15   # meters from obstacle

class BounceStateMachine:
    def __init__(self, supervisor):
        self.supervisor = supervisor

        # Initialize state
        self.transition_to_state_forward()

    def update_state(self):
        if self.current_state == ControlState.FORWARD:
            self.execute_state_forward()
        elif self.current_state == ControlState.BOUNCE:
            self.execute_state_bounce()
        else:
            raise Exception("Undefined state")

    # === STATE PROCEDURES ===
    def execute_state_forward(self):
        # If a danger condition is detected, transition to bounce state
        if self.condition_danger():
            self.transition_to_state_bounce()

    def execute_state_bounce(self):
        # Once the danger condition is resolved, return to forward movement
        if not self.condition_danger():
            self.transition_to_state_forward()

    # === STATE TRANSITIONS ===
    def transition_to_state_forward(self):
        self.current_state = ControlState.FORWARD
        self.supervisor.current_controller = self.supervisor.forward_controller

    def transition_to_state_bounce(self):
        self.current_state = ControlState.BOUNCE
        self._reflect_heading()
        self.supervisor.current_controller = self.supervisor.forward_controller

    # === CONDITIONS ===
    def condition_danger(self):
        # Check if any forward sensors detect an obstacle within the danger range
        return any(d < D_DANGER for d in self._forward_sensor_distances())

    # === HELPER METHODS ===
    def _forward_sensor_distances(self):
        # Assumes the forward sensors are from indices 1 to 7
        return self.supervisor.proximity_sensor_distances[1:7]

    def _reflect_heading(self):
        # Reflect the current heading based on detected obstacle
        current_heading = self.supervisor.estimated_pose.theta
        # Reflect heading by adding 180 degrees (pi radians) to change direction
        new_heading = (current_heading + 3.14159) % (2 * 3.14159)
        self.supervisor.estimated_pose.update_theta(new_heading)

    # === FOR DEBUGGING ===
    def _print_debug_info(self):
        print("\n ======== \n")
        print(
            "STATE: "
            + str(
                [
                    "Forward",
                    "Bounce",
                ][self.current_state]
                + "\n"
            )
        )
        print("CONDITIONS:")
        print("Danger: " + str(self.condition_danger()))

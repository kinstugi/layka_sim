from robot_control.custom_behavior.behavior_controller_interface import BehaviorControllerInterface


class ReverseController:
    def __init__(self, supervisor):
        # Bind the supervisor
        self.supervisor = supervisor

    def execute(self):
        # Get the maximum translational velocity
        v = -self.supervisor.v_max()  # Negative velocity for reverse motion

        # Set angular velocity to zero to maintain current heading
        omega = 0.0

        # Send the control outputs to the supervisor
        self.supervisor.set_outputs(v, omega)

        # === FOR DEBUGGING ===
        # self._print_vars(v, omega)

    def _print_vars(self, v, omega):
        print("\n\n")
        print("==============")
        print("OUTPUTS:")
        print("omega: " + str(omega))
        print("v    : " + str(v))

from robot_control.custom_behavior.swarm_behavior import SwarmBehavior


class SearchRobotsController:
    def __init__(self, supervisor: SwarmBehavior) -> None:
        self.supervisor = supervisor

        # gains
        self.kP = 5.0
        self.kI = 0.0
        self.kD = 0.0

        # stored values - for computing next results
        self.prev_time = 0.0
        self.prev_eP = 0.0
        self.prev_eI = 0.0
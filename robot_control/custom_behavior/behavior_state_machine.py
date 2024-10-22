# event parameters
from robot_control.custom_behavior.swarm_behavior import SwarmBehavior


D_STOP = 0.05  # meters from goal
D_CAUTION = 0.15  # meters from obstacle
D_DANGER = 0.04  # meters from obstacle

# progress margin
PROGRESS_EPSILON = 0.05

class BehaviorStateMachine:
    def __init__(self, supervisor: SwarmBehavior) -> None:
        self.supervisor = supervisor

        #set the initial state
        self.transition_to_search_state()
    
    def update_state(self):
        pass


    def transition_to_search_state(self):
        pass

    def transition_to_wait_state(self):
        pass

    def transition_to_depart_swarm_state(self):
        pass
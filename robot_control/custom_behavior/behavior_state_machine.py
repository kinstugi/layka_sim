# event parameters
from robot_control.control_state import ControlState
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
        if self.current_state == ControlState.SEARCH_ROBOTS:
            self.execute_state_search_for_robot()
        elif self.current_state == ControlState.WAIT_IN_SWARM:
            self.execute_state_wait_in_swarm()
        elif self.current_state == ControlState.DEPART_SWARM:
            self.execute_state_depart_swarm()
        elif self.current_state == ControlState.AVOID_OBSTACLES:
            self.execute_state_avoid_obstacle()


    def execute_state_search_for_robot(self):
        if self.condition_at_obstacle():
            pass
        elif self.condition_robots_nearby():
            self.transition_to_wait_state()
        elif self.condition_no_robots_nearby():
            self.transition_to_search_state()
        else:
            raise("Error state")

    def execute_state_avoid_obstacle(self):
        pass

    def execute_state_wait_in_swarm(self):
        pass

    def execute_state_depart_swarm(self):
        pass

    def execute_state_slide_left(self):
        pass

    def execute_state_slide_right(self):
        pass

    def transition_to_search_state(self):
        self.current_state = ControlState.SEARCH_ROBOTS
        self.supervisor.current_controller = self.supervisor.search_robots_controller

    def transition_to_wait_state(self):
        self.current_state = ControlState.WAIT_IN_SWARM
        self.supervisor.current_controller = self.supervisor.wait_swarm_controller

    def transition_to_depart_swarm_state(self):
        self.current_state = ControlState.DEPART_SWARM
        self.supervisor.current_controller = self.supervisor.depart_swarm_controller
    
    def transition_to_slide_left_state(self):
        self.current_state = ControlState.SLIDE_LEFT
        # maybe get best distance to goal but there is no goal
        self.supervisor.current_controller = self.supervisor.follow_wall_controller

    def transition_to_slide_right_state(self):
        self.current_state = ControlState.SLIDE_RIGHT
        # maybe get best distance to goal but there is no goal
        self.supervisor.current_controller = self.supervisor.follow_wall_controller
    
    # conditions for the robot
    def condition_no_robots_nearby(self):
        return False

    def condition_robots_nearby(self):
        return False
    
    def condition_at_obstacle(self):
        pass

    def condition_at_slide_left(self):
        pass

    def condition_at_slide_right(self):
        pass

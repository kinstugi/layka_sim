# event parameters
D_STOP = 0.05  # meters from goal
D_CAUTION = 0.15  # meters from obstacle
D_DANGER = 0.04  # meters from obstacle

# progress margin
PROGRESS_EPSILON = 0.05

class BehaviorStateMachine:
    def __init__(self, supervisor) -> None:
        self.supervisor = supervisor

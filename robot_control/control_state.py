# a simple enumeration of control states


class ControlState:
    AT_GOAL = 0
    GO_TO_GOAL = 1
    AVOID_OBSTACLES = 2
    GTG_AND_AO = 3
    SLIDE_LEFT = 4
    SLIDE_RIGHT = 5
    SEARCH_ROBOTS = 6
    WAIT_IN_SWARM = 7
    DEPART_SWARM = 8
    FORWARD = 9
    BOUNCE = 10

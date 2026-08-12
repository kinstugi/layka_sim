from models.physics import Physics


class World:
    """
    Represents the simulated world environment.

    Attributes:
        physics (Physics): The physics engine for the world.
        world_time (float): The current time in seconds.
        dt (float): The time step for simulation updates.
        supervisors (list[Supervisor]): A list of supervisors in the world.
        robots (list[Robot]): A list of robots in the world.
        obstacles (list[Obstacle]): A list of obstacles in the world.

    Methods:
        step(): Steps the simulation forward by one time step.
        add_robot(robot): Adds a robot to the world.
        add_obstacle(obstacle): Adds an obstacle to the world.
        colliders(): Returns a list of objects that might collide in the simulation.
        solids(): Returns a list of all solid objects in the world.
    """
    def __init__(self, dt=0.05):
        # initialize physics engine
        self.physics = Physics(self)

        # initialize world time
        self.world_time = 0.0  # seconds
        self.dt = dt  # seconds

        # initialize lists of world objects
        self.supervisors = []
        self.robots = []
        self.obstacles = []

    # step the simulation through one time interval
    def step(self):
        dt = self.dt

        # step all the robots
        for robot in self.robots:
            # step robot motion
            robot.step_motion(dt)

        # apply physics interactions
        self.physics.apply_physics()

        # NOTE: the supervisors must run last to ensure they are observing the "current"
        # world step all of the supervisors
        for supervisor in self.supervisors:
            supervisor.step(dt)

        # increment world time
        self.world_time += dt

    def add_robot(self, robot):
        self.robots.append(robot)
        self.supervisors.append(robot.supervisor)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    # return all objects in the world that might collide with other objects in the
    # world during simulation
    def colliders(self):
        # moving objects only
        return (
            self.robots
        )  # as obstacles are static we should not test them against each other

    # return all solids in the world
    def solids(self):
        return self.robots + self.obstacles

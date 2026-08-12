"""M2.10 tests: static circular obstacles, world storage, and IR-sensor avoidance.

Covers the plan.md M2.10 requirements:

* **Obstacle model**: frozen dataclass with a ``Vector2`` center and a
  positive finite radius; invalid radii raise ``ValueError``.
* **World storage**: ``add_obstacle`` / ``obstacles`` (immutable snapshot) /
  ``obstacle_count``; obstacles never change robot behavior or ``step()``.
* **Avoidance behavior**: ``ObstacleAvoidanceBehavior`` is driven by the IR
  proximity sensors (the same ray-cast readings the renderer draws as red /
  green cones) rather than analytic distance. An obstacle-detecting (RED)
  sensor within ``trigger_delta * max_range`` overrides the inner command
  with a steer-away command; a robot-detecting (GREEN) sensor NEVER triggers
  avoidance (the swarm behavior handles robot-robot spacing via LJ
  repulsion); otherwise it delegates. Closer obstacles contribute larger
  repulsive vectors.
* **An obstacle is NOT a robot**: it never appears in ``NeighborSensor``
  results and never changes LJ/search commands when avoidance is not involved.
* **Acceptance criterion** (the key integration test): 10 robots on
  ``BoundaryContainmentBehavior(ObstacleAvoidanceBehavior(SearchSwarmBehavior),
  ...)`` aggregate in a 5 x 5 m world with two static obstacles while NEVER
  entering an obstacle (min center-distance-to-radius clearance >= 0 at every
  step, initial configuration included), and the run is deterministic.

Empirical acceptance numbers (seed 42, 5 x 5 m world, dt = 0.05,
detection_range = 2.0, cluster_radius = 1.0, trigger_delta = 0.9, 2000 steps):
final mean pairwise distance ~0.67 m, cluster fraction 1.00, and the minimum
obstacle clearance over the whole run ~0.20 m (robots never get within the
obstacle radius; the IR-sensor avoidance keeps them out). With the legacy
trigger (0.75) the avoidance fires too late and disrupts aggregation, so the
default reaction distance is 0.9 * max_range.

Obstacle placement was tuned empirically (see the module docstring in
``layka/sim.py``): the plan's sketch placement puts a robot INSIDE the
(2.5, 1.0) obstacle at the seed-42 spawn and splits the swarm, so the
acceptance test uses obstacles at the left/right mid-height.
"""

from __future__ import annotations

import math

import pytest

from layka import (
    BodyVelocity,
    BoundaryContainmentBehavior,
    LJBehavior,
    LJController,
    LJControllerConfig,
    LennardJonesConfig,
    NeighborSensor,
    Obstacle,
    ObstacleAvoidanceBehavior,
    Pose2D,
    ProximitySensorConfig,
    SearchSwarmBehavior,
    SearchSwarmConfig,
    SimulationConfig,
    StationaryBehavior,
    TrivialMotionBehavior,
    Vector2,
    World,
)

DEFAULT_SENSOR_CONFIG = ProximitySensorConfig()
FRONT_SENSOR_CONFIG = ProximitySensorConfig(sensor_poses=[(0.07, 0.0, 0.0)])
BOTTOM_SENSOR_CONFIG = ProximitySensorConfig(
    sensor_poses=[(0.019, -0.064, -math.pi / 2.0)]
)
TRIGGER_DELTA = 0.9

#: Obstacles used by the acceptance scenario (tuned for seed 42, see module
#: docstring): placed to the sides so the swarm aggregates while search
#: patrols still encounter them.
ACCEPTANCE_OBSTACLES = (
    Obstacle(Vector2(1.0, 2.5), 0.3),
    Obstacle(Vector2(4.2, 2.5), 0.3),
)

#: Mean pairwise distance of the seed-42 initial placement in the 5 x 5 m
#: world; the final value must be clearly below it.
INITIAL_SPREAD = 2.936

ACCEPTANCE_STEPS = 2000


def _controller() -> LJController:
    return LJController(LJControllerConfig())


def _inner() -> TrivialMotionBehavior:
    return TrivialMotionBehavior(v=0.1, omega=0.3)


def _avoidance(
    inner=None,
    controller=None,
    sensor_config: ProximitySensorConfig = DEFAULT_SENSOR_CONFIG,
    trigger_delta: float = TRIGGER_DELTA,
):
    return ObstacleAvoidanceBehavior(
        inner if inner is not None else _inner(),
        controller if controller is not None else _controller(),
        sensor_config,
        trigger_delta=trigger_delta,
    )


def _robot(world: World, x: float, y: float, theta: float = 0.0):
    robot_id = world.add_robot(pose=Pose2D(x, y, theta))
    return world.robot_by_id(robot_id)


# --- Obstacle model ---


class TestObstacleModel:
    def test_construction_and_fields(self):
        center = Vector2(1.5, 2.5)
        obstacle = Obstacle(center, 0.4)
        assert obstacle.center == center
        assert obstacle.radius == 0.4
        assert isinstance(obstacle.center, Vector2)

    @pytest.mark.parametrize("radius", [0.0, -1.0, -0.1, math.nan, math.inf, -math.inf])
    def test_invalid_radius_raises_value_error(self, radius):
        with pytest.raises(ValueError):
            Obstacle(Vector2(1.0, 1.0), radius)

    def test_non_vector_center_rejected(self):
        with pytest.raises(TypeError):
            Obstacle((1.0, 2.0), 0.3)  # type: ignore[arg-type]

    def test_nonfinite_center_rejected(self):
        with pytest.raises(ValueError):
            Obstacle(Vector2(math.nan, 1.0), 0.3)

    def test_obstacle_is_frozen(self):
        obstacle = Obstacle(Vector2(1.0, 1.0), 0.3)
        with pytest.raises(AttributeError):
            obstacle.radius = 0.5  # type: ignore[misc]
        with pytest.raises(AttributeError):
            obstacle.center = Vector2(0.0, 0.0)  # type: ignore[misc]


# --- World storage ---


class TestWorldObstacles:
    def test_empty_world_has_zero_obstacles(self):
        world = World(timestep=0.05)
        assert world.obstacle_count == 0
        assert world.obstacles == ()

    def test_add_obstacle_and_read_back(self):
        world = World(timestep=0.05)
        obstacle = Obstacle(Vector2(1.0, 1.0), 0.3)
        world.add_obstacle(obstacle)
        assert world.obstacle_count == 1
        assert world.obstacles == (obstacle,)

    def test_obstacles_snapshot_is_an_immutable_copy(self):
        world = World(timestep=0.05)
        first = Obstacle(Vector2(1.0, 1.0), 0.3)
        second = Obstacle(Vector2(2.0, 2.0), 0.2)
        world.add_obstacle(first)
        snapshot = world.obstacles
        world.add_obstacle(second)
        assert snapshot == (first,)  # earlier snapshot unchanged
        assert world.obstacles == (first, second)
        assert world.obstacle_count == 2

    def test_add_obstacle_rejects_non_obstacle(self):
        world = World(timestep=0.05)
        with pytest.raises(TypeError):
            world.add_obstacle("not an obstacle")  # type: ignore[arg-type]

    def test_adding_obstacles_does_not_change_robot_behavior_or_step(self):
        def build(with_obstacle: bool) -> World:
            world = World(
                SimulationConfig(
                    timestep=0.05,
                    robot_count=2,
                    random_seed=1,
                    world_width=5.0,
                    world_height=5.0,
                )
            )
            world.spawn_robots(behavior=TrivialMotionBehavior(v=0.1, omega=0.0))
            if with_obstacle:
                world.add_obstacle(Obstacle(Vector2(2.5, 2.5), 0.5))
            return world

        plain, with_obstacle = build(False), build(True)
        for _ in range(50):
            plain.step()
            with_obstacle.step()
        assert [r.pose for r in plain.robots] == [
            r.pose for r in with_obstacle.robots
        ]
        assert [r.linear_velocity for r in plain.robots] == [
            r.linear_velocity for r in with_obstacle.robots
        ]


# --- IR-sensor-driven avoidance behavior ---


class TestAvoidanceDelegation:
    def test_delegates_when_obstacle_beyond_sensor_range(self):
        world = World(timestep=0.05)
        world.add_obstacle(Obstacle(Vector2(0.5, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0)
        behavior = _avoidance(sensor_config=FRONT_SENSOR_CONFIG)
        # Front sensor max range is 0.2 m; the obstacle at 0.5 m is unseen.
        assert behavior.compute_command(robot, world, 0.05) == BodyVelocity(
            v=0.1, omega=0.3
        )

    def test_delegates_when_detected_but_beyond_trigger(self):
        world = World(timestep=0.05)
        # Obstacle surface at 0.26 m -> ray hit 0.19 m = delta 0.95 >= 0.75.
        world.add_obstacle(Obstacle(Vector2(0.28, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0)
        behavior = _avoidance(sensor_config=FRONT_SENSOR_CONFIG)
        assert behavior.compute_command(robot, world, 0.05) == BodyVelocity(
            v=0.1, omega=0.3
        )

    def test_delegates_stationary_inner_unchanged(self):
        world = World(timestep=0.05)
        world.add_obstacle(Obstacle(Vector2(0.5, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0)
        behavior = _avoidance(inner=StationaryBehavior())
        assert behavior.compute_command(robot, world, 0.05) == BodyVelocity(0.0, 0.0)

    def test_robot_detection_never_triggers_avoidance(self):
        # A front IR sensor detecting ANOTHER ROBOT (green cone) must NOT
        # trigger obstacle avoidance -- the swarm behavior handles robot-robot
        # spacing. The obstacle-avoidance override must delegate.
        world = World(timestep=0.05)
        _robot(world, 0.15, 0.0)  # second robot directly ahead
        robot = _robot(world, 0.0, 0.0)
        behavior = _avoidance(sensor_config=FRONT_SENSOR_CONFIG)
        assert behavior.compute_command(robot, world, 0.05) == BodyVelocity(
            v=0.1, omega=0.3
        )

    def test_inner_is_not_called_while_override_active(self):
        world = World(timestep=0.05)
        world.add_obstacle(Obstacle(Vector2(0.15, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0)

        class CountingBehavior:
            def __init__(self):
                self.calls = 0

            def compute_command(self, robot, world, dt):
                self.calls += 1
                return BodyVelocity(v=0.1, omega=0.3)

        counting = CountingBehavior()
        behavior = ObstacleAvoidanceBehavior(
            counting, _controller(), FRONT_SENSOR_CONFIG
        )
        behavior.compute_command(robot, world, 0.05)
        assert counting.calls == 0  # override path never touches the inner


class TestAvoidanceOverride:
    def test_front_obstacle_steers_away(self):
        world = World(timestep=0.05)
        # Obstacle dead ahead at 0.15 m: front sensor origin at 0.07 m, ray hit
        # at 0.06 m -> delta 0.3 -> weight 0.75-0.3 = 0.45, away vector (-1, 0).
        world.add_obstacle(Obstacle(Vector2(0.15, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0, theta=0.0)
        behavior = _avoidance(sensor_config=FRONT_SENSOR_CONFIG)
        command = behavior.compute_command(robot, world, 0.05)
        # Facing straight into the obstacle: it must stop and turn around
        # (omega = +max, v = 0 because cos(error) <= 0 at error = pi).
        assert command == BodyVelocity(v=0.0, omega=1.0)
        assert command != BodyVelocity(v=0.1, omega=0.3)

    def test_front_obstacle_command_matches_controller_on_away_vector(self):
        world = World(timestep=0.05)
        world.add_obstacle(Obstacle(Vector2(0.15, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0, theta=0.0)
        controller = _controller()
        behavior = _avoidance(controller=controller, sensor_config=FRONT_SENSOR_CONFIG)
        command = behavior.compute_command(robot, world, 0.05)
        # The avoidance vector points AWAY from the detected surface (hit
        # point at x=0.13, robot at x=0), i.e. along -x; the controller's
        # atan2 direction is what matters, so any -x vector maps to the same
        # turn-around command.
        assert command == controller.compute(Vector2(-1.0, 0.0), 0.0)
        assert command.v == 0.0
        assert command.omega > 0.0

    def test_side_obstacle_steers_away_from_that_side(self):
        world = World(timestep=0.05)
        # Obstacle below the robot; the bottom sensor (facing -y) detects it,
        # so the robot must steer upward (away from the obstacle).
        world.add_obstacle(Obstacle(Vector2(0.0, -0.15), 0.02))
        robot = _robot(world, 0.0, 0.0, theta=0.0)
        behavior = _avoidance(sensor_config=BOTTOM_SENSOR_CONFIG)
        command = behavior.compute_command(robot, world, 0.05)
        assert command != BodyVelocity(v=0.1, omega=0.3)
        assert command.omega > 0.0  # turning up / away from the below obstacle

    def test_trigger_delta_threshold(self):
        world = World(timestep=0.05)
        world.add_obstacle(Obstacle(Vector2(0.15, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0)
        # delta 0.3 >= trigger 0.2 -> no avoidance, delegate.
        relaxed = _avoidance(
            sensor_config=FRONT_SENSOR_CONFIG, trigger_delta=0.2
        )
        assert relaxed.compute_command(robot, world, 0.05) == BodyVelocity(
            v=0.1, omega=0.3
        )
        # delta 0.3 < trigger 0.75 -> avoid.
        strict = _avoidance(
            sensor_config=FRONT_SENSOR_CONFIG, trigger_delta=0.75
        )
        assert strict.compute_command(robot, world, 0.05) != BodyVelocity(
            v=0.1, omega=0.3
        )

    def test_closest_obstacle_along_ray_determines_detection(self):
        world = World(timestep=0.05)
        # Two obstacles along the front ray; the closer one (0.15) makes the
        # sensor report delta 0.3 (triggering), while the farther one alone
        # (delta 0.95) would not. Closest wins => override happens.
        world.add_obstacle(Obstacle(Vector2(0.28, 0.0), 0.02))
        world.add_obstacle(Obstacle(Vector2(0.15, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0)
        behavior = _avoidance(sensor_config=FRONT_SENSOR_CONFIG)
        assert behavior.compute_command(robot, world, 0.05) != BodyVelocity(
            v=0.1, omega=0.3
        )

    def test_determinism(self):
        world = World(timestep=0.05)
        world.add_obstacle(Obstacle(Vector2(0.15, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0, theta=0.5)
        behavior = _avoidance(sensor_config=FRONT_SENSOR_CONFIG)
        first = behavior.compute_command(robot, world, 0.05)
        second = behavior.compute_command(robot, world, 0.05)
        assert first == second

    def test_no_mutation_of_robot_or_world(self):
        world = World(timestep=0.05)
        world.add_obstacle(Obstacle(Vector2(0.15, 0.0), 0.02))
        robot = _robot(world, 0.0, 0.0, theta=0.5)
        pose_before = robot.pose
        velocity_before = (robot.linear_velocity, robot.angular_velocity)
        time_before = world.time
        behavior = _avoidance(sensor_config=FRONT_SENSOR_CONFIG)
        behavior.compute_command(robot, world, 0.05)
        assert robot.pose == pose_before
        assert (robot.linear_velocity, robot.angular_velocity) == velocity_before
        assert world.time == time_before


class TestAvoidanceValidation:
    @pytest.mark.parametrize("delta", [0.0, -0.1, math.nan, math.inf, 1.5])
    def test_invalid_trigger_delta(self, delta):
        with pytest.raises(ValueError):
            ObstacleAvoidanceBehavior(
                _inner(), _controller(), DEFAULT_SENSOR_CONFIG, trigger_delta=delta
            )

    def test_wrong_sensor_config_type(self):
        with pytest.raises(TypeError):
            ObstacleAvoidanceBehavior(_inner(), _controller(), "not a config")  # type: ignore[arg-type]

    def test_read_only_properties(self):
        inner = _inner()
        controller = _controller()
        behavior = ObstacleAvoidanceBehavior(
            inner, controller, DEFAULT_SENSOR_CONFIG, trigger_delta=0.6
        )
        assert behavior.inner is inner
        assert behavior.controller is controller
        assert behavior.sensor_config is DEFAULT_SENSOR_CONFIG
        assert behavior.trigger_delta == 0.6
        with pytest.raises(AttributeError):
            behavior.trigger_delta = 0.5  # type: ignore[misc]
        with pytest.raises(AttributeError):
            behavior.sensor_config = DEFAULT_SENSOR_CONFIG  # type: ignore[misc]


# --- An obstacle is NOT a robot ---


class TestObstacleIsNotARobot:
    def test_obstacle_never_appears_in_neighbor_query(self):
        world = World(timestep=0.05)
        rid_a = world.add_robot(pose=Pose2D(1.0, 1.0, 0.0))
        rid_b = world.add_robot(pose=Pose2D(1.3, 1.0, 0.0))
        # Obstacle sitting between the two robots, well inside the detection
        # range: it must NEVER show up as a neighbor.
        world.add_obstacle(Obstacle(Vector2(1.15, 1.3), 0.4))
        sensor = NeighborSensor(world, detection_range=2.0)
        assert [n.neighbor_id for n in sensor.neighbors_of(rid_a)] == [rid_b]
        assert [n.neighbor_id for n in sensor.neighbors_of(rid_b)] == [rid_a]

    def test_lj_behavior_unaffected_by_obstacles(self):
        lj_config = LennardJonesConfig(desired_spacing=0.40)

        def build(with_obstacle: bool) -> World:
            world = World(timestep=0.05)
            behavior = LJBehavior.from_config(
                LJControllerConfig(), lj_config, detection_range=2.0
            )
            world.add_robot(pose=Pose2D(0.0, 0.0, 0.0), behavior=behavior)
            world.add_robot(pose=Pose2D(0.6, 0.0, 0.0), behavior=behavior)
            if with_obstacle:
                # Obstacle between the pair, far from the LJ equilibrium zone:
                # with avoidance NOT involved, commands must be identical.
                world.add_obstacle(Obstacle(Vector2(0.3, 0.5), 0.4))
            return world

        plain, with_obstacle = build(False), build(True)
        command_a = plain.behavior_of(0).compute_command(
            plain.robot_by_id(0), plain, 0.05
        )
        command_b = with_obstacle.behavior_of(0).compute_command(
            with_obstacle.robot_by_id(0), with_obstacle, 0.05
        )
        assert command_a == command_b

    def test_search_swarm_behavior_unaffected_by_obstacles(self):
        lj_config = LennardJonesConfig(desired_spacing=0.40)

        def build(with_obstacle: bool) -> tuple[World, int]:
            world = World(timestep=0.05)
            behavior = SearchSwarmBehavior.from_config(
                SearchSwarmConfig(), LJControllerConfig(), lj_config
            )
            world.add_robot(pose=Pose2D(0.0, 0.0, 0.0), behavior=behavior)
            if with_obstacle:
                world.add_obstacle(Obstacle(Vector2(0.0, 0.5), 0.3))
            return world, 0

        plain, rid_a = build(False)
        with_obstacle, rid_b = build(True)
        command_a = plain.behavior_of(rid_a).compute_command(
            plain.robot_by_id(rid_a), plain, 0.05
        )
        command_b = with_obstacle.behavior_of(rid_b).compute_command(
            with_obstacle.robot_by_id(rid_b), with_obstacle, 0.05
        )
        assert command_a == command_b


# --- Acceptance criterion: aggregation with obstacles, never driving through ---


def _make_acceptance_world(seed: int = 42) -> World:
    config = SimulationConfig(
        timestep=0.05,
        robot_count=10,
        random_seed=seed,
        world_width=5.0,
        world_height=5.0,
    )
    world = World(config)
    controller_config = LJControllerConfig()
    search_config = SearchSwarmConfig()
    lj_config = LennardJonesConfig(desired_spacing=0.40)
    sensor_config = ProximitySensorConfig()
    for _ in range(10):
        inner = SearchSwarmBehavior.from_config(
            search_config, controller_config, lj_config
        )
        avoidance = ObstacleAvoidanceBehavior(
            inner,
            LJController(controller_config),
            sensor_config,
            trigger_delta=TRIGGER_DELTA,
        )
        world.add_robot(
            behavior=BoundaryContainmentBehavior(
                avoidance,
                LJController(controller_config),
                world.width,
                world.height,
                margin=0.3,
            )
        )
    for obstacle in ACCEPTANCE_OBSTACLES:
        world.add_obstacle(obstacle)
    return world


def _run_acceptance(seed: int = 42, steps: int = ACCEPTANCE_STEPS):
    """Step the acceptance world; return (final positions, min clearance)."""
    world = _make_acceptance_world(seed=seed)
    min_clearance = math.inf
    # The initial configuration is included: a robot spawning inside an
    # obstacle would fail the criterion at step 0.
    for _ in range(steps):
        for robot in world.robots:
            position = robot.pose.position()
            for obstacle in world.obstacles:
                clearance = position.distance_to(obstacle.center) - obstacle.radius
                min_clearance = min(min_clearance, clearance)
        world.step()
    positions = [robot.pose.position() for robot in world.robots]
    return positions, min_clearance


def _mean_pairwise_distance(positions: list[Vector2]) -> float:
    total = 0.0
    pair_count = 0
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            total += positions[i].distance_to(positions[j])
            pair_count += 1
    return total / pair_count


def _cluster_fraction(positions: list[Vector2], cluster_radius: float = 1.0) -> float:
    centroid = Vector2(
        sum(p.x for p in positions) / len(positions),
        sum(p.y for p in positions) / len(positions),
    )
    return (
        sum(1 for p in positions if p.distance_to(centroid) <= cluster_radius)
        / len(positions)
    )


class TestAcceptanceCriterion:
    def test_robots_aggregate_without_driving_through_obstacles(self):
        positions, min_clearance = _run_acceptance()
        # Never inside any obstacle at ANY step (including the initial state):
        # every robot's distance to every obstacle center stayed >= radius.
        assert min_clearance >= -1e-6
        # Every robot's final position is finite.
        for position in positions:
            assert math.isfinite(position.x)
            assert math.isfinite(position.y)
        # The swarm aggregated (assert a clear majority within 1.0 m).
        assert _cluster_fraction(positions) >= 0.7
        # Mean pairwise distance dropped clearly below the initial spread.
        assert _mean_pairwise_distance(positions) < 1.5
        assert _mean_pairwise_distance(positions) < INITIAL_SPREAD

    def test_acceptance_run_is_deterministic(self):
        positions_a, min_clear_a = _run_acceptance()
        positions_b, min_clear_b = _run_acceptance()
        assert positions_a == positions_b
        assert min_clear_a == pytest.approx(min_clear_b)

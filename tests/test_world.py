"""Tests for the minimal multi-robot world (M1.6).

A world must hold at least 10 robots, update every robot exactly once per
``step()``, keep spawning deterministic under a seed, and expose read-only
accessors. Headless and deterministic (no randomness outside the seeded
spawning RNG).
"""

import math

import pytest

from layka import (
    BodyVelocity,
    Pose2D,
    RobotState,
    SimulationClock,
    SimulationConfig,
    StationaryBehavior,
    TrivialMotionBehavior,
    World,
    integrate_pose,
)


def _config(
    *, count: int = 10, timestep: float = 0.1, seed: int | None = None
) -> SimulationConfig:
    return SimulationConfig(timestep=timestep, robot_count=count, random_seed=seed)


class CountingBehavior:
    """Test-only behavior that records how many times it was called."""

    def __init__(self) -> None:
        self.calls = 0

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        self.calls += 1
        return BodyVelocity(v=0.0, omega=0.0)


# --- spawning ---


def test_spawn_creates_ten_robots_with_unique_ids_and_poses():
    world = World(_config(count=10, seed=42))
    ids = world.spawn_robots()
    assert len(world) == 10
    assert world.robot_count == 10
    assert ids == list(range(10))
    assert [r.robot_id for r in world.robots] == list(range(10))
    assert len({r.robot_id for r in world.robots}) == 10
    for robot in world.robots:
        assert isinstance(robot.pose, Pose2D)


def test_spawn_defaults_count_to_config_robot_count():
    world = World(_config(count=25, seed=3))
    world.spawn_robots()
    assert len(world) == 25
    assert [r.robot_id for r in world.robots] == list(range(25))


def test_each_robot_gets_a_behavior_and_zero_initial_velocity():
    world = World(_config(count=10, seed=42))
    world.spawn_robots()
    for robot in world.robots:
        assert isinstance(world.behavior_of(robot.robot_id), StationaryBehavior)
        assert robot.linear_velocity == 0.0
        assert robot.angular_velocity == 0.0


def test_add_robot_with_explicit_pose_assigns_next_id():
    world = World(_config(count=2, seed=42))
    world.spawn_robots()
    pose = Pose2D(1.0, 2.0, 0.5)
    robot_id = world.add_robot(pose, behavior=TrivialMotionBehavior(v=0.3))
    assert robot_id == 2
    assert world.robot_by_id(2).pose == pose
    assert isinstance(world.behavior_of(2), TrivialMotionBehavior)
    assert len(world) == 3


def test_spawn_rejects_negative_and_nonint_counts():
    world = World(_config(count=0))
    with pytest.raises(ValueError):
        world.spawn_robots(-1)
    with pytest.raises(ValueError):
        world.spawn_robots(2.5)


# --- step semantics ---


def test_step_advances_exactly_one_timestep():
    dt = 0.1
    world = World(_config(count=10, timestep=dt, seed=1))
    world.spawn_robots()
    world.step()
    assert world.time == pytest.approx(dt)
    assert world.step_count == 1
    assert world.dt == pytest.approx(dt)
    assert world.timestep == pytest.approx(dt)


def test_step_sets_velocity_from_behavior_and_integrates_pose():
    v, dt = 0.2, 0.1
    world = World(_config(count=10, timestep=dt, seed=7))
    world.spawn_robots(behavior=TrivialMotionBehavior(v=v, omega=0.0))
    before = [r.pose for r in world.robots]
    world.step()
    for before_pose, robot in zip(before, world.robots):
        assert robot.linear_velocity == pytest.approx(v)
        assert robot.angular_velocity == pytest.approx(0.0)
        assert robot.pose == integrate_pose(before_pose, v, 0.0, dt)


def test_stationary_robots_remain_stationary_for_n_steps():
    n_steps, dt = 25, 0.1
    world = World(_config(count=10, timestep=dt, seed=5))
    world.spawn_robots()  # default StationaryBehavior
    initial = [r.pose for r in world.robots]
    for _ in range(n_steps):
        world.step()
    assert world.time == pytest.approx(n_steps * dt)
    assert world.step_count == n_steps
    assert [r.pose for r in world.robots] == initial
    for robot in world.robots:
        assert robot.linear_velocity == 0.0
        assert robot.angular_velocity == 0.0


def test_trivial_straight_motion_advances_x_along_heading():
    """omega=0: after N steps each robot's x advances by v*N*dt along its
    (constant) heading; matches the closed form exactly within float error."""
    v, dt, n_steps = 0.15, 0.1, 30
    world = World(_config(count=10, timestep=dt, seed=11))
    world.spawn_robots(behavior=TrivialMotionBehavior(v=v, omega=0.0))
    initial = [r.pose for r in world.robots]
    for _ in range(n_steps):
        world.step()
    assert world.step_count == n_steps
    for start, robot in zip(initial, world.robots):
        dx = v * n_steps * dt * math.cos(start.theta)
        dy = v * n_steps * dt * math.sin(start.theta)
        assert robot.pose.x == pytest.approx(start.x + dx)
        assert robot.pose.y == pytest.approx(start.y + dy)
        assert robot.pose.theta == pytest.approx(start.theta)


def test_trivial_motion_trajectory_matches_loop_of_integrate_pose():
    """The world's per-step integration is exactly the repeated
    ``integrate_pose`` sequence (same float ops), so poses must be identical."""
    v, omega, dt, n_steps = 0.1, 0.4, 0.05, 40
    world = World(_config(count=10, timestep=dt, seed=3))
    world.spawn_robots(behavior=TrivialMotionBehavior(v=v, omega=omega))
    initial = [r.pose for r in world.robots]
    for _ in range(n_steps):
        world.step()
    for start, robot in zip(initial, world.robots):
        expected = start
        for _ in range(n_steps):
            expected = integrate_pose(expected, v, omega, dt)
        assert robot.pose == expected


def test_every_robot_updated_exactly_once_per_step():
    """A shared counting behavior sees exactly N calls per step (no robot is
    updated twice, none is skipped)."""
    n = 10
    counter = CountingBehavior()
    world = World(_config(count=n, seed=2))
    world.spawn_robots(behavior=counter)
    assert counter.calls == 0
    for _ in range(5):
        world.step()
    assert counter.calls == n * 5
    assert world.step_count == 5


# --- determinism under a seed ---


def test_same_seed_spawns_identical_robot_configurations():
    a = World(_config(count=10, seed=42))
    b = World(_config(count=10, seed=42))
    a.spawn_robots()
    b.spawn_robots()
    assert [r.pose for r in a.robots] == [r.pose for r in b.robots]


def test_different_seed_spawns_different_configurations():
    a = World(_config(count=10, seed=42))
    c = World(_config(count=10, seed=43))
    a.spawn_robots()
    c.spawn_robots()
    assert [r.pose for r in a.robots] != [r.pose for r in c.robots]


def test_seeded_worlds_produce_identical_trajectories():
    def build() -> World:
        world = World(_config(count=10, seed=42))
        world.spawn_robots(behavior=TrivialMotionBehavior(v=0.1, omega=0.5))
        return world

    a, b = build(), build()
    for _ in range(50):
        a.step()
        b.step()
    assert a.time == pytest.approx(b.time)
    assert a.step_count == b.step_count == 50
    assert [r.pose for r in a.robots] == [r.pose for r in b.robots]
    assert [r.linear_velocity for r in a.robots] == [
        r.linear_velocity for r in b.robots
    ]


def test_world_without_config_is_deterministic_under_explicit_seed():
    a = World(timestep=0.1, seed=7)
    b = World(timestep=0.1, seed=7)
    a.spawn_robots(10)
    b.spawn_robots(10)
    assert [r.pose for r in a.robots] == [r.pose for r in b.robots]


# --- accessors ---


def test_robot_by_id_returns_correct_robot_and_raises_for_unknown():
    world = World(_config(count=10, seed=42))
    world.spawn_robots()
    for robot in world.robots:
        assert world.robot_by_id(robot.robot_id) is robot
    with pytest.raises(KeyError):
        world.robot_by_id(10_000)


def test_time_and_step_count_are_read_only_properties():
    world = World(_config(count=3, timestep=0.05, seed=1))
    assert isinstance(world.clock, SimulationClock)
    assert world.time == 0.0
    assert world.step_count == 0
    assert world.dt == pytest.approx(0.05)
    assert world.timestep == pytest.approx(0.05)
    with pytest.raises(AttributeError):
        world.time = 1.0  # type: ignore[misc]
    with pytest.raises(AttributeError):
        world.step_count = 1  # type: ignore[misc]


def test_config_and_timestep_together_raise():
    with pytest.raises(ValueError):
        World(_config(count=2), timestep=0.1)


def test_trivial_motion_behavior_rejects_nonfinite_velocities():
    with pytest.raises(ValueError):
        TrivialMotionBehavior(v=math.nan)
    with pytest.raises(ValueError):
        TrivialMotionBehavior(v=0.1, omega=math.inf)


# --- acceptance criterion: a scene of >= 10 robots simulates ---


def test_scene_of_twelve_robots_simulates_for_fixed_steps():
    world = World(_config(count=12, timestep=0.05, seed=9))
    world.spawn_robots(behavior=TrivialMotionBehavior(v=0.1, omega=0.2))
    for _ in range(100):
        world.step()
    assert world.step_count == 100
    assert world.time == pytest.approx(100 * 0.05)
    assert all(math.isfinite(r.pose.x) and math.isfinite(r.pose.y) for r in world.robots)

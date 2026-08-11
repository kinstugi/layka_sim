"""Unit tests for M2.9: the SEARCH <-> SWARM state-machine behavior.

Covers the plan.md M2.9 acceptance criteria plus the task requirements: the
SEARCH patrol pattern (forward -> turn -> forward -> ... with timing pinned by
``dt`` accumulation), the SEARCH -> SWARM and SWARM -> SEARCH transitions, the
read-only ``state`` property matching the emitted command, config validation,
determinism, non-mutation, and behavior in an empty/single-robot world.
Headless and deterministic: all poses are explicit and no RNG is used.
"""

import math

import pytest
from pydantic import ValidationError

from layka.behavior import StationaryBehavior
from layka.config import LennardJonesConfig
from layka.kinematics import BodyVelocity
from layka.lj_controller import LJControllerConfig
from layka.pose import Pose2D
from layka.search_behavior import SEARCH, SWARM, SearchSwarmBehavior, SearchSwarmConfig
from layka.world import World

#: Defaults: search_velocity 0.1, forward_interval 1.0, turn_duration 0.5,
#: turn_rate 1.0, detection_range 2.0.
DEFAULT = SearchSwarmConfig()


def make_search_config(**overrides: float) -> SearchSwarmConfig:
    kwargs: dict[str, float] = {}
    kwargs.update(overrides)
    return SearchSwarmConfig(**kwargs)


def make_lj_config(**overrides: object) -> LennardJonesConfig:
    kwargs: dict[str, object] = {"desired_spacing": 0.40}
    kwargs.update(overrides)
    return LennardJonesConfig(**kwargs)


def make_behavior(**overrides: float) -> SearchSwarmBehavior:
    return SearchSwarmBehavior.from_config(
        make_search_config(**overrides),
        LJControllerConfig(),
        make_lj_config(),
    )


def _lone_robot_world(behavior: SearchSwarmBehavior) -> tuple[World, int]:
    world = World(seed=1)
    robot_id = world.add_robot(Pose2D(0.0, 0.0, 0.0), behavior=behavior)
    return world, robot_id


def _two_robot_world(
    behavior: SearchSwarmBehavior, neighbor_pose: Pose2D
) -> tuple[World, int, int]:
    world = World(seed=1)
    self_id = world.add_robot(Pose2D(0.0, 0.0, 0.0), behavior=behavior)
    other_id = world.add_robot(neighbor_pose, behavior=StationaryBehavior())
    return world, self_id, other_id


# --- SEARCH: move forward when there are no neighbors ---


def test_search_moves_forward_with_no_neighbors():
    behavior = make_behavior()
    world, robot_id = _lone_robot_world(behavior)
    robot = world.robot_by_id(robot_id)
    for _ in range(5):
        command = behavior.compute_command(robot, world, dt=0.1)
        assert command == BodyVelocity(DEFAULT.search_velocity, 0.0)
        assert behavior.state == SEARCH


def test_lone_robot_in_empty_world_is_searching():
    behavior = make_behavior()
    world, robot_id = _lone_robot_world(behavior)
    robot = world.robot_by_id(robot_id)
    command = behavior.compute_command(robot, world, dt=world.dt)
    assert behavior.state == SEARCH
    assert command == BodyVelocity(DEFAULT.search_velocity, 0.0)


def test_robots_outside_detection_range_do_not_trigger_swarm():
    behavior = make_behavior()  # detection_range 2.0
    world, self_id, _ = _two_robot_world(
        behavior, Pose2D(10.0, 0.0, 0.0)  # far outside detection range
    )
    robot = world.robot_by_id(self_id)
    command = behavior.compute_command(robot, world, dt=0.1)
    assert behavior.state == SEARCH
    assert command == BodyVelocity(DEFAULT.search_velocity, 0.0)


# --- SEARCH: deterministic patrol pattern (forward -> turn -> forward) ---


def test_search_turns_after_forward_interval_and_alternates_phases():
    # Defaults: forward_interval 1.0, turn_duration 0.5, turn_rate 1.0.
    # dt = 0.25 (exactly representable) so accumulation is exact: the SEARCH
    # time after call k is 0.25 * k, so
    #   calls 1..3 (elapsed 0.25..0.75)      -> forward
    #   calls 4..5 (elapsed 1.0..1.25)       -> turn in place
    #   call 6     (elapsed 1.5 -> phase 0.0) -> forward again
    behavior = make_behavior()
    world, robot_id = _lone_robot_world(behavior)
    robot = world.robot_by_id(robot_id)
    commands = [
        behavior.compute_command(robot, world, dt=0.25) for _ in range(6)
    ]
    forward = BodyVelocity(DEFAULT.search_velocity, 0.0)
    turning = BodyVelocity(0.0, DEFAULT.turn_rate)
    assert commands[:3] == [forward] * 3
    assert commands[3:5] == [turning] * 2
    assert commands[5] == forward


def test_turn_direction_is_positive_counterclockwise():
    # A positive omega is a CCW turn (math convention, theta CCW positive).
    behavior = make_behavior()
    world, robot_id = _lone_robot_world(behavior)
    robot = world.robot_by_id(robot_id)
    for _ in range(4):  # accumulate 4 * 0.25 = 1.0 s (forward_interval)
        behavior.compute_command(robot, world, dt=0.25)
    command = behavior.compute_command(robot, world, dt=0.25)  # elapsed 1.25
    assert command == BodyVelocity(0.0, DEFAULT.turn_rate)
    assert command.omega > 0.0


# --- transitions: SEARCH <-> SWARM ---


def test_search_to_swarm_when_neighbor_detected():
    behavior = make_behavior()  # detection_range 2.0, LJ desired_spacing 0.4
    world, self_id, _ = _two_robot_world(behavior, Pose2D(0.5, 0.0, 0.0))
    robot = world.robot_by_id(self_id)
    assert behavior.state == SEARCH
    command = behavior.compute_command(robot, world, dt=world.dt)
    assert behavior.state == SWARM
    # Neighbor at 0.5 > r_eq = 0.4 ahead: attractive, heading aligned ->
    # full speed straight ahead (the M2.6 LJ controller command).
    assert command.v == pytest.approx(LJControllerConfig().max_linear_velocity)
    assert command.omega == pytest.approx(0.0, abs=1e-12)


def test_swarm_to_search_when_neighbors_gone_then_back_to_swarm():
    behavior = make_behavior()
    world, self_id, other_id = _two_robot_world(
        behavior, Pose2D(0.5, 0.0, 0.0)
    )
    robot = world.robot_by_id(self_id)
    other = world.robot_by_id(other_id)

    command = behavior.compute_command(robot, world, dt=world.dt)
    assert behavior.state == SWARM
    assert command.v > 0.0  # LJ attraction

    other.pose = Pose2D(10.0, 10.0, 0.0)  # move out of detection range
    command = behavior.compute_command(robot, world, dt=0.1)
    assert behavior.state == SEARCH
    # Freshly re-entered SEARCH always starts with the forward phase.
    assert command == BodyVelocity(DEFAULT.search_velocity, 0.0)

    other.pose = Pose2D(0.5, 0.0, 0.0)  # back in range
    command = behavior.compute_command(robot, world, dt=world.dt)
    assert behavior.state == SWARM
    assert command.v == pytest.approx(LJControllerConfig().max_linear_velocity)
    assert command.omega == pytest.approx(0.0, abs=1e-12)


# --- state correctness ---


def test_state_matches_the_emitted_command():
    behavior = make_behavior()
    world, self_id, other_id = _two_robot_world(
        behavior, Pose2D(0.5, 0.0, 0.0)
    )
    robot = world.robot_by_id(self_id)
    other = world.robot_by_id(other_id)

    behavior.compute_command(robot, world, dt=0.1)
    assert behavior.state == SWARM

    other.pose = Pose2D(10.0, 0.0, 0.0)
    command = behavior.compute_command(robot, world, dt=0.1)
    assert behavior.state == SEARCH
    assert command == BodyVelocity(DEFAULT.search_velocity, 0.0)


def test_initial_state_is_search():
    behavior = make_behavior()
    assert behavior.state == SEARCH


# --- phase timing integrates dt, not wall-clock ---


def test_phase_timing_same_phase_for_same_total_dt():
    def commands_after(chunks: list[float]) -> tuple[BodyVelocity, str]:
        behavior = make_behavior()
        world, robot_id = _lone_robot_world(behavior)
        robot = world.robot_by_id(robot_id)
        last: BodyVelocity | None = None
        for chunk in chunks:
            last = behavior.compute_command(robot, world, dt=chunk)
        assert last is not None
        return last, behavior.state

    # 2 x 0.25 vs 1 x 0.5: same accumulated 0.5 s -> same forward phase.
    assert commands_after([0.25, 0.25]) == commands_after([0.5])
    # 4 x 0.25 vs 1 x 1.0: same accumulated 1.0 s -> same turn phase.
    assert commands_after([0.25] * 4) == commands_after([1.0])


def test_dt_accumulation_matches_sequence_of_small_steps():
    chunked = make_behavior()
    big = make_behavior()
    world_chunked, id_c = _lone_robot_world(chunked)
    world_big, id_b = _lone_robot_world(big)
    robot_c = world_chunked.robot_by_id(id_c)
    robot_b = world_big.robot_by_id(id_b)
    for _ in range(6):  # 6 * 0.25 = 1.5 s of SEARCH time
        chunked.compute_command(robot_c, world_chunked, dt=0.25)
    for _ in range(3):  # 3 * 0.5 = 1.5 s of SEARCH time
        big.compute_command(robot_b, world_big, dt=0.5)
    # Both accumulated 1.5 s (phase time 0.0 -> forward again).
    assert chunked.compute_command(robot_c, world_chunked, dt=0.25) == (
        big.compute_command(robot_b, world_big, dt=0.25)
    )
    assert chunked.state == big.state == SEARCH


# --- determinism ---


def test_identical_sequences_produce_identical_commands():
    def run_sequence(behavior: SearchSwarmBehavior) -> list[BodyVelocity]:
        world, robot_id = _lone_robot_world(behavior)
        robot = world.robot_by_id(robot_id)
        return [
            behavior.compute_command(robot, world, dt=0.1) for _ in range(20)
        ]

    assert run_sequence(make_behavior()) == run_sequence(make_behavior())


# --- config validation ---


@pytest.mark.parametrize("bad", [0.0, -1.0, math.inf, -math.inf, math.nan])
@pytest.mark.parametrize(
    "field",
    [
        "search_velocity",
        "forward_interval",
        "turn_duration",
        "turn_rate",
        "detection_range",
    ],
)
def test_all_search_config_fields_must_be_positive_and_finite(field, bad):
    with pytest.raises(ValidationError):
        SearchSwarmConfig(**{field: bad})


def test_search_config_defaults():
    assert DEFAULT.search_velocity == pytest.approx(0.1)
    assert DEFAULT.forward_interval == pytest.approx(1.0)
    assert DEFAULT.turn_duration == pytest.approx(0.5)
    assert DEFAULT.turn_rate == pytest.approx(1.0)
    assert DEFAULT.detection_range == pytest.approx(2.0)


def test_behavior_exposes_readonly_config():
    config = make_search_config()
    behavior = SearchSwarmBehavior.from_config(
        config, LJControllerConfig(), make_lj_config()
    )
    assert behavior.config is config


# --- non-mutation ---


def test_compute_command_does_not_mutate_robot_state():
    behavior = make_behavior()
    world, self_id, other_id = _two_robot_world(
        behavior, Pose2D(0.5, 0.0, 0.0)
    )
    robot = world.robot_by_id(self_id)
    pose_before = robot.pose
    linear_before = robot.linear_velocity
    angular_before = robot.angular_velocity

    behavior.compute_command(robot, world, dt=0.1)
    assert robot.pose == pose_before
    assert robot.linear_velocity == linear_before
    assert robot.angular_velocity == angular_before

    # SEARCH path also leaves the robot untouched.
    world.robot_by_id(other_id).pose = Pose2D(10.0, 10.0, 0.0)
    behavior.compute_command(robot, world, dt=0.1)
    assert robot.pose == pose_before
    assert robot.linear_velocity == linear_before
    assert robot.angular_velocity == angular_before


def test_invalid_dt_is_rejected():
    behavior = make_behavior()
    world, robot_id = _lone_robot_world(behavior)
    robot = world.robot_by_id(robot_id)
    for bad_dt in (0.0, -0.1, math.nan, math.inf):
        with pytest.raises(ValueError):
            behavior.compute_command(robot, world, dt=bad_dt)

"""Tests for the neighbor query abstraction (M1.7).

Covers the acceptance criteria: robots inside/outside range, multiple
neighbors, self-exclusion (never detect yourself, even co-located), exact
inclusive boundary behavior, the world-frame relative-position convention,
relative velocity, validation, non-mutation, and determinism. Headless and
deterministic: all poses are given explicitly, so no RNG is involved.
"""

import math

import pytest

from layka import NeighborSensor, Pose2D, Vector2, World


def _world_with_poses(poses: list[Pose2D]) -> World:
    """Build a world whose robot ``i`` sits at ``poses[i]``."""
    world = World()
    for pose in poses:
        world.add_robot(pose)
    return world


# --- acceptance: inside / outside / multiple neighbors ---


def test_robot_inside_range_is_returned_with_correct_geometry():
    world = _world_with_poses(
        [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)]
    )
    sensor = NeighborSensor(world, detection_range=1.5)
    (neighbor,) = sensor.neighbors_of(0)
    assert neighbor.neighbor_id == 1
    assert neighbor.relative_position == Vector2(1.0, 0.0)
    assert neighbor.distance == pytest.approx(1.0)


def test_robot_outside_range_is_not_returned():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(2.0, 0.0, 0.0),
            Pose2D(3.0, 4.0, 0.0),
        ]
    )
    sensor = NeighborSensor(world, detection_range=1.5)
    assert sensor.neighbors_of(0) == []


def test_single_robot_world_has_no_neighbors():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0)])
    sensor = NeighborSensor(world, detection_range=1.0)
    assert sensor.neighbors_of(0) == []


def test_multiple_neighbors_all_returned_in_ascending_id_order():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, 0.0),  # 0: query
            Pose2D(0.5, 0.0, 0.0),  # 1: in (0.5)
            Pose2D(3.0, 3.0, 0.0),  # 2: out
            Pose2D(0.0, 0.8, 0.0),  # 3: in (0.8)
            Pose2D(2.5, 0.0, 0.0),  # 4: out
            Pose2D(0.3, 0.3, 0.0),  # 5: in (~0.424)
        ]
    )
    sensor = NeighborSensor(world, detection_range=1.0)
    neighbors = sensor.neighbors_of(0)
    assert [n.neighbor_id for n in neighbors] == [1, 3, 5]
    assert all(n.distance <= 1.0 for n in neighbors)


# --- self exclusion ---


def test_self_never_detected_even_when_co_located():
    world = _world_with_poses(
        [
            Pose2D(1.0, 2.0, 0.3),  # 0
            Pose2D(1.0, 2.0, 0.0),  # 1: exactly co-located with 0
            Pose2D(0.5, 0.5, 0.0),  # 2
            Pose2D(0.5, 0.5, 0.0),  # 3: exactly co-located with 2
        ]
    )
    sensor = NeighborSensor(world, detection_range=1.0)
    for robot_id in range(4):
        ids = [n.neighbor_id for n in sensor.neighbors_of(robot_id)]
        assert robot_id not in ids, f"robot {robot_id} detected itself"
        assert len(ids) <= 3  # at most N-1 neighbors
    # Co-located non-self robots are still detected.
    assert 1 in [n.neighbor_id for n in sensor.neighbors_of(0)]
    assert 0 in [n.neighbor_id for n in sensor.neighbors_of(1)]
    assert 3 in [n.neighbor_id for n in sensor.neighbors_of(2)]


# --- exact boundary semantics ---


def test_boundary_distance_equal_to_range_is_inclusive():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, 0.0),  # 0: query
            Pose2D(1.0, 0.0, 0.0),  # 1: distance exactly == range
            Pose2D(1.0 + 1e-9, 0.0, 0.0),  # 2: just beyond range
            Pose2D(0.0, 0.0, 0.0),  # 3: distance 0 (co-located, non-self)
        ]
    )
    sensor = NeighborSensor(world, detection_range=1.0)
    neighbors = sensor.neighbors_of(0)
    assert [n.neighbor_id for n in neighbors] == [1, 3]
    assert neighbors[0].distance == pytest.approx(1.0)
    assert neighbors[1].distance == pytest.approx(0.0)


def test_zero_distance_co_located_robot_is_in_range():
    world = _world_with_poses(
        [Pose2D(2.0, -1.0, 0.0), Pose2D(2.0, -1.0, 0.7)]
    )
    sensor = NeighborSensor(world, detection_range=0.5)
    (neighbor,) = sensor.neighbors_of(0)
    assert neighbor.neighbor_id == 1
    assert neighbor.distance == pytest.approx(0.0)
    assert neighbor.relative_position == Vector2(0.0, 0.0)


# --- world-frame relative position convention ---


def test_relative_position_is_world_frame_independent_of_heading():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, math.pi),  # 0: query, heading pi
            Pose2D(1.0, 0.0, 0.0),  # 1: at (1, 0)
            Pose2D(0.0, -2.0, 0.0),  # 2: at (0, -2)
        ]
    )
    sensor = NeighborSensor(world, detection_range=3.0)
    by_id = {n.neighbor_id: n for n in sensor.neighbors_of(0)}
    assert by_id[1].relative_position == Vector2(1.0, 0.0)
    assert by_id[1].distance == pytest.approx(1.0)
    assert by_id[2].relative_position == Vector2(0.0, -2.0)
    assert by_id[2].distance == pytest.approx(2.0)


# --- relative velocity ---


def test_relative_velocity_is_v_neighbor_minus_v_self_resolved_along_heading():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, 0.0),  # 0: v=0.1 heading 0 -> (0.1, 0)
            Pose2D(1.0, 0.0, math.pi / 2),  # 1: v=0.2 heading pi/2 -> (0, 0.2)
        ]
    )
    world.robot_by_id(0).linear_velocity = 0.1
    world.robot_by_id(1).linear_velocity = 0.2
    sensor = NeighborSensor(world, detection_range=2.0)
    (neighbor,) = sensor.neighbors_of(0)
    assert neighbor.relative_velocity is not None
    assert neighbor.relative_velocity.x == pytest.approx(-0.1)
    assert neighbor.relative_velocity.y == pytest.approx(0.2)


def test_relative_velocity_is_antisymmetric_between_a_pair():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(1.0, 0.0, math.pi / 2),
        ]
    )
    world.robot_by_id(0).linear_velocity = 0.1
    world.robot_by_id(1).linear_velocity = 0.2
    sensor = NeighborSensor(world, detection_range=2.0)
    v_01 = sensor.neighbors_of(0)[0].relative_velocity
    v_10 = sensor.neighbors_of(1)[0].relative_velocity
    assert v_01 is not None and v_10 is not None
    assert v_01.x == pytest.approx(-v_10.x)
    assert v_01.y == pytest.approx(-v_10.y)


def test_relative_velocity_is_zero_for_stationary_robots():
    world = _world_with_poses(
        [Pose2D(0.0, 0.0, 0.0), Pose2D(0.5, 0.5, 1.0)]
    )
    sensor = NeighborSensor(world, detection_range=1.0)
    (neighbor,) = sensor.neighbors_of(0)
    assert neighbor.relative_velocity is not None
    assert neighbor.relative_velocity.x == pytest.approx(0.0)
    assert neighbor.relative_velocity.y == pytest.approx(0.0)


# --- validation ---


def test_detection_range_must_be_positive_and_finite():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0)])
    with pytest.raises(ValueError):
        NeighborSensor(world, detection_range=0.0)
    with pytest.raises(ValueError):
        NeighborSensor(world, detection_range=-1.0)
    with pytest.raises(ValueError):
        NeighborSensor(world, detection_range=math.inf)
    with pytest.raises(ValueError):
        NeighborSensor(world, detection_range=math.nan)


def test_unknown_robot_id_raises_key_error():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0)])
    sensor = NeighborSensor(world, detection_range=1.0)
    with pytest.raises(KeyError):
        sensor.neighbors_of(99)


# --- non-mutation and determinism ---


def test_queries_do_not_mutate_world_state():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(0.5, 0.2, 0.1),
            Pose2D(1.0, 0.0, -0.5),
        ]
    )
    world.robot_by_id(1).linear_velocity = 0.3
    sensor = NeighborSensor(world, detection_range=2.0)
    before_poses = [r.pose for r in world.robots]
    before_velocities = [
        (r.linear_velocity, r.angular_velocity) for r in world.robots
    ]
    before_time = world.time
    before_steps = world.step_count
    sensor.neighbors_of(0)
    sensor.neighbors_of(1)
    sensor.neighbors_of(2)
    assert [r.pose for r in world.robots] == before_poses
    assert [
        (r.linear_velocity, r.angular_velocity) for r in world.robots
    ] == before_velocities
    assert world.time == before_time
    assert world.step_count == before_steps


def test_repeated_queries_are_deterministic():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(0.4, 0.1, 0.0),
            Pose2D(1.1, 0.0, 0.0),
            Pose2D(0.0, 0.9, 0.0),
        ]
    )
    sensor = NeighborSensor(world, detection_range=1.0)
    first = sensor.neighbors_of(0)
    for _ in range(5):
        assert sensor.neighbors_of(0) == first

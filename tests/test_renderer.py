"""Tests for the minimal text-based debug renderer (M1.8).

Covers the acceptance criteria: render() carries header/grid/legend content
(world size, robot count, ids, positions, time, step_count), grid dimension
validation, heading-arrow octant mapping, neighbor links only in debug mode
with exact distances, one-sided detection, TrajectoryRecorder semantics,
determinism, and non-mutation. Headless and deterministic: all poses are
given explicitly, so no RNG is involved.
"""

import math

import pytest

from layka import (
    DebugRenderer,
    Pose2D,
    TrajectoryRecorder,
    Vector2,
    World,
    heading_arrow,
)


def _world_with_poses(poses: list[Pose2D]) -> World:
    """Build a world whose robot ``i`` sits at ``poses[i]``."""
    world = World()
    for pose in poses:
        world.add_robot(pose)
    return world


# --- render(): header, grid, legend ---


def test_render_is_nonempty_and_contains_header_robot_ids_positions_and_clock():
    world = _world_with_poses(
        [Pose2D(1.0, 2.0, 0.5), Pose2D(3.0, 4.0, 1.2), Pose2D(0.5, 0.25, -0.7)]
    )
    text = DebugRenderer(world).render()
    assert text
    assert "world 5.00 x 5.00 m" in text
    assert "3 robots" in text
    assert "t=0.00 s" in text
    assert "step=0" in text
    for robot_id in (0, 1, 2):
        assert f"R{robot_id}" in text
    assert "(1.00, 2.00)" in text
    assert "(3.00, 4.00)" in text
    assert "(0.50, 0.25)" in text


def test_render_marks_cells_with_heading_arrows():
    world = _world_with_poses(
        [Pose2D(0.5, 2.5, 0.0), Pose2D(4.5, 2.5, math.pi)]
    )
    text = DebugRenderer(world).render()
    grid_lines = [line for line in text.splitlines() if line.startswith("|")]
    assert len(grid_lines) == 24
    # Robot 0 (theta=0) draws a right arrow; robot 1 (theta=pi) a left arrow.
    assert any("→" in line[1:-1] for line in grid_lines)
    assert any("←" in line[1:-1] for line in grid_lines)


# --- constructor validation ---


def test_grid_dimensions_must_be_positive_ints():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0)])
    with pytest.raises(ValueError):
        DebugRenderer(world, grid_width=0)
    with pytest.raises(ValueError):
        DebugRenderer(world, grid_height=-3)
    with pytest.raises(ValueError):
        DebugRenderer(world, grid_width=2.5)
    with pytest.raises(ValueError):
        DebugRenderer(world, grid_height=True)


def test_trajectory_points_and_detection_range_are_validated():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0)])
    with pytest.raises(ValueError):
        DebugRenderer(world, trajectory_points=-1)
    with pytest.raises(ValueError):
        DebugRenderer(world, trajectory_points=2.5)
    with pytest.raises(ValueError):
        DebugRenderer(world, debug=True, detection_range=0.0)
    with pytest.raises(ValueError):
        DebugRenderer(world, debug=True, detection_range=-1.0)
    with pytest.raises(ValueError):
        DebugRenderer(world, debug=True, detection_range=math.inf)
    with pytest.raises(ValueError):
        DebugRenderer(world, debug=True, detection_range=math.nan)
    with pytest.raises(ValueError):
        DebugRenderer(world, detection_range=-1.0)  # validated even in non-debug


# --- heading-arrow octant mapping ---


def test_heading_arrow_cardinal_and_diagonal_octants():
    assert heading_arrow(0.0) == "→"
    assert heading_arrow(math.pi / 2) == "↑"
    assert heading_arrow(math.pi) == "←"
    assert heading_arrow(-math.pi / 2) == "↓"
    assert heading_arrow(math.pi / 4) == "↗"
    assert heading_arrow(3 * math.pi / 4) == "↖"
    assert heading_arrow(-math.pi / 4) == "↘"
    assert heading_arrow(-3 * math.pi / 4) == "↙"


def test_heading_arrow_octant_boundaries_round_to_cardinal_direction():
    # Exact boundaries at odd multiples of pi/8 round to the even octant.
    assert heading_arrow(math.pi / 8) == "→"
    assert heading_arrow(-math.pi / 8) == "→"
    assert heading_arrow(5 * math.pi / 8) == "↑"
    assert heading_arrow(7 * math.pi / 8) == "←"


def test_heading_arrow_rejects_nonfinite_angles():
    with pytest.raises(ValueError):
        heading_arrow(math.nan)
    with pytest.raises(ValueError):
        heading_arrow(math.inf)


# --- report(): authoritative text output ---


def test_report_contains_positions_heading_degrees_and_arrows():
    world = _world_with_poses(
        [Pose2D(1.0, 2.0, math.pi), Pose2D(4.0, 0.5, math.pi / 2)]
    )
    text = DebugRenderer(world).report()
    assert "(1.00, 2.00)" in text
    assert "(4.00, 0.50)" in text
    assert "180.0 deg" in text  # theta = pi
    assert "90.0 deg" in text  # theta = pi/2
    assert "←" in text
    assert "↑" in text


def test_report_shows_neighbor_links_only_in_debug_mode():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)])
    debug_text = DebugRenderer(world, debug=True, detection_range=1.5).report()
    assert "R0 neighbors: R1 (d=1.000 m)" in debug_text
    assert "R1 neighbors: R0 (d=1.000 m)" in debug_text
    # Same renderer configuration but debug OFF: no neighbor links at all.
    plain_text = DebugRenderer(world, detection_range=1.5).report()
    assert "neighbors" not in plain_text


def test_detection_range_covers_exactly_one_robot():
    world = _world_with_poses(
        [
            Pose2D(0.0, 0.0, 0.0),  # 0: query
            Pose2D(1.0, 0.0, 0.0),  # 1: in range (d=1.0)
            Pose2D(5.0, 0.0, 0.0),  # 2: out of range (d=5.0)
        ]
    )
    text = DebugRenderer(world, debug=True, detection_range=1.5).report()
    assert "R0 neighbors: R1 (d=1.000 m)" in text
    assert "R1 neighbors: R0 (d=1.000 m)" in text
    assert "R2 neighbors: none" in text
    # Sanity check against NeighborSensor directly.
    from layka import NeighborSensor

    sensor = NeighborSensor(world, detection_range=1.5)
    assert [n.neighbor_id for n in sensor.neighbors_of(0)] == [1]
    assert sensor.neighbors_of(2) == []


def test_report_shows_trajectory_points_in_debug_mode():
    world = _world_with_poses([Pose2D(0.5, 0.5, 0.0)])
    recorder = TrajectoryRecorder(max_points=10)
    recorder.record(world)
    text = DebugRenderer(
        world, debug=True, trajectory_points=5, trajectory=recorder
    ).report()
    assert "-- debug: trajectories (last 5 points) --" in text
    assert "(0.50, 0.50)" in text


def test_report_omits_trajectory_when_debug_is_off():
    world = _world_with_poses([Pose2D(0.5, 0.5, 0.0)])
    recorder = TrajectoryRecorder(max_points=10)
    recorder.record(world)
    text = DebugRenderer(world, trajectory_points=5, trajectory=recorder).report()
    assert "trajectory" not in text


def test_co_located_robots_are_disambiguated_in_report():
    world = _world_with_poses(
        [Pose2D(1.0, 1.0, 0.0), Pose2D(1.0, 1.0, math.pi / 2)]
    )
    text = DebugRenderer(world).report()
    assert "R0: pos=(1.00, 1.00)" in text
    assert "R1: pos=(1.00, 1.00)" in text


# --- neighbor links on the grid (debug mode) ---


def test_debug_grid_draws_horizontal_neighbor_links():
    world = _world_with_poses([Pose2D(0.5, 2.5, 0.0), Pose2D(4.5, 2.5, 0.0)])
    text = DebugRenderer(world, debug=True, detection_range=5.0).render()
    grid_lines = [line for line in text.splitlines() if line.startswith("|")]
    assert any("-" in line[1:-1] for line in grid_lines)
    plain = DebugRenderer(world, detection_range=5.0).render()
    plain_grid = [line for line in plain.splitlines() if line.startswith("|")]
    assert all("-" not in line[1:-1] for line in plain_grid)


def test_debug_grid_draws_vertical_neighbor_links():
    world = _world_with_poses([Pose2D(2.5, 0.5, 0.0), Pose2D(2.5, 4.5, 0.0)])
    text = DebugRenderer(world, debug=True, detection_range=5.0).render()
    grid_lines = [line for line in text.splitlines() if line.startswith("|")]
    assert any("|" in line[1:-1] for line in grid_lines)


# --- TrajectoryRecorder ---


def test_trajectory_recorder_keeps_positions_in_order():
    world = World()
    robot_id = world.add_robot(Pose2D(0.0, 0.0, 0.0))
    recorder = TrajectoryRecorder(max_points=4)
    for i in range(3):
        world.robot_by_id(robot_id).pose = Pose2D(float(i), 0.0, 0.0)
        recorder.record(world)
    assert recorder.positions(robot_id) == [
        Vector2(0.0, 0.0),
        Vector2(1.0, 0.0),
        Vector2(2.0, 0.0),
    ]


def test_trajectory_recorder_caps_to_last_max_points():
    world = World()
    robot_id = world.add_robot(Pose2D(0.0, 0.0, 0.0))
    recorder = TrajectoryRecorder(max_points=4)
    for i in range(6):
        world.robot_by_id(robot_id).pose = Pose2D(float(i), 0.0, 0.0)
        recorder.record(world)
    assert recorder.positions(robot_id) == [
        Vector2(2.0, 0.0),
        Vector2(3.0, 0.0),
        Vector2(4.0, 0.0),
        Vector2(5.0, 0.0),
    ]
    assert recorder.max_points == 4


def test_trajectory_recorder_clear_empties_buffers():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0)])
    recorder = TrajectoryRecorder(max_points=3)
    recorder.record(world)
    assert recorder.positions(0)
    recorder.clear()
    assert recorder.positions(0) == []


def test_trajectory_recorder_unknown_robot_id_returns_empty():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0)])
    recorder = TrajectoryRecorder(max_points=3)
    recorder.record(world)
    assert recorder.positions(999) == []


def test_trajectory_recorder_does_not_mutate_world():
    world = _world_with_poses([Pose2D(0.0, 0.0, 0.0)])
    recorder = TrajectoryRecorder(max_points=5)
    before_poses = [r.pose for r in world.robots]
    recorder.record(world)
    recorder.record(world)
    assert [r.pose for r in world.robots] == before_poses
    assert world.time == 0.0
    assert world.step_count == 0


def test_trajectory_recorder_validates_max_points():
    with pytest.raises(ValueError):
        TrajectoryRecorder(max_points=0)
    with pytest.raises(ValueError):
        TrajectoryRecorder(max_points=-2)
    with pytest.raises(ValueError):
        TrajectoryRecorder(max_points=True)


# --- determinism and non-mutation ---


def test_render_and_report_are_deterministic():
    world = _world_with_poses(
        [Pose2D(1.0, 1.0, 0.3), Pose2D(2.0, 3.0, 1.0), Pose2D(0.4, 4.1, -2.0)]
    )
    renderer_a = DebugRenderer(world, debug=True, detection_range=3.0)
    renderer_b = DebugRenderer(world, debug=True, detection_range=3.0)
    assert renderer_a.render() == renderer_b.render()
    assert renderer_a.report() == renderer_b.report()
    assert renderer_a.render() == renderer_a.render()
    assert renderer_a.report() == renderer_a.report()


def test_render_and_report_do_not_mutate_world():
    world = _world_with_poses(
        [Pose2D(1.0, 2.0, 0.5), Pose2D(3.0, 1.0, -1.0)]
    )
    world.robot_by_id(0).linear_velocity = 0.3
    renderer = DebugRenderer(world, debug=True, detection_range=2.0)
    before_poses = [r.pose for r in world.robots]
    before_velocities = [
        (r.linear_velocity, r.angular_velocity) for r in world.robots
    ]
    before_time = world.time
    before_steps = world.step_count
    renderer.render()
    renderer.report()
    assert [r.pose for r in world.robots] == before_poses
    assert [
        (r.linear_velocity, r.angular_velocity) for r in world.robots
    ] == before_velocities
    assert world.time == before_time
    assert world.step_count == before_steps

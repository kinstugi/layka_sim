"""Unit tests for the pure frame-primitive builder (simulator wiring)."""

from __future__ import annotations

import math

import pytest

from layka.renderer import TrajectoryRecorder
from layka.sim_view import (
    BOUNDARY_COLOR,
    GRID_COLOR,
    HEADING_COLOR,
    LINK_COLOR,
    ROBOT_COLOR,
    TRAIL_COLOR,
    build_frame_items,
)
from layka.vector import Vector2
from layka.world import World


def _world_with(world: World, poses: list[tuple[float, float, float]]) -> World:
    from layka.pose import Pose2D

    for (x, y, theta) in poses:
        world.add_robot(pose=Pose2D(x, y, theta))
    return world


def _circles(items: list[dict]) -> list[dict]:
    return [i for i in items if i["type"] == "circle"]


def _lines(items: list[dict]) -> list[dict]:
    return [i for i in items if i["type"] == "lines"]


def _all_line_segments(items: list[dict]) -> list[list[list[float]]]:
    segments = []
    for item in _lines(items):
        segments.extend(item["lines"])
    return segments


class TestEmptyWorld:
    def test_grid_and_boundary_present_no_robots(self):
        world = World(timestep=0.05)
        items = build_frame_items(world)
        assert _circles(items) == []
        line_items = _lines(items)
        assert line_items  # grid + boundary present
        colors = [i["color"] for i in line_items]
        assert GRID_COLOR in colors
        assert BOUNDARY_COLOR in colors

    def test_boundary_rectangle_covers_world(self):
        world = World(timestep=0.05)
        items = build_frame_items(world)
        boundary = next(i for i in _lines(items) if i["color"] == BOUNDARY_COLOR)
        segments = boundary["lines"]
        assert len(segments) == 4
        expected = {
            ((0.0, 0.0), (world.width, 0.0)),
            ((world.width, 0.0), (world.width, world.height)),
            ((world.width, world.height), (0.0, world.height)),
            ((0.0, world.height), (0.0, 0.0)),
        }
        got = {(tuple(s[0]), tuple(s[1])) for s in segments}
        assert got == expected


class TestRobots:
    def test_robot_drawn_as_circle_at_pose(self):
        world = _world_with(World(timestep=0.05), [(1.0, 2.0, 0.0)])
        items = build_frame_items(world)
        circles = _circles(items)
        assert len(circles) == 1
        assert circles[0]["pos"] == [1.0, 2.0]
        assert circles[0]["radius"] == 0.06
        assert circles[0]["color"] == ROBOT_COLOR

    def test_heading_line_orientation(self):
        world = _world_with(World(timestep=0.05), [(0.0, 0.0, 0.0)])
        items = build_frame_items(world, heading_length=0.5)
        heading = next(i for i in _lines(items) if i["color"] == HEADING_COLOR)
        (start, end) = heading["lines"][0]
        assert start == [0.0, 0.0]
        assert end == [0.5, 0.0]  # heading 0 -> +x

    def test_heading_line_points_up_for_pi_over_two(self):
        world = _world_with(World(timestep=0.05), [(0.0, 0.0, math.pi / 2.0)])
        items = build_frame_items(world, heading_length=0.5)
        heading = next(i for i in _lines(items) if i["color"] == HEADING_COLOR)
        (start, end) = heading["lines"][0]
        assert start == [0.0, 0.0]
        assert pytest.approx(end[0]) == 0.0
        assert pytest.approx(end[1]) == 0.5


class TestNeighborLinks:
    def test_no_links_when_debug_off(self):
        world = _world_with(World(timestep=0.05), [(1.0, 1.0, 0.0), (1.3, 1.0, 0.0)])
        items = build_frame_items(world, detection_range=1.0)
        assert not any(i["color"] == LINK_COLOR for i in _lines(items))

    def test_links_when_debug_on_and_in_range(self):
        world = _world_with(World(timestep=0.05), [(1.0, 1.0, 0.0), (1.3, 1.0, 0.0)])
        items = build_frame_items(world, debug=True, detection_range=1.0)
        link = next(i for i in _lines(items) if i["color"] == LINK_COLOR)
        assert link["lines"] == [[[1.0, 1.0], [1.3, 1.0]]]

    def test_no_links_when_out_of_range(self):
        world = _world_with(World(timestep=0.05), [(0.0, 0.0, 0.0), (4.0, 4.0, 0.0)])
        items = build_frame_items(world, debug=True, detection_range=1.0)
        assert not any(i["color"] == LINK_COLOR for i in _lines(items))

    def test_single_link_not_duplicated(self):
        world = _world_with(World(timestep=0.05), [(1.0, 1.0, 0.0), (1.2, 1.0, 0.0)])
        items = build_frame_items(world, debug=True, detection_range=1.0)
        link = next(i for i in _lines(items) if i["color"] == LINK_COLOR)
        assert len(link["lines"]) == 1


class TestTrails:
    def test_trail_polyline_emitted(self):
        from layka.pose import Pose2D

        world = _world_with(World(timestep=0.05), [(1.0, 1.0, 0.0)])
        recorder = TrajectoryRecorder(max_points=10)
        recorder.record(world)
        world.robot_by_id(0).pose = Pose2D(1.5, 1.0, 0.0)
        recorder.record(world)
        items = build_frame_items(world, trail=recorder)
        trail = next(i for i in _lines(items) if i["color"] == TRAIL_COLOR)
        assert trail["lines"] == [[[1.0, 1.0], [1.5, 1.0]]]

    def test_no_trail_item_when_only_one_point(self):
        world = _world_with(World(timestep=0.05), [(1.0, 1.0, 0.0)])
        recorder = TrajectoryRecorder(max_points=10)
        recorder.record(world)
        items = build_frame_items(world, trail=recorder)
        assert not any(i["color"] == TRAIL_COLOR for i in _lines(items))


class TestPurityAndValidation:
    def test_world_not_mutated(self):
        world = _world_with(World(timestep=0.05), [(1.0, 1.0, 0.0)])
        time_before = world.time
        step_before = world.step_count
        build_frame_items(world, debug=True, detection_range=1.0)
        assert world.time == time_before
        assert world.step_count == step_before
        assert world.robot_count == 1

    def test_deterministic_output(self):
        world = _world_with(World(timestep=0.05), [(1.0, 1.0, 0.0), (1.2, 1.1, 0.3)])
        a = build_frame_items(world, debug=True, detection_range=1.0)
        b = build_frame_items(world, debug=True, detection_range=1.0)
        assert a == b

    def test_invalid_detection_range(self):
        world = World(timestep=0.05)
        for bad in (0.0, -1.0, math.nan, math.inf):
            with pytest.raises(ValueError):
                build_frame_items(world, debug=True, detection_range=bad)

    def test_invalid_sizes(self):
        world = World(timestep=0.05)
        for bad in (0.0, -1.0, math.nan):
            with pytest.raises(ValueError):
                build_frame_items(world, robot_radius=bad)
            with pytest.raises(ValueError):
                build_frame_items(world, heading_length=bad)
            with pytest.raises(ValueError):
                build_frame_items(world, grid_interval=bad)

"""Unit tests for the world-boundary containment behavior (wiring)."""

from __future__ import annotations

import math

import pytest

from layka.behavior import StationaryBehavior, TrivialMotionBehavior
from layka.boundary import BoundaryContainmentBehavior
from layka.kinematics import BodyVelocity
from layka.lj_controller import LJController, LJControllerConfig
from layka.pose import Pose2D
from layka.vector import Vector2
from layka.world import World

WIDTH = 5.0
HEIGHT = 4.0
MARGIN = 0.3


def _controller() -> LJController:
    return LJController(LJControllerConfig())


def _inner() -> TrivialMotionBehavior:
    return TrivialMotionBehavior(v=0.1, omega=0.3)


def _containment(inner=None, margin: float = MARGIN) -> BoundaryContainmentBehavior:
    return BoundaryContainmentBehavior(
        inner if inner is not None else _inner(),
        _controller(),
        WIDTH,
        HEIGHT,
        margin=margin,
    )


def _robot(world: World, x: float, y: float, theta: float = 0.0):
    robot_id = world.add_robot(pose=Pose2D(x, y, theta))
    return world.robot_by_id(robot_id)


class TestDelegation:
    def test_delegates_when_far_from_edges(self):
        world = World(timestep=0.05)
        robot = _robot(world, WIDTH / 2.0, HEIGHT / 2.0)
        behavior = _containment()
        command = behavior.compute_command(robot, world, 0.05)
        assert command == BodyVelocity(v=0.1, omega=0.3)

    def test_delegates_inside_margin_exactly_at_center_of_world(self):
        world = World(timestep=0.05)
        robot = _robot(world, 0.0, 0.0)  # degenerate tiny world not used here
        behavior = _containment()
        # Center of the 5x4 world is far from every edge.
        robot.pose = Pose2D(2.5, 2.0, 0.0)
        command = behavior.compute_command(robot, world, 0.05)
        assert command == BodyVelocity(v=0.1, omega=0.3)

    def test_stationary_inner_is_delegated_unchanged(self):
        world = World(timestep=0.05)
        robot = _robot(world, 2.5, 2.0)
        behavior = _containment(inner=StationaryBehavior())
        assert behavior.compute_command(robot, world, 0.05) == BodyVelocity(0.0, 0.0)


class TestContainmentOverride:
    @pytest.mark.parametrize(
        ("pose", "expected_omega_sign"),
        [
            # Right edge, facing straight into the wall (heading 0 = +x):
            # center is to the left (desired heading pi), so it must turn
            # counter-clockwise (omega > 0).
            (Pose2D(WIDTH - MARGIN / 2.0, HEIGHT / 2.0, 0.0), 1),
            # Left edge, facing into the wall (heading pi = -x). Center is to
            # the right (desired heading 0); the wrapped error is -pi, so the
            # robot turns clockwise (omega < 0).
            (Pose2D(MARGIN / 2.0, HEIGHT / 2.0, math.pi), -1),
            # Top edge, facing into the wall (heading pi/2 = +y). Center is
            # below (desired heading -pi/2); wrapped error -pi => omega < 0.
            (Pose2D(WIDTH / 2.0, HEIGHT - MARGIN / 2.0, math.pi / 2.0), -1),
            # Bottom edge, facing into the wall (heading -pi/2 = -y). Center is
            # above (desired heading pi/2); wrapped error pi => omega > 0.
            (Pose2D(WIDTH / 2.0, MARGIN / 2.0, -math.pi / 2.0), 1),
        ],
    )
    def test_near_edge_overrides_inner_and_turns_away(self, pose, expected_omega_sign):
        world = World(timestep=0.05)
        robot = _robot(world, pose.x, pose.y, pose.theta)
        behavior = _containment()
        command = behavior.compute_command(robot, world, 0.05)
        assert command != BodyVelocity(v=0.1, omega=0.3)
        assert math.isfinite(command.v)
        assert math.isfinite(command.omega)
        assert command.v >= 0.0
        assert (
            expected_omega_sign > 0 and command.omega >= 0.0
        ) or (expected_omega_sign < 0 and command.omega <= 0.0)

    def test_override_matches_controller_toward_center(self):
        world = World(timestep=0.05)
        robot = _robot(world, 0.15, HEIGHT / 2.0, math.pi)
        behavior = _containment()
        command = behavior.compute_command(robot, world, 0.05)
        expected = behavior.controller.compute(
            Vector2(WIDTH / 2.0 - robot.pose.x, HEIGHT / 2.0 - robot.pose.y),
            robot.pose.theta,
        )
        assert command == expected

    def test_outside_world_is_contained(self):
        world = World(timestep=0.05)
        robot = _robot(world, WIDTH + 5.0, HEIGHT / 2.0, 0.0)
        behavior = _containment()
        command = behavior.compute_command(robot, world, 0.05)
        assert command != BodyVelocity(v=0.1, omega=0.3)
        assert math.isfinite(command.v) and math.isfinite(command.omega)

    def test_determinism(self):
        world = World(timestep=0.05)
        robot = _robot(world, 0.15, HEIGHT / 2.0, 0.0)
        behavior = _containment()
        first = behavior.compute_command(robot, world, 0.05)
        second = behavior.compute_command(robot, world, 0.05)
        assert first == second


class TestNoMutation:
    def test_does_not_mutate_robot_or_world(self):
        world = World(timestep=0.05)
        robot = _robot(world, 0.15, HEIGHT / 2.0, 0.0)
        pose_before = robot.pose
        velocity_before = (robot.linear_velocity, robot.angular_velocity)
        time_before = world.time
        behavior = _containment()
        behavior.compute_command(robot, world, 0.05)
        assert robot.pose == pose_before
        assert (robot.linear_velocity, robot.angular_velocity) == velocity_before
        assert world.time == time_before


class TestValidation:
    def test_invalid_margin(self):
        for bad in (0.0, -1.0, math.nan, math.inf):
            with pytest.raises(ValueError):
                _containment(margin=bad)

    def test_margin_too_large(self):
        with pytest.raises(ValueError):
            _containment(margin=3.0)  # > min(5, 4) / 2 = 2.0

    def test_invalid_dimensions(self):
        for bad_width, bad_height in ((0.0, 1.0), (1.0, -1.0), (math.nan, 1.0)):
            with pytest.raises(ValueError):
                BoundaryContainmentBehavior(
                    _inner(), _controller(), bad_width, bad_height, margin=0.3
                )

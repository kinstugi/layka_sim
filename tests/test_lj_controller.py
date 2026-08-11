"""Unit tests for M2.6: the LJ controller (interaction vector -> (v, omega)).

Covers the plan.md M2.6 acceptance criteria: ``desired_heading`` from the
resultant vector, heading-error wrapping to [-pi, pi] via ``normalize_angle``,
proportional angular control with sign preservation and clamping at
``max_angular_velocity``, the bounded linear-velocity law
(``v = max_linear_velocity * cos(|error|)`` clamped into
``[min_linear_velocity, max_linear_velocity]``), the zero-resultant stop
policy, config validation, the ``LJBehavior`` wrapper integration
(sensor -> interaction -> controller), and determinism.
"""

import math

import pytest
from pydantic import ValidationError

from layka.behavior import StationaryBehavior
from layka.config import LennardJonesConfig
from layka.kinematics import BodyVelocity
from layka.lj_controller import LJBehavior, LJController, LJControllerConfig
from layka.lj_interaction import LJInteraction
from layka.pose import Pose2D, normalize_angle
from layka.vector import Vector2
from layka.world import World

#: Defaults: angular_gain 2.0, max_linear_velocity 0.2, max_angular_velocity
#: 1.0, min_linear_velocity 0.0.
DEFAULT = LJControllerConfig()


def make_controller(**overrides: float) -> LJController:
    return LJController(LJControllerConfig(**overrides))


def make_lj_config(**overrides: object) -> LennardJonesConfig:
    kwargs: dict[str, object] = {"desired_spacing": 0.40}
    kwargs.update(overrides)
    return LennardJonesConfig(**kwargs)


# --- desired_heading: atan2(F_y, F_x) ---


def test_desired_heading_cardinal_and_diagonal_directions():
    controller = make_controller()
    assert controller.desired_heading(Vector2(1.0, 0.0)) == pytest.approx(0.0)
    assert controller.desired_heading(Vector2(0.0, 1.0)) == pytest.approx(
        math.pi / 2
    )
    assert controller.desired_heading(Vector2(-1.0, 0.0)) == pytest.approx(math.pi)
    assert controller.desired_heading(Vector2(1.0, 1.0)) == pytest.approx(
        math.pi / 4
    )


def test_desired_heading_of_zero_vector_is_pinned_to_zero():
    # Pinned: atan2(0, 0) == 0.0. compute() never acts on this value because
    # it short-circuits zero resultants to BodyVelocity(0, 0) first.
    assert make_controller().desired_heading(Vector2(0.0, 0.0)) == 0.0


# --- heading error wrapping via normalize_angle ---


def test_omega_uses_wrapped_error_not_raw_difference():
    controller = make_controller()
    # Desired heading just below -pi, current heading just below +pi: the raw
    # difference is ~ -2pi, which must wrap to a small positive error.
    resultant = Vector2(-1.0, -1e-9)
    current_heading = math.pi - 1e-9
    desired = controller.desired_heading(resultant)
    raw_error = desired - current_heading
    assert raw_error < -math.pi  # without wrapping this would be ~ -2pi
    command = controller.compute(resultant, current_heading)
    wrapped = normalize_angle(desired - current_heading)
    # Pins that the controller uses normalize_angle AND the proportional law.
    assert command.omega == pytest.approx(
        controller.config.angular_gain * wrapped
    )
    assert 0.0 < wrapped < 1e-6  # small positive error, not a hard turn


def test_heading_error_always_wrapped_to_pm_pi():
    controller = make_controller()
    for current_heading in (-4.0, -1.0, 0.5, 2.0, 5.0):
        for resultant in (
            Vector2(1.0, 0.0),
            Vector2(-0.5, 1.0),
            Vector2(0.3, -0.7),
        ):
            desired = controller.desired_heading(resultant)
            error = normalize_angle(desired - current_heading)
            assert -math.pi <= error <= math.pi


# --- omega: proportional with sign preservation and clamping ---


def test_omega_proportional_to_heading_error_with_sign_preserved():
    controller = make_controller()  # k_omega = 2.0, max_angular = 1.0
    error = 0.3
    command = controller.compute(
        Vector2(math.cos(error), math.sin(error)), 0.0
    )
    assert command.omega == pytest.approx(controller.config.angular_gain * error)
    negated = controller.compute(
        Vector2(math.cos(-error), math.sin(-error)), 0.0
    )
    assert negated.omega == pytest.approx(-command.omega)


def test_omega_clamped_at_max_angular_velocity():
    controller = make_controller()
    # error = pi/2 -> k_omega * error = pi > max_angular = 1.0 (sign preserved).
    command = controller.compute(Vector2(0.0, 1.0), 0.0)
    assert command.omega == pytest.approx(controller.config.max_angular_velocity)
    assert command.omega > 0
    # Facing exactly away: |error| = pi -> clamped to -max_angular.
    away = controller.compute(Vector2(1.0, 0.0), math.pi)
    assert away.omega == pytest.approx(-controller.config.max_angular_velocity)


# --- linear velocity: bounded and reduced when heading error is large ---


def test_linear_velocity_max_when_heading_aligned():
    controller = make_controller()
    command = controller.compute(Vector2(1.0, 0.0), 0.0)  # error == 0
    assert command.v == pytest.approx(controller.config.max_linear_velocity)


def test_linear_velocity_min_when_facing_away():
    controller = make_controller()  # min_linear_velocity == 0.0 by default
    away = controller.compute(Vector2(1.0, 0.0), math.pi)  # |error| ~ pi
    assert away.v == pytest.approx(controller.config.min_linear_velocity)
    # With a nonzero floor the robot still crawls forward while turning.
    crawl = make_controller(min_linear_velocity=0.05)
    assert crawl.compute(Vector2(1.0, 0.0), math.pi).v == pytest.approx(0.05)


def test_linear_velocity_monotonically_decreases_with_heading_error():
    controller = make_controller()
    errors = [0.0, math.pi / 6, math.pi / 3, math.pi / 2, math.pi]
    speeds = [
        controller.compute(Vector2(math.cos(e), math.sin(e)), 0.0).v
        for e in errors
    ]
    assert speeds[0] == pytest.approx(controller.config.max_linear_velocity)
    # cos(|e|) is strictly decreasing on [0, pi]: distinct speeds, then the
    # clamp floor once the raw value drops below min_linear_velocity.
    assert speeds[0] > speeds[1] > speeds[2] > speeds[3]
    assert speeds[-1] == pytest.approx(controller.config.min_linear_velocity)


def test_linear_velocity_never_exceeds_max():
    controller = make_controller()
    for error in (-math.pi, -1.0, 0.0, 0.5, math.pi):
        command = controller.compute(
            Vector2(math.cos(error), math.sin(error)), 0.0
        )
        assert 0.0 <= command.v <= controller.config.max_linear_velocity


# --- zero resultant -> no motion ---


def test_zero_resultant_stops_the_robot():
    controller = make_controller()
    assert controller.compute(Vector2(0.0, 0.0), 0.0) == BodyVelocity(0.0, 0.0)
    assert controller.compute(Vector2(0.0, 0.0), 1.7) == BodyVelocity(0.0, 0.0)
    # The numerical residue of canceling forces is also treated as no motion.
    assert controller.compute(Vector2(1e-13, 0.0), 0.0) == BodyVelocity(0.0, 0.0)


def test_nonfinite_resultant_is_invalid_input():
    controller = make_controller()
    with pytest.raises(ValueError):
        controller.compute(Vector2(math.nan, 0.0), 0.0)
    with pytest.raises(ValueError):
        controller.compute(Vector2(1.0, math.inf), 0.0)


# --- config validation ---


@pytest.mark.parametrize("bad", [0.0, -1.0, math.inf, -math.inf, math.nan])
def test_angular_gain_must_be_positive_and_finite(bad):
    with pytest.raises(ValidationError):
        LJControllerConfig(angular_gain=bad)


@pytest.mark.parametrize("bad", [0.0, -1.0, math.inf, -math.inf, math.nan])
def test_max_linear_velocity_must_be_positive_and_finite(bad):
    with pytest.raises(ValidationError):
        LJControllerConfig(max_linear_velocity=bad)


@pytest.mark.parametrize("bad", [0.0, -1.0, math.inf, -math.inf, math.nan])
def test_max_angular_velocity_must_be_positive_and_finite(bad):
    with pytest.raises(ValidationError):
        LJControllerConfig(max_angular_velocity=bad)


@pytest.mark.parametrize("bad", [-0.1, -1.0, math.inf, -math.inf, math.nan])
def test_min_linear_velocity_must_be_nonnegative_and_finite(bad):
    with pytest.raises(ValidationError):
        LJControllerConfig(min_linear_velocity=bad)


def test_min_linear_velocity_must_not_exceed_max():
    with pytest.raises(ValidationError):
        LJControllerConfig(min_linear_velocity=0.3, max_linear_velocity=0.2)
    # Equality is allowed.
    config = LJControllerConfig(min_linear_velocity=0.2, max_linear_velocity=0.2)
    assert config.min_linear_velocity == pytest.approx(0.2)


def test_config_defaults():
    config = LJControllerConfig()
    assert config.angular_gain == pytest.approx(2.0)
    assert config.max_linear_velocity == pytest.approx(0.2)
    assert config.max_angular_velocity == pytest.approx(1.0)
    assert config.min_linear_velocity == pytest.approx(0.0)


def test_controller_exposes_readonly_config():
    config = LJControllerConfig()
    controller = LJController(config)
    assert controller.config is config


# --- determinism (pure controller) ---


def test_controller_is_deterministic():
    controller = make_controller()
    resultant = Vector2(0.5, -0.3)
    first = controller.compute(resultant, 0.9)
    for _ in range(5):
        assert controller.compute(resultant, 0.9) == first


# --- LJBehavior integration: sensor -> interaction -> controller ---


def _two_robot_world(neighbor_x: float) -> tuple[World, LJBehavior]:
    world = World(seed=7)
    behavior = LJBehavior.from_config(
        DEFAULT, make_lj_config(), detection_range=1.0
    )
    world.add_robot(Pose2D(0.0, 0.0, 0.0), behavior=behavior)
    world.add_robot(Pose2D(neighbor_x, 0.0, 0.0), behavior=StationaryBehavior())
    return world, behavior


def test_behavior_returns_velocity_toward_attractive_neighbor():
    world, behavior = _two_robot_world(neighbor_x=0.6)  # 0.6 > r_eq = 0.4
    robot0 = world.robot_by_id(0)
    command = behavior.compute_command(robot0, world, dt=world.dt)
    assert isinstance(command, BodyVelocity)
    # Resultant points +x -> desired heading 0 == robot heading 0: full speed
    # straight ahead, zero turn.
    assert command.v == pytest.approx(DEFAULT.max_linear_velocity)
    assert command.omega == pytest.approx(0.0, abs=1e-12)


def test_behavior_with_no_neighbors_stops():
    world, behavior = _two_robot_world(neighbor_x=2.0)  # beyond detection_range
    command = behavior.compute_command(world.robot_by_id(0), world, dt=world.dt)
    assert command == BodyVelocity(0.0, 0.0)


def test_world_step_moves_robot_toward_attractive_neighbor():
    world, _ = _two_robot_world(neighbor_x=0.6)
    world.step()
    robot0 = world.robot_by_id(0)
    assert robot0.linear_velocity == pytest.approx(DEFAULT.max_linear_velocity)
    assert robot0.angular_velocity == pytest.approx(0.0)
    # Heading 0, so a positive v moves it along +x, closer to the neighbor.
    assert robot0.pose.x > 0.0
    assert robot0.pose.y == pytest.approx(0.0)


def test_behavior_is_deterministic():
    world, behavior = _two_robot_world(neighbor_x=0.6)
    robot0 = world.robot_by_id(0)
    first = behavior.compute_command(robot0, world, dt=world.dt)
    for _ in range(5):
        assert behavior.compute_command(robot0, world, dt=world.dt) == first


def test_behavior_rejects_invalid_detection_range():
    with pytest.raises(ValueError):
        LJBehavior(
            controller=LJController(DEFAULT),
            interaction=LJInteraction(make_lj_config()),
            detection_range=0.0,
        )

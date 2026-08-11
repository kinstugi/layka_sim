"""Unit tests for differential-drive kinematics (M1.5).

Covers the body <-> wheel speed transformations and the forward-Euler pose
integration, matching the legacy ``models/differential_drive_dynamics.py``
convention exactly (heading held at the old theta; theta accumulates without
normalization).
"""

import math

import pytest

from layka import (
    BodyVelocity,
    DifferentialDriveRobot,
    Pose2D,
    RobotConfig,
    WheelSpeeds,
    body_to_wheels,
    integrate_pose,
    wheels_to_body,
)


# --- body <-> wheels conversions ---


@pytest.mark.parametrize(
    ("v", "omega", "wheel_base"),
    [
        (0.0, 0.0, 0.0885),
        (0.315, 0.0, 0.0885),
        (0.1, 0.5, 0.0885),
        (-0.2, 1.0, 0.1),
        (0.5, -2.0, 0.0885),
    ],
)
def test_body_to_wheels_round_trip(v, omega, wheel_base):
    wheels = body_to_wheels(v, omega, wheel_base)
    back = wheels_to_body(wheels.v_left, wheels.v_right, wheel_base)
    assert back.v == pytest.approx(v)
    assert back.omega == pytest.approx(omega)


@pytest.mark.parametrize(
    ("v_left", "v_right", "wheel_base"),
    [
        (0.0, 0.0, 0.0885),
        (0.15, 0.15, 0.0885),
        (0.1, 0.3, 0.0885),
        (-0.2, 0.2, 0.1),
        (0.3, -0.1, 0.0885),
    ],
)
def test_wheels_to_body_round_trip(v_left, v_right, wheel_base):
    body = wheels_to_body(v_left, v_right, wheel_base)
    wheels = body_to_wheels(body.v, body.omega, wheel_base)
    assert wheels.v_left == pytest.approx(v_left)
    assert wheels.v_right == pytest.approx(v_right)


def test_body_to_wheels_exact_values():
    wheels = body_to_wheels(v=0.2, omega=0.5, wheel_base=0.1)
    assert wheels.v_left == pytest.approx(0.2 - (0.5 * 0.1) / 2.0)
    assert wheels.v_right == pytest.approx(0.2 + (0.5 * 0.1) / 2.0)


def test_wheels_to_body_exact_values():
    body = wheels_to_body(v_left=0.1, v_right=0.3, wheel_base=0.0885)
    assert body.v == pytest.approx(0.2)
    assert body.omega == pytest.approx((0.3 - 0.1) / 0.0885)


def test_zero_angular_velocity_yields_equal_wheel_speeds():
    wheels = body_to_wheels(v=0.315, omega=0.0, wheel_base=0.0885)
    assert wheels.v_left == pytest.approx(0.315)
    assert wheels.v_right == pytest.approx(0.315)
    assert wheels.v_left == pytest.approx(wheels.v_right)


def test_zero_linear_velocity_yields_counter_rotating_wheels():
    wheels = body_to_wheels(v=0.0, omega=2.0, wheel_base=0.1)
    assert wheels.v_left == pytest.approx(-0.1)
    assert wheels.v_right == pytest.approx(0.1)
    assert wheels.v_left == pytest.approx(-wheels.v_right)


@pytest.mark.parametrize("bad_wheel_base", [0.0, -0.0885])
def test_nonpositive_wheel_base_raises(bad_wheel_base):
    with pytest.raises(ValueError):
        body_to_wheels(0.1, 0.5, bad_wheel_base)
    with pytest.raises(ValueError):
        wheels_to_body(0.1, 0.1, bad_wheel_base)


# --- integrate_pose ---


@pytest.mark.parametrize("bad_dt", [0.0, -0.1])
def test_nonpositive_dt_raises(bad_dt):
    with pytest.raises(ValueError):
        integrate_pose(Pose2D(0.0, 0.0, 0.0), 0.1, 0.5, bad_dt)


def test_zero_velocity_keeps_pose_unchanged():
    pose = Pose2D(1.0, 2.0, 0.7)
    result = integrate_pose(pose, v=0.0, omega=0.0, dt=0.25)
    assert result == pose


def test_straight_motion_along_heading():
    v, dt = 0.5, 0.1

    heading_zero = integrate_pose(Pose2D(0.0, 0.0, 0.0), v=v, omega=0.0, dt=dt)
    assert heading_zero.x == pytest.approx(v * dt)
    assert heading_zero.y == pytest.approx(0.0)
    assert heading_zero.theta == pytest.approx(0.0)

    heading_pi_2 = integrate_pose(Pose2D(0.0, 0.0, math.pi / 2), v=v, omega=0.0, dt=dt)
    assert heading_pi_2.x == pytest.approx(0.0)
    assert heading_pi_2.y == pytest.approx(v * dt)
    assert heading_pi_2.theta == pytest.approx(math.pi / 2)


def test_pure_rotation_keeps_position():
    pose = Pose2D(1.0, -1.0, 0.2)
    result = integrate_pose(pose, v=0.0, omega=0.5, dt=0.1)
    assert result.x == pytest.approx(pose.x)
    assert result.y == pytest.approx(pose.y)
    assert result.theta == pytest.approx(pose.theta + 0.5 * 0.1)


def test_positive_angular_velocity_increases_theta():
    pose = Pose2D(0.0, 0.0, 0.3)
    v, omega, dt = 0.1, 0.5, 0.1
    result = integrate_pose(pose, v=v, omega=omega, dt=dt)
    assert result.x == pytest.approx(0.0 + v * dt * math.cos(pose.theta))
    assert result.y == pytest.approx(0.0 + v * dt * math.sin(pose.theta))
    assert result.theta == pytest.approx(pose.theta + omega * dt)
    assert result.theta > pose.theta


def test_negative_angular_velocity_decreases_theta():
    pose = Pose2D(0.0, 0.0, 0.3)
    v, omega, dt = 0.1, -0.5, 0.1
    result = integrate_pose(pose, v=v, omega=omega, dt=dt)
    assert result.x == pytest.approx(0.0 + v * dt * math.cos(pose.theta))
    assert result.y == pytest.approx(0.0 + v * dt * math.sin(pose.theta))
    assert result.theta == pytest.approx(pose.theta + omega * dt)
    assert result.theta < pose.theta


def test_translation_uses_old_heading_and_theta_is_not_wrapped():
    """Legacy-convention check: translation uses the old theta and theta
    accumulates past [-pi, pi] without normalization."""
    pose = Pose2D(1.0, 1.0, 2.0 * math.pi - 0.1)
    v, omega, dt = 0.2, 3.0, 0.1
    result = integrate_pose(pose, v=v, omega=omega, dt=dt)
    d = v * dt
    assert result.x == pytest.approx(pose.x + d * math.cos(pose.theta))
    assert result.y == pytest.approx(pose.y + d * math.sin(pose.theta))
    assert result.theta == pytest.approx(pose.theta + omega * dt)


# --- determinism ---


def test_repeated_integration_is_deterministic():
    start = Pose2D(0.5, -0.25, 0.9)
    v, omega, dt = 0.1, 0.5, 0.1
    first = start
    for _ in range(50):
        first = integrate_pose(first, v, omega, dt)
    second = start
    for _ in range(50):
        second = integrate_pose(second, v, omega, dt)
    assert first == second


def test_same_sequence_of_integrations_is_reproducible():
    """Two runs of the same N-step sequence from the same state produce
    bit-identical poses (no randomness anywhere in the loop)."""
    dt = 0.1
    velocities = [(0.1, 0.5), (0.0, -0.3), (0.2, 0.0), (-0.05, 0.7)]
    trajectory = _run_trajectory(velocities, dt)
    trajectory_again = _run_trajectory(velocities, dt)
    assert trajectory == trajectory_again


def _run_trajectory(velocities: list[tuple[float, float]], dt: float) -> list[Pose2D]:
    pose = Pose2D(0.0, 0.0, 0.0)
    poses = []
    for v, omega in velocities:
        for _ in range(10):
            pose = integrate_pose(pose, v, omega, dt)
        poses.append(pose)
    return poses


# --- DifferentialDriveRobot wrapper ---


def test_robot_wrapper_from_config_reuses_robot_config():
    config = RobotConfig()
    robot = DifferentialDriveRobot.from_config(config)
    assert robot.wheel_base == pytest.approx(config.wheel_base)
    assert robot.max_linear_velocity == pytest.approx(config.max_linear_velocity)
    assert robot.max_angular_velocity == pytest.approx(config.max_angular_velocity)


def test_robot_wrapper_velocities_to_wheel_speeds():
    robot = DifferentialDriveRobot(wheel_base=0.1)
    wheels = robot.velocities_to_wheel_speeds(v=0.2, omega=1.0)
    assert wheels.v_left == pytest.approx(0.15)
    assert wheels.v_right == pytest.approx(0.25)


def test_robot_wrapper_update_pose():
    robot = DifferentialDriveRobot(wheel_base=0.1)
    pose = Pose2D(1.0, 2.0, 0.5)
    result = robot.update_pose(pose, v=0.1, omega=0.0, dt=0.1)
    assert result.x == pytest.approx(1.0 + 0.01 * math.cos(0.5))
    assert result.y == pytest.approx(2.0 + 0.01 * math.sin(0.5))
    assert result.theta == pytest.approx(0.5)


def test_robot_wrapper_rejects_nonpositive_wheel_base():
    with pytest.raises(ValueError):
        DifferentialDriveRobot(wheel_base=0.0)


def test_robot_wrapper_rejects_nonpositive_velocity_limits():
    with pytest.raises(ValueError):
        DifferentialDriveRobot(wheel_base=0.1, max_linear_velocity=0.0)
    with pytest.raises(ValueError):
        DifferentialDriveRobot(wheel_base=0.1, max_angular_velocity=-1.0)


def test_body_velocity_and_wheel_speeds_are_frozen_dataclasses():
    with pytest.raises(AttributeError):
        BodyVelocity(v=0.1, omega=0.2).v = 0.3  # type: ignore[misc]
    with pytest.raises(AttributeError):
        WheelSpeeds(v_left=0.1, v_right=0.2).v_left = 0.3  # type: ignore[misc]

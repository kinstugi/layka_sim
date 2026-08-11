"""Unit tests for the typed core models introduced in M1.3."""

import math

import pytest
from pydantic import ValidationError

from layka.config import LennardJonesConfig, SimulationConfig
from layka.pose import Pose2D, normalize_angle
from layka.robot import RobotConfig, RobotState
from layka.vector import Vector2

TWO_ONE_SIXTH = 2.0 ** (1.0 / 6.0)


# --- Vector2 ---


def test_vector_add():
    assert Vector2(1.0, 2.0) + Vector2(3.0, 4.0) == Vector2(4.0, 6.0)


def test_vector_sub():
    assert Vector2(3.0, 4.0) - Vector2(1.0, 2.0) == Vector2(2.0, 2.0)


def test_vector_scalar_mul_and_rmul():
    assert Vector2(2.0, 3.0) * 2.0 == Vector2(4.0, 6.0)
    assert 2.0 * Vector2(2.0, 3.0) == Vector2(4.0, 6.0)


def test_vector_scalar_div():
    assert Vector2(4.0, 6.0) / 2.0 == Vector2(2.0, 3.0)


def test_vector_neg():
    assert -Vector2(1.0, -2.0) == Vector2(-1.0, 2.0)


def test_vector_dot():
    assert Vector2(1.0, 0.0).dot(Vector2(0.0, 1.0)) == 0.0
    assert Vector2(1.0, 2.0).dot(Vector2(3.0, 4.0)) == 11.0


def test_vector_norm():
    assert Vector2(3.0, 4.0).norm() == pytest.approx(5.0)


def test_vector_normalized():
    result = Vector2(3.0, 4.0).normalized()
    assert result.x == pytest.approx(0.6)
    assert result.y == pytest.approx(0.8)


def test_vector_normalized_zero_vector_returns_zero():
    assert Vector2(0.0, 0.0).normalized() == Vector2(0.0, 0.0)


def test_vector_rotate():
    quarter = Vector2(1.0, 0.0).rotate(math.pi / 2)
    assert quarter.x == pytest.approx(0.0)
    assert quarter.y == pytest.approx(1.0)
    half = Vector2(1.0, 0.0).rotate(math.pi)
    assert half.x == pytest.approx(-1.0)
    assert half.y == pytest.approx(0.0)
    back = Vector2(1.0, 0.0).rotate(-math.pi / 2)
    assert back.x == pytest.approx(0.0)
    assert back.y == pytest.approx(-1.0)


def test_vector_distance():
    assert Vector2(0.0, 0.0).distance_to(Vector2(3.0, 4.0)) == pytest.approx(5.0)


# --- Pose2D ---


def test_normalize_angle_maps_to_minus_pi_to_pi():
    assert normalize_angle(0.5) == pytest.approx(0.5)
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(2.0 * math.pi) == pytest.approx(0.0, abs=1e-12)
    assert normalize_angle(-0.5) == pytest.approx(-0.5)


def test_pose_position():
    pose = Pose2D(1.0, 2.0, 0.5)
    assert pose.position() == Vector2(1.0, 2.0)


def test_pose_wrapped_returns_normalized_copy():
    pose = Pose2D(1.0, 2.0, 3.0 * math.pi)
    wrapped = pose.wrapped()
    assert wrapped.x == 1.0
    assert wrapped.y == 2.0
    assert wrapped.theta == pytest.approx(math.pi)
    assert pose.theta == 3.0 * math.pi  # original unchanged (frozen dataclass)


def test_pose_relative_to():
    pose_a = Pose2D(0.0, 0.0, 0.0)
    pose_b = Pose2D(3.0, 4.0, 0.0)
    displacement, bearing = pose_a.relative_to(pose_b)
    assert displacement == Vector2(3.0, 4.0)
    assert bearing == pytest.approx(math.atan2(4.0, 3.0))


# --- LennardJonesConfig ---


def test_lj_config_derives_sigma_from_desired_spacing():
    desired = 0.4
    config = LennardJonesConfig(desired_spacing=desired)
    assert config.sigma == pytest.approx(desired / TWO_ONE_SIXTH)
    assert config.equilibrium_distance == pytest.approx(desired)


def test_lj_config_sigma_is_not_equilibrium_distance():
    sigma = 0.4
    config = LennardJonesConfig(sigma=sigma)
    r_eq = TWO_ONE_SIXTH * sigma
    assert config.equilibrium_distance == pytest.approx(r_eq)
    assert config.sigma != config.equilibrium_distance


def test_lj_config_nonpositive_epsilon_raises():
    for bad in (0.0, -1.0):
        with pytest.raises(ValidationError):
            LennardJonesConfig(epsilon=bad, sigma=0.4)


def test_lj_config_nonpositive_sigma_raises():
    for bad in (0.0, -1.0):
        with pytest.raises(ValidationError):
            LennardJonesConfig(sigma=bad)


def test_lj_config_nonpositive_cutoff_distance_raises():
    for bad in (0.0, -1.0):
        with pytest.raises(ValidationError):
            LennardJonesConfig(sigma=0.4, cutoff_distance=bad)


def test_lj_config_nonpositive_max_force_raises():
    for bad in (0.0, -1.0):
        with pytest.raises(ValidationError):
            LennardJonesConfig(sigma=0.4, max_force=bad)


def test_lj_config_sigma_and_desired_spacing_conflict():
    with pytest.raises(ValidationError):
        LennardJonesConfig(sigma=0.3, desired_spacing=0.4)


def test_lj_config_requires_sigma_or_desired_spacing():
    with pytest.raises(ValidationError):
        LennardJonesConfig()


# --- SimulationConfig ---


def test_simulation_config_validation():
    with pytest.raises(ValidationError):
        SimulationConfig(timestep=0.0)
    with pytest.raises(ValidationError):
        SimulationConfig(robot_count=-1)
    with pytest.raises(ValidationError):
        SimulationConfig(world_width=0.0)
    with pytest.raises(ValidationError):
        SimulationConfig(world_height=-2.0)
    config = SimulationConfig(timestep=0.05, robot_count=10, random_seed=42)
    assert config.timestep == 0.05
    assert config.robot_count == 10
    assert config.random_seed == 42


# --- RobotConfig ---


def test_robot_config_validation():
    with pytest.raises(ValidationError):
        RobotConfig(wheel_base=0.0)
    with pytest.raises(ValidationError):
        RobotConfig(max_linear_velocity=0.0)
    with pytest.raises(ValidationError):
        RobotConfig(max_angular_velocity=-1.0)
    config = RobotConfig()
    assert config.wheel_base > 0
    assert config.max_linear_velocity > 0
    assert config.max_angular_velocity > 0


# --- RobotState ---


def test_robot_state_defaults():
    state = RobotState(robot_id=3, pose=Pose2D(0.0, 0.0, 0.0))
    assert state.robot_id == 3
    assert state.linear_velocity == 0.0
    assert state.angular_velocity == 0.0


def test_robot_state_dataclass_semantics():
    a = RobotState(1, Pose2D(0.0, 0.0, 0.0))
    b = RobotState(1, Pose2D(0.0, 0.0, 0.0))
    c = RobotState(2, Pose2D(0.0, 0.0, 0.0))
    assert a == b
    assert a != c

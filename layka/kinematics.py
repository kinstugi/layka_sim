"""Differential-drive (unicycle) kinematics for mobile robots.

Pure math, no GTK/rendering/controller dependency: converts between body
velocities ``(v, omega)`` and linear wheel speeds ``(v_left, v_right)``, and
integrates a pose forward by one timestep.

The integration below is the forward-Euler approximation already used by the
legacy ``models/differential_drive_dynamics.py``: the robot travels
``d = v * dt`` along the *old* heading ``theta``, and the heading itself
advances by ``omega * dt``. These exact conventions are preserved so later
milestones (e.g. M2.6's controller) are drop-in compatible.

Pose updates are independent of rendering: they are driven only by the
simulation clock ``SimulationClock`` (M1.4) providing ``dt``; renderers never
call :func:`integrate_pose`.

Coordinate conventions (from :mod:`layka.pose`): x right, y up, theta measured
from the +x axis, counter-clockwise positive. Units are SI: m, rad, m/s, rad/s.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from layka.pose import Pose2D
from layka.robot import RobotConfig


@dataclass(frozen=True, slots=True)
class WheelSpeeds:
    """Linear speeds of the left and right drive wheels (m/s)."""

    v_left: float
    v_right: float


@dataclass(frozen=True, slots=True)
class BodyVelocity:
    """Body-frame velocity of a unicycle robot: forward speed and turn rate."""

    v: float
    omega: float


def _validate_wheel_base(wheel_base: float) -> None:
    if not math.isfinite(wheel_base) or wheel_base <= 0:
        raise ValueError(
            f"wheel_base must be a finite positive number, got {wheel_base!r}"
        )


def _validate_dt(dt: float) -> None:
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError(f"dt must be a finite positive number, got {dt!r}")


def body_to_wheels(v: float, omega: float, wheel_base: float) -> WheelSpeeds:
    """Convert body velocities ``(v, omega)`` to linear wheel speeds.

    Standard differential-drive body-to-wheel relations (wheel speeds in m/s,
    wheel base in m):

        v_left  = v - (omega * wheel_base) / 2
        v_right = v + (omega * wheel_base) / 2
    """
    _validate_wheel_base(wheel_base)
    half_omega_base = omega * wheel_base / 2.0
    return WheelSpeeds(v_left=v - half_omega_base, v_right=v + half_omega_base)


def wheels_to_body(v_left: float, v_right: float, wheel_base: float) -> BodyVelocity:
    """Convert linear wheel speeds to body velocities ``(v, omega)``.

    Inverse of :func:`body_to_wheels`:

        v     = (v_left + v_right) / 2
        omega = (v_right - v_left) / wheel_base
    """
    _validate_wheel_base(wheel_base)
    return BodyVelocity(
        v=(v_left + v_right) / 2.0,
        omega=(v_right - v_left) / wheel_base,
    )


def integrate_pose(pose: Pose2D, v: float, omega: float, dt: float) -> Pose2D:
    """Advance ``pose`` by one timestep ``dt`` using the legacy forward-Euler
    scheme (heading held at the old ``theta``):

        d         = v * dt
        new_x     = x     + d * cos(theta)
        new_y     = y     + d * sin(theta)
        new_theta = theta + omega * dt

    This matches ``models/differential_drive_dynamics.py`` exactly. ``theta``
    is intentionally NOT normalized here (the legacy model accumulates it);
    callers needing [-pi, pi) can use :meth:`Pose2D.wrapped`. Pose updates are
    independent of rendering and are driven only by the simulation clock
    ``SimulationClock`` (M1.4) supplying ``dt``.
    """
    _validate_dt(dt)
    distance = v * dt
    return Pose2D(
        x=pose.x + distance * math.cos(pose.theta),
        y=pose.y + distance * math.sin(pose.theta),
        theta=pose.theta + omega * dt,
    )


@dataclass(frozen=True, slots=True)
class DifferentialDriveRobot:
    """Minimal differential-drive kinematics wrapper.

    Holds the wheel base and optional velocity limits sourced from
    ``RobotConfig``. Deliberately thin: no controllers, sensors, or physics
    (Rule 5 / Rule 8). Velocity limits are stored for later milestones (e.g.
    M2.6's controller) but are NOT enforced here — kinematics only maps
    velocities to motion.
    """

    wheel_base: float
    max_linear_velocity: float | None = None
    max_angular_velocity: float | None = None

    def __post_init__(self) -> None:
        _validate_wheel_base(self.wheel_base)
        if self.max_linear_velocity is not None and (
            not math.isfinite(self.max_linear_velocity)
            or self.max_linear_velocity <= 0
        ):
            raise ValueError(
                "max_linear_velocity must be a finite positive number or None"
            )
        if self.max_angular_velocity is not None and (
            not math.isfinite(self.max_angular_velocity)
            or self.max_angular_velocity <= 0
        ):
            raise ValueError(
                "max_angular_velocity must be a finite positive number or None"
            )

    @classmethod
    def from_config(cls, config: RobotConfig) -> DifferentialDriveRobot:
        """Build from a validated ``RobotConfig`` (reused, never modified)."""
        return cls(
            wheel_base=config.wheel_base,
            max_linear_velocity=config.max_linear_velocity,
            max_angular_velocity=config.max_angular_velocity,
        )

    def velocities_to_wheel_speeds(self, v: float, omega: float) -> WheelSpeeds:
        return body_to_wheels(v, omega, self.wheel_base)

    def update_pose(self, pose: Pose2D, v: float, omega: float, dt: float) -> Pose2D:
        return integrate_pose(pose, v, omega, dt)

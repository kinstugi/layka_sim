"""Minimal robot behavior abstraction (M1.6).

A behavior decides *how a robot should move*: it maps ``(robot, world, dt)``
to a body velocity ``(v, omega)``. It does NOT integrate poses, touch wheel
speeds, or sense anything (sensors/neighbor queries arrive in M1.7/M2.x). The
world owns one behavior per robot, calls ``compute_command`` exactly once per
robot per step, stores the resulting velocity on the robot state, and
integrates the pose via :func:`layka.kinematics.integrate_pose` (Rule 5:
behavior -> control -> kinematics stay separate).

Only trivial behaviors ship in M1.6: stationary and constant-velocity motion.
Search/LJ/obstacle behaviors are deliberately deferred to M2.x (Design
Correction 4: no swarm behavior yet).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

from layka.kinematics import BodyVelocity
from layka.robot import RobotState

if TYPE_CHECKING:
    from layka.world import World


class Behavior(Protocol):
    """A robot behavior: produce a body velocity command for one robot.

    ``world`` is passed for future read-only queries (e.g. M1.7 neighbor
    detection), but a behavior must never mutate the world or other robots —
    ``World.step`` is the only mutation driver.
    """

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        """Return the body velocity ``(v, omega)`` for ``robot`` this step."""
        ...


class StationaryBehavior:
    """Do not move: always commands ``BodyVelocity(0.0, 0.0)``.

    Stateless, so a single instance may be shared by many robots.
    """

    __slots__ = ()

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        return BodyVelocity(v=0.0, omega=0.0)


@dataclass(frozen=True, slots=True)
class TrivialMotionBehavior:
    """Straight-line / constant-velocity motion for debugging and tests.

    Commands a constant body velocity ``(v, omega)`` regardless of robot or
    world state. With ``omega = 0`` the robot drives straight along its
    heading; with ``omega != 0`` it turns at a constant rate. Negative ``v``
    is allowed (reverse driving).
    """

    v: float = 0.1
    omega: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.v) or not math.isfinite(self.omega):
            raise ValueError(
                f"v and omega must be finite numbers, got {(self.v, self.omega)!r}"
            )

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        return BodyVelocity(v=self.v, omega=self.omega)


__all__ = ["Behavior", "StationaryBehavior", "TrivialMotionBehavior"]

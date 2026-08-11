"""Obstacle-avoidance behavior wrapper (M2.10).

Priority (plan.md M2.10): collision safety > obstacle avoidance > swarm
interaction. :class:`ObstacleAvoidanceBehavior` implements the middle layer:
it wraps the inner swarm behavior and OVERRIDES its command with a steering
command directly away from the nearest obstacle whenever the robot comes
within ``obstacle.radius + clearance`` of one.

Design (mirrors :class:`layka.boundary.BoundaryContainmentBehavior` exactly):

- When the robot is within ``radius + clearance`` of an obstacle center, the
  wrapper computes ``away = robot_pos - obstacle.center`` (which points from
  the obstacle center toward the robot, i.e. directly away from the obstacle)
  and steers along it with the shared :class:`layka.lj_controller.LJController`
  (proportional heading error, bounded velocities) -- the same control math
  already used for boundary containment; no new control code.
- If several obstacles are in range, the CLOSEST one (smallest center
  distance) wins: that is the most imminent collision.
- Otherwise it delegates to the inner behavior unchanged. The inner behavior
  is NOT called while an override is active, so a wrapped stateful behavior
  (e.g. search) pauses its patrol timing until the robot is clear.

Rule 5 separation: an obstacle is WORLD-level static geometry. It is NOT a
robot: it never enters the neighbor/LJ pipeline (:class:`NeighborSensor`
iterates only ``world.robots``) and never carries a behavior, pose, or
velocity. This behavior only READS ``world.obstacles``; it never mutates them,
the robot, the world, or other robots (``World.step`` is the only mutation
driver).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from layka.kinematics import BodyVelocity
from layka.lj_controller import LJController
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.behavior import Behavior
    from layka.obstacle import Obstacle
    from layka.robot import RobotState
    from layka.world import World


def _validate_nonnegative(name: str, value: float) -> None:
    if not math.isfinite(value) or value < 0:
        raise ValueError(
            f"{name} must be a finite non-negative number, got {value!r}"
        )


class ObstacleAvoidanceBehavior:
    """``Behavior`` wrapper that steers robots away from nearby obstacles.

    Wraps ``inner`` and delegates to it while the robot is clear of every
    obstacle. When the robot comes within ``radius + clearance`` of an
    obstacle center, the command is overridden with a heading-directly-away
    body velocity computed by the supplied :class:`LJController`. If several
    obstacles are within range, the closest one (most imminent) wins.

    Deterministic and side-effect free; apart from the ``world.obstacles`` it
    reads it is independent of any particular world instance.
    """

    __slots__ = ("_inner", "_controller", "_clearance")

    def __init__(
        self,
        inner: Behavior,
        controller: LJController,
        *,
        clearance: float = 0.15,
    ) -> None:
        """Wrap ``inner``; ``clearance`` is the extra distance beyond an
        obstacle's radius at which avoidance kicks in (m, finite and >= 0)."""
        _validate_nonnegative("clearance", clearance)
        self._inner = inner
        self._controller = controller
        self._clearance = clearance

    @property
    def inner(self) -> Behavior:
        """The wrapped behavior (read-only)."""
        return self._inner

    @property
    def controller(self) -> LJController:
        """The controller used for the avoidance override (read-only)."""
        return self._controller

    @property
    def clearance(self) -> float:
        """Extra distance beyond an obstacle's radius that triggers avoidance (m)."""
        return self._clearance

    def _nearest_in_range(self, robot_pos: Vector2, world: World) -> Obstacle | None:
        """Closest obstacle within ``radius + clearance`` of ``robot_pos``.

        Returns ``None`` when no obstacle is in range. Strict ``<`` compares
        the center distance against the influence radius, and the closest
        obstacle (smallest center distance, i.e. most imminent collision)
        wins.
        """
        nearest: Obstacle | None = None
        nearest_distance = math.inf
        for obstacle in world.obstacles:
            distance = robot_pos.distance_to(obstacle.center)
            if distance < obstacle.radius + self._clearance and distance < nearest_distance:
                nearest = obstacle
                nearest_distance = distance
        return nearest

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        """Return the body velocity commanded for ``robot``.

        If the robot is within ``radius + clearance`` of the closest obstacle
        center, steer directly away from that center via the LJController
        (``controller.compute(robot_pos - obstacle.center, robot.pose.theta)``);
        otherwise delegate to the inner behavior. The inner behavior is not
        invoked while an override is active.
        """
        robot_pos = robot.pose.position()
        nearest = self._nearest_in_range(robot_pos, world)
        if nearest is not None:
            away = robot_pos - nearest.center
            return self._controller.compute(away, robot.pose.theta)
        return self._inner.compute_command(robot, world, dt)


__all__ = ["ObstacleAvoidanceBehavior"]

"""World-boundary containment behavior (wiring; stopgap before M2.10).

A raw :class:`layka.world.World` has no walls: robots keep whatever velocity
their behavior commands and can wander (or, in the M2.8 "runaway pair"
pathology, flee) out of the visible world forever. M2.10 introduces proper
obstacle interaction; until then this module provides a simple, deterministic
containment wrapper that keeps robots inside the world bounds.

:class:`BoundaryContainmentBehavior` wraps any inner :class:`Behavior`
(typically :class:`layka.search_behavior.SearchSwarmBehavior`):

- When the robot is within ``margin`` meters of any world edge (or has already
  left the world), the wrapper OVERRIDES the inner command with a steering
  command back toward the world center. It reuses the tested
  :class:`layka.lj_controller.LJController` to convert "head toward the
  center" into a feasible ``(v, omega)`` (proportional heading error, bounded
  velocities) -- no new control math.
- Otherwise it delegates to the inner behavior unchanged.

Priority follows the plan's rough ordering (collision safety / obstacle
avoidance above swarm interaction): the containment override wins whenever it
triggers, even in the SWARM state. The inner behavior is NOT called while the
override is active, so a wrapped stateful behavior (e.g. search) simply
pauses its patrol timing until the robot is back inside the interior.

Deterministic and read-only: ``compute_command`` never mutates the robot, the
world, or other robots (``World.step`` is the only mutation driver).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from layka.kinematics import BodyVelocity
from layka.lj_controller import LJController
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.behavior import Behavior
    from layka.robot import RobotState
    from layka.world import World


def _validate_positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number, got {value!r}")


class BoundaryContainmentBehavior:
    """``Behavior`` wrapper that steers robots back inside the world bounds.

    Wraps ``inner`` and delegates to it while the robot is comfortably inside
    the world. When the robot comes within ``margin`` of any edge (or leaves
    the world), the command is overridden with a heading-toward-center body
    velocity computed by the supplied :class:`LJController`.

    Deterministic, side-effect free, and (apart from the width/height it is
    constructed with) independent of any particular world instance.
    """

    __slots__ = ("_inner", "_controller", "_width", "_height", "_margin")

    def __init__(
        self,
        inner: Behavior,
        controller: LJController,
        width: float,
        height: float,
        *,
        margin: float = 0.3,
    ) -> None:
        _validate_positive("width", width)
        _validate_positive("height", height)
        _validate_positive("margin", margin)
        if margin > min(width, height) / 2.0:
            raise ValueError(
                "margin must be <= min(width, height) / 2 so the containment "
                "zone is well-defined, got "
                f"margin={margin!r} vs {min(width, height) / 2.0!r}"
            )
        self._inner = inner
        self._controller = controller
        self._width = width
        self._height = height
        self._margin = margin

    @property
    def inner(self) -> Behavior:
        """The wrapped behavior (read-only)."""
        return self._inner

    @property
    def controller(self) -> LJController:
        """The controller used for the containment override (read-only)."""
        return self._controller

    @property
    def width(self) -> float:
        """World width the containment zone is computed for (m)."""
        return self._width

    @property
    def height(self) -> float:
        """World height the containment zone is computed for (m)."""
        return self._height

    @property
    def margin(self) -> float:
        """Distance from an edge that triggers containment (m)."""
        return self._margin

    def _needs_containment(self, x: float, y: float) -> bool:
        return (
            x < self._margin
            or x > self._width - self._margin
            or y < self._margin
            or y > self._height - self._margin
        )

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        """Return the body velocity commanded for ``robot``.

        If the robot is within ``margin`` of an edge (or outside the world),
        steer back toward the world center via the LJController; otherwise
        delegate to the inner behavior. The inner behavior is not invoked
        while the override is active.
        """
        if self._needs_containment(robot.pose.x, robot.pose.y):
            direction = Vector2(
                self._width / 2.0 - robot.pose.x,
                self._height / 2.0 - robot.pose.y,
            )
            return self._controller.compute(direction, robot.pose.theta)
        return self._inner.compute_command(robot, world, dt)


__all__ = ["BoundaryContainmentBehavior"]

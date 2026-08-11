"""Static circular obstacles for the simulation world (M2.10).

An :class:`Obstacle` is WORLD-level static geometry: a circle defined by a
``center`` position and a ``radius``. The analytic distance from a point to an
obstacle is ``|pos - center| - radius``; there is no rigid-body physics, no
sliding contact, and no path planning (Design Correction 4) -- a behavior
steering override when the robot is within an influence range is enough to
satisfy "not intentionally driving through".

An obstacle is NOT a robot. It never enters the neighbor/LJ pipeline
(:class:`layka.neighbors.NeighborSensor` iterates only ``world.robots``, so
obstacles can never appear in neighbor queries by construction) and never
carries a behavior, pose, velocity, or state. Obstacles are read-only geometry
consumed by behaviors (e.g.
:class:`layka.obstacle_avoidance.ObstacleAvoidanceBehavior`) and rendered by
:func:`layka.sim_view.build_frame_items`; nothing ever mutates them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from layka.vector import Vector2


@dataclass(frozen=True, slots=True)
class Obstacle:
    """A simple static circular obstacle.

    Fields:
        center: obstacle center position (m), a :class:`Vector2`.
        radius: obstacle radius (m); must be finite and strictly positive.

    Frozen and immutable: obstacles are static world geometry that never moves
    or changes during a simulation. ``__post_init__`` validates ``center`` is
    a ``Vector2`` with finite components and ``radius`` is finite and ``> 0``
    (``ValueError``/``TypeError`` otherwise).
    """

    center: Vector2
    radius: float

    def __post_init__(self) -> None:
        if not isinstance(self.center, Vector2):
            raise TypeError(f"center must be a Vector2, got {self.center!r}")
        if not math.isfinite(self.center.x) or not math.isfinite(self.center.y):
            raise ValueError(f"center must have finite components, got {self.center!r}")
        if not math.isfinite(self.radius) or self.radius <= 0:
            raise ValueError(
                f"radius must be a finite positive number, got {self.radius!r}"
            )


__all__ = ["Obstacle"]

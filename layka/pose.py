"""2D pose (position + heading) for differential-drive robots.

Math convention: x right, y up, theta measured from the +x axis, counter-
clockwise positive. Angles are normalized to the interval [-pi, pi].
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from layka.vector import Vector2


def normalize_angle(theta: float) -> float:
    """Map ``theta`` to the equivalent angle in [-pi, pi]."""
    return math.atan2(math.sin(theta), math.cos(theta))


@dataclass(frozen=True, slots=True)
class Pose2D:
    """Position ``(x, y)`` and heading ``theta`` (radians)."""

    x: float
    y: float
    theta: float

    def position(self) -> Vector2:
        return Vector2(self.x, self.y)

    def wrapped(self) -> Pose2D:
        """Return a copy whose heading is normalized to [-pi, pi]."""
        return Pose2D(self.x, self.y, normalize_angle(self.theta))

    def relative_to(self, other: Pose2D) -> tuple[Vector2, float]:
        """Displacement vector and bearing angle from this pose to ``other``."""
        displacement = other.position() - self.position()
        bearing = math.atan2(displacement.y, displacement.x)
        return displacement, bearing

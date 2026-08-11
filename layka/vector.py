"""2D vector math for the simulation core.

``Vector2`` is a plain frozen dataclass (not Pydantic) because it lives in
high-frequency numerical paths where validation overhead would dominate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Vector2:
    """A 2D Euclidean vector with components ``x`` and ``y``."""

    x: float
    y: float

    def __add__(self, other: Vector2) -> Vector2:
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector2) -> Vector2:
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> Vector2:
        return Vector2(self.x * scalar, self.y * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> Vector2:
        return Vector2(self.x / scalar, self.y / scalar)

    def __neg__(self) -> Vector2:
        return Vector2(-self.x, -self.y)

    def dot(self, other: Vector2) -> float:
        return self.x * other.x + self.y * other.y

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> Vector2:
        """Unit vector in this direction; the zero vector maps to itself."""
        length = self.norm()
        if length == 0.0:
            return Vector2(0.0, 0.0)
        return Vector2(self.x / length, self.y / length)

    def rotate(self, angle: float) -> Vector2:
        """Rotate counter-clockwise by ``angle`` radians (math convention)."""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        return Vector2(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a,
        )

    def distance_to(self, other: Vector2) -> float:
        return (other - self).norm()

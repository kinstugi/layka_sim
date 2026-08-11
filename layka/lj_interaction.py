"""Pairwise Lennard-Jones interaction as 2D vectors (M2.4).

For a robot at ``self_position`` and a neighbor at ``other_position``:

    displacement = other_position - self_position
    r = |displacement|
    direction = displacement / r            # unit vector, world frame, self -> other
    f = safe_lj_force(r, config)            # signed scalar from M2.1 via M2.2
    F_AB = -f * direction

``f`` is the signed radial LJ scalar: positive = repulsive, negative =
attractive (M2.1/M2.2). The force on the self robot is ``F_AB = -f * direction``
-- note the negation. Why: ``direction`` points from self TOWARD the other
robot, while the scalar ``f = -dV/dr`` acts along the line joining the pair in
the direction of INCREASING separation (away from the other robot). The vector
force on self is therefore the scalar times ``-direction``:

* r > r_eq (attractive, f < 0): F_AB = -f * direction points from self TOWARD
  the neighbor (magnitude |f|), pulling the pair together;
* r < r_eq (repulsive, f > 0):  F_AB = -f * direction points from self AWAY
  from the neighbor (magnitude f), pushing the pair apart.

A sign flip here would invert the whole swarm behavior (robots beyond
equilibrium would repel and never aggregate), so the negation is deliberate
and pinned by the unit tests in ``tests/test_lj_interaction.py``.

With multiple neighbors the contributions are additive (superposition):

    F_A = sum(F_AB) over all neighbors B

This module is a pure geometric/math component: it produces a resultant
VECTOR and knows nothing about robot controllers, kinematics, the world, or
rendering (independence required by plan.md M2.4). Configuring/selecting the
force is M2.5, converting the vector to robot motion is M2.6.
"""

from __future__ import annotations

from collections.abc import Iterable

from layka.config import LennardJonesConfig
from layka.lj_safety import safe_lj_force
from layka.vector import Vector2


def pairwise_lj_force(
    self_position: Vector2,
    other_position: Vector2,
    config: LennardJonesConfig,
) -> Vector2:
    """LJ force vector on ``self_position`` due to ``other_position``.

    ``F_AB = -safe_lj_force(r, config) * direction`` with
    ``direction = (other_position - self_position) / r`` (see the module
    docstring for the sign rationale).

    r == 0 policy: co-located robots have no defined separation direction, so
    the pairwise force is the zero vector. The safety layer (M2.2) cannot
    provide a direction for a zero displacement, and zero force is the safe,
    symmetric choice: neither robot is influenced by the other.
    """
    displacement = other_position - self_position
    r = displacement.norm()
    if r == 0.0:
        return Vector2(0.0, 0.0)
    direction = displacement / r
    f = safe_lj_force(r, config)
    return direction * (-f)


def resultant_lj_force(
    self_position: Vector2,
    other_positions: Iterable[Vector2],
    config: LennardJonesConfig,
) -> Vector2:
    """Resultant LJ force on ``self_position`` summed over all neighbors.

    Superposition: ``F_A = sum(pairwise_lj_force(self, other, config))``.
    Returns the zero vector for an empty neighbor list. Co-located neighbors
    contribute zero (see :func:`pairwise_lj_force`). Deterministic: the sum
    order follows the iteration order of ``other_positions``.
    """
    total = Vector2(0.0, 0.0)
    for other_position in other_positions:
        total = total + pairwise_lj_force(self_position, other_position, config)
    return total

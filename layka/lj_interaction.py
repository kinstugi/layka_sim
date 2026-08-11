"""Pairwise Lennard-Jones interaction as 2D vectors (M2.4) and the
interaction component that assembles them (M2.5).

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

The functions above are pure geometric/math: they produce a resultant VECTOR
and know nothing about robot controllers, kinematics, the world, or rendering
(independence required by plan.md M2.4).

The :class:`LJInteraction` component (M2.5) is the thin orchestration layer on
top: it receives the self pose and already-detected neighbor poses (detection
is :class:`layka.neighbors.NeighborSensor`'s job, M1.7) and returns the
resultant vector computed by :func:`resultant_lj_force`. It has NO side
effects -- it never mutates poses/robots/world and never sets wheel
velocities or any robot motion. Converting the resultant vector into
``(v, omega)`` is the controller's job (M2.6), and the robot model performs
the kinematic integration. This three-way split (interaction vector ->
controller -> kinematics) is the M2.x separation of concerns.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence

from layka.config import LennardJonesConfig
from layka.lj_safety import safe_lj_force
from layka.neighbors import Neighbor
from layka.pose import Pose2D
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


class LJInteraction:
    """M2.5 component: neighbor poses -> resultant LJ interaction vector.

    Consumes already-detected neighbor poses (detection is
    :class:`layka.neighbors.NeighborSensor`'s job, M1.7) and computes the
    resultant LJ vector by delegating to :func:`resultant_lj_force` (the pure
    M2.4 math).

    This component is deliberately side-effect free: it never mutates poses,
    robots, or the world, and it never sets wheel velocities or any robot
    motion. The resultant vector is a desired interaction direction/speed
    input for the controller; converting it into ``(v, omega)`` is the
    controller's responsibility (M2.6), not this component's.
    """

    __slots__ = ("_config",)

    def __init__(self, config: LennardJonesConfig) -> None:
        self._config = config

    @property
    def config(self) -> LennardJonesConfig:
        """The LJ configuration this component was created with (read-only)."""
        return self._config

    def compute(
        self,
        self_pose: Pose2D,
        neighbor_poses: Sequence[Pose2D],
    ) -> Vector2:
        """Resultant LJ interaction vector on ``self_pose`` over ``neighbor_poses``.

        Delegates to ``resultant_lj_force`` (M2.4) with the world-frame
        positions of the self pose and each neighbor pose. An empty neighbor
        list yields the zero vector. Deterministic: identical inputs produce
        identical results across repeated calls.
        """
        return resultant_lj_force(
            self_pose.position(),
            (neighbor.position() for neighbor in neighbor_poses),
            self._config,
        )

    def compute_from_neighbors(
        self,
        self_pose: Pose2D,
        neighbors: Sequence[Neighbor],
    ) -> Vector2:
        """Like :meth:`compute` but consumes ``Neighbor`` records (M1.7).

        Absolute neighbor positions are reconstructed as
        ``self_position + neighbor.relative_position``. Per M1.7's convention
        ``Neighbor.relative_position`` is a world-frame displacement (NOT a
        body-frame vector), so this sum is exact world-frame addition and the
        result equals passing the equivalent absolute poses to
        :meth:`compute`.
        """
        self_position = self_pose.position()
        return resultant_lj_force(
            self_position,
            (self_position + neighbor.relative_position for neighbor in neighbors),
            self._config,
        )

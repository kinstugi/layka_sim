"""Neighbor query abstraction for the swarm (M1.7).

A :class:`NeighborSensor` answers the question "which robots are within
detection range of robot X?" as a list of :class:`Neighbor` records, each
carrying the neighbor's id, its position relative to X, the distance, and the
relative linear velocity. The query is a pure sensor function of the world's
robot state: no state mutation, no side effects, no rendering, and no physics
(Design Corrections 2/3, Rule 5). It builds on :class:`layka.world.World`
(M1.6) without modifying it.

Conventions (documented here and pinned by tests):

- **World frame**: ``relative_position`` is the displacement vector from X to
  the neighbor in the world frame (``neighbor.pose.position() -
  query.pose.position()``), independent of X's heading.
- **Inclusive boundary**: a robot is a neighbor iff ``distance <=
  detection_range``; a robot exactly at range IS detected, just beyond it is
  not.
- **Self exclusion**: X never appears in its own neighbor list, even when
  another robot is exactly co-located with X.
- **Distance 0**: a non-self robot exactly co-located with X has distance
  ``0 <= detection_range`` and IS a neighbor (the LJ math handles ``r = 0``
  separately in M2.x; here it is pure geometry).
- **Ordering**: neighbors are returned in ascending robot id order (the
  world's insertion order), making repeated queries deterministic.
- **Complexity**: the scan is O(N^2) over all robots (loop over every robot,
  skip self) by design for M1. Spatial hashing / quadtrees / KD-trees / GPU
  acceleration are deliberately out of scope (Rule 7).

``relative_velocity`` is the relative linear velocity vector ``v_other -
v_self`` in the world frame, where each robot's world-frame velocity is its
``linear_velocity`` resolved along its heading: ``Vector2(v * cos(theta),
v * sin(theta))``. It is always computed and returned by
:meth:`NeighborSensor.neighbors_of`; the field is typed optional only so a
``Neighbor`` can also be constructed by hand without it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from layka.pose import Pose2D
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.world import World


def _validate_detection_range(detection_range: float) -> None:
    if not math.isfinite(detection_range) or detection_range <= 0:
        raise ValueError(
            f"detection_range must be a finite positive number, "
            f"got {detection_range!r}"
        )


def _world_frame_velocity(pose: Pose2D, linear_velocity: float) -> Vector2:
    """Resolve a robot's body-frame forward speed into a world-frame vector."""
    return Vector2(
        linear_velocity * math.cos(pose.theta),
        linear_velocity * math.sin(pose.theta),
    )


@dataclass(frozen=True, slots=True)
class Neighbor:
    """One detected robot, expressed relative to the querying robot.

    ``relative_position`` is the world-frame displacement vector from the
    querying robot to the neighbor; ``distance`` is its norm.
    ``relative_velocity`` is ``v_neighbor - v_self`` in the world frame, where
    each ``v`` is the robot's ``linear_velocity`` resolved along its heading.
    """

    neighbor_id: int
    relative_position: Vector2
    distance: float
    relative_velocity: Vector2 | None = None


class NeighborSensor:
    """Detects robots within ``detection_range`` of a queried robot.

    Read-only: queries never mutate the world, robots, or clock.
    """

    __slots__ = ("_world", "_detection_range")

    def __init__(self, world: World, detection_range: float) -> None:
        _validate_detection_range(detection_range)
        self._world = world
        self._detection_range = detection_range

    @property
    def world(self) -> World:
        """The world this sensor reads from."""
        return self._world

    @property
    def detection_range(self) -> float:
        """Detection radius (m); the boundary is inclusive (``<=``)."""
        return self._detection_range

    def neighbors_of(self, robot_id: int) -> list[Neighbor]:
        """Return all robots within ``detection_range`` of ``robot_id``.

        Scans every robot in the world and keeps those with
        ``distance <= detection_range``, excluding the querying robot itself.
        Results are returned in ascending neighbor id order. Raises
        ``KeyError`` for an unknown ``robot_id``. O(N^2) by design for M1.
        """
        query = self._world.robot_by_id(robot_id)
        query_position = query.pose.position()
        query_velocity = _world_frame_velocity(query.pose, query.linear_velocity)
        neighbors: list[Neighbor] = []
        for other in self._world.robots:
            if other.robot_id == robot_id:
                continue
            relative_position = other.pose.position() - query_position
            distance = relative_position.norm()
            if distance <= self._detection_range:
                relative_velocity = (
                    _world_frame_velocity(other.pose, other.linear_velocity)
                    - query_velocity
                )
                neighbors.append(
                    Neighbor(
                        neighbor_id=other.robot_id,
                        relative_position=relative_position,
                        distance=distance,
                        relative_velocity=relative_velocity,
                    )
                )
        return neighbors


__all__ = ["Neighbor", "NeighborSensor"]

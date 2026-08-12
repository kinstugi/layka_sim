"""M2.13 debug overlay: pure geometry for visualizing LJ interaction vectors.

The purpose of this module is debugging sign and coordinate-frame mistakes:
for each robot it produces the exact vectors the physics layer computes, so a
developer can see at a glance whether a robot is attracted toward its
neighbors (correct) or pushed away (a sign bug), and whether the vectors are
in the world frame or some rotated body frame.

Sign convention (pinned by M2.4/M2.5 and rendered here UNCHANGED): the
resultant and pairwise vectors are the physically-correct interaction
vectors produced by :func:`layka.lj_interaction.pairwise_lj_force` /
:class:`layka.lj_interaction.LJInteraction`:

* r > r_eq (attractive): the vector points from the robot TOWARD the neighbor;
* r < r_eq (repulsive):  the vector points from the robot AWAY from the
  neighbor.

These are world-frame vectors (x right, y up). They are NOT forces to
integrate: they are the virtual interaction vector that the M2.6 controller
converts into a desired velocity/heading (Design Correction 2). This module
only computes geometry for drawing arrows; it never mutates the world and
never imports ``gi``, so it is fully headless-testable.

Data model: :func:`lj_debug_overlay` returns ``dict[int, list[LJDebugVector]]``
-- one record (list) per robot id. Each record's FIRST element is always the
resultant interaction vector (``kind == "resultant"``); when ``pairwise=True``
the record then contains one ``kind == "pairwise"`` entry per detected
neighbor, in ascending neighbor id order (the ``NeighborSensor`` order). An
isolated robot's record contains exactly one entry with a zero resultant
vector. The overlay is DISABLED BY DEFAULT: the renderer draws nothing unless
the caller passes an overlay to ``build_frame_items``.

:func:`scaled_vector` bounds arrow length for drawing; it is a pure helper
(zero vector -> zero vector, direction preserved).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

from layka.config import LennardJonesConfig
from layka.lj_interaction import LJInteraction, pairwise_lj_force
from layka.neighbors import NeighborSensor
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.world import World


@dataclass(frozen=True, slots=True)
class LJDebugVector:
    """One world-frame debug arrow: ``origin`` + ``vector``, tagged by kind.

    ``kind`` distinguishes the resultant interaction vector (one per robot)
    from individual pairwise vectors (one per neighbor, only in pairwise
    mode). Pure data: world-frame ``Vector2`` values, NOT drawing primitives.
    """

    origin: Vector2
    vector: Vector2
    kind: Literal["resultant", "pairwise"]


def scaled_vector(vector: Vector2, max_length: float) -> Vector2:
    """Scale ``vector`` so its length is at most ``max_length`` meters.

    Pure arrow-drawing helper: a vector shorter than (or equal to)
    ``max_length`` is returned unchanged; a longer one is scaled down
    proportionally, so the direction is preserved. The zero vector maps to
    itself. ``max_length`` must be a finite positive number.
    """
    if not math.isfinite(max_length) or max_length <= 0:
        raise ValueError(
            f"max_length must be a finite positive number, got {max_length!r}"
        )
    length = vector.norm()
    if length == 0.0 or length <= max_length:
        return vector
    return vector * (max_length / length)


def lj_debug_overlay(
    world: World,
    lj_config: LennardJonesConfig,
    detection_range: float,
    *,
    pairwise: bool = False,
) -> dict[int, list[LJDebugVector]]:
    """Compute the M2.13 debug overlay for every robot in ``world``.

    For each robot: find its neighbors with
    :class:`layka.neighbors.NeighborSensor` (M1.7), compute the resultant
    interaction vector with :class:`layka.lj_interaction.LJInteraction`
    (M2.5), and -- when ``pairwise=True`` -- one pairwise vector per neighbor
    via :func:`layka.lj_interaction.pairwise_lj_force` (M2.4), from the
    robot's position toward/away from that neighbor, world frame.

    Returns ``dict[int, list[LJDebugVector]]`` keyed by robot id (ascending,
    the world's robot order). Each record's first entry is the resultant
    (kind ``"resultant"``), followed by the pairwise entries (kind
    ``"pairwise"``) when requested. An isolated robot has exactly one entry:
    a zero resultant vector. Pure and deterministic: never mutates the world,
    robots, or clock; identical inputs produce identical output.
    """
    sensor = NeighborSensor(world, detection_range)
    interaction = LJInteraction(lj_config)
    overlay: dict[int, list[LJDebugVector]] = {}
    for robot in world.robots:
        rid = robot.robot_id
        pose = robot.pose
        neighbors = sensor.neighbors_of(rid)
        origin = pose.position()
        resultant = interaction.compute_from_neighbors(pose, neighbors)
        record = [LJDebugVector(origin=origin, vector=resultant, kind="resultant")]
        if pairwise:
            for neighbor in neighbors:
                pairwise_vector = pairwise_lj_force(
                    origin, origin + neighbor.relative_position, lj_config
                )
                record.append(
                    LJDebugVector(origin=origin, vector=pairwise_vector, kind="pairwise")
                )
        overlay[rid] = record
    return overlay


__all__ = ["LJDebugVector", "lj_debug_overlay", "scaled_vector"]

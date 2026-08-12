"""M2.11 swarm metrics: pure, read-only measurements over robot positions.

All functions are deterministic, mutate nothing (the input sequence is only
read; ``World`` objects are never touched), and work headless (no ``gi``).
They operate on robot positions only (Design Correction 2: robots are
differential-drive, so metrics make no dynamics assumptions).

Signature choice: every function takes a ``Sequence[Vector2]`` (list, tuple,
or any read-only sequence) as its single positions argument -- one clean
signature instead of accepting both ``World`` and lists. The thin helper
:func:`positions_from_world` extracts positions from a :class:`World` when
needed.

Input policy (M2.2 ethos: metrics never silently produce NaN):

* Non-finite positions (NaN/inf in x or y) raise ``ValueError`` everywhere.
* Empty input: ``centroid``, ``mean_centroid_distance``,
  ``distance_variance``, and ``cluster_radius`` raise ``ValueError`` (they
  need a centroid, and ``(0, 0)`` would silently mask bugs);
  ``mean_pairwise_distance`` returns ``0.0`` (no pairs exist) and
  ``aggregation_score`` returns ``0.0`` (no robot can be within threshold).
* Single robot: ``mean_pairwise_distance``, ``mean_centroid_distance``,
  ``distance_variance``, and ``cluster_radius`` return ``0.0`` (trivial
  values); ``centroid`` returns the position itself; ``aggregation_score``
  returns ``1.0`` iff the single robot is within ``threshold`` of itself
  (always true), i.e. ``1.0`` for any valid threshold.

Formulas (all documented per function):

* ``centroid``: ``c = (1/N) * sum_i pos_i``.
* ``mean_centroid_distance``: ``mean_d = (1/N) * sum_i |pos_i - c|``.
* ``distance_variance``: population variance of the centroid distances,
  ``var = (1/N) * sum_i (|pos_i - c| - mean_d)^2``. "Distance variance" in the
  plan measures how tightly the robots cluster, and distance-from-centroid
  spread is the natural tightness measure.
* ``cluster_radius``: ``max_i |pos_i - c|`` -- the MAXIMUM distance from the
  centroid, i.e. the radius of the tightest centroid-centered circle enclosing
  every robot (the swarm's spatial extent).
* ``mean_pairwise_distance``: ``(1/P) * sum_{i<j} |pos_i - pos_j|`` with
  ``P = N*(N-1)/2`` unordered pairs.
* ``aggregation_score``: ``(1/N) * |{i : |pos_i - c| <= threshold}|`` -- the
  fraction of robots within ``threshold`` of the centroid (inclusive boundary,
  matching the M2.8 inline ``_cluster_size`` convention), in ``[0, 1]``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from layka.vector import Vector2
from layka.world import World


def _validated(positions: Sequence[Vector2]) -> list[Vector2]:
    """Snapshot ``positions`` and reject any non-finite coordinate."""
    result = list(positions)
    for position in result:
        if not math.isfinite(position.x) or not math.isfinite(position.y):
            raise ValueError(
                f"positions must be finite, got non-finite coordinate {position!r}"
            )
    return result


def _centroid(positions: list[Vector2]) -> Vector2:
    """Mean of a non-empty, already-validated position list."""
    total = Vector2(0.0, 0.0)
    for position in positions:
        total = total + position
    return total / len(positions)


def centroid(positions: Sequence[Vector2]) -> Vector2:
    """Swarm centroid ``c = (1/N) * sum_i pos_i`` (mean position).

    Raises ``ValueError`` on empty input or any non-finite coordinate (a
    silent ``(0, 0)`` would mask bugs).
    """
    pts = _validated(positions)
    if not pts:
        raise ValueError("centroid requires at least one position")
    return _centroid(pts)


def mean_centroid_distance(positions: Sequence[Vector2]) -> float:
    """Mean distance of the robots from the centroid,
    ``mean_d = (1/N) * sum_i |pos_i - c|``.

    Single robot -> ``0.0``; empty input -> ``ValueError`` (needs a centroid).
    """
    pts = _validated(positions)
    if not pts:
        raise ValueError("mean_centroid_distance requires at least one position")
    c = _centroid(pts)
    return sum(position.distance_to(c) for position in pts) / len(pts)


def distance_variance(positions: Sequence[Vector2]) -> float:
    """Distance variance (how tightly the robots cluster).

    Population variance of the robot distances from the centroid:

        var = (1/N) * sum_i (d_i - mean_d)^2

    with ``d_i = |pos_i - c|`` and ``mean_d`` = :func:`mean_centroid_distance`.
    Divide by N (population), not N-1. Single robot -> ``0.0``; empty input ->
    ``ValueError``.
    """
    pts = _validated(positions)
    if not pts:
        raise ValueError("distance_variance requires at least one position")
    c = _centroid(pts)
    mean_d = sum(position.distance_to(c) for position in pts) / len(pts)
    return (
        sum((position.distance_to(c) - mean_d) ** 2 for position in pts)
        / len(pts)
    )


def cluster_radius(positions: Sequence[Vector2]) -> float:
    """Cluster radius: the MAXIMUM distance from the swarm centroid.

    Interpretation: the radius of the tightest centroid-centered circle that
    encloses all robots, i.e. the swarm's spatial extent (``max_i |pos_i - c|``).
    Single robot -> ``0.0``; empty input -> ``ValueError``.
    """
    pts = _validated(positions)
    if not pts:
        raise ValueError("cluster_radius requires at least one position")
    c = _centroid(pts)
    return max(position.distance_to(c) for position in pts)


def mean_pairwise_distance(positions: Sequence[Vector2]) -> float:
    """Mean distance over all unordered robot pairs:

        mean = (1/P) * sum_{i<j} |pos_i - pos_j|,  P = N*(N-1)/2

    Empty or single-robot input -> ``0.0`` (no pairs exist); guards the
    division by zero.
    """
    pts = _validated(positions)
    n = len(pts)
    if n < 2:
        return 0.0
    total = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            total += pts[i].distance_to(pts[j])
    return total / (n * (n - 1) / 2)


def aggregation_score(positions: Sequence[Vector2], threshold: float) -> float:
    """Aggregation score: fraction of robots within ``threshold`` of the
    centroid, ``(1/N) * |{i : |pos_i - c| <= threshold}|``, in ``[0, 1]``.

    The boundary is inclusive (``<=``), matching the M2.8 inline
    ``_cluster_size`` convention. Empty input -> ``0.0``. ``threshold`` must
    be a finite positive number (``ValueError`` otherwise).
    """
    if not math.isfinite(threshold) or threshold <= 0:
        raise ValueError(
            f"threshold must be a finite positive number, got {threshold!r}"
        )
    pts = _validated(positions)
    if not pts:
        return 0.0
    c = _centroid(pts)
    within = sum(1 for position in pts if position.distance_to(c) <= threshold)
    return within / len(pts)


def positions_from_world(world: World) -> list[Vector2]:
    """Robot positions in id (insertion) order, extracted from a ``World``.

    Read-only: the world and its robots are never mutated. The returned
    positions are a snapshot; pass them to the metrics functions above.
    """
    return [robot.pose.position() for robot in world.robots]


__all__ = [
    "aggregation_score",
    "centroid",
    "cluster_radius",
    "distance_variance",
    "mean_centroid_distance",
    "mean_pairwise_distance",
    "positions_from_world",
]

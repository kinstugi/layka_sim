"""M2.11 unit tests: pure, read-only swarm metrics.

Hand-verified formulas (Rule 4):

* Mean pairwise distance: (0,0),(0,3),(4,0) -> (3+4+5)/3 = 4.0.
* Centroid: (0,0),(2,0),(0,4) -> (2/3, 4/3).
* Centroid distances for (0,0),(2,0),(4,0): centroid (2,0), distances
  (2,0,2), mean 4/3, population variance
  ((2-4/3)^2 + (0-4/3)^2 + (2-4/3)^2)/3 = (24/9)/3 = 8/9.
* Cluster radius (max distance from centroid) for the same line: 2.0.
* Aggregation score: 4 robots, 3 within 1.0 of the centroid, 1 far -> 0.75.

Pinned API choices (documented in ``layka/metrics.py``): empty input raises
``ValueError`` for centroid-derived metrics and returns ``0.0`` for
``mean_pairwise_distance`` / ``aggregation_score``; non-finite positions
always raise ``ValueError``; ``cluster_radius`` is the MAXIMUM distance from
the centroid; ``distance_variance`` is the population variance of the
centroid distances.
"""

import math

import pytest

from layka import (
    Pose2D,
    SimulationConfig,
    Vector2,
    World,
    aggregation_score,
    centroid,
    cluster_radius,
    distance_variance,
    mean_centroid_distance,
    mean_pairwise_distance,
    positions_from_world,
)

LINE = (Vector2(0.0, 0.0), Vector2(2.0, 0.0), Vector2(4.0, 0.0))
TRIANGLE = (Vector2(0.0, 0.0), Vector2(0.0, 3.0), Vector2(4.0, 0.0))


# --- mean pairwise distance ---


def test_mean_pairwise_distance_three_robots():
    assert mean_pairwise_distance(TRIANGLE) == pytest.approx(4.0)


def test_mean_pairwise_distance_empty_and_single_are_zero():
    assert mean_pairwise_distance([]) == 0.0
    assert mean_pairwise_distance([Vector2(1.0, 2.0)]) == 0.0


def test_mean_pairwise_distance_two_robots_is_their_distance():
    assert mean_pairwise_distance([Vector2(0.0, 0.0), Vector2(3.0, 4.0)]) == 5.0


# --- centroid ---


def test_centroid_of_triangle():
    assert centroid([Vector2(0.0, 0.0), Vector2(2.0, 0.0), Vector2(0.0, 4.0)]) == Vector2(
        2.0 / 3.0, 4.0 / 3.0
    )


def test_centroid_of_line_points():
    assert centroid(LINE) == Vector2(2.0, 0.0)


def test_centroid_empty_raises_value_error():
    with pytest.raises(ValueError):
        centroid([])


def test_centroid_single_robot_is_itself():
    assert centroid([Vector2(3.0, -1.0)]) == Vector2(3.0, -1.0)


# --- mean centroid distance + distance variance ---


def test_mean_centroid_distance_line_points():
    assert mean_centroid_distance(LINE) == pytest.approx(4.0 / 3.0)


def test_mean_centroid_distance_empty_raises():
    with pytest.raises(ValueError):
        mean_centroid_distance([])


def test_mean_centroid_distance_single_robot_is_zero():
    assert mean_centroid_distance([Vector2(5.0, 5.0)]) == 0.0


def test_distance_variance_line_points():
    assert distance_variance(LINE) == pytest.approx(8.0 / 9.0)


def test_distance_variance_zero_when_all_robots_share_a_point():
    points = [Vector2(1.0, 1.0), Vector2(1.0, 1.0), Vector2(1.0, 1.0)]
    assert distance_variance(points) == pytest.approx(0.0)


def test_tighter_cluster_has_smaller_variance():
    tight = [Vector2(0.0, 0.0), Vector2(0.1, 0.0), Vector2(0.0, 0.1)]
    spread = [Vector2(0.0, 0.0), Vector2(1.0, 0.0), Vector2(0.0, 1.0)]
    assert distance_variance(tight) < distance_variance(spread)


def test_distance_variance_empty_raises():
    with pytest.raises(ValueError):
        distance_variance([])


def test_distance_variance_single_robot_is_zero():
    assert distance_variance([Vector2(5.0, 5.0)]) == 0.0


# --- cluster radius ---


def test_cluster_radius_is_max_distance_from_centroid():
    assert cluster_radius(LINE) == pytest.approx(2.0)


def test_cluster_radius_single_robot_is_zero():
    assert cluster_radius([Vector2(5.0, 5.0)]) == 0.0


def test_cluster_radius_empty_raises():
    with pytest.raises(ValueError):
        cluster_radius([])


# --- aggregation score ---


AGGREGATION_POINTS = (
    Vector2(0.0, 0.0),
    Vector2(0.5, 0.0),
    Vector2(0.0, 0.5),
    Vector2(3.0, 0.0),
)


def test_aggregation_score_fraction_within_threshold():
    # 3 of the 4 robots are within 1.0 of the centroid (0.875, 0.125).
    assert aggregation_score(AGGREGATION_POINTS, threshold=1.0) == pytest.approx(0.75)


def test_aggregation_score_huge_threshold_is_one():
    assert aggregation_score(AGGREGATION_POINTS, threshold=10.0) == pytest.approx(1.0)


def test_aggregation_score_tiny_threshold_is_zero():
    assert aggregation_score(AGGREGATION_POINTS, threshold=0.1) == pytest.approx(0.0)


def test_aggregation_score_all_inside_is_one():
    points = [Vector2(0.0, 0.0), Vector2(0.2, 0.0), Vector2(0.0, 0.2)]
    assert aggregation_score(points, threshold=1.0) == pytest.approx(1.0)


def test_aggregation_score_inclusive_boundary():
    # Both robots sit exactly at distance 1.0 from the centroid (1,0); the
    # inclusive boundary (<= threshold) counts them (matches M2.8 _cluster_size).
    points = [Vector2(0.0, 0.0), Vector2(2.0, 0.0)]
    assert centroid(points) == Vector2(1.0, 0.0)
    assert aggregation_score(points, threshold=1.0) == pytest.approx(1.0)


def test_aggregation_score_empty_is_zero():
    assert aggregation_score([], threshold=1.0) == 0.0


def test_aggregation_score_single_robot_is_one():
    assert aggregation_score([Vector2(3.0, 3.0)], threshold=0.5) == pytest.approx(1.0)


def test_aggregation_score_rejects_nonpositive_threshold():
    for bad in (0.0, -1.0):
        with pytest.raises(ValueError):
            aggregation_score(AGGREGATION_POINTS, threshold=bad)


def test_aggregation_score_rejects_nonfinite_threshold():
    for bad in (math.inf, float("nan")):
        with pytest.raises(ValueError):
            aggregation_score(AGGREGATION_POINTS, threshold=bad)


# --- non-finite positions ---


def test_nonfinite_positions_raise_value_error():
    cases = [
        [Vector2(0.0, 0.0), Vector2(float("nan"), 0.0)],
        [Vector2(0.0, 0.0), Vector2(0.0, float("inf"))],
        [Vector2(float("-inf"), 0.0)],
    ]
    for points in cases:
        with pytest.raises(ValueError):
            centroid(points)
        with pytest.raises(ValueError):
            mean_centroid_distance(points)
        with pytest.raises(ValueError):
            distance_variance(points)
        with pytest.raises(ValueError):
            cluster_radius(points)
        with pytest.raises(ValueError):
            mean_pairwise_distance(points)
        with pytest.raises(ValueError):
            aggregation_score(points, threshold=1.0)


# --- purity and determinism ---


def test_metrics_do_not_mutate_input_sequence():
    points = list(AGGREGATION_POINTS)
    snapshot = list(points)
    mean_pairwise_distance(points)
    centroid(points)
    mean_centroid_distance(points)
    distance_variance(points)
    cluster_radius(points)
    aggregation_score(points, threshold=1.0)
    assert points == snapshot


def test_metrics_are_deterministic():
    points = list(AGGREGATION_POINTS)
    assert mean_pairwise_distance(points) == mean_pairwise_distance(tuple(points))
    assert distance_variance(points) == distance_variance(points)
    assert aggregation_score(points, threshold=1.0) == aggregation_score(
        points, threshold=1.0
    )
    assert centroid(points) == centroid(points)


# --- from_world helper ---


def _world_with_poses(poses: list[tuple[float, float]]) -> World:
    world = World(
        SimulationConfig(timestep=0.1, world_width=10.0, world_height=10.0)
    )
    for x, y in poses:
        world.add_robot(pose=Pose2D(x, y, 0.0))
    return world


def test_positions_from_world_matches_direct_list_calls():
    poses = [(0.0, 0.0), (0.0, 3.0), (4.0, 0.0)]
    world = _world_with_poses(poses)
    positions = positions_from_world(world)
    assert positions == [Vector2(x, y) for x, y in poses]
    assert positions == list(TRIANGLE)
    assert mean_pairwise_distance(positions) == pytest.approx(
        mean_pairwise_distance(TRIANGLE)
    )
    assert centroid(positions) == centroid(TRIANGLE)
    assert aggregation_score(positions, threshold=4.0) == aggregation_score(
        TRIANGLE, threshold=4.0
    )


def test_positions_from_world_empty_world():
    world = World(SimulationConfig(timestep=0.1))
    assert positions_from_world(world) == []


def test_metrics_do_not_mutate_world():
    world = _world_with_poses([(0.0, 0.0), (0.5, 0.0), (0.0, 0.5), (3.0, 0.0)])
    before_positions = [r.pose.position() for r in world.robots]
    before_steps = world.step_count
    positions = positions_from_world(world)
    aggregation_score(positions, threshold=1.0)
    distance_variance(positions)
    cluster_radius(positions)
    after_positions = [r.pose.position() for r in world.robots]
    assert after_positions == before_positions
    assert world.step_count == before_steps

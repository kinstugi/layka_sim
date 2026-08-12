"""Tests for M2.13 -- the pure LJ debug overlay (geometry + headless rendering).

Covers ``lj_debug_overlay`` (resultant equals the M2.5 interaction output
exactly, per-robot coverage, pairwise mode with equal-and-opposite pair
vectors, isolated robots, purity/determinism), ``scaled_vector`` (bounding
with direction preserved), and ``build_frame_items`` (arrow lines emitted
only when an overlay is supplied, endpoints match origin + scaled vector).

Sign convention pinned here: the overlay MUST render the exact vectors the
physics layer computes -- attractive pulls toward the neighbor, repulsive
pushes away (M2.4/M2.5). The overlay exists to spot sign mistakes, so it
must not introduce any.
"""

from __future__ import annotations

import math

import pytest

from layka.config import LennardJonesConfig
from layka.lj_interaction import LJInteraction, pairwise_lj_force
from layka.lj_overlay import LJDebugVector, lj_debug_overlay, scaled_vector
from layka.neighbors import NeighborSensor
from layka.pose import Pose2D
from layka.sim_view import (
    PAIRWISE_VECTOR_COLOR,
    RESULTANT_VECTOR_COLOR,
    build_frame_items,
)
from layka.vector import Vector2
from layka.world import World

#: Same config as the sim: desired_spacing 0.40 m -> r_eq 0.40 m, sigma
#: derived as desired_spacing / 2^(1/6). Default max_force (10.0) is fine:
#: the test separations (0.60 m attractive, ~0.30 m repulsive) stay under it.
CONFIG = LennardJonesConfig(desired_spacing=0.40)
DETECTION_RANGE = 2.0


def _two_robot_world() -> World:
    world = World(timestep=0.05)
    world.add_robot(pose=Pose2D(0.0, 0.0, 0.0))  # robot 0
    world.add_robot(pose=Pose2D(0.6, 0.0, 0.0))  # robot 1
    return world


def assert_vec_approx(actual: Vector2, expected: Vector2) -> None:
    assert actual.x == pytest.approx(expected.x, rel=1e-12, abs=1e-12)
    assert actual.y == pytest.approx(expected.y, rel=1e-12, abs=1e-12)


# --- resultant vector correct (matches the M2.5 interaction output) ---


def test_resultant_matches_interaction_output_for_two_robots():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    sensor = NeighborSensor(world, DETECTION_RANGE)
    interaction = LJInteraction(CONFIG)
    a_pose = world.robot_by_id(0).pose
    expected = interaction.compute_from_neighbors(a_pose, sensor.neighbors_of(0))
    record = overlay[0]
    assert len(record) == 1  # pairwise disabled -> resultant only
    resultant = record[0]
    assert resultant.kind == "resultant"
    assert resultant.origin == Vector2(0.0, 0.0)
    assert_vec_approx(resultant.vector, expected)
    # Attractive region (r = 0.6 > r_eq = 0.40): robot 0 is pulled toward
    # robot 1 at +x -- the physically-correct sign, not a repulsion bug.
    assert resultant.vector.x > 0
    assert resultant.vector.y == pytest.approx(0.0, abs=1e-12)


def test_second_robot_resultant_points_back_toward_first():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    assert overlay[1][0].vector.x < 0  # robot 1 pulled toward -x (robot 0)
    assert overlay[1][0].origin == Vector2(0.6, 0.0)


# --- per-robot coverage ---


def test_overlay_has_one_entry_per_robot():
    world = World(timestep=0.05)
    for i in range(4):
        world.add_robot(pose=Pose2D(float(i) * 0.3, 0.0, 0.0))
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    assert set(overlay.keys()) == {0, 1, 2, 3}
    assert len(overlay) == 4


# --- pairwise mode ---


def test_pairwise_mode_one_vector_per_neighbor():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE, pairwise=True)
    record_a = overlay[0]
    assert len(record_a) == 2  # resultant + one pairwise vector (neighbor 1)
    pairwise_ab = record_a[1]
    assert pairwise_ab.kind == "pairwise"
    expected_ab = pairwise_lj_force(Vector2(0.0, 0.0), Vector2(0.6, 0.0), CONFIG)
    assert_vec_approx(pairwise_ab.vector, expected_ab)
    assert pairwise_ab.origin == Vector2(0.0, 0.0)


def test_pairwise_vectors_are_equal_and_opposite():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE, pairwise=True)
    pairwise_ab = overlay[0][1].vector  # force on A due to B
    pairwise_ba = overlay[1][1].vector  # force on B due to A
    assert overlay[1][1].origin == Vector2(0.6, 0.0)
    # Newton's third law must hold exactly (same config, symmetric pair).
    assert pairwise_ba.x == pytest.approx(-pairwise_ab.x, rel=1e-12, abs=1e-12)
    assert pairwise_ba.y == pytest.approx(-pairwise_ab.y, rel=1e-12, abs=1e-12)


def test_pairwise_mode_multiple_neighbors():
    world = World(timestep=0.05)
    world.add_robot(pose=Pose2D(0.0, 0.0, 0.0))  # robot 0
    world.add_robot(pose=Pose2D(0.6, 0.0, 0.0))  # robot 1
    world.add_robot(pose=Pose2D(0.0, 0.6, 0.0))  # robot 2
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE, pairwise=True)
    record_a = overlay[0]
    assert len(record_a) == 3  # resultant + 2 pairwise vectors
    kinds = [item.kind for item in record_a]
    assert kinds == ["resultant", "pairwise", "pairwise"]
    # Pairwise vectors follow neighbor id order (robot 1 then robot 2).
    assert_vec_approx(
        record_a[1].vector,
        pairwise_lj_force(Vector2(0.0, 0.0), Vector2(0.6, 0.0), CONFIG),
    )
    assert_vec_approx(
        record_a[2].vector,
        pairwise_lj_force(Vector2(0.0, 0.0), Vector2(0.0, 0.6), CONFIG),
    )


def test_pairwise_disabled_by_default():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    for record in overlay.values():
        assert len(record) == 1
        assert record[0].kind == "resultant"


# --- empty / isolated robots ---


def test_isolated_robot_record_has_zero_resultant():
    world = World(timestep=0.05)
    world.add_robot(pose=Pose2D(1.5, 1.5, 0.0))
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    assert set(overlay.keys()) == {0}  # record still present
    assert len(overlay[0]) == 1
    assert overlay[0][0].kind == "resultant"
    assert overlay[0][0].vector == Vector2(0.0, 0.0)
    assert overlay[0][0].origin == Vector2(1.5, 1.5)


# --- scaled_vector ---


def test_scaled_vector_short_vector_unchanged():
    v = Vector2(0.1, 0.2)  # norm ~0.224 < 0.3
    assert scaled_vector(v, 0.3) is v  # same object, unchanged


def test_scaled_vector_long_vector_scaled_to_exact_max_length():
    v = Vector2(3.0, 4.0)  # norm 5.0
    out = scaled_vector(v, 1.0)
    assert out.norm() == pytest.approx(1.0)
    assert out.x == pytest.approx(0.6)
    assert out.y == pytest.approx(0.8)


def test_scaled_vector_direction_preserved():
    v = Vector2(-2.0, -2.0)
    out = scaled_vector(v, 1.0)
    assert out.x < 0 and out.y < 0
    assert out.norm() == pytest.approx(1.0)


def test_scaled_vector_zero_vector_stays_zero():
    assert scaled_vector(Vector2(0.0, 0.0), 1.0) == Vector2(0.0, 0.0)


def test_scaled_vector_equal_length_unchanged():
    v = Vector2(3.0, 4.0)
    out = scaled_vector(v, 5.0)
    assert out is v  # norm == max_length -> not scaled


def test_scaled_vector_invalid_max_length():
    for bad in (0.0, -1.0, math.nan, math.inf):
        with pytest.raises(ValueError):
            scaled_vector(Vector2(1.0, 0.0), bad)


# --- build_frame_items rendering ---


def _lj_arrow_items(items: list[dict]) -> list[dict]:
    return [
        i
        for i in items
        if i["type"] == "lines"
        and i["color"] in (RESULTANT_VECTOR_COLOR, PAIRWISE_VECTOR_COLOR)
    ]


def test_arrow_lines_emitted_with_overlay():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    items = build_frame_items(world, lj_overlay=overlay)
    arrows = _lj_arrow_items(items)
    assert any(i["color"] == RESULTANT_VECTOR_COLOR for i in arrows)
    resultant_item = next(
        i for i in arrows if i["color"] == RESULTANT_VECTOR_COLOR
    )
    # Robot 0's arrow: origin (0,0) -> origin + scaled_vector(resultant, 0.3).
    scaled = scaled_vector(overlay[0][0].vector, 0.3)
    assert [0.0, 0.0] in [segment[0] for segment in resultant_item["lines"]]
    assert [scaled.x, scaled.y] in [
        segment[1] for segment in resultant_item["lines"]
    ]
    # No pairwise arrows in default (pairwise=False) mode.
    assert not any(i["color"] == PAIRWISE_VECTOR_COLOR for i in arrows)


def test_pairwise_arrows_emitted_in_pairwise_mode():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE, pairwise=True)
    items = build_frame_items(world, lj_overlay=overlay)
    arrows = _lj_arrow_items(items)
    assert any(i["color"] == PAIRWISE_VECTOR_COLOR for i in arrows)
    pairwise_item = next(i for i in arrows if i["color"] == PAIRWISE_VECTOR_COLOR)
    scaled_ab = scaled_vector(overlay[0][1].vector, 0.3)
    assert [scaled_ab.x, scaled_ab.y] in [
        segment[1] for segment in pairwise_item["lines"]
    ]


def test_no_arrow_lines_when_overlay_none():
    world = _two_robot_world()
    # Debug on (neighbor links) but no overlay -> no LJ arrows at all.
    items = build_frame_items(world, debug=True, detection_range=1.0)
    assert _lj_arrow_items(items) == []


def test_arrows_respect_offset():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    items = build_frame_items(world, lj_overlay=overlay, offset=(2.5, 2.5))
    resultant_item = next(
        i
        for i in _lj_arrow_items(items)
        if i["color"] == RESULTANT_VECTOR_COLOR
    )
    scaled = scaled_vector(overlay[0][0].vector, 0.3)
    assert [-2.5, -2.5] in [segment[0] for segment in resultant_item["lines"]]
    assert [scaled.x - 2.5, scaled.y - 2.5] in [
        segment[1] for segment in resultant_item["lines"]
    ]


def test_custom_arrow_max_length_shortens_arrows():
    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    items = build_frame_items(world, lj_overlay=overlay, arrow_max_length=0.1)
    resultant_item = next(
        i
        for i in _lj_arrow_items(items)
        if i["color"] == RESULTANT_VECTOR_COLOR
    )
    scaled = scaled_vector(overlay[0][0].vector, 0.1)
    assert [scaled.x, scaled.y] in [
        segment[1] for segment in resultant_item["lines"]
    ]


def test_build_frame_items_validation():
    world = _two_robot_world()
    with pytest.raises(TypeError):
        build_frame_items(world, lj_overlay=[(0, [])])  # not a dict
    with pytest.raises(ValueError):
        build_frame_items(world, arrow_max_length=0.0)
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    overlay[0] = [
        LJDebugVector(Vector2(0.0, 0.0), Vector2(1.0, 0.0), "bogus")
    ]
    with pytest.raises(ValueError):
        build_frame_items(world, lj_overlay=overlay)


def test_layka_world_view_passes_overlay_through():
    from layka.sim_view import LaykaWorldView

    class FakeFrame:
        def __init__(self):
            self.lines = []

        def add_circle(self, pos, radius, color, alpha):
            pass

        def add_lines(self, lines, linewidth, color, alpha):
            self.lines.append((lines, linewidth, color, alpha))

        def add_polygons(self, polygons, color, alpha):
            pass

    class FakeViewer:
        current_frame = None

    world = _two_robot_world()
    overlay = lj_debug_overlay(world, CONFIG, DETECTION_RANGE)
    viewer = FakeViewer()
    view = LaykaWorldView(world, viewer)
    # Default: overlay disabled -> no arrow lines emitted.
    viewer.current_frame = FakeFrame()
    view.draw_world_to_frame()
    assert not any(
        color in (RESULTANT_VECTOR_COLOR, PAIRWISE_VECTOR_COLOR)
        for (_, _, color, _) in viewer.current_frame.lines
    )
    # Enable via the settable property (mirrors sensor_readings handling).
    view.lj_overlay = overlay
    viewer.current_frame = FakeFrame()
    view.draw_world_to_frame()
    assert any(
        color == RESULTANT_VECTOR_COLOR for (_, _, color, _) in viewer.current_frame.lines
    )
    # Disable again via the setter.
    view.lj_overlay = None
    viewer.current_frame = FakeFrame()
    view.draw_world_to_frame()
    assert not any(
        color in (RESULTANT_VECTOR_COLOR, PAIRWISE_VECTOR_COLOR)
        for (_, _, color, _) in viewer.current_frame.lines
    )


# --- purity / determinism ---


def test_overlay_does_not_mutate_world():
    world = _two_robot_world()
    time_before = world.time
    step_before = world.step_count
    poses_before = [world.robot_by_id(i).pose for i in (0, 1)]
    lj_debug_overlay(world, CONFIG, DETECTION_RANGE, pairwise=True)
    assert world.time == time_before
    assert world.step_count == step_before
    assert world.robot_count == 2
    assert [world.robot_by_id(i).pose for i in (0, 1)] == poses_before


def test_overlay_is_deterministic():
    world = _two_robot_world()
    first = lj_debug_overlay(world, CONFIG, DETECTION_RANGE, pairwise=True)
    second = lj_debug_overlay(world, CONFIG, DETECTION_RANGE, pairwise=True)
    assert first == second

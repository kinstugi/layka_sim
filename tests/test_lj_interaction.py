"""Unit tests for M2.4 (pairwise/resultant LJ force vectors) and M2.5 (the
LJInteraction component: poses -> resultant vector).

M2.4 coverage follows the plan.md acceptance criteria: one neighbor to the
left, one neighbor to the right, one neighbor above, two symmetric neighbors
(attractive and repulsive regions), three asymmetric neighbors, co-located
robots (r == 0), empty neighbor lists, magnitude/sign sanity against
``safe_lj_force``, M2.2 clamp integration, determinism, and controller
independence (the pure functions import no robot/world/controller code).

M2.5 coverage checks that ``LJInteraction`` delegates to the M2.4 functions,
returns the same vectors for single/multiple/empty neighbor lists, keeps the
attractive/repulsive sign convention, is deterministic, has no side effects
(inputs unchanged, returns only a Vector2 -- it never sets wheel velocities;
motion is the M2.6 controller's job), and that ``compute_from_neighbors``
reconstructs absolute positions from ``Neighbor.relative_position`` (a
world-frame displacement per M1.7).

Sign convention (pinned here): the M2.1 scalar ``f = -dV/dr`` is positive-
repulsive / negative-attractive and acts in the direction of increasing
separation (away from the other robot). With
``direction = (other - self) / r`` the force on self is ``F_AB = -f * direction``:

* r > r_eq (attractive, f < 0): F_AB points from self TOWARD the neighbor;
* r < r_eq (repulsive, f > 0):  F_AB points from self AWAY from the neighbor.

The test config raises ``max_force`` to 1e6 so the repulsive-region distance
``r = 0.20`` (raw |F| ~ 2.4e5) is NOT clamped by the M2.2 safeguard: these
tests must exercise the true LJ formula, not the clamp. With the default
``max_force = 10.0`` every repulsive distance below ~0.375 m is clamped for
``desired_spacing = 0.40``, so the unclamped config is deliberate.
"""

import inspect
import math

import pytest

from layka.config import LennardJonesConfig
from layka.lj_interaction import (
    LJInteraction,
    pairwise_lj_force,
    resultant_lj_force,
)
from layka.lj_safety import safe_lj_force
from layka.neighbors import Neighbor
from layka.pose import Pose2D
from layka.vector import Vector2
import layka.lj_interaction as lj_interaction

#: Unclamped test config: desired_spacing = 0.40 m -> sigma = 0.40/2^(1/6),
#: r_eq = 2^(1/6)*sigma = 0.40 m. max_force raised so repulsive-region
#: distances stay raw (default max_force = 10.0 would clamp them).
CONFIG = LennardJonesConfig(desired_spacing=0.40, max_force=1e6)

ORIGIN = Vector2(0.0, 0.0)

#: Distances clearly inside each LJ region, far from the min_distance (0.01)
#: and max_force (1e6) clamps.
R_ATTRACTIVE = 0.60  # > r_eq = 0.40  -> f < 0 (attractive)
R_REPULSIVE = 0.20  #  < r_eq = 0.40  -> f > 0 (repulsive)


def f_attr() -> float:
    """Signed LJ scalar in the attractive region (negative)."""
    return safe_lj_force(R_ATTRACTIVE, CONFIG)


def f_rep() -> float:
    """Signed LJ scalar in the repulsive region (positive)."""
    return safe_lj_force(R_REPULSIVE, CONFIG)


def assert_vec_approx(actual: Vector2, expected: Vector2) -> None:
    assert actual.x == pytest.approx(expected.x, rel=1e-12, abs=1e-12)
    assert actual.y == pytest.approx(expected.y, rel=1e-12, abs=1e-12)


# --- one neighbor to the left (attractive region) ---


def test_one_neighbor_to_the_left_attracts_toward_the_neighbor():
    f = f_attr()
    assert f < 0  # sanity: we are in the attractive region
    # Neighbor at (-0.6, 0): direction (-1, 0); F_AB = -f * direction = (f, 0).
    force = pairwise_lj_force(ORIGIN, Vector2(-R_ATTRACTIVE, 0.0), CONFIG)
    assert force.x == pytest.approx(f)  # f < 0 -> -x, toward the left neighbor
    assert force.y == pytest.approx(0.0)
    # Hard-coded hand value (f = safe_lj_force(0.6, config) = -1.60168...).
    assert_vec_approx(force, Vector2(-1.601682971392874, 0.0))


# --- one neighbor to the right (attractive region) ---


def test_one_neighbor_to_the_right_attracts_toward_the_neighbor():
    f = f_attr()
    assert f < 0
    # Neighbor at (0.6, 0): direction (1, 0); F_AB = -f * direction = (-f, 0).
    force = pairwise_lj_force(ORIGIN, Vector2(R_ATTRACTIVE, 0.0), CONFIG)
    assert force.x == pytest.approx(-f)  # -f > 0 -> +x, toward the right neighbor
    assert force.y == pytest.approx(0.0)


def test_attraction_points_toward_the_neighbor_on_both_sides():
    # A central potential must pull toward the neighbor regardless of side.
    # Left neighbor -> -x; right neighbor -> +x. (This is the M2.4 sign
    # sanity check; the M2.1 scalar is positive-repulsive, so the vector
    # force on self is -f * direction, not f * direction.)
    left = pairwise_lj_force(ORIGIN, Vector2(-R_ATTRACTIVE, 0.0), CONFIG)
    right = pairwise_lj_force(ORIGIN, Vector2(R_ATTRACTIVE, 0.0), CONFIG)
    assert left.x < 0 and right.x > 0
    assert left.x == pytest.approx(-right.x)


# --- one neighbor above (attractive region) ---


def test_one_neighbor_above_attracts_toward_the_neighbor():
    f = f_attr()
    assert f < 0
    # Neighbor at (0, 0.6): direction (0, 1); F_AB = -f * direction = (0, -f).
    force = pairwise_lj_force(ORIGIN, Vector2(0.0, R_ATTRACTIVE), CONFIG)
    assert force.x == pytest.approx(0.0)
    assert force.y == pytest.approx(-f)  # -f > 0 -> +y, toward the upper neighbor


# --- two symmetric neighbors ---


def test_two_symmetric_neighbors_in_attractive_region_cancel():
    others = [Vector2(-R_ATTRACTIVE, 0.0), Vector2(R_ATTRACTIVE, 0.0)]
    result = resultant_lj_force(ORIGIN, others, CONFIG)
    assert_vec_approx(result, Vector2(0.0, 0.0))
    # Equal-and-opposite pairwise forces, each of magnitude |f_attr|.
    left = pairwise_lj_force(ORIGIN, others[0], CONFIG)
    right = pairwise_lj_force(ORIGIN, others[1], CONFIG)
    assert left.norm() == pytest.approx(abs(f_attr()))
    assert left == -right


def test_two_symmetric_neighbors_in_repulsive_region_cancel():
    others = [Vector2(-R_REPULSIVE, 0.0), Vector2(R_REPULSIVE, 0.0)]
    result = resultant_lj_force(ORIGIN, others, CONFIG)
    assert_vec_approx(result, Vector2(0.0, 0.0))
    # Individual forces are nonzero (repulsive) before cancellation.
    assert pairwise_lj_force(ORIGIN, others[1], CONFIG).x == pytest.approx(-f_rep())


def test_two_symmetric_neighbors_off_axis_cancel():
    # Symmetry must hold for any axis: neighbors at (0, d) and (0, -d).
    others = [Vector2(0.0, R_ATTRACTIVE), Vector2(0.0, -R_ATTRACTIVE)]
    result = resultant_lj_force(ORIGIN, others, CONFIG)
    assert_vec_approx(result, Vector2(0.0, 0.0))


# --- three asymmetric neighbors ---


def test_three_asymmetric_neighbors_sum_pairwise_forces():
    f = f_attr()
    assert f < 0
    others = [
        Vector2(R_ATTRACTIVE, 0.0),  # direction (1, 0)  -> F = (-f, 0)
        Vector2(0.0, R_ATTRACTIVE),  # direction (0, 1)  -> F = (0, -f)
        Vector2(0.0, -R_ATTRACTIVE),  # direction (0, -1) -> F = (0, f)
    ]
    result = resultant_lj_force(ORIGIN, others, CONFIG)
    # The upper and lower neighbors cancel vertically; the right neighbor
    # dominates: resultant = (-f, 0) + (0, -f) + (0, f) = (-f, 0).
    assert_vec_approx(result, Vector2(-f, 0.0))
    # Same as summing the three pairwise vectors explicitly.
    manual = Vector2(0.0, 0.0)
    for other in others:
        manual = manual + pairwise_lj_force(ORIGIN, other, CONFIG)
    assert_vec_approx(result, manual)


# --- r == 0 (co-located robots) ---


def test_colocated_robots_produce_zero_pairwise_force():
    force = pairwise_lj_force(ORIGIN, ORIGIN, CONFIG)
    assert force == Vector2(0.0, 0.0)


def test_colocated_neighbor_in_resultant_contributes_zero():
    others = [ORIGIN, Vector2(R_ATTRACTIVE, 0.0)]
    result = resultant_lj_force(ORIGIN, others, CONFIG)
    expected = pairwise_lj_force(ORIGIN, Vector2(R_ATTRACTIVE, 0.0), CONFIG)
    assert result == expected


# --- empty neighbor list ---


def test_resultant_force_with_no_neighbors_is_zero():
    assert resultant_lj_force(ORIGIN, [], CONFIG) == Vector2(0.0, 0.0)
    assert resultant_lj_force(ORIGIN, (), CONFIG) == Vector2(0.0, 0.0)


# --- magnitude/sign sanity against safe_lj_force ---


def test_repulsive_force_magnitude_and_direction():
    f = f_rep()
    assert f > 0  # sanity: repulsive region
    direction = Vector2(1.0, 0.0)  # unit vector self -> other
    force = pairwise_lj_force(ORIGIN, Vector2(R_REPULSIVE, 0.0), CONFIG)
    assert force.norm() == pytest.approx(f)  # magnitude == safe_lj_force
    assert force.dot(direction) == pytest.approx(-f)  # opposite to direction: AWAY


def test_attractive_force_magnitude_and_direction():
    f = f_attr()
    assert f < 0  # sanity: attractive region
    direction = Vector2(1.0, 0.0)
    force = pairwise_lj_force(ORIGIN, Vector2(R_ATTRACTIVE, 0.0), CONFIG)
    assert force.norm() == pytest.approx(-f)  # magnitude == |safe_lj_force|
    assert force.dot(direction) == pytest.approx(-f)  # along direction: TOWARD


def test_resultant_is_additive_with_safe_lj_force():
    # Linearity check in the attractive region: the resultant of two
    # neighbors on the same side is the sum of their magnitudes.
    others = [Vector2(R_ATTRACTIVE, 0.0), Vector2(2.0 * R_ATTRACTIVE, 0.0)]
    result = resultant_lj_force(ORIGIN, others, CONFIG)
    expected = (
        -safe_lj_force(R_ATTRACTIVE, CONFIG) + -safe_lj_force(2.0 * R_ATTRACTIVE, CONFIG)
    )
    assert result.y == pytest.approx(0.0)
    assert result.x == pytest.approx(expected)


# --- M2.2 safety integration (default config clamps) ---


def test_default_config_clamped_force_is_finite_and_direction_correct():
    config = LennardJonesConfig(desired_spacing=0.40)  # default max_force = 10.0
    force = pairwise_lj_force(ORIGIN, Vector2(R_REPULSIVE, 0.0), config)
    assert math.isfinite(force.x) and math.isfinite(force.y)
    assert force.norm() == pytest.approx(config.max_force)  # clamped to 10.0
    assert force.x < 0  # still points AWAY from the right-hand neighbor
    assert force.y == pytest.approx(0.0)


def test_r_above_cutoff_produces_zero_vector():
    config = LennardJonesConfig(desired_spacing=0.40, cutoff_distance=0.5)
    force = pairwise_lj_force(ORIGIN, Vector2(0.6, 0.0), config)
    assert force == Vector2(0.0, 0.0)


# --- determinism (pure functions) ---


def test_pairwise_and_resultant_are_deterministic():
    others = [Vector2(-0.6, 0.0), Vector2(0.3, 0.4), Vector2(0.2, -0.2)]
    assert pairwise_lj_force(ORIGIN, others[0], CONFIG) == pairwise_lj_force(
        ORIGIN, others[0], CONFIG
    )
    assert resultant_lj_force(ORIGIN, others, CONFIG) == resultant_lj_force(
        ORIGIN, others, CONFIG
    )


# --- independence from robot controllers / world ---


def test_module_imports_no_robot_world_or_controller_code():
    source = inspect.getsource(lj_interaction)
    # "pose" and "neighbors" are deliberately NOT in the forbidden list: the
    # M2.5 LJInteraction component (same module) consumes Pose2D poses and
    # Neighbor records. The remaining tokens pin the M2.4 invariant that the
    # LJ math never couples to robot/controller/world/kinematics/UI code.
    forbidden = (
        "robot",
        "world",
        "behavior",
        "controller",
        "kinematics",
        "renderer",
        "gui",
        "view",
    )
    for line in source.splitlines():
        stripped = line.strip()
        if stripped.startswith("import ") or stripped.startswith("from "):
            assert not any(token in stripped for token in forbidden), (
                f"unexpected dependency in layka/lj_interaction.py: {stripped!r}"
            )


# --- M2.5 LJInteraction component ---


def make_component() -> LJInteraction:
    return LJInteraction(CONFIG)


# --- single neighbors: component delegates to the M2.4 pure functions ---


def test_component_single_neighbor_left_matches_pairwise_force():
    component = make_component()
    self_pose = Pose2D(0.0, 0.0, 0.0)
    neighbor = Pose2D(-R_ATTRACTIVE, 0.0, 0.0)
    expected = pairwise_lj_force(self_pose.position(), neighbor.position(), CONFIG)
    assert_vec_approx(component.compute(self_pose, [neighbor]), expected)


def test_component_single_neighbor_right_matches_pairwise_force():
    component = make_component()
    self_pose = Pose2D(0.0, 0.0, 0.0)
    neighbor = Pose2D(R_ATTRACTIVE, 0.0, 0.0)
    expected = pairwise_lj_force(self_pose.position(), neighbor.position(), CONFIG)
    assert_vec_approx(component.compute(self_pose, [neighbor]), expected)


def test_component_single_neighbor_above_matches_pairwise_force():
    component = make_component()
    self_pose = Pose2D(0.0, 0.0, 0.0)
    neighbor = Pose2D(0.0, R_ATTRACTIVE, 0.0)
    expected = pairwise_lj_force(self_pose.position(), neighbor.position(), CONFIG)
    assert_vec_approx(component.compute(self_pose, [neighbor]), expected)


# --- multiple / empty neighbor lists ---


def test_component_multiple_neighbors_matches_resultant_force():
    component = make_component()
    self_pose = Pose2D(0.0, 0.0, 0.0)
    neighbors = [
        Pose2D(-R_ATTRACTIVE, 0.0, 0.0),
        Pose2D(0.0, R_ATTRACTIVE, 0.0),
        Pose2D(0.3, 0.4, 0.0),
        Pose2D(0.2, -0.2, 0.0),
    ]
    expected = resultant_lj_force(
        self_pose.position(), [n.position() for n in neighbors], CONFIG
    )
    assert_vec_approx(component.compute(self_pose, neighbors), expected)


def test_component_empty_neighbor_list_is_zero_vector():
    component = make_component()
    assert component.compute(Pose2D(0.0, 0.0, 0.0), []) == Vector2(0.0, 0.0)


# --- sign convention preserved (M2.4 sign, reviewer-verified) ---


def test_component_repulsive_neighbor_points_away():
    component = make_component()
    self_pose = Pose2D(0.0, 0.0, 0.0)
    neighbor = Pose2D(R_REPULSIVE, 0.0, 0.0)
    result = component.compute(self_pose, [neighbor])
    direction = neighbor.position() - self_pose.position()  # self -> other
    assert result.dot(direction) < 0  # opposite the direction: AWAY
    assert_vec_approx(
        result, pairwise_lj_force(self_pose.position(), neighbor.position(), CONFIG)
    )


def test_component_attractive_neighbor_points_toward():
    component = make_component()
    self_pose = Pose2D(0.0, 0.0, 0.0)
    neighbor = Pose2D(R_ATTRACTIVE, 0.0, 0.0)
    result = component.compute(self_pose, [neighbor])
    direction = neighbor.position() - self_pose.position()  # self -> other
    assert result.dot(direction) > 0  # along the direction: TOWARD
    assert_vec_approx(
        result, pairwise_lj_force(self_pose.position(), neighbor.position(), CONFIG)
    )


# --- separation of concerns: returns a vector, no side effects, no motion ---


def test_component_returns_vector2_and_does_not_mutate_inputs():
    component = make_component()
    self_pose = Pose2D(0.0, 0.0, 0.5)
    neighbor = Pose2D(R_ATTRACTIVE, 0.0, -0.3)
    result = component.compute(self_pose, [neighbor])
    # The component's contract: it returns a Vector2 (a desired interaction
    # vector), NOT a velocity, wheel speed, or RobotState. Motion is the
    # M2.6 controller's job; the component never sets it.
    assert isinstance(result, Vector2)
    assert result == resultant_lj_force(
        self_pose.position(), [neighbor.position()], CONFIG
    )
    # No side effects: input poses (position AND heading) are unchanged.
    assert self_pose == Pose2D(0.0, 0.0, 0.5)
    assert neighbor == Pose2D(R_ATTRACTIVE, 0.0, -0.3)


def test_component_is_deterministic_across_repeated_calls():
    component = make_component()
    self_pose = Pose2D(0.0, 0.0, 0.0)
    neighbors = [
        Pose2D(-R_ATTRACTIVE, 0.0, 0.0),
        Pose2D(0.3, 0.4, 0.0),
        Pose2D(0.2, -0.2, 0.0),
    ]
    first = component.compute(self_pose, neighbors)
    for _ in range(5):
        assert component.compute(self_pose, neighbors) == first


def test_component_exposes_readonly_config():
    component = make_component()
    assert component.config is CONFIG
    assert component.config.sigma == pytest.approx(CONFIG.sigma)
    assert component.config.equilibrium_distance == pytest.approx(CONFIG.equilibrium_distance)


# --- compute_from_neighbors: Neighbor records -> same vector as poses ---


def test_component_from_neighbors_matches_equivalent_absolute_poses():
    component = make_component()
    self_pose = Pose2D(1.0, -2.0, 0.0)
    neighbor_poses = [
        Pose2D(1.0 + R_ATTRACTIVE, -2.0, 0.0),
        Pose2D(1.0 - R_REPULSIVE, -2.0, 0.0),
        Pose2D(1.0 + 0.3, -2.0 + 0.4, 0.0),
    ]
    neighbors = [
        Neighbor(
            neighbor_id=i,
            relative_position=n.position() - self_pose.position(),
            distance=(n.position() - self_pose.position()).norm(),
        )
        for i, n in enumerate(neighbor_poses)
    ]
    # relative_position is a world-frame displacement (M1.7), so reconstructing
    # absolute positions and delegating must equal the direct pose path.
    assert_vec_approx(
        component.compute_from_neighbors(self_pose, neighbors),
        component.compute(self_pose, neighbor_poses),
    )


def test_component_from_neighbors_empty_is_zero_vector():
    component = make_component()
    assert component.compute_from_neighbors(Pose2D(0.0, 0.0, 0.0), []) == Vector2(
        0.0, 0.0
    )

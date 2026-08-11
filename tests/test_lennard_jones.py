"""Unit tests for the pure Lennard-Jones potential/force module (M2.1).

Covers the 8 required cases from plan.md M2.1: r <= 0 handling, very small
distances, r = sigma, r = 2^(1/6)*sigma, large distances, attractive region,
repulsive region, and force direction/sign. Also covers parameter validation
(epsilon/sigma/r must be finite and positive) and a finite-difference
``F = -dV/dr`` consistency check, satisfying the "verify the implementation
mathematically rather than copying" instruction.
"""

import math

import pytest

from layka.lennard_jones import (
    TWO_ONE_SIXTH,
    lennard_jones_force,
    lennard_jones_potential,
)

EPSILON = 1.0
SIGMA = 1.0
R_EQ = TWO_ONE_SIXTH * SIGMA  # 2^(1/6) * sigma


# --- Case 1: r <= 0 handling (choice: raise ValueError) ---


@pytest.mark.parametrize("func", [lennard_jones_potential, lennard_jones_force])
def test_nonpositive_r_raises(func):
    for bad_r in (0.0, -1e-9, -1.0, -math.inf):
        with pytest.raises(ValueError):
            func(bad_r, EPSILON, SIGMA)


@pytest.mark.parametrize("func", [lennard_jones_potential, lennard_jones_force])
def test_nonfinite_r_raises(func):
    for bad_r in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            func(bad_r, EPSILON, SIGMA)


# --- Case 2: very small distances (repulsive, large, finite) ---


@pytest.mark.parametrize("r", [1e-6 * SIGMA, 1e-9 * SIGMA])
def test_very_small_distances_are_large_positive_and_finite(r):
    potential = lennard_jones_potential(r, EPSILON, SIGMA)
    force = lennard_jones_force(r, EPSILON, SIGMA)
    assert math.isfinite(potential) and potential > 0
    assert math.isfinite(force) and force > 0
    assert potential > EPSILON  # far above the well depth |V(r_eq)| = epsilon
    assert force > EPSILON / SIGMA


def test_very_small_distances_grow_monotonically_as_r_shrinks():
    smaller = 1e-9 * SIGMA
    larger = 1e-6 * SIGMA
    assert (
        lennard_jones_potential(smaller, EPSILON, SIGMA)
        > lennard_jones_potential(larger, EPSILON, SIGMA)
        > 0
    )
    assert (
        lennard_jones_force(smaller, EPSILON, SIGMA)
        > lennard_jones_force(larger, EPSILON, SIGMA)
        > 0
    )


# --- Case 3: r = sigma (zero-crossing of the potential) ---


def test_at_sigma_potential_is_zero():
    assert lennard_jones_potential(SIGMA, EPSILON, SIGMA) == pytest.approx(0.0)


def test_at_sigma_force_is_24_epsilon_over_sigma():
    assert lennard_jones_force(SIGMA, EPSILON, SIGMA) == pytest.approx(
        24.0 * EPSILON / SIGMA
    )


# --- Case 4: r = 2^(1/6) * sigma (zero-force equilibrium, well depth) ---


def test_at_equilibrium_force_is_zero_and_potential_is_minus_epsilon():
    # Hand check: at r_eq = 2^(1/6)*sigma, (sigma/r)^6 = 1/2 and
    # (sigma/r)^12 = 1/4, so the force bracket 2*(1/4) - (1/2) == 0 exactly
    # and V = 4*epsilon*(1/4 - 1/2) = -epsilon. The float computation lands
    # within ~1e-15 of zero (verified numerically), hence the abs tolerance.
    assert lennard_jones_force(R_EQ, EPSILON, SIGMA) == pytest.approx(0.0, abs=1e-9)
    assert lennard_jones_potential(R_EQ, EPSILON, SIGMA) == pytest.approx(-EPSILON)


def test_equilibrium_force_zero_bracket_holds_exactly_in_math():
    sigma_over_r_6 = 0.5
    sigma_over_r_12 = 0.25
    assert 2.0 * sigma_over_r_12 - sigma_over_r_6 == 0.0
    assert 4.0 * (sigma_over_r_12 - sigma_over_r_6) == -1.0


# --- Case 5: large distances (attractive, tending to zero) ---


def test_large_distance_potential_and_force_tend_to_zero_from_below():
    r = 10.0 * SIGMA
    potential = lennard_jones_potential(r, EPSILON, SIGMA)
    force = lennard_jones_force(r, EPSILON, SIGMA)
    assert potential < 0 and abs(potential) < 1e-4 * EPSILON
    assert force < 0 and abs(force) < 1e-4 * EPSILON / SIGMA


# --- Case 6: attractive region (r > r_eq) ---


def test_attractive_region_beyond_equilibrium():
    r = 2.0 * SIGMA
    assert lennard_jones_force(r, EPSILON, SIGMA) < 0
    potential = lennard_jones_potential(r, EPSILON, SIGMA)
    assert potential < 0 and potential > -EPSILON


# --- Case 7: repulsive region (r < r_eq) ---


def test_repulsive_region_below_equilibrium():
    r = 0.5 * SIGMA
    assert lennard_jones_force(r, EPSILON, SIGMA) > 0
    assert lennard_jones_potential(r, EPSILON, SIGMA) > 0


# --- Case 8: force direction/sign and monotonicity spot check ---


def test_force_is_repulsive_below_and_attractive_above_equilibrium():
    for r in (0.25 * SIGMA, 0.5 * SIGMA, 0.75 * SIGMA, 0.99 * SIGMA, 1.1 * SIGMA):
        assert r < R_EQ
        assert lennard_jones_force(r, EPSILON, SIGMA) > 0
    for r in (1.15 * SIGMA, 1.25 * SIGMA, 1.5 * SIGMA, 2.0 * SIGMA, 5.0 * SIGMA, 10.0 * SIGMA):
        assert r > R_EQ
        assert lennard_jones_force(r, EPSILON, SIGMA) < 0
    assert lennard_jones_force(R_EQ, EPSILON, SIGMA) == pytest.approx(0.0, abs=1e-9)


def test_force_is_monotonically_decreasing_through_repulsive_region():
    rs = (0.5 * SIGMA, 0.9 * SIGMA, 0.99 * SIGMA, SIGMA, 1.1 * SIGMA)
    forces = [lennard_jones_force(r, EPSILON, SIGMA) for r in rs]
    assert all(f > 0 for f in forces)
    assert forces == sorted(forces, reverse=True)


# --- parameter validation ---


@pytest.mark.parametrize("func", [lennard_jones_potential, lennard_jones_force])
def test_invalid_epsilon_raises(func):
    for bad in (0.0, -1.0, math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            func(1.0, bad, SIGMA)


@pytest.mark.parametrize("func", [lennard_jones_potential, lennard_jones_force])
def test_invalid_sigma_raises(func):
    for bad in (0.0, -1.0, math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            func(1.0, EPSILON, bad)


# --- mathematical verification: F = -dV/dr (finite difference) ---


@pytest.mark.parametrize(
    "r", [0.6 * SIGMA, SIGMA, R_EQ, 1.5 * SIGMA, 3.0 * SIGMA]
)
def test_force_matches_negative_gradient_of_potential(r):
    h = 1e-6 * r
    numerical = -(
        lennard_jones_potential(r + h, EPSILON, SIGMA)
        - lennard_jones_potential(r - h, EPSILON, SIGMA)
    ) / (2.0 * h)
    assert lennard_jones_force(r, EPSILON, SIGMA) == pytest.approx(
        numerical, rel=1e-6, abs=1e-9
    )


# --- determinism (pure function) ---


def test_pure_function_is_deterministic():
    args = (1.37, 0.5, 0.356)
    assert lennard_jones_potential(*args) == lennard_jones_potential(*args)
    assert lennard_jones_force(*args) == lennard_jones_force(*args)

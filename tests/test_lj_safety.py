"""Unit tests for the M2.2 numerical-safety guards (``layka.lj_safety``).

Covers the plan.md M2.2 acceptance criterion (no NaN/inf over a dense r
sweep), the min-distance clamp (including r == 0 and negative r), the
max-force clamp with sign preservation, the optional cutoff (including the
pinned r == cutoff boundary behavior), NaN/inf input rejection, config
validation, clamp logging, defaults, and consistency with the pure M2.1
module in the healthy (unclamped) region.
"""

import logging
import math

import pytest
from pydantic import ValidationError

from layka.config import LennardJonesConfig, TWO_ONE_SIXTH
from layka.lennard_jones import lennard_jones_force, lennard_jones_potential
from layka.lj_safety import clamp, safe_lj_force, safe_lj_potential

EPSILON = 1.0
#: Typical M2.3-style config: desired_spacing = 0.40 m -> sigma = 0.40 / 2^(1/6).
SIGMA = 0.4 / TWO_ONE_SIXTH
R_EQ = TWO_ONE_SIXTH * SIGMA  # == 0.4


def make_config(**overrides) -> LennardJonesConfig:
    kwargs = {"desired_spacing": 0.4}
    kwargs.update(overrides)
    return LennardJonesConfig(**kwargs)


def _raw_force_at_min_distance(config: LennardJonesConfig) -> float:
    """Raw (unclamped) pure LJ force at the clamped floor r = min_distance."""
    return lennard_jones_force(config.min_distance, config.epsilon, config.sigma)


# --- acceptance criterion: no NaN or inf over a dense r sweep ---


def test_no_nan_or_inf_over_dense_log_sweep():
    config = make_config()
    # Dense log-spaced sweep from 1e-12 m to well past the attractive tail,
    # including the exact points required by the task list.
    rs: list[float] = [1e-12, 1e-6, config.min_distance / 2, config.min_distance]
    rs.append(SIGMA)
    rs.append(R_EQ)
    rs.append(2 * SIGMA)
    rs.append(10 * SIGMA)
    r = 1e-12
    while r <= 10 * SIGMA:
        rs.append(r)
        r *= 1.15
    for r in rs:
        force = safe_lj_force(r, config)
        potential = safe_lj_potential(r, config)
        assert math.isfinite(force), f"non-finite force at r={r!r}: {force!r}"
        assert math.isfinite(potential), f"non-finite potential at r={r!r}: {potential!r}"


def test_no_nan_or_inf_around_cutoff():
    config = make_config(cutoff_distance=2.0)
    for r in (1.0, config.cutoff_distance / 2, config.cutoff_distance, 2.5, 10 * SIGMA):
        force = safe_lj_force(r, config)
        potential = safe_lj_potential(r, config)
        assert math.isfinite(force), f"non-finite force at r={r!r}"
        assert math.isfinite(potential), f"non-finite potential at r={r!r}"


# --- min-distance clamp ---


def test_r_below_min_distance_is_clamped_to_min_distance():
    config = make_config()
    expected_force = clamp(
        _raw_force_at_min_distance(config),
        -config.max_force,
        config.max_force,
    )
    expected_potential = lennard_jones_potential(
        config.min_distance, config.epsilon, config.sigma
    )
    for r in (config.min_distance / 2, 1e-12, 0.0, -1.0):
        force = safe_lj_force(r, config)
        potential = safe_lj_potential(r, config)
        assert force == pytest.approx(expected_force), f"r={r!r}"
        assert potential == pytest.approx(expected_potential), f"r={r!r}"
        assert math.isfinite(force) and math.isfinite(potential), f"r={r!r}"


def test_r_equal_min_distance_is_not_clamped():
    config = make_config()
    assert safe_lj_force(config.min_distance, config) == pytest.approx(
        clamp(_raw_force_at_min_distance(config), -config.max_force, config.max_force)
    )


# --- max-force clamp ---


def test_extreme_small_r_clamps_to_max_force_with_positive_sign():
    config = make_config()
    # Raw force at 1e-9 vastly exceeds max_force; min-distance clamp also
    # applies, so the result is exactly +max_force (repulsive sign preserved).
    assert safe_lj_force(1e-9, config) == pytest.approx(config.max_force)


def test_attractive_force_below_max_force_is_not_clamped():
    config = make_config()
    r = 2 * SIGMA
    raw = lennard_jones_force(r, config.epsilon, config.sigma)
    assert raw < 0  # attractive region
    assert abs(raw) < config.max_force
    assert safe_lj_force(r, config) == pytest.approx(raw)


# --- cutoff ---


def test_r_above_cutoff_returns_zero_for_both_functions():
    config = make_config(cutoff_distance=2.0)
    r = 2.5
    assert safe_lj_force(r, config) == pytest.approx(0.0)
    assert safe_lj_potential(r, config) == pytest.approx(0.0)


def test_cutoff_boundary_r_equal_cutoff_is_still_computed():
    # Pinned boundary choice: cutoff excludes r > cutoff only; r == cutoff
    # is still evaluated (nonzero attractive force for this config).
    config = make_config(cutoff_distance=2.0)
    force_at_boundary = safe_lj_force(config.cutoff_distance, config)
    assert force_at_boundary != pytest.approx(0.0, abs=1e-12)
    assert force_at_boundary < 0
    assert safe_lj_force(1.9, config) != pytest.approx(0.0, abs=1e-12)


def test_r_below_cutoff_in_repulsive_region_is_nonzero():
    config = make_config(cutoff_distance=2.0)
    r = 0.2  # < sigma so both potential and force are positive (repulsive)
    assert safe_lj_force(r, config) > 0
    assert safe_lj_potential(r, config) > 0


# --- NaN/inf input is an invalid state, not a clampable value ---


@pytest.mark.parametrize("func", [safe_lj_force, safe_lj_potential])
@pytest.mark.parametrize("bad_r", [math.nan, math.inf, -math.inf])
def test_nonfinite_r_raises_value_error(func, bad_r):
    config = make_config()
    with pytest.raises(ValueError):
        func(bad_r, config)


# --- config validation ---


@pytest.mark.parametrize("bad", [0.0, -1.0, math.nan, math.inf, -math.inf])
def test_min_distance_validation_rejects_invalid_values(bad):
    with pytest.raises(ValidationError):
        LennardJonesConfig(sigma=0.4, min_distance=bad)


def test_min_distance_accepts_finite_positive_value():
    config = LennardJonesConfig(sigma=0.4, min_distance=0.05)
    assert config.min_distance == pytest.approx(0.05)


# --- logging: clamping must not be silent ---


def test_r_clamp_logs_warning(caplog):
    config = make_config()
    with caplog.at_level(logging.WARNING, logger="layka.lj_safety"):
        safe_lj_force(config.min_distance / 2, config)
    messages = [rec.message for rec in caplog.records]
    assert any("min_distance" in m and "clamping" in m for m in messages)


def test_force_clamp_logs_warning(caplog):
    config = make_config()
    with caplog.at_level(logging.WARNING, logger="layka.lj_safety"):
        safe_lj_force(1e-9, config)
    messages = [rec.message for rec in caplog.records]
    assert any("max_force" in m and "clamping" in m for m in messages)


def test_healthy_region_logs_no_warnings(caplog):
    config = make_config()
    with caplog.at_level(logging.WARNING, logger="layka.lj_safety"):
        safe_lj_force(0.5, config)
        safe_lj_potential(0.5, config)
    assert not any(rec.levelno >= logging.WARNING for rec in caplog.records)


# --- defaults and existing M1.3 constructions ---


def test_min_distance_has_positive_default():
    config = LennardJonesConfig(sigma=0.4)  # all safety fields at their defaults
    assert config.min_distance == pytest.approx(0.01)
    assert config.min_distance > 0
    assert config.max_force == pytest.approx(10.0)
    assert config.cutoff_distance is None
    # Existing M1.3-style constructions still validate.
    assert LennardJonesConfig(desired_spacing=0.4).min_distance == pytest.approx(0.01)
    assert LennardJonesConfig(desired_spacing=0.4).sigma == pytest.approx(
        0.4 / TWO_ONE_SIXTH
    )


def test_sigma_still_required():
    with pytest.raises(ValidationError):
        LennardJonesConfig()


# --- consistency with the pure M2.1 module (healthy region) ---


def test_safe_functions_match_pure_module_in_healthy_region():
    config = make_config()
    r = 0.5  # min_distance < r < r_eq/... attractive but unclamped, no cutoff
    assert safe_lj_force(r, config) == pytest.approx(
        lennard_jones_force(r, config.epsilon, config.sigma)
    )
    assert safe_lj_potential(r, config) == pytest.approx(
        lennard_jones_potential(r, config.epsilon, config.sigma)
    )


# --- clamp helper ---


def test_clamp_helper():
    assert clamp(5.0, 0.0, 10.0) == 5.0
    assert clamp(-5.0, 0.0, 10.0) == 0.0
    assert clamp(15.0, 0.0, 10.0) == 10.0
    assert clamp(-3.0, -10.0, 10.0) == -3.0
    assert clamp(10.0, -10.0, 10.0) == 10.0


def test_clamp_helper_requires_lower_less_equal_upper():
    with pytest.raises(ValueError):
        clamp(0.0, 10.0, 0.0)


# --- determinism (pure functions) ---


def test_safe_functions_are_deterministic():
    config = make_config()
    assert safe_lj_force(0.13, config) == safe_lj_force(0.13, config)
    assert safe_lj_potential(0.13, config) == safe_lj_potential(0.13, config)

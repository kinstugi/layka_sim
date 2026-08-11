"""Numerical-safety guards for the Lennard-Jones interaction (M2.2).

The pure LJ formulas (M2.1) diverge as r -> 0 and raise ValueError for
r <= 0 or any non-finite argument. This module is the layer the simulator
calls instead: it clamps r up to a configured minimum distance, applies an
optional interaction cutoff, clamps the force magnitude, and verifies every
returned value is finite. It is pure math + config: it knows nothing about
robots, sensors, or controllers.

Safeguard order (``safe_lj_force``):

    1. r NaN/inf              -> raise ValueError (invalid state, NOT clampable)
    2. cutoff (r > cutoff)    -> return 0.0 (no interaction; r == cutoff is
                                 still computed -- cutoff excludes only r > cutoff)
    3. r < min_distance       -> r = min_distance (log a warning; this branch
                                 also covers r <= 0, so r == 0 never reaches
                                 the pure function)
    4. f = lennard_jones_force(r, epsilon, sigma)
    5. |f| > max_force        -> f = clamp(f, -max_force, +max_force)
                                 (log a warning; sign is preserved)
    6. final finite check     -> raise RuntimeError if f is not finite

``safe_lj_potential`` follows the same order for steps 1-4 and 6 but applies
NO magnitude clamp (only the force is clamped per plan.md M2.2).

Given a valid ``LennardJonesConfig`` (all values finite and positive), this
guarantees a finite force for every finite r: the pure force is bounded
because r >= min_distance > 0, and the magnitude clamp bounds it further to
[+/-max_force]. Invalid states (NaN/inf r) raise instead of being silently
clamped.
"""

from __future__ import annotations

import logging
import math

from layka.config import LennardJonesConfig
from layka.lennard_jones import lennard_jones_force, lennard_jones_potential

logger = logging.getLogger(__name__)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp ``value`` into the inclusive range ``[lower, upper]``."""
    if lower > upper:
        raise ValueError(
            f"clamp bounds must satisfy lower <= upper, got lower={lower!r}, upper={upper!r}"
        )
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def safe_lj_force(r: float, config: LennardJonesConfig) -> float:
    """Evaluate the LJ force with the M2.2 numerical-safety guards.

    Returns 0.0 beyond the optional cutoff, clamps ``r`` up to
    ``config.min_distance`` and the magnitude up to ``config.max_force``
    (sign-preserving). Raises ValueError for NaN/inf ``r`` and RuntimeError
    if the guarded result is somehow still non-finite.
    """
    if not math.isfinite(r):
        raise ValueError(f"r must be finite, got {r!r}")
    if config.cutoff_distance is not None and r > config.cutoff_distance:
        return 0.0
    r_safe = r
    if r < config.min_distance:
        logger.warning(
            "r=%.6g below min_distance=%.6g; clamping r to min_distance",
            r,
            config.min_distance,
        )
        r_safe = config.min_distance
    force = lennard_jones_force(r_safe, config.epsilon, config.sigma)
    if abs(force) > config.max_force:
        logger.warning(
            "|force|=%.6g exceeds max_force=%.6g at r=%.6g; clamping force",
            abs(force),
            config.max_force,
            r,
        )
        force = clamp(force, -config.max_force, config.max_force)
    if not math.isfinite(force):
        raise RuntimeError(
            f"non-finite LJ force produced for r={r!r} with config={config}"
        )
    return force


def safe_lj_potential(r: float, config: LennardJonesConfig) -> float:
    """Evaluate the LJ potential with the M2.2 numerical-safety guards.

    Same clamp/cutoff semantics as :func:`safe_lj_force` (0.0 beyond the
    optional cutoff, ``r`` clamped up to ``config.min_distance`` with a
    warning) but no magnitude clamp, since plan.md M2.2 only clamps the
    force. Raises ValueError for NaN/inf ``r`` and RuntimeError if the
    guarded result is somehow still non-finite.
    """
    if not math.isfinite(r):
        raise ValueError(f"r must be finite, got {r!r}")
    if config.cutoff_distance is not None and r > config.cutoff_distance:
        return 0.0
    r_safe = r
    if r < config.min_distance:
        logger.warning(
            "r=%.6g below min_distance=%.6g; clamping r to min_distance",
            r,
            config.min_distance,
        )
        r_safe = config.min_distance
    potential = lennard_jones_potential(r_safe, config.epsilon, config.sigma)
    if not math.isfinite(potential):
        raise RuntimeError(
            f"non-finite LJ potential produced for r={r!r} with config={config}"
        )
    return potential

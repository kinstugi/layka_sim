"""Pure scalar Lennard-Jones (12-6) potential and force (M2.1).

Pure mathematical functions only: they know nothing about robots, sensors,
rendering, or the simulator. The 2D vector assembly of pairwise forces is
M2.4, and the numerical-safety guards (min distance, force clamp, cutoff) are
M2.2; neither belongs here. Parameter names mirror ``LennardJonesConfig``
(``epsilon``, ``sigma``), which is deliberately NOT imported so this module
stays standalone.

Formulas (standard 12-6 Lennard-Jones, SI units):

    V(r) = 4 * epsilon * ((sigma / r)**12 - (sigma / r)**6)
    F(r) = 24 * epsilon / r * (2 * (sigma / r)**12 - (sigma / r)**6)
         = -dV/dr

Sign convention for F: positive F is REPULSIVE (the radial force pushes the
pair apart, i.e. it points in the direction of increasing r); negative F is
ATTRACTIVE. F is the scalar force along the radial direction; callers that
need a 2D vector multiply it by the radial unit vector (M2.4).

sigma vs equilibrium distance: ``sigma`` is the zero-crossing of the potential
(V(sigma) == 0); it is NOT the equilibrium distance. The potential minimum --
and therefore the zero-force equilibrium -- is at

    r_eq = 2^(1/6) * sigma

with V(r_eq) = -epsilon (the well depth) and F(r_eq) == 0. A desired
robot-to-robot spacing of 0.40 m therefore means sigma = 0.40 / 2^(1/6)
(see M2.3), never sigma = 0.40.

Singularity / invalid input handling: r <= 0 (and any non-finite r, epsilon,
or sigma) raises ValueError, consistent with the validation style elsewhere in
``layka/`` (e.g. :mod:`layka.clock`, :mod:`layka.kinematics`). The potential
diverges as r -> 0+; callers must guard against unbounded values before
reaching this module (the M2.2 min-distance/clamp logic lives in the
interaction layer, not here).
"""

from __future__ import annotations

import math

#: Factor relating sigma to the zero-force equilibrium distance:
#: ``r_eq = TWO_ONE_SIXTH * sigma``.
TWO_ONE_SIXTH = 2.0 ** (1.0 / 6.0)


def _validate_lj_params(epsilon: float, sigma: float) -> None:
    if not math.isfinite(epsilon) or epsilon <= 0:
        raise ValueError(
            f"epsilon must be a finite positive number, got {epsilon!r}"
        )
    if not math.isfinite(sigma) or sigma <= 0:
        raise ValueError(f"sigma must be a finite positive number, got {sigma!r}")


def _validate_r(r: float) -> None:
    if not math.isfinite(r) or r <= 0:
        raise ValueError(f"r must be a finite positive number, got {r!r}")


def lennard_jones_potential(r: float, epsilon: float, sigma: float) -> float:
    """Evaluate the Lennard-Jones potential ``V(r) = 4ε((σ/r)^12 − (σ/r)^6)``.

    V(sigma) == 0 (zero-crossing; sigma is NOT the equilibrium distance),
    V(r_eq) == −epsilon at r_eq = 2^(1/6)·sigma (the well depth), V → +inf as
    r → 0⁺, and V → 0⁻ as r → ∞. Raises ValueError for r <= 0 or any
    non-finite argument.
    """
    _validate_lj_params(epsilon, sigma)
    _validate_r(r)
    sigma_over_r_6 = (sigma / r) ** 6
    return 4.0 * epsilon * (sigma_over_r_6 * sigma_over_r_6 - sigma_over_r_6)


def lennard_jones_force(r: float, epsilon: float, sigma: float) -> float:
    """Evaluate the radial LJ force ``F(r) = (24ε/r)(2(σ/r)^12 − (σ/r)^6) = −dV/dr``.

    Sign convention: positive F is REPULSIVE (points in the direction of
    increasing r), negative F is ATTRACTIVE. F == 0 at r_eq = 2^(1/6)·sigma;
    F > 0 for r < r_eq; F < 0 for r > r_eq. Returns the scalar force along
    the radial direction; callers multiply by the radial unit vector to obtain
    a 2D force (M2.4). Raises ValueError for r <= 0 or any non-finite
    argument.
    """
    _validate_lj_params(epsilon, sigma)
    _validate_r(r)
    sigma_over_r_6 = (sigma / r) ** 6
    return 24.0 * epsilon / r * (2.0 * sigma_over_r_6 * sigma_over_r_6 - sigma_over_r_6)

"""M2.3: pin the interaction-distance convention with the plan's 0.40 m example.

``sigma`` is the LJ zero-crossing, NOT the equilibrium distance. Users set
``desired_spacing`` (the equilibrium distance r_eq); sigma is derived as
``desired_spacing / 2^(1/6)``. These tests pin the exact 0.40 m example from
plan.md M2.3 numerically.
"""

import pytest

from layka.config import LennardJonesConfig

TWO_ONE_SIXTH = 2.0 ** (1.0 / 6.0)


def test_desired_spacing_0_40_derives_sigma_and_restores_r_eq():
    config = LennardJonesConfig(desired_spacing=0.40)
    assert config.sigma == pytest.approx(0.40 / TWO_ONE_SIXTH)
    assert config.sigma == pytest.approx(0.35636, abs=1e-4)
    assert config.equilibrium_distance == pytest.approx(0.40)
    assert config.sigma != pytest.approx(0.40)


def test_sigma_0_40_equilibrium_is_not_0_40():
    config = LennardJonesConfig(sigma=0.40)
    assert config.equilibrium_distance == pytest.approx(TWO_ONE_SIXTH * 0.40)
    assert config.equilibrium_distance == pytest.approx(0.4490, abs=1e-4)
    assert abs(config.equilibrium_distance - 0.40) > 0.04


def test_round_trip_desired_spacing_and_sigma_agree():
    from_desired = LennardJonesConfig(desired_spacing=0.40)
    from_sigma = LennardJonesConfig(sigma=from_desired.sigma)
    assert from_desired.equilibrium_distance == pytest.approx(
        from_sigma.equilibrium_distance
    )

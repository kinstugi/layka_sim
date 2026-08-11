"""M2.8 integration tests: the deterministic multi-robot aggregation experiment.

Pins the plan.md M2.8 acceptance criteria for the default example
(swarm_size=10, desired_spacing=0.40, random_seed=42):

* Determinism: identical parameters -> identical result records (positions,
  steps, metrics) run-to-run, because all randomness flows from the seeded
  world RNG. A different seed changes the outcome.
* A genuine cluster forms: cluster_fraction reaches the empirically-observed
  0.70 (threshold 0.6) and the mean pairwise distance (1.21) is well below
  the initial random spread (1.41 in the 2.4 x 2.4 m spawn world). A rigid
  formation is NOT required -- only emergent aggregation.
* Every robot is simulated: final_positions has exactly ``swarm_size`` finite
  entries (no NaN/inf).
* Input validation rejects non-positive swarm_size / detection_range /
  cluster_radius / timestep and max_steps < 1.
* The result record is complete: every field populated.

The spawn world is deliberately 2.4 x 2.4 m (not the plan's 5 x 5 m example)
so the seed-42 placement stays connected long enough for the aggregate to
form under the M2.6 unit-speed controller; see the module docstring in
``layka/experiments.py`` for the full justification.
"""

import math
from dataclasses import fields

import pytest

from layka import AggregationResult, run_aggregation_experiment

#: Empirically-observed default-run values (seed 42, world 2.4 x 2.4 m,
#: detection_range 2.0 m, cluster_radius 1.0 m, stop_fraction 0.7). Assert
#: with a margin, never exactly: the run is deterministic, but the test pins
#: the *behavior* (aggregation) rather than specific floats.
OBSERVED_CLUSTER_FRACTION = 0.70
OBSERVED_CLUSTER_SIZE = 7
#: Mean pairwise distance of the default final state.
OBSERVED_MEAN_PAIRWISE = 1.21
#: Mean pairwise distance of the initial seeded placement (2.4 x 2.4 m
#: world); the final value must be clearly below it.
INITIAL_SPREAD = 1.41


def assert_fields_populated(result: AggregationResult) -> None:
    for field in fields(result):
        assert getattr(result, field.name) is not None


# --- determinism / reproducibility ---


def test_identical_params_produce_identical_results():
    first = run_aggregation_experiment()
    second = run_aggregation_experiment()
    assert first.final_positions == second.final_positions
    assert first.num_steps == second.num_steps
    assert first.converged is second.converged
    assert first.cluster_fraction == pytest.approx(second.cluster_fraction)
    assert first.mean_pairwise_distance == pytest.approx(
        second.mean_pairwise_distance
    )
    assert first.centroid == second.centroid
    assert first.cluster_size == second.cluster_size


def test_different_seed_produces_different_final_positions():
    first = run_aggregation_experiment(random_seed=42)
    second = run_aggregation_experiment(random_seed=43)
    assert first.final_positions != second.final_positions


# --- a cluster genuinely forms ---


def test_default_run_converges_and_forms_a_cluster():
    result = run_aggregation_experiment()
    assert result.converged is True
    # Observed 0.70; assert with a margin above the 0.6 aggregation bar.
    assert result.cluster_fraction >= 0.65
    assert result.cluster_size >= OBSERVED_CLUSTER_SIZE - 1
    assert result.cluster_size >= 1
    # The aggregate is loose (not a rigid formation): cluster_size may be less
    # than swarm_size -- only a clear majority must sit inside the radius.
    assert result.cluster_size > result.swarm_size / 2


def test_mean_pairwise_distance_shrinks_below_initial_spread():
    result = run_aggregation_experiment()
    assert result.mean_pairwise_distance < 1.5
    assert result.mean_pairwise_distance < INITIAL_SPREAD
    assert result.mean_pairwise_distance == pytest.approx(
        OBSERVED_MEAN_PAIRWISE, abs=0.15
    )


# --- all robots simulated ---


def test_final_positions_cover_every_robot_and_are_finite():
    result = run_aggregation_experiment()
    assert len(result.final_positions) == result.swarm_size == 10
    for position in result.final_positions:
        assert math.isfinite(position.x)
        assert math.isfinite(position.y)


# --- result completeness ---


def test_result_fields_all_populated():
    assert_fields_populated(run_aggregation_experiment())
    assert_fields_populated(run_aggregation_experiment(random_seed=7))


# --- input validation ---


def test_nonpositive_swarm_size_rejected():
    with pytest.raises(ValueError):
        run_aggregation_experiment(swarm_size=0)
    with pytest.raises(ValueError):
        run_aggregation_experiment(swarm_size=-3)


def test_swarm_size_capped_at_500():
    with pytest.raises(ValueError):
        run_aggregation_experiment(swarm_size=501)


def test_nonpositive_detection_range_rejected():
    with pytest.raises(ValueError):
        run_aggregation_experiment(detection_range=0.0)
    with pytest.raises(ValueError):
        run_aggregation_experiment(detection_range=-1.0)


def test_nonpositive_cluster_radius_rejected():
    with pytest.raises(ValueError):
        run_aggregation_experiment(cluster_radius=0.0)
    with pytest.raises(ValueError):
        run_aggregation_experiment(cluster_radius=-0.5)


def test_nonpositive_timestep_rejected():
    with pytest.raises(ValueError):
        run_aggregation_experiment(timestep=0.0)
    with pytest.raises(ValueError):
        run_aggregation_experiment(timestep=-0.05)


def test_max_steps_below_one_rejected():
    with pytest.raises(ValueError):
        run_aggregation_experiment(max_steps=0)


def test_invalid_stop_fraction_rejected():
    with pytest.raises(ValueError):
        run_aggregation_experiment(stop_fraction=0.0)
    with pytest.raises(ValueError):
        run_aggregation_experiment(stop_fraction=1.5)
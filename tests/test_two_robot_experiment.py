"""M2.7 integration tests: the deterministic two-robot LJ experiment.

Pins the plan.md M2.7 acceptance criteria for both scenarios:

* Scenario A (too far): robots attract and settle near ``desired_spacing``
  without passing through one another (min separation stays clearly positive).
* Scenario B (too close): robots repel and settle near the equilibrium.
* Both experiments are reproducible: identical parameters -> identical result
  records (no randomness anywhere in the clean chain).
* The result record is complete: all fields populated, equilibrium target is
  ``desired_spacing`` (Design Correction 1: ``r_eq``, not sigma).
"""

from dataclasses import fields

import pytest

from layka.experiments import TwoRobotResult, run_two_robot_experiment

EQUILIBRIUM = 0.40


def assert_fields_populated(result: TwoRobotResult) -> None:
    for field in fields(result):
        assert getattr(result, field.name) is not None


# --- Scenario A: too far (attraction) ---


def test_scenario_a_too_far_moves_toward_and_settles():
    result = run_two_robot_experiment(initial_separation=0.60)
    assert result.initial_distance == pytest.approx(0.60)
    assert result.final_distance < result.initial_distance  # attracted
    assert result.final_distance == pytest.approx(EQUILIBRIUM, abs=0.02)
    assert result.equilibrium_target == EQUILIBRIUM  # r_eq, not sigma
    assert result.converged is True
    assert isinstance(result.num_steps, int) and result.num_steps > 0
    assert result.min_distance > 0.05  # no passing through


# --- Scenario B: too close (repulsion) ---


def test_scenario_b_too_close_moves_apart_and_settles():
    result = run_two_robot_experiment(initial_separation=0.20)
    assert result.final_distance > result.initial_distance  # repelled
    assert result.final_distance == pytest.approx(EQUILIBRIUM, abs=0.02)
    assert result.equilibrium_target == EQUILIBRIUM
    assert result.converged is True
    assert isinstance(result.num_steps, int) and result.num_steps > 0
    assert result.min_distance > 0.05  # never crossed


# --- Reproducibility ---


def test_scenario_a_is_reproducible_identical_params_identical_result():
    first = run_two_robot_experiment(initial_separation=0.60)
    second = run_two_robot_experiment(initial_separation=0.60)
    assert first == second
    assert first.initial_distance == second.initial_distance
    assert first.final_distance == pytest.approx(second.final_distance)
    assert first.equilibrium_target == second.equilibrium_target
    assert first.num_steps == second.num_steps
    assert first.min_distance == pytest.approx(second.min_distance)
    assert first.converged is second.converged


# --- Result completeness ---


@pytest.mark.parametrize("initial_separation", [0.60, 0.20])
def test_result_fields_all_populated(initial_separation: float):
    assert_fields_populated(run_two_robot_experiment(initial_separation))


# --- Robustness at other starting distances (recommended check) ---


@pytest.mark.parametrize("initial_separation", [0.50, 0.30])
def test_other_starting_distances_settle_near_equilibrium(initial_separation: float):
    result = run_two_robot_experiment(initial_separation)
    assert result.final_distance == pytest.approx(EQUILIBRIUM, abs=0.02)
    assert result.converged is True


# --- Input validation ---


def test_nonpositive_initial_separation_rejected():
    with pytest.raises(ValueError):
        run_two_robot_experiment(initial_separation=0.0)
    with pytest.raises(ValueError):
        run_two_robot_experiment(initial_separation=-0.1)

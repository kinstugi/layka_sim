"""Unit tests for the explicit simulation clock (M1.4).

The clock is the only time source: ``step()`` advances exactly one ``dt`` and
nothing else can change its state, so runs are deterministic and rendering
cannot influence simulation time.
"""

import math

import pytest

from layka import SimulationClock
from layka.config import SimulationConfig


def _step_times(clock: SimulationClock, n: int) -> list[float]:
    times = []
    for _ in range(n):
        clock.step()
        times.append(clock.time)
    return times


# --- step() semantics ---


def test_step_advances_time_by_dt_and_increments_step_count():
    clock = SimulationClock(timestep=0.1)
    clock.step()
    assert clock.time == pytest.approx(0.1)
    assert clock.step_count == 1
    clock.step()
    assert clock.time == pytest.approx(0.2)
    assert clock.step_count == 2


def test_after_n_steps_time_is_n_times_dt():
    dt = 0.05
    n = 200
    clock = SimulationClock(timestep=dt)
    for _ in range(n):
        clock.step()
    assert clock.time == pytest.approx(n * dt)
    assert clock.step_count == n


# --- configurability ---


def test_dt_is_configurable_and_differs_between_clocks():
    fast = SimulationClock(timestep=0.01)
    slow = SimulationClock(timestep=0.1)
    for _ in range(10):
        fast.step()
        slow.step()
    assert fast.time == pytest.approx(10 * 0.01)
    assert slow.time == pytest.approx(10 * 0.1)
    assert fast.time != slow.time
    assert fast.step_count == slow.step_count  # same step count, different time


def test_clock_reuses_simulation_config_timestep():
    config = SimulationConfig(timestep=0.05, robot_count=10)
    clock = SimulationClock(config)
    assert clock.dt == pytest.approx(0.05)
    assert clock.timestep == pytest.approx(0.05)
    clock.step()
    assert clock.time == pytest.approx(0.05)


def test_timestep_dt_and_time_are_read_only():
    clock = SimulationClock(timestep=0.1)
    with pytest.raises(AttributeError):
        clock.time = 1.0  # type: ignore[misc]
    with pytest.raises(AttributeError):
        clock.step_count = 1  # type: ignore[misc]
    with pytest.raises(AttributeError):
        clock.dt = 0.5  # type: ignore[misc]
    with pytest.raises(AttributeError):
        clock.timestep = 0.5  # type: ignore[misc]


def test_nonpositive_dt_raises():
    for bad in (0.0, -0.1):
        with pytest.raises(ValueError):
            SimulationClock(timestep=bad)


def test_nan_dt_raises():
    with pytest.raises(ValueError):
        SimulationClock(timestep=math.nan)


# --- epochs and reset ---


def test_initial_epoch_supported():
    clock = SimulationClock(timestep=0.1, initial_time=2.0, initial_step_count=20)
    assert clock.time == pytest.approx(2.0)
    assert clock.step_count == 20
    clock.step()
    assert clock.time == pytest.approx(2.1)
    assert clock.step_count == 21


def test_reset_restores_initial_epoch():
    clock = SimulationClock(timestep=0.1)
    for _ in range(100):
        clock.step()
    clock.reset()
    assert clock.time == pytest.approx(0.0)
    assert clock.step_count == 0


def test_reset_restores_custom_initial_epoch():
    clock = SimulationClock(timestep=0.1, initial_time=1.0, initial_step_count=10)
    for _ in range(50):
        clock.step()
    clock.reset()
    assert clock.time == pytest.approx(1.0)
    assert clock.step_count == 10


def test_reset_with_explicit_state():
    clock = SimulationClock(timestep=0.1)
    for _ in range(10):
        clock.step()
    clock.reset(time=0.5, step_count=5)
    assert clock.time == pytest.approx(0.5)
    assert clock.step_count == 5
    clock.step()
    assert clock.time == pytest.approx(0.6)
    assert clock.step_count == 6


def test_reset_rejects_negative_epoch():
    clock = SimulationClock(timestep=0.1)
    with pytest.raises(ValueError):
        clock.reset(time=-1.0)
    with pytest.raises(ValueError):
        clock.reset(step_count=-1)


# --- determinism (core acceptance criterion) ---


def test_deterministic_n_step_run_is_reproducible():
    dt = 0.1
    n = 100
    first = SimulationClock(timestep=dt)
    for _ in range(n):
        first.step()
    second = SimulationClock(timestep=dt)
    for _ in range(n):
        second.step()
    assert second.time == pytest.approx(first.time)
    assert second.step_count == first.step_count


def test_reset_reproduces_identical_trajectory():
    dt = 0.1
    n = 100
    clock = SimulationClock(timestep=dt)
    first_trajectory = _step_times(clock, n)
    clock.reset()
    second_trajectory = _step_times(clock, n)
    assert second_trajectory == pytest.approx(first_trajectory)
    assert second_trajectory == pytest.approx([dt * i for i in range(1, n + 1)])


def test_render_calls_do_not_affect_clock_state():
    """Rendering is decoupled: interleaved no-op 'render' calls change nothing."""

    def render() -> None:
        return None

    n = 100
    clock = SimulationClock(timestep=0.1)
    reference = SimulationClock(timestep=0.1)
    for _ in range(n):
        reference.step()

    for i in range(n):
        for _ in range(i % 7):  # arbitrary, varying number of renders per step
            render()
        clock.step()

    assert clock.time == pytest.approx(reference.time)
    assert clock.step_count == reference.step_count

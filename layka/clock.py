"""Explicit simulation timekeeping, decoupled from rendering speed.

Intended loop pattern (world/renderer arrive in later milestones):

    clock = SimulationClock(SimulationConfig(timestep=0.1))
    while simulation_running:
        world.step(clock)   # simulation update; advances exactly one dt
        renderer.render()   # does NOT advance the clock

The clock is the single source of simulation time. It has no access to
wall-clock time, and nothing outside ``step()``/``reset()`` can change its
state, so simulation state cannot drift merely because rendering FPS changes.
"""

from __future__ import annotations

import math

from layka.config import SimulationConfig


class SimulationClock:
    """Advances simulation time in fixed, explicit timesteps.

    This is hot simulation state: a plain class with ``__slots__`` (not
    Pydantic) because it is touched on every simulation step. ``dt`` is fixed
    for the lifetime of the clock; ``time`` and ``step_count`` change only via
    ``step()`` (advance one timestep) and ``reset()`` (restore an epoch for
    reproducible runs).
    """

    __slots__ = (
        "_timestep",
        "_time",
        "_step_count",
        "_initial_time",
        "_initial_step_count",
    )

    def __init__(
        self,
        timestep: float | SimulationConfig,
        *,
        initial_time: float = 0.0,
        initial_step_count: int = 0,
    ) -> None:
        """Create a clock with a fixed step duration ``dt``.

        ``timestep`` may be a bare float (seconds) or a ``SimulationConfig``,
        whose validated ``timestep`` field is reused. ``initial_time`` /
        ``initial_step_count`` define the epoch that ``reset()`` restores.
        """
        if isinstance(timestep, SimulationConfig):
            timestep = timestep.timestep
        self._validate_epoch(initial_time, initial_step_count)
        if not math.isfinite(timestep) or timestep <= 0:
            raise ValueError(
                f"timestep must be a finite positive number, got {timestep!r}"
            )
        self._timestep = float(timestep)
        self._initial_time = float(initial_time)
        self._initial_step_count = int(initial_step_count)
        self._time = self._initial_time
        self._step_count = self._initial_step_count

    @staticmethod
    def _validate_epoch(time: float, step_count: int) -> None:
        if time < 0:
            raise ValueError(f"time must be >= 0, got {time!r}")
        if step_count < 0:
            raise ValueError(f"step_count must be >= 0, got {step_count!r}")

    def step(self) -> None:
        """Advance exactly one simulation timestep: ``time += dt``, ``step_count += 1``."""
        self._time += self._timestep
        self._step_count += 1

    def reset(self, *, time: float | None = None, step_count: int | None = None) -> None:
        """Restore the clock to its initial epoch (or an explicit one).

        For deterministic runs, call ``reset()`` before re-applying the same
        initial world state so the next N steps reproduce the previous run.
        """
        new_time = self._initial_time if time is None else time
        new_step_count = (
            self._initial_step_count if step_count is None else step_count
        )
        self._validate_epoch(new_time, new_step_count)
        self._time = float(new_time)
        self._step_count = int(new_step_count)

    @property
    def time(self) -> float:
        """Current simulation time (s). Read-only."""
        return self._time

    @property
    def step_count(self) -> int:
        """Number of completed simulation steps. Read-only."""
        return self._step_count

    @property
    def timestep(self) -> float:
        """Fixed step duration ``dt`` (s). Read-only."""
        return self._timestep

    #: Alias of :attr:`timestep`, so callers can write ``clock.dt``.
    dt = timestep

    def __repr__(self) -> str:
        return (
            f"SimulationClock(timestep={self._timestep!r}, "
            f"time={self._time!r}, step_count={self._step_count!r})"
        )

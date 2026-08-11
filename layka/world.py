"""Minimal multi-robot world (M1.6).

Holds N robots, owns the simulation clock, and drives the per-step update:
every robot's behavior is called exactly once to obtain a body velocity
``(v, omega)``, the robot's velocity state is set from it, and the pose is
integrated via :func:`layka.kinematics.integrate_pose`. Matching the legacy
``models/world.py`` convention, robots are updated first and the clock
advances at the end of each step.

Design notes:

- Robots are stored in a ``dict`` keyed by unique robot id, which keeps
  insertion order (= ascending id) and gives O(1) ``robot_by_id`` lookups for
  M1.7 neighbor queries. Iteration order is fixed, so ``step()`` updates
  robots in a deterministic order.
- Rendering is NOT coupled to stepping: ``step()`` is the only scheduled
  mutation driver. Renderers (M1.8) only read robot state; the clock is never
  advanced by anything but ``step()``.
- Spawning is deterministic when a seed is set: the world owns a private
  ``random.Random(seed)`` instance (never the global ``random`` module), so
  two worlds built from the same ``SimulationConfig`` spawn identical robot
  configurations. With no seed, spawns are nondeterministic.
"""

from __future__ import annotations

import math
import random
from collections.abc import ValuesView

from layka.behavior import Behavior, StationaryBehavior
from layka.clock import SimulationClock
from layka.config import SimulationConfig
from layka.kinematics import integrate_pose
from layka.pose import Pose2D
from layka.robot import RobotState

#: Defaults used when a ``World`` is built without a ``SimulationConfig``;
#: they mirror the ``SimulationConfig`` field defaults.
DEFAULT_TIMESTEP = 0.1
DEFAULT_WORLD_WIDTH = 5.0
DEFAULT_WORLD_HEIGHT = 5.0


class World:
    """A 2D world holding multiple robots and a simulation clock."""

    __slots__ = (
        "_config",
        "_clock",
        "_rng",
        "_robots",
        "_behaviors",
        "_next_id",
        "_world_width",
        "_world_height",
    )

    def __init__(
        self,
        config: SimulationConfig | None = None,
        *,
        timestep: float | None = None,
        seed: int | None = None,
    ) -> None:
        """Build a world with an empty robot set.

        ``config`` is the preferred single source for the clock's ``dt``, the
        spawn bounds, and the RNG seed. Alternatively pass ``timestep`` (and
        optionally ``seed``) directly; world dimensions then default to 5 x 5.
        When both ``config`` and an explicit ``seed`` are given, the explicit
        seed wins.
        """
        if config is not None and timestep is not None:
            raise ValueError("provide either config or timestep, not both")
        if config is not None:
            self._config = config
            self._clock = SimulationClock(config)
            effective_seed = config.random_seed if seed is None else seed
            self._world_width = config.world_width
            self._world_height = config.world_height
        else:
            self._config = None
            self._clock = SimulationClock(
                DEFAULT_TIMESTEP if timestep is None else timestep
            )
            effective_seed = seed
            self._world_width = DEFAULT_WORLD_WIDTH
            self._world_height = DEFAULT_WORLD_HEIGHT
        self._rng = random.Random(effective_seed)
        self._robots: dict[int, RobotState] = {}
        self._behaviors: dict[int, Behavior] = {}
        self._next_id = 0

    # --- robot management ---

    def spawn_robots(
        self,
        count: int | None = None,
        *,
        behavior: Behavior | None = None,
    ) -> list[int]:
        """Spawn ``count`` robots with unique sequential ids (0..count-1).

        ``count`` defaults to ``SimulationConfig.robot_count`` when the world
        was built from a config. Each robot gets a random pose drawn from the
        world's seeded RNG (deterministic when a seed is set) and zero initial
        velocity; robots given no explicit behavior get a fresh
        ``StationaryBehavior``. Returns the spawned robot ids.
        """
        if count is None:
            count = self._config.robot_count if self._config is not None else 0
        if isinstance(count, bool) or not isinstance(count, int):
            raise ValueError(f"count must be an int, got {count!r}")
        if count < 0:
            raise ValueError(f"count must be >= 0, got {count!r}")
        return [self.add_robot(behavior=behavior) for _ in range(count)]

    def add_robot(
        self,
        pose: Pose2D | None = None,
        *,
        behavior: Behavior | None = None,
    ) -> int:
        """Add one robot and return its unique id.

        ``pose`` defaults to a random pose (uniform position within the world
        bounds, uniform heading in [-pi, pi)) drawn from the world's seeded
        RNG. ``behavior`` defaults to a fresh ``StationaryBehavior``. The new
        robot starts with zero linear/angular velocity.
        """
        if pose is None:
            pose = self._random_pose()
        robot_id = self._next_id
        self._next_id += 1
        self._robots[robot_id] = RobotState(robot_id=robot_id, pose=pose)
        self._behaviors[robot_id] = (
            behavior if behavior is not None else StationaryBehavior()
        )
        return robot_id

    def _random_pose(self) -> Pose2D:
        return Pose2D(
            x=self._rng.uniform(0.0, self._world_width),
            y=self._rng.uniform(0.0, self._world_height),
            theta=self._rng.uniform(-math.pi, math.pi),
        )

    # --- simulation step ---

    def step(self) -> None:
        """Advance the simulation by exactly one timestep.

        Every robot is updated exactly once, in deterministic id (insertion)
        order: its behavior is called to produce ``(v, omega)``, the robot's
        velocity state is set from it, and its pose is integrated with
        ``integrate_pose`` using the clock's ``dt``. The clock advances once
        per call (``time += dt``, ``step_count += 1``) after all robots are
        updated, matching the legacy ``models/world.py`` convention. Rendering
        never calls this and never changes simulation state.
        """
        dt = self._clock.dt
        for robot_id, robot in self._robots.items():
            command = self._behaviors[robot_id].compute_command(robot, self, dt)
            robot.linear_velocity = command.v
            robot.angular_velocity = command.omega
            robot.pose = integrate_pose(robot.pose, command.v, command.omega, dt)
        self._clock.step()

    # --- read-only accessors ---

    @property
    def robots(self) -> ValuesView[RobotState]:
        """Live view of all robots in id (insertion) order.

        The view cannot add or remove robots; the ``RobotState`` objects
        themselves are mutable live state (e.g. ``step`` updates them in
        place).
        """
        return self._robots.values()

    def robot_by_id(self, robot_id: int) -> RobotState:
        """Return the robot with ``robot_id``; raises ``KeyError`` if unknown."""
        return self._robots[robot_id]

    def behavior_of(self, robot_id: int) -> Behavior:
        """Return the behavior attached to ``robot_id``; raises ``KeyError`` if unknown."""
        return self._behaviors[robot_id]

    @property
    def clock(self) -> SimulationClock:
        """The world's simulation clock (read-only time source)."""
        return self._clock

    @property
    def time(self) -> float:
        """Current simulation time (s)."""
        return self._clock.time

    @property
    def step_count(self) -> int:
        """Number of completed simulation steps."""
        return self._clock.step_count

    @property
    def dt(self) -> float:
        """Fixed step duration (s)."""
        return self._clock.dt

    @property
    def timestep(self) -> float:
        """Alias of :attr:`dt`."""
        return self._clock.dt

    @property
    def width(self) -> float:
        """World width (m)."""
        return self._world_width

    @property
    def height(self) -> float:
        """World height (m)."""
        return self._world_height

    @property
    def robot_count(self) -> int:
        """Number of robots currently in the world."""
        return len(self._robots)

    def __len__(self) -> int:
        return len(self._robots)

    def __repr__(self) -> str:
        return (
            f"World(robots={len(self._robots)}, time={self._clock.time!r}, "
            f"step_count={self._clock.step_count!r}, dt={self._clock.dt!r})"
        )


__all__ = ["World"]

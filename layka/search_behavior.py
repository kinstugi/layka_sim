"""M2.9 search behavior: a SEARCH <-> SWARM state machine for one robot.

A robot with no neighbors must not stop forever, and a robot that has
neighbors must aggregate. This module provides a single robot behavior that
implements exactly that two-state machine:

    SEARCH:
        move forward at ``search_velocity`` for ``forward_interval`` seconds
        then turn in place at ``turn_rate`` for ``turn_duration`` seconds,
        cycling forward -> turn -> forward -> ... (deterministic patrol)
        detect neighbors every call

    when a neighbor is detected:    SEARCH -> SWARM
    when no neighbors remain:       SWARM -> SEARCH

    SWARM:
        reuse the M2.6 LJ chain (neighbors -> LJInteraction resultant ->
        LJController body velocity); this is the M2.5/M2.6 pipeline, signs
        verified in M2.4/M2.5/M2.6.

The patrol pattern is deliberately simple (Design Correction 4): no obstacle
avoidance (that is M2.10), no flocking/alignment, no goals, and NO random
noise. The pattern is fully deterministic: same inputs -> same outputs, so
identical initial states produce identical trajectories.

Design rules applied:

- Rule 5 (separation): this is a BEHAVIOR -- it decides ``(v, omega)``. It
  composes :class:`layka.neighbors.NeighborSensor` (M1.7),
  :class:`layka.lj_interaction.LJInteraction` (M2.5), and
  :class:`layka.lj_controller.LJController` (M2.6); it never integrates poses
  or sets wheel speeds directly.
- Each robot gets its OWN ``SearchSwarmBehavior`` instance (the world stores
  ``dict[int, Behavior]``): per-robot search state (current state, accumulated
  patrol time) lives on the instance, so the behavior works for any robot id
  and is independent of other robots' instances.
- Read-only: ``compute_command`` never mutates the robot, other robots, or the
  world (``World.step`` is the only mutation driver).

Phase convention (pinned by the tests): within one patrol cycle of length
``forward_interval + turn_duration``, accumulated elapsed time ``t`` in SEARCH
selects the command as

    t % (forward_interval + turn_duration) <  forward_interval   -> forward
    otherwise                                                   -> turn

so a robot that has just entered SEARCH always starts with a forward phase,
and the turn phase is entered exactly when ``forward_interval`` has fully
elapsed. Time is accumulated per call from ``dt`` (never wall-clock).
``turn_rate`` is positive, and a positive ``omega`` is a counter-clockwise
turn (the math convention: theta increases CCW from +x).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from pydantic import BaseModel, Field, field_validator

from layka.config import LennardJonesConfig
from layka.kinematics import BodyVelocity
from layka.lj_controller import LJController, LJControllerConfig
from layka.lj_interaction import LJInteraction
from layka.neighbors import NeighborSensor

if TYPE_CHECKING:
    from layka.robot import RobotState
    from layka.world import World

#: Behavior state labels returned by :attr:`SearchSwarmBehavior.state`.
SEARCH = "SEARCH"
SWARM = "SWARM"


def _validate_dt(dt: float) -> None:
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError(f"dt must be a finite positive number, got {dt!r}")


class SearchSwarmConfig(BaseModel):
    """Validated parameters of the SEARCH patrol pattern (M2.9)."""

    search_velocity: float = Field(
        default=0.1, gt=0, description="Forward speed while searching (m/s)."
    )
    forward_interval: float = Field(
        default=1.0,
        gt=0,
        description="Seconds to move straight forward before turning.",
    )
    turn_duration: float = Field(
        default=0.5,
        gt=0,
        description="Seconds to turn in place after the forward interval.",
    )
    turn_rate: float = Field(
        default=1.0,
        gt=0,
        description="Angular speed while turning (rad/s); positive = CCW.",
    )
    detection_range: float = Field(
        default=2.0,
        gt=0,
        description="Neighbor detection radius (m); independent of the LJ cutoff.",
    )

    @field_validator(
        "search_velocity",
        "forward_interval",
        "turn_duration",
        "turn_rate",
        "detection_range",
    )
    @classmethod
    def _finite_positive(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be a finite number")
        return value


class SearchSwarmBehavior:
    """M2.9 ``Behavior``: SEARCH <-> SWARM state machine for one robot.

    SEARCH produces the deterministic patrol pattern described in the module
    docstring; SWARM reuses the M2.6 LJ controller chain. The current state is
    exposed read-only via :attr:`state`. One instance per robot: all mutable
    search state (state, accumulated patrol time) is per-instance.
    """

    __slots__ = ("_config", "_controller", "_interaction", "_state", "_search_elapsed")

    def __init__(
        self,
        search_config: SearchSwarmConfig,
        controller: LJController,
        interaction: LJInteraction,
    ) -> None:
        self._config = search_config
        self._controller = controller
        self._interaction = interaction
        self._state = SEARCH
        self._search_elapsed = 0.0

    @classmethod
    def from_config(
        cls,
        search_config: SearchSwarmConfig,
        controller_config: LJControllerConfig,
        lj_config: LennardJonesConfig,
    ) -> SearchSwarmBehavior:
        """Build from validated configs, constructing the components internally."""
        return cls(
            search_config=search_config,
            controller=LJController(controller_config),
            interaction=LJInteraction(lj_config),
        )

    @property
    def config(self) -> SearchSwarmConfig:
        """The configuration this behavior was built with (read-only)."""
        return self._config

    @property
    def state(self) -> str:
        """Current state: ``"SEARCH"`` or ``"SWARM"`` (read-only)."""
        return self._state

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        """Return the body velocity ``(v, omega)`` commanded for ``robot``.

        1. Detect neighbors within ``config.detection_range``.
        2. Neighbors present -> state becomes SWARM; return the M2.6 LJ
           controller command for the resultant interaction vector.
        3. No neighbors -> state becomes SEARCH (resetting the patrol timer so
           a re-entered SEARCH always starts with a forward phase) and return
           the deterministic patrol command, advancing the accumulated patrol
           time by ``dt``.

        Side-effect free with respect to the robot/world: returns a
        ``BodyVelocity`` that the world applies.
        """
        _validate_dt(dt)
        sensor = NeighborSensor(world, self._config.detection_range)
        neighbors = sensor.neighbors_of(robot.robot_id)
        if neighbors:
            self._state = SWARM
            resultant = self._interaction.compute_from_neighbors(
                robot.pose, neighbors
            )
            return self._controller.compute(resultant, robot.pose.theta)
        if self._state is not SEARCH:
            self._state = SEARCH
            self._search_elapsed = 0.0
        self._search_elapsed += dt
        return self._search_command()

    def _search_command(self) -> BodyVelocity:
        """Deterministic patrol command from the accumulated SEARCH time."""
        cycle = self._config.forward_interval + self._config.turn_duration
        phase_time = self._search_elapsed % cycle
        if phase_time < self._config.forward_interval:
            return BodyVelocity(v=self._config.search_velocity, omega=0.0)
        return BodyVelocity(v=0.0, omega=self._config.turn_rate)


__all__ = ["SEARCH", "SearchSwarmBehavior", "SearchSwarmConfig", "SWARM"]

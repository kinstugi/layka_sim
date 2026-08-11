"""M2.6 controller: resultant interaction vector -> body velocity ``(v, omega)``.

This module connects the M2.4/M2.5 Lennard-Jones interaction vector to a
differential-drive (unicycle) robot. The resultant vector expresses a DESIRED
direction of motion, NOT a Newtonian force and NOT wheel velocities (Design
Correction 2 / Rule 8): the controller converts it into a feasible body
velocity ``(v, omega)`` and the robot model (M1.5, ``integrate_pose``)
performs the kinematic integration. Nothing here touches wheel speeds.

Controller law (documented and pinned by the unit tests):

    desired_heading = atan2(F_y, F_x)                 # world frame
    heading_error   = normalize_angle(desired_heading - robot_heading)
    omega           = clamp(k_omega * heading_error,
                            -max_angular_velocity, +max_angular_velocity)
    v               = clamp(max_linear_velocity * cos(|heading_error|),
                            min_linear_velocity, max_linear_velocity)

``v`` peaks at ``max_linear_velocity`` when the robot already faces the
desired direction and decays monotonically (cosine law) toward
``min_linear_velocity`` as the heading error grows to pi, so a robot that is
facing the wrong way crawls (or stops, when ``min_linear_velocity == 0``)
while turning around.

A zero (or near-zero) resultant vector means no net interaction, so the robot
stops: ``BodyVelocity(0.0, 0.0)``. A robot with no neighbors stays put; search
behavior for isolated robots is a later milestone (M2.9).

:class:`LJBehavior` is the M1.6 ``Behavior`` wrapper that composes the full
pipeline for one robot per step: neighbor detection (M1.7) -> resultant vector
(M2.4/M2.5) -> body velocity. It is pure and read-only: it never mutates the
world or other robots.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from pydantic import BaseModel, Field, field_validator, model_validator

from layka.config import LennardJonesConfig
from layka.kinematics import BodyVelocity
from layka.lj_interaction import LJInteraction
from layka.lj_safety import clamp
from layka.neighbors import NeighborSensor
from layka.pose import normalize_angle
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.robot import RobotState
    from layka.world import World

#: Resultant vectors with norm below this magnitude are treated as "no
#: interaction": exact zeros plus the numerical residue of canceling forces.
ZERO_RESULTANT_TOLERANCE = 1e-12


class LJControllerConfig(BaseModel):
    """Validated gains and velocity limits for the :class:`LJController`."""

    angular_gain: float = Field(
        default=2.0, gt=0, description="Proportional angular gain k_omega."
    )
    max_linear_velocity: float = Field(
        default=0.2, gt=0, description="Maximum forward speed (m/s)."
    )
    max_angular_velocity: float = Field(
        default=1.0, gt=0, description="Maximum turn rate (rad/s)."
    )
    min_linear_velocity: float = Field(
        default=0.0, ge=0, description="Forward-speed floor while turning (m/s)."
    )

    @field_validator("angular_gain", "max_linear_velocity", "max_angular_velocity")
    @classmethod
    def _finite_positive(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be a finite number")
        return value

    @field_validator("min_linear_velocity")
    @classmethod
    def _finite_nonnegative(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be a finite number")
        return value

    @model_validator(mode="after")
    def _min_linear_does_not_exceed_max(self) -> LJControllerConfig:
        if self.min_linear_velocity > self.max_linear_velocity:
            raise ValueError(
                "min_linear_velocity must be <= max_linear_velocity, got "
                f"{self.min_linear_velocity!r} > {self.max_linear_velocity!r}"
            )
        return self


class LJController:
    """Proportional heading controller: interaction vector -> ``(v, omega)``.

    Pure and deterministic: a function of the resultant vector and the current
    heading only. It does NOT run neighbor detection (M1.7) or force math
    (M2.4/M2.5); compose those in :class:`LJBehavior` (Rule 5).
    """

    __slots__ = ("_config",)

    def __init__(self, config: LJControllerConfig) -> None:
        self._config = config

    @property
    def config(self) -> LJControllerConfig:
        """The configuration this controller was built with (read-only)."""
        return self._config

    def desired_heading(self, resultant: Vector2) -> float:
        """World-frame heading (radians) the resultant vector points toward.

        ``atan2(resultant.y, resultant.x)``. For the zero vector this evaluates
        to ``atan2(0, 0) == 0.0`` (pinned by tests); :meth:`compute`
        short-circuits zero resultants before this value is acted on.
        """
        return math.atan2(resultant.y, resultant.x)

    def compute(self, resultant: Vector2, current_heading: float) -> BodyVelocity:
        """Convert ``resultant`` into a feasible body velocity for a robot
        facing ``current_heading``.

        A zero (or near-zero) resultant means no net interaction: the robot
        stops with ``BodyVelocity(0.0, 0.0)`` (a robot with no neighbors stays
        put; search behavior is M2.9). Otherwise the proportional law from the
        module docstring applies. Non-finite resultants are invalid input and
        raise ``ValueError`` (never silently clamped). Side-effect free.
        """
        if not (math.isfinite(resultant.x) and math.isfinite(resultant.y)):
            raise ValueError(f"resultant must be finite, got {resultant!r}")
        if resultant.norm() < ZERO_RESULTANT_TOLERANCE:
            return BodyVelocity(v=0.0, omega=0.0)
        desired = self.desired_heading(resultant)
        error = normalize_angle(desired - current_heading)
        omega = clamp(
            self._config.angular_gain * error,
            -self._config.max_angular_velocity,
            self._config.max_angular_velocity,
        )
        v = clamp(
            self._config.max_linear_velocity * math.cos(abs(error)),
            self._config.min_linear_velocity,
            self._config.max_linear_velocity,
        )
        return BodyVelocity(v=v, omega=omega)


class LJBehavior:
    """M1.6 ``Behavior`` wrapper: neighbors -> LJ resultant -> ``(v, omega)``.

    Composes the sensing/control pipeline for one robot:
    :class:`layka.neighbors.NeighborSensor` (M1.7) detects neighbors within
    ``detection_range``, :class:`LJInteraction` (M2.5) computes the resultant
    vector, and :class:`LJController` converts it into a body velocity.
    Read-only: never mutates the world or other robots.

    ``detection_range`` (what the sensor can see) is independent of the LJ
    ``cutoff_distance`` (beyond which the interaction is zero): neighbors
    farther than the cutoff contribute zero force, neighbors closer than the
    cutoff contribute. Building a fresh ``NeighborSensor`` per call is O(N^2)
    by design (Rule 7).
    """

    __slots__ = ("_controller", "_interaction", "_detection_range")

    def __init__(
        self,
        controller: LJController,
        interaction: LJInteraction,
        detection_range: float,
    ) -> None:
        if not math.isfinite(detection_range) or detection_range <= 0:
            raise ValueError(
                "detection_range must be a finite positive number, "
                f"got {detection_range!r}"
            )
        self._controller = controller
        self._interaction = interaction
        self._detection_range = detection_range

    @classmethod
    def from_config(
        cls,
        controller_config: LJControllerConfig,
        lj_config: LennardJonesConfig,
        detection_range: float,
    ) -> LJBehavior:
        """Build from validated configs, constructing the components internally."""
        return cls(
            controller=LJController(controller_config),
            interaction=LJInteraction(lj_config),
            detection_range=detection_range,
        )

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        """Return the body velocity ``(v, omega)`` commanded for ``robot``.

        ``dt`` is part of the M1.6 ``Behavior`` protocol but this velocity
        controller does not need it.
        """
        sensor = NeighborSensor(world, self._detection_range)
        neighbors = sensor.neighbors_of(robot.robot_id)
        resultant = self._interaction.compute_from_neighbors(robot.pose, neighbors)
        return self._controller.compute(resultant, robot.pose.theta)


__all__ = ["LJBehavior", "LJController", "LJControllerConfig"]

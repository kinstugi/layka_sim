"""Validated simulation configuration models (Pydantic).

These models are validated once at setup time; they are not used inside
high-frequency numerical loops.
"""

from __future__ import annotations

import math

from pydantic import BaseModel, Field, field_validator, model_validator

#: 2^(1/6): factor relating the LJ zero-crossing ``sigma`` to the potential
#: minimum / zero-force equilibrium distance ``r_eq = 2^(1/6) * sigma``.
TWO_ONE_SIXTH = 2.0 ** (1.0 / 6.0)


class SimulationConfig(BaseModel):
    """Top-level simulation parameters."""

    timestep: float = Field(default=0.1, gt=0, description="Simulation step duration (s).")
    robot_count: int = Field(default=0, ge=0, description="Number of robots in the world.")
    random_seed: int | None = Field(default=None, description="Seed for reproducible runs.")
    world_width: float = Field(default=5.0, gt=0, description="World width (m).")
    world_height: float = Field(default=5.0, gt=0, description="World height (m).")


class LennardJonesConfig(BaseModel):
    """Lennard-Jones interaction parameters.

    Important: ``sigma`` is the zero-crossing of the LJ potential, NOT the
    equilibrium distance. The equilibrium (zero-force) distance is
    ``r_eq = 2^(1/6) * sigma``. Users who think in terms of the desired
    robot-to-robot spacing should set ``desired_spacing`` instead; ``sigma``
    is then derived internally as ``desired_spacing / 2^(1/6)``.

    Example: ``desired_spacing = 0.40`` m means ``r_eq ~= 0.40`` m (with
    ``sigma ~= 0.3564`` m), never ``sigma = 0.40`` m (which would give
    ``r_eq ~= 0.4490`` m).
    """

    epsilon: float = Field(default=1.0, gt=0, description="Potential well depth.")
    sigma: float | None = Field(
        default=None,
        gt=0,
        description="LJ length scale; zero-crossing of the potential (m). "
        "This is NOT the equilibrium distance.",
    )
    desired_spacing: float | None = Field(
        default=None,
        gt=0,
        description="Desired robot-to-robot equilibrium spacing r_eq (m). "
        "When set, sigma is derived as desired_spacing / 2^(1/6).",
    )
    cutoff_distance: float | None = Field(
        default=None,
        description="Interaction cutoff radius (m); None disables the cutoff.",
    )
    max_force: float = Field(default=10.0, gt=0, description="Interaction force clamp.")
    min_distance: float = Field(
        default=0.01,
        gt=0,
        description="Minimum interaction distance for numerical safety (m). "
        "Distances below this are clamped up before evaluating the LJ "
        "potential/force (M2.2). Must be finite and positive. The default "
        "0.01 m is small relative to the typical desired_spacing of 0.40 m "
        "(sigma ~= 0.3564 m).",
    )

    @field_validator("cutoff_distance")
    @classmethod
    def _cutoff_must_be_positive(cls, value: float | None) -> float | None:
        if value is not None and value <= 0:
            raise ValueError("cutoff_distance must be greater than 0 when set")
        return value

    @field_validator("min_distance")
    @classmethod
    def _min_distance_must_be_finite_and_positive(cls, value: float) -> float:
        if not math.isfinite(value) or value <= 0:
            raise ValueError("min_distance must be a finite positive number")
        return value

    @model_validator(mode="after")
    def _resolve_sigma(self) -> LennardJonesConfig:
        if self.sigma is not None and self.desired_spacing is not None:
            raise ValueError("provide either sigma or desired_spacing, not both")
        if self.desired_spacing is not None:
            self.sigma = self.desired_spacing / TWO_ONE_SIXTH
        if self.sigma is None:
            raise ValueError("one of sigma or desired_spacing must be provided")
        return self

    @property
    def equilibrium_distance(self) -> float:
        """Zero-force equilibrium distance r_eq = 2^(1/6) * sigma (m)."""
        return TWO_ONE_SIXTH * self.sigma

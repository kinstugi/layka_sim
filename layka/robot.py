"""Robot domain models: hot runtime state and validated configuration.

Robots are differential-drive/unicycle vehicles: state carries pose plus the
linear velocity ``v`` and angular velocity ``omega``. There is deliberately no
Newtonian acceleration state.
"""

from __future__ import annotations

from dataclasses import dataclass

from pydantic import BaseModel, Field

from layka.pose import Pose2D


@dataclass(slots=True)
class RobotState:
    """Per-robot runtime state (hot simulation state, hence a dataclass)."""

    robot_id: int
    pose: Pose2D
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


class RobotConfig(BaseModel):
    """Validated physical/control configuration of a differential-drive robot.

    Defaults mirror the legacy KheperaIII/Layka robot: wheel base 0.0885 m and
    max wheel drive rate 15 rad/s at wheel radius 0.021 m, which gives
    v_max = 0.315 m/s and omega_max = 2 * v_max / wheel_base ~= 7.12 rad/s.
    """

    wheel_base: float = Field(
        default=0.0885, gt=0, description="Distance between the drive wheels (m)."
    )
    max_linear_velocity: float = Field(
        default=0.315, gt=0, description="Maximum forward speed (m/s)."
    )
    max_angular_velocity: float = Field(
        default=7.12, gt=0, description="Maximum turn rate (rad/s)."
    )

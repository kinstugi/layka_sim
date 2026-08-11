"""Layka simulator core (M1.3 / M1.4 / M1.5).

Re-exports the public typed models, the explicit simulation clock, and the
differential-drive kinematics. Pure Python: no GTK/PyGObject dependency, so it
imports headlessly for tests and experiments.
"""

from layka.clock import SimulationClock
from layka.config import LennardJonesConfig, SimulationConfig
from layka.kinematics import (
    BodyVelocity,
    DifferentialDriveRobot,
    WheelSpeeds,
    body_to_wheels,
    integrate_pose,
    wheels_to_body,
)
from layka.pose import Pose2D, normalize_angle
from layka.robot import RobotConfig, RobotState
from layka.vector import Vector2

__all__ = [
    "BodyVelocity",
    "DifferentialDriveRobot",
    "LennardJonesConfig",
    "Pose2D",
    "RobotConfig",
    "RobotState",
    "SimulationClock",
    "SimulationConfig",
    "Vector2",
    "WheelSpeeds",
    "body_to_wheels",
    "integrate_pose",
    "normalize_angle",
    "wheels_to_body",
]

"""Layka simulator core (M1.3 / M1.4 / M1.5 / M1.6).

Re-exports the public typed models, the explicit simulation clock, the
differential-drive kinematics, the minimal robot behavior abstraction, and
the multi-robot world. Pure Python: no GTK/PyGObject dependency, so it
imports headlessly for tests and experiments.
"""

from layka.behavior import Behavior, StationaryBehavior, TrivialMotionBehavior
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
from layka.world import World

__all__ = [
    "Behavior",
    "BodyVelocity",
    "DifferentialDriveRobot",
    "LennardJonesConfig",
    "Pose2D",
    "RobotConfig",
    "RobotState",
    "SimulationClock",
    "SimulationConfig",
    "StationaryBehavior",
    "TrivialMotionBehavior",
    "Vector2",
    "WheelSpeeds",
    "World",
    "body_to_wheels",
    "integrate_pose",
    "normalize_angle",
    "wheels_to_body",
]

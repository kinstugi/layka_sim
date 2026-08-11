"""Layka simulator core (M1.3 / M1.4).

Re-exports the public typed models and the explicit simulation clock. Pure
Python: no GTK/PyGObject dependency, so it imports headlessly for tests and
experiments.
"""

from layka.clock import SimulationClock
from layka.config import LennardJonesConfig, SimulationConfig
from layka.pose import Pose2D, normalize_angle
from layka.robot import RobotConfig, RobotState
from layka.vector import Vector2

__all__ = [
    "LennardJonesConfig",
    "Pose2D",
    "RobotConfig",
    "RobotState",
    "SimulationClock",
    "SimulationConfig",
    "Vector2",
    "normalize_angle",
]

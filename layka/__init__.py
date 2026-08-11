"""Layka simulator core (M1.3 / M1.4 / M1.5 / M1.6 / M1.7 / M1.8 / M2.1).

Re-exports the public typed models, the explicit simulation clock, the
differential-drive kinematics, the minimal robot behavior abstraction, the
multi-robot world, the neighbor query abstraction, the minimal text-based
debug renderer, and the pure Lennard-Jones potential/force functions. Pure
Python: no GTK/PyGObject dependency, so it imports headlessly for tests and
experiments.
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
from layka.lennard_jones import lennard_jones_force, lennard_jones_potential
from layka.neighbors import Neighbor, NeighborSensor
from layka.pose import Pose2D, normalize_angle
from layka.renderer import DebugRenderer, TrajectoryRecorder, heading_arrow
from layka.robot import RobotConfig, RobotState
from layka.vector import Vector2
from layka.world import World

__all__ = [
    "Behavior",
    "BodyVelocity",
    "DebugRenderer",
    "DifferentialDriveRobot",
    "LennardJonesConfig",
    "Neighbor",
    "NeighborSensor",
    "Pose2D",
    "RobotConfig",
    "RobotState",
    "SimulationClock",
    "SimulationConfig",
    "StationaryBehavior",
    "TrajectoryRecorder",
    "TrivialMotionBehavior",
    "Vector2",
    "WheelSpeeds",
    "World",
    "body_to_wheels",
    "heading_arrow",
    "integrate_pose",
    "lennard_jones_force",
    "lennard_jones_potential",
    "normalize_angle",
    "wheels_to_body",
]

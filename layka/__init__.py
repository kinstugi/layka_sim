"""Layka simulator core (M1.3 / M1.4 / M1.5 / M1.6 / M1.7 / M1.8 / M2.1 / M2.2 / M2.4 / M2.5 / M2.6).

Re-exports the public typed models, the explicit simulation clock, the
differential-drive kinematics, the minimal robot behavior abstraction, the
multi-robot world, the neighbor query abstraction, the minimal text-based
debug renderer, the pure Lennard-Jones potential/force functions, the
M2.2 numerical-safety guards (min-distance clamp, force clamp, cutoff,
finite checks), the M2.4 2D assembly of pairwise LJ forces into
resultant vectors, the M2.5 LJ interaction component (poses ->
resultant vector, no side effects), and the M2.6 LJ controller (resultant
vector -> body velocity (v, omega)) plus its LJBehavior wrapper. Pure
Python: no GTK/PyGObject dependency, so it imports headlessly for tests
and experiments.
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
from layka.lj_controller import LJBehavior, LJController, LJControllerConfig
from layka.lj_interaction import (
    LJInteraction,
    pairwise_lj_force,
    resultant_lj_force,
)
from layka.lj_safety import clamp, safe_lj_force, safe_lj_potential
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
    "LJBehavior",
    "LJController",
    "LJControllerConfig",
    "LJInteraction",
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
    "clamp",
    "heading_arrow",
    "integrate_pose",
    "lennard_jones_force",
    "lennard_jones_potential",
    "normalize_angle",
    "pairwise_lj_force",
    "resultant_lj_force",
    "safe_lj_force",
    "safe_lj_potential",
    "wheels_to_body",
]

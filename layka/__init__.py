"""Layka simulator core (M1.3-M1.8 / M2.1-M2.8).

Re-exports the public typed models, the explicit simulation clock, the
differential-drive kinematics, the minimal robot behavior abstraction, the
multi-robot world, the neighbor query abstraction, the minimal text-based
debug renderer, the pure Lennard-Jones potential/force functions, the
M2.2 numerical-safety guards (min-distance clamp, force clamp, cutoff,
finite checks), the M2.4 2D assembly of pairwise LJ forces into
resultant vectors, the M2.5 LJ interaction component (poses ->
resultant vector, no side effects), the M2.6 LJ controller (resultant
vector -> body velocity (v, omega)) plus its LJBehavior wrapper, the
M2.7 deterministic two-robot experiment (``run_two_robot_experiment`` /
``TwoRobotResult``), and the M2.8 deterministic multi-robot aggregation
experiment (``run_aggregation_experiment`` / ``AggregationResult``).
M2.9 adds the SEARCH <-> SWARM state-machine behavior for isolated robots
(``SearchSwarmBehavior`` / ``SearchSwarmConfig``). The simulator wiring adds
the world-boundary containment wrapper (``BoundaryContainmentBehavior``), the
headless frame-primitive builder for the GTK renderer (``build_frame_items``),
and its thin viewer glue (``LaykaWorldView``); the GTK entry point itself
lives in ``layka.sim`` (imports ``gi``, so it is not re-exported here).
M2.10 adds static circular obstacles (``Obstacle``), world storage for them
(``World.add_obstacle`` / ``World.obstacles`` / ``World.obstacle_count``),
and the ``ObstacleAvoidanceBehavior`` wrapper that overrides the inner swarm
command with a steer-away command near an obstacle (obstacles are world
geometry, never robots: they never enter the neighbor/LJ pipeline).
M2.11 adds the pure, read-only swarm metrics (``mean_pairwise_distance``,
``centroid``, ``mean_centroid_distance``, ``distance_variance``,
``cluster_radius``, ``aggregation_score``) plus the ``positions_from_world``
extraction helper.
Pure Python: no GTK/PyGObject dependency, so it imports headlessly for
tests and experiments.
"""

from layka.behavior import Behavior, StationaryBehavior, TrivialMotionBehavior
from layka.boundary import BoundaryContainmentBehavior
from layka.clock import SimulationClock
from layka.config import LennardJonesConfig, SimulationConfig
from layka.experiments import (
    AggregationResult,
    TwoRobotResult,
    run_aggregation_experiment,
    run_two_robot_experiment,
)
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
from layka.metrics import (
    aggregation_score,
    centroid,
    cluster_radius,
    distance_variance,
    mean_centroid_distance,
    mean_pairwise_distance,
    positions_from_world,
)
from layka.neighbors import Neighbor, NeighborSensor
from layka.obstacle import Obstacle
from layka.obstacle_avoidance import ObstacleAvoidanceBehavior
from layka.pose import Pose2D, normalize_angle
from layka.proximity_sensor import (
    ProximitySensorConfig,
    SensorReading,
    compute_sensor_readings,
)
from layka.renderer import DebugRenderer, TrajectoryRecorder, heading_arrow
from layka.robot import RobotConfig, RobotState
from layka.search_behavior import SearchSwarmBehavior, SearchSwarmConfig
from layka.sim_view import LaykaWorldView, build_frame_items
from layka.vector import Vector2
from layka.world import World

__all__ = [
    "AggregationResult",
    "Behavior",
    "BodyVelocity",
    "BoundaryContainmentBehavior",
    "DebugRenderer",
    "DifferentialDriveRobot",
    "LJBehavior",
    "LJController",
    "LJControllerConfig",
    "LJInteraction",
    "LaykaWorldView",
    "LennardJonesConfig",
    "Neighbor",
    "NeighborSensor",
    "Obstacle",
    "ObstacleAvoidanceBehavior",
    "Pose2D",
    "ProximitySensorConfig",
    "RobotConfig",
    "RobotState",
    "SearchSwarmBehavior",
    "SearchSwarmConfig",
    "SensorReading",
    "SimulationClock",
    "SimulationConfig",
    "StationaryBehavior",
    "TrajectoryRecorder",
    "TrivialMotionBehavior",
    "TwoRobotResult",
    "Vector2",
    "WheelSpeeds",
    "World",
    "body_to_wheels",
    "build_frame_items",
    "aggregation_score",
    "centroid",
    "clamp",
    "cluster_radius",
    "compute_sensor_readings",
    "distance_variance",
    "heading_arrow",
    "integrate_pose",
    "lennard_jones_force",
    "lennard_jones_potential",
    "mean_centroid_distance",
    "mean_pairwise_distance",
    "normalize_angle",
    "pairwise_lj_force",
    "positions_from_world",
    "resultant_lj_force",
    "run_aggregation_experiment",
    "run_two_robot_experiment",
    "safe_lj_force",
    "safe_lj_potential",
    "wheels_to_body",
]

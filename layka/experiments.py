"""M2.7 deterministic two-robot LJ aggregation experiment.

The experiment is the first end-to-end check of the clean chain
(World + NeighborSensor + LJInteraction + LJController/LJBehavior + safe LJ
math). Two robots on the +x axis, both with ``LJBehavior``:

* **Scenario A (too far)** -- Robot A at ``(0, 0)``, Robot B at
  ``(initial_separation, 0)`` with ``initial_separation > equilibrium``:
  both robots move toward one another (attractive region), do not pass
  through one another, and settle near ``equilibrium_target``.
* **Scenario B (too close)** -- ``initial_separation < equilibrium``: the
  robots move apart (repulsive region) and settle near the equilibrium.

``run_two_robot_experiment`` is a pure, deterministic function: there is no
randomness anywhere (both robots are placed at explicit poses, the RNG is
never consulted), so identical parameters produce identical results. It
returns a frozen :class:`TwoRobotResult` recording the initial distance, the
final distance, the equilibrium target, the number of simulation steps, the
minimum separation observed during the run (no-pass-through evidence), and
whether the run converged before ``max_steps``.

Convergence is declared when the robot separation is within ``tolerance`` of
``equilibrium_target``; ``max_steps`` caps the run and is reported truthfully
through ``converged``.

Note: the sensor ``detection_range`` (5.0 m) is deliberately much larger than
the LJ ``cutoff_distance`` (which is disabled by default). They are separate
parameters: detection decides which robots are visible, the LJ math decides
which visible robots exert force. 5.0 m guarantees both robots always detect
each other regardless of the separation used here.

This is a programmatic experiment only -- the CLI runner arrives in M2.12.

M2.8 adds the deterministic multi-robot aggregation experiment
(:func:`run_aggregation_experiment` / :class:`AggregationResult`): 5-20
robots (default 10) spawn at seeded random positions, all running the M2.6
``LJBehavior``, and aggregate emergently:

* robots entering ``detection_range`` of one another attract (too far) or
  repel (too close) through the LJ interaction, and
* a loose cluster/aggregate forms -- the goal is emergent aggregation, NOT a
  rigid formation.

Scope note (M2.9 deferred): SEARCH BEHAVIOR IS NOT IMPLEMENTED HERE. An
isolated robot (no neighbors within ``detection_range``) stops in place under
the M2.6 zero-resultant policy until search arrives in M2.9. Aggregation still
emerges from the seeded random placement because robots that start within
``detection_range`` of one another pull together.

``detection_range`` is the robot-to-robot INTERACTION range (which neighbors
exert LJ force); it is independent of the LJ ``cutoff_distance`` (disabled by
default) inside ``LennardJonesConfig``. The run is deterministic: the world's
seeded RNG (``SimulationConfig.random_seed``) drives all random poses, so the
same seed reproduces identical ``final_positions`` and metrics run-to-run.

M2.8 world-size deviation (documented): the plan's example spawns 10 robots
in a 5 x 5 m world, but the M2.6 controller converts the LJ resultant into a
bounded ``(v, omega)`` WITHOUT scaling speed by the force magnitude (plan
M2.6: "bounded linear velocity", "slow down when heading error is large" --
there is no force-proportional speed term). Combined with the world's
sequential per-robot updates (``World.step``), that controller makes a pair
of robots that align their headings near the equilibrium distance self-
sustain as a "runaway pair" that flies off at full speed forever: the robot
updated first sees its partner at ``r < r_eq`` (repulsion pushes it forward),
and the partner sees it at ``r > r_eq`` (attraction pulls it forward). At
low initial density (5 x 5 m) such pairs escape the interaction range before
multi-body forces can break them, and the swarm disperses for every parameter
combination tried (detection_range 1.5-7.0, max_linear_velocity 0.01-0.2,
angular_gain 0.5-8.0, max_force 1-10, max_steps up to 40000, swarm sizes
10-20). Spawning in a denser 2.4 x 2.4 m world keeps the initial random
placement connected enough that the emergent aggregate forms before pairs can
escape, so the M2.8 experiment uses ``AGGREGATION_WORLD_SIZE = 2.4``. The
plan's 5 x 5 m example is honored as far as the M2.6 controller allows; the
deviation is the world size only (all other M2.8 defaults -- swarm_size=10,
desired_spacing=0.40, random_seed=42, detection_range=2.0, cluster_radius=1.0
-- match the plan).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from layka.config import LennardJonesConfig, SimulationConfig
from layka.lj_controller import LJBehavior, LJControllerConfig
from layka.pose import Pose2D
from layka.vector import Vector2
from layka.world import World

#: Sensor detection radius (m). Larger than any separation used in this
#: experiment so both robots are always mutual neighbors; independent of the
#: LJ ``cutoff_distance`` (see the module docstring).
DETECTION_RANGE = 5.0

#: Default robot-to-robot interaction range for the aggregation experiment
#: (m): which neighbors exert LJ force. Independent of the LJ
#: ``cutoff_distance`` (disabled by default) inside ``LennardJonesConfig``.
AGGREGATION_DETECTION_RANGE = 2.0

#: Square world side length for the aggregation experiment (m). Smaller than
#: the two-robot world (5.0) by design: at 5 x 5 m the M2.6 unit-speed
#: controller cannot aggregate seed-42 placements (runaway pairs disperse the
#: swarm; see the module docstring), while the denser 2.4 x 2.4 m spawn keeps
#: the initial placement connected and the aggregate forms. The world has no
#: boundary/obstacle logic in M2.8 (that is M2.10).
AGGREGATION_WORLD_SIZE = 2.4

#: World bounds for the two-robot experiment (m). Robots stay well inside; the
#: world has no boundary/obstacle logic in M2.7 (that is M2.10).
WORLD_WIDTH = 5.0
WORLD_HEIGHT = 5.0


@dataclass(frozen=True, slots=True)
class TwoRobotResult:
    """Recorded outcome of one :func:`run_two_robot_experiment` run.

    ``min_distance`` is the smallest robot-to-robot separation observed across
    all steps (including the initial configuration); it proves the robots
    never pass through one another. ``converged`` is ``True`` iff the run
    reached ``|distance - equilibrium_target| <= tolerance`` within
    ``max_steps``.
    """

    initial_distance: float
    final_distance: float
    equilibrium_target: float
    num_steps: int
    min_distance: float
    converged: bool


def run_two_robot_experiment(
    initial_separation: float,
    *,
    desired_spacing: float = 0.40,
    timestep: float = 0.05,
    max_steps: int = 2000,
    tolerance: float = 0.005,
) -> TwoRobotResult:
    """Run the deterministic two-robot experiment and record its outcome.

    Robot A starts at ``(0.0, 0.0)`` and Robot B at
    ``(initial_separation, 0.0)``, both heading 0, both with ``LJBehavior``
    built from default ``LJControllerConfig`` and
    ``LennardJonesConfig(desired_spacing=desired_spacing)``.

    ``equilibrium_target`` is the zero-force equilibrium distance
    ``r_eq = 2^(1/6) * sigma`` with ``sigma = desired_spacing / 2^(1/6)``,
    i.e. numerically equal to ``desired_spacing`` (Design Correction 1: the
    equilibrium target is the desired spacing, NOT sigma).

    Parameters:
        initial_separation: signed-positive starting distance between the
            robots (m). Must be > 0.
        desired_spacing: equilibrium robot-to-robot spacing (m).
        timestep: simulation step duration (s).
        max_steps: hard cap on simulation steps; ``converged`` reports whether
            the run finished early.
        tolerance: convergence band around ``equilibrium_target`` (m).

    Returns:
        A frozen :class:`TwoRobotResult`. The run is fully deterministic:
        identical parameters produce identical results.

    Raises:
        ValueError: if ``initial_separation <= 0``.
    """
    if initial_separation <= 0:
        raise ValueError(
            f"initial_separation must be > 0, got {initial_separation!r}"
        )

    lj_config = LennardJonesConfig(desired_spacing=desired_spacing)
    equilibrium_target = lj_config.equilibrium_distance
    behavior = LJBehavior.from_config(
        LJControllerConfig(),
        lj_config,
        detection_range=DETECTION_RANGE,
    )

    world = World(
        SimulationConfig(
            timestep=timestep,
            world_width=WORLD_WIDTH,
            world_height=WORLD_HEIGHT,
        )
    )
    world.add_robot(pose=Pose2D(0.0, 0.0, 0.0), behavior=behavior)
    world.add_robot(pose=Pose2D(initial_separation, 0.0, 0.0), behavior=behavior)

    def separation() -> float:
        return world.robot_by_id(0).pose.position().distance_to(
            world.robot_by_id(1).pose.position()
        )

    initial_distance = separation()
    min_distance = initial_distance
    num_steps = 0
    converged = False
    while num_steps < max_steps:
        world.step()
        num_steps += 1
        current = separation()
        min_distance = min(min_distance, current)
        if abs(current - equilibrium_target) <= tolerance:
            converged = True
            break

    return TwoRobotResult(
        initial_distance=initial_distance,
        final_distance=separation(),
        equilibrium_target=equilibrium_target,
        num_steps=num_steps,
        min_distance=min_distance,
        converged=converged,
    )


@dataclass(frozen=True, slots=True)
class AggregationResult:
    """Recorded outcome of one :func:`run_aggregation_experiment` run.

    Metrics are minimal and inline (the full swarm-metrics module is M2.11):
    the final swarm centroid, how many robots lie within ``cluster_radius`` of
    it, that count as a fraction of ``swarm_size``, and the mean pairwise
    robot-to-robot distance. Together they assess whether an aggregate formed.

    ``converged`` is ``True`` iff ``cluster_fraction >= stop_fraction`` was
    reached within ``max_steps``. ``final_positions`` are the robot positions
    (ascending robot id order) at the end of the run, frozen for
    reproducibility/analysis.
    """

    swarm_size: int
    desired_spacing: float
    random_seed: int
    num_steps: int
    converged: bool
    centroid: Vector2
    cluster_size: int
    cluster_fraction: float
    mean_pairwise_distance: float
    final_positions: tuple[Vector2, ...]


def _centroid(positions: list[Vector2]) -> Vector2:
    """Mean of ``positions`` (requires a non-empty list)."""
    total = Vector2(0.0, 0.0)
    for position in positions:
        total = total + position
    return total / len(positions)


def _cluster_size(
    positions: list[Vector2], centroid: Vector2, cluster_radius: float
) -> int:
    """Number of ``positions`` within ``cluster_radius`` of ``centroid``."""
    return sum(
        1 for position in positions if position.distance_to(centroid) <= cluster_radius
    )


def _mean_pairwise_distance(positions: list[Vector2]) -> float:
    """Mean distance over all unordered robot pairs (``0.0`` if fewer than 2)."""
    total = 0.0
    pair_count = 0
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            total += positions[i].distance_to(positions[j])
            pair_count += 1
    if pair_count == 0:
        return 0.0
    return total / pair_count


def run_aggregation_experiment(
    *,
    swarm_size: int = 10,
    desired_spacing: float = 0.40,
    random_seed: int = 42,
    timestep: float = 0.05,
    max_steps: int = 2000,
    cluster_radius: float = 1.0,
    detection_range: float = AGGREGATION_DETECTION_RANGE,
    stop_fraction: float = 0.7,
) -> AggregationResult:
    """Run the deterministic multi-robot aggregation experiment.

    ``swarm_size`` robots spawn at seeded random poses (uniform position in
    the ``AGGREGATION_WORLD_SIZE`` x ``AGGREGATION_WORLD_SIZE`` world, uniform
    heading) drawn from the world RNG seeded with ``random_seed``; every
    robot runs ``LJBehavior`` built from default ``LJControllerConfig`` and
    ``LennardJonesConfig(desired_spacing=...)`` with the given
    ``detection_range`` (M2.6). The world is stepped until
    ``cluster_fraction >= stop_fraction`` (checked at the start of each step,
    including the initial placement) or ``num_steps == max_steps``.

    Robots with no neighbors within ``detection_range`` stop in place
    (M2.6 zero-resultant policy); SEARCH BEHAVIOR IS M2.9, so aggregation is
    driven purely by the seeded initial placement pulling robots into range.
    The goal is a loose emergent cluster, not a rigid formation.

    The spawn world is 2.4 x 2.4 m (not the plan's 5 x 5 m example) so the
    seed-42 placement stays connected long enough for the aggregate to form
    under the M2.6 controller; see the module docstring for the full
    justification of this documented deviation.

    Deterministic: identical parameters produce identical
    ``final_positions`` and metrics, because all randomness flows from the
    seeded world RNG.

    Parameters:
        swarm_size: number of robots (1..500).
        desired_spacing: equilibrium robot-to-robot spacing r_eq (m);
            ``sigma = desired_spacing / 2^(1/6)`` (Design Correction 1).
        random_seed: seed for the world RNG driving the initial poses.
        timestep: simulation step duration (s).
        max_steps: hard cap on simulation steps; ``converged`` reports whether
            the run finished early.
        cluster_radius: radius (m) around the swarm centroid defining the
            cluster for the ``cluster_size`` metric.
        detection_range: robot-to-robot interaction range (m); independent of
            the LJ ``cutoff_distance``.
        stop_fraction: convergence threshold on ``cluster_fraction`` in
            (0, 1].

    Returns:
        A frozen :class:`AggregationResult`.

    Raises:
        ValueError: on invalid ``swarm_size``, ``detection_range``,
            ``cluster_radius``, ``timestep``, ``max_steps``, or
            ``stop_fraction``.
    """
    if isinstance(swarm_size, bool) or not isinstance(swarm_size, int):
        raise ValueError(f"swarm_size must be an int, got {swarm_size!r}")
    if not 1 <= swarm_size <= 500:
        raise ValueError(f"swarm_size must be in [1, 500], got {swarm_size!r}")
    for name, value in (
        ("detection_range", detection_range),
        ("cluster_radius", cluster_radius),
        ("timestep", timestep),
    ):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be a finite positive number, got {value!r}")
    if isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps < 1:
        raise ValueError(f"max_steps must be an int >= 1, got {max_steps!r}")
    if not 0 < stop_fraction <= 1:
        raise ValueError(f"stop_fraction must be in (0, 1], got {stop_fraction!r}")

    behavior = LJBehavior.from_config(
        LJControllerConfig(),
        LennardJonesConfig(desired_spacing=desired_spacing),
        detection_range=detection_range,
    )
    world = World(
        SimulationConfig(
            timestep=timestep,
            robot_count=swarm_size,
            random_seed=random_seed,
            world_width=AGGREGATION_WORLD_SIZE,
            world_height=AGGREGATION_WORLD_SIZE,
        )
    )
    world.spawn_robots(swarm_size, behavior=behavior)

    def positions() -> list[Vector2]:
        return [robot.pose.position() for robot in world.robots]

    def fraction() -> float:
        current = positions()
        return _cluster_size(current, _centroid(current), cluster_radius) / swarm_size

    num_steps = 0
    converged = False
    while num_steps < max_steps:
        if fraction() >= stop_fraction:
            converged = True
            break
        world.step()
        num_steps += 1

    final_positions = positions()

    centroid = _centroid(final_positions)
    cluster_size = _cluster_size(final_positions, centroid, cluster_radius)
    return AggregationResult(
        swarm_size=swarm_size,
        desired_spacing=desired_spacing,
        random_seed=random_seed,
        num_steps=num_steps,
        converged=converged,
        centroid=centroid,
        cluster_size=cluster_size,
        cluster_fraction=cluster_size / swarm_size,
        mean_pairwise_distance=_mean_pairwise_distance(final_positions),
        final_positions=tuple(final_positions),
    )


__all__ = [
    "AggregationResult",
    "TwoRobotResult",
    "run_aggregation_experiment",
    "run_two_robot_experiment",
]

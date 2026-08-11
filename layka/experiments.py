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
"""

from __future__ import annotations

from dataclasses import dataclass

from layka.config import LennardJonesConfig, SimulationConfig
from layka.lj_controller import LJBehavior, LJControllerConfig
from layka.pose import Pose2D
from layka.world import World

#: Sensor detection radius (m). Larger than any separation used in this
#: experiment so both robots are always mutual neighbors; independent of the
#: LJ ``cutoff_distance`` (see the module docstring).
DETECTION_RANGE = 5.0

#: World bounds for the experiment (m). Robots stay well inside; the world has
#: no boundary/obstacle logic in M2.7 (that is M2.10).
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


__all__ = ["TwoRobotResult", "run_two_robot_experiment"]

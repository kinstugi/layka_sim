# Layka Simulator

A small, clean 2D multi-robot simulator where robot-to-robot interaction is
driven primarily by a Lennard-Jones (LJ) potential.

The simulator demonstrates:

- multiple robots moving in a 2D world;
- robots detecting nearby robots (and their relative pose/distance);
- attraction when robots are too far apart and repulsion when they are too
  close;
- stable aggregation at a configurable preferred separation;
- a SEARCH <-> SWARM state machine so isolated robots patrol until they find
  the swarm;
- sensor-driven obstacle avoidance (IR proximity sensors), world-boundary
  containment, and obstacle interaction;
- deterministic, reproducible experiments with configurable parameters and
  quantitative swarm metrics;
- a GTK visualization with debug overlays for neighbor links, IR sensor
  cones, and LJ force vectors.

The implementation prioritizes correctness, simplicity, and inspectability
over realism. The core math lives in the `layka/` package and is covered by
an extensive headless test suite (`uv run pytest`).

## Table of Contents

- [Getting Started](#getting-started)
- [User Interface](#user-interface)
- [Architecture](#architecture)
- [Swarm Behavior](#swarm-behavior)
- [Experiments and Metrics](#experiments-and-metrics)
- [Tests](#tests)
- [Legacy Code](#legacy-code)

## Getting Started

### Requirements

- Python >= 3.10 (the repository is tested with 3.12).
- [Gtk 3](https://www.gtk.org/) with
  [PyGObject 3](https://pygobject.readthedocs.io/) for the UI, plus the
  system GTK3 / gobject-introspection development packages.
- Runtime Python dependencies: `PyGObject` and `pydantic`. Development adds
  `pytest`.

See [docs/installation.md](docs/installation.md) for the full installation
procedure (both `uv` and `pip` + `venv` paths), the required system packages,
and how to run the tests.

### Running the Simulator

From the repository root:

```
uv run python simulator.py
```

(or `python simulator.py` inside an activated virtual environment, or
`uv run python -m layka.sim`).

The simulator is a GTK application and requires a display; without one it
exits at startup. The test suite and the experiment runners do **not**
require a display.

### Running the Experiments

Experiments can be run headlessly from the terminal — no GUI needed:

```
uv run python -m experiments.aggregate --robots 10 --spacing 0.40 --seed 42 --steps 5000
uv run python -m experiments.two_robots --separation 0.60 --steps 2000
```

Same parameters + same seed always produce the same result (fully
deterministic). Add `--out result.json` (or `.csv`) to save the result.

## User Interface

The simulator window shows the world in the center of a GTK drawing area
(centered and auto-fitted to the world bounds). The control panel has three
button rows:

1. **Play / Stop / Step / Reset** — standard simulation transport controls.
   "Step" advances exactly one simulation timestep (`dt = 0.05` s); "Reset"
   rebuilds the same seeded world.
2. **Random Map** — respawns the robots at new random positions (new seed).
   (The legacy "Save Map" / "Load Map" buttons are **not** supported by the
   new `layka` World, which has no map system.)
3. **Show Invisibles** — toggles the debug overlay. When enabled it draws:
   - the robot **IR sensor cones** (green when detecting another robot, red
     when detecting an obstacle, fading with distance);
   - **neighbor links** between robots within detection range;
   - each robot's **resultant LJ vector** (orange arrow) and, in some views,
     the **pairwise force vectors** (brown arrows).

### What is drawn

- **Robots** — blue circles with a red heading line.
- **Obstacles** — filled dark-gray circles (static).
- **Grid + world boundary** — light-gray 1 m grid and a dark boundary.
- **Trajectories** — each robot's recent path (steel-blue), shown as part of
  the debug overlay.

## Architecture

The simulator is built from a clean, typed, layered `layka/` package. Each
component is pure Python and headless-testable; only the GTK entry points
import `gi`.

| Component | Module | Purpose |
|---|---|---|
| Core models | `layka/vector.py`, `layka/pose.py`, `layka/robot.py`, `layka/config.py` | `Vector2`, `Pose2D`, `RobotState`/`RobotConfig`, `SimulationConfig`/`LennardJonesConfig` (Pydantic-validated) |
| Clock | `layka/clock.py` | Explicit timestep `dt`; simulation time is decoupled from rendering |
| Kinematics | `layka/kinematics.py` | Differential-drive `(v, ω) ↔ (v_left, v_right)` and pose integration |
| Behaviors | `layka/behavior.py`, `layka/search_behavior.py`, `layka/obstacle_avoidance.py`, `layka/boundary.py` | `Behavior` protocol; SEARCH↔SWARM; IR-sensor obstacle avoidance; boundary containment |
| World | `layka/world.py` | Holds robots + static obstacles, drives one step per `dt` |
| Sensing | `layka/neighbors.py`, `layka/proximity_sensor.py` | Neighbor queries; ray-cast IR sensors (robot vs obstacle discrimination) |
| LJ interaction | `layka/lennard_jones.py`, `layka/lj_safety.py`, `layka/lj_interaction.py`, `layka/lj_controller.py` | Pure potential/force; numerical safety; pairwise→resultant vectors; vector→motion controller |
| Metrics | `layka/metrics.py` | Mean pairwise distance, cluster radius, variance, aggregation score |
| Experiments | `layka/experiments.py`, `experiments/` | Deterministic two-robot and multi-robot runs + CLI runners |
| Debug overlay | `layka/lj_overlay.py`, `layka/renderer.py` | LJ force vectors; text-based debug rendering |
| GUI | `layka/sim_view.py`, `layka/sim.py` | GTK renderer glue and the simulator controller |

## Swarm Behavior

The robot behaviors follow the plan's intended chain:

```
search -> detect robots -> LJ interaction -> aggregate
```

### SEARCH <-> SWARM state machine

- **SEARCH**: a robot with no neighbors patrols deterministically — move
  forward for a fixed interval, then turn in place — detecting neighbors
  every step.
- **SWARM**: when neighbors are detected, the robot aggregates via the
  Lennard-Jones interaction.
- Transitions: `SEARCH -> SWARM` on detection, `SWARM -> SEARCH` when no
  neighbors remain.

### Lennard-Jones potential

The standard LJ potential and force are implemented as pure functions in
`layka/lennard_jones.py`:

```
V(r) = 4ε((σ/r)¹² − (σ/r)⁶)
F(r) = (24ε/r)(2(σ/r)¹² − (σ/r)⁶)     # positive = repulsive, negative = attractive
```

> **Important**: `sigma` is the potential's zero-crossing, **not** the
> equilibrium distance. The zero-force equilibrium distance is
> `r_eq = 2^(1/6) · σ`. The user-facing parameter is `desired_spacing`
> (e.g. 0.40 m), and `sigma` is derived internally as
> `desired_spacing / 2^(1/6)`.

Numerical safety (`layka/lj_safety.py`) clamps the interaction distance and
the force magnitude so no NaN/infinity can appear during a normal simulation.

### Motion

The resultant LJ vector is converted into a feasible differential-drive
velocity: desired heading `= atan2(F_y, F_x)`, proportional angular control on
the wrapped heading error, and a bounded linear velocity that slows down when
the heading error is large.

### Obstacle interaction

Obstacles are static geometry and are **never treated as robots**. Each robot
carries a ring of ray-cast IR proximity sensors; an obstacle-detecting sensor
(green/red cone rendering) drives the avoidance override, steering the robot
around the obstacle before it commits to it. World boundaries are handled by a
containment wrapper. Priority: collision safety > obstacle avoidance > swarm
interaction.

## Experiments and Metrics

- `run_two_robot_experiment(initial_separation, ...)` — robots too far move
  together, robots too close move apart, both settle at the equilibrium
  spacing.
- `run_aggregation_experiment(swarm_size=10, desired_spacing=0.40,
  random_seed=42, ...)` — seeded random spawns aggregate into a cluster
  (cluster fraction, cluster size, mean pairwise distance recorded).
- `layka.metrics` provides the measurements: `mean_pairwise_distance`,
  `centroid`, `distance_variance`, `cluster_radius`, `aggregation_score`.

Both experiments are deterministic: the same parameters and seed produce
identical results, and results can be saved as JSON or CSV via the CLI.

## Tests

```
uv run pytest
```

The suite is headless (no display required) and covers the models, clock,
kinematics, behaviors, neighbor queries, LJ math and safety, sensor ray
casting, obstacle avoidance, swarm metrics, the experiment CLI, and the
rendering primitives.

## Legacy Code

The original Sobot-Rimulator-style code (the GTK simulator, `models/`,
`robot_control/`, `views/`, `utils/`, `maps/`, ...) is preserved untouched
under [`legacy_code/`](legacy_code/) and can still be launched with:

```
uv run python legacy_code/simulator.py
```

The new `layka` simulator reuses only the generic GTK window/button chrome
from `legacy_code/gui`; it does not use the legacy robot/world/physics code.

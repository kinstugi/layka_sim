# Current Architecture (M1.1 Repository Audit)

Audit date: 2026-08-11. This document describes the repository **as it is**,
before any M1/M2 restructuring. Every claim was checked against the source
files listed below. No source file was modified to produce this document.

---

## 1. High-level module map

```
simulator.py  (entry point, GTK glue, control buttons)
   |
   +--> gui/viewer.py           Gtk window, control panel, file dialogs
   |        +--> gui/frame.py, gui/painter.py, gui/color_palette.py
   |
   +--> models/world.py         World container, owns dt, step()
   |        +--> models/physics.py      collision detection + sensor ray-casting
   |        +--> models/custom_robots/layka.py  (default robot: 10 spawned)
   |        |        +--> models/custom_robots/mobile_robot.py
   |        |              +--> robot_control/custom_behavior/swarm_force_behavior.py  (active "supervisor")
   |        |              +--> models/differential_drive_dynamics.py
   |        |              +--> models/proximity_sensor.py x9, models/wheel_encoder.py x2
   |        |              +--> models/pose.py, models/polygon.py
   |        +--> models/map_manager.py   load/save/random maps
   |        +--> models/rectangle_obstacle.py
   |
   +--> views/world_view.py     draws grid, robots, obstacles
             +--> views/robot_view.py -> views/swarm_force_behavior_view.py,
             |                       views/proximity_sensor_view.py
             +--> views/obstacle_view.py

utils/            linalg2_util, geometrics_util, math_util, constants (plain math)
sim_exceptions/   CollisionException (live), GoalReachedException (dead path)
```

Direction of information flow at runtime: `Simulator -> World.step() ->`
`robot motion -> physics (collisions, sensor rays) -> supervisor (reads
sensors, decides, writes wheel rates) -> draw`.

---

## 2. Simulation loop

Two distinct layers:

1. **Driving loop (GTK-based, wall-clock):** `simulator.py`,
   `Simulator._run_sim()` schedules itself with
   `GLib.timeout_add(int(self.period * 1000), ...)` at `REFRESH_RATE = 20.0`
   Hz (`period = 0.05 s`). Each tick calls `_step_sim()`, which calls
   `self.world.step()` and then redraws. Play/stop/step/reset buttons map to
   `play_sim` / `pause_sim` / `step_sim_once` / `reset_sim`, all of which
   add/remove this `GLib` source.
2. **Fixed-timestep world update:** `models/world.py`, `World.step()`.
   Order is fixed:
   `robot.step_motion(dt)` for every robot -> `physics.apply_physics()`
   (collision check, then proximity-sensor ray-casting) -> `supervisor.step(dt)`
   for every supervisor (observes the *current* step, as the comment notes) ->
   `world_time += dt`.

Note: `dt` is captured once at construction (`World(self.period)`) and the
physics/control update is **not** tied to render FPS (GLib fires the timeout,
but the timeout only triggers one fixed `dt` step). This already satisfies the
"explicit timestep" requirement at a basic level.

The sim ends when `Physics` raises `CollisionException` (displayed as
"Collision!") or when a supervisor raises `GoalReachedException` (dead path,
see section 7).

---

## 3. Robot model

Default robot is `Layka` (`models/custom_robots/layka.py`), a
`MobileRobot` subclass (`models/custom_robots/mobile_robot.py`). Physical
parameters (Khepera III profile, duplicated as constants in several files):

| Parameter | Value |
|---|---|
| wheel radius | 0.021 m |
| wheel base length | 0.0885 m |
| max wheel drive rate | 15.0 rad/s (per wheel) |
| IR sensors | 9, min range 0.02 m, max range 0.2 m, FOV 40° (Layka) / 20° (KheperaIII) |
| body | 8-vertex convex polygon (`ROBOT_OUTLINE`) |

- **Kinematics:** `models/differential_drive_dynamics.py` integrates the
  unicycle-equivalent differential drive pose update directly from wheel
  rates:
  `v_l, v_r -> d_left, d_right -> d_center -> new_x, new_y, new_theta`,
  and increments both `WheelEncoder`s. `robot.step_motion(dt)` applies
  dynamics, refreshes `global_geometry`, and repositions sensors.
- **Interface boundary:** the supervisor never touches the robot directly; it
  goes through `robot_control/robot_supervisor_interface.py`
  (`read_proximity_sensors`, `read_wheel_encoders`,
  `set_wheel_drive_rates`, plus robot-specific
  `read_robot_detection_array`, `read_robot_neighbors_pose`).
- `models/robot.py` defines a separate, older `Robot` class with identical
  Khepera III constants. It is imported by `simulator.py` but **never
  instantiated** (dead code).

**Classification: working.** The kinematic/pose/sensor plumbing is intact and
runs. Candidate for *replacement of the constant soup* by typed config
(M1.3), but the differential-drive math itself is reusable (M1.5).

---

## 4. World / map representation

- `models/world.py`: holds `robots`, `obstacles`, `supervisors`, `physics`,
  `world_time`, `dt`. `add_robot()` also appends
  `robot.supervisor` to `world.supervisors` (implicit coupling).
- `models/map_manager.py`: `random_map(world)` generates a goal and 10-50
  random `RectangleObstacle`s at 0.4-6.0 m, avoiding overlap with robots and
  the goal; `save_map`/`load_map` pickle the obstacle list + goal;
  `apply_to_world` adds obstacles and writes `robot.supervisor.goal`.
  - Latent bug: a fresh `MapManager` has `current_goal = None`;
    `apply_to_world` does `self.current_goal[:]`, so calling it before any
    map exists raises `TypeError`. The default startup path avoids this by
    always generating a random map first (see section 9).
- Map files in `maps/` are Python pickle (protocol 0) dumps of
  `[list[RectangleObstacle], goal]`, e.g. `maps/box`. They load correctly
  with `MapManager.load_map`.
- `models/rectangle_obstacle.py` / `models/polygon.py` /
  `models/line_segment.py` / `models/geometry.py`: convex-polygon world
  objects with bounding circles and SAT collision support. Static obstacles
  have a `pose` but no dynamics; robots carry a `global_geometry` updated
  every step.
- `World.colliders()` returns only robots; obstacles are never tested against
  each other.

**Classification: working** (with the latent `apply_to_world` ordering bug
noted above). Geometry code is sound; the map/obstacle layer is a candidate
for *reuse* rather than replacement.

---

## 5. Sensors

- `models/sensor.py`: abstract base (`read()` raises `NotImplementedError`).
- `models/proximity_sensor.py`: a `LineSegment` "detector line" from the
  sensor out to `max_range`, transformed to the world each step. `Physics`
  ray-casts it against every solid, records the closest hit distance, and
  maps it through an exponential model to an integer `read_value` in
  [18, 3960] (`0.02 - log(read/3960)/30` is used to recover meters).
  Also carries `detecting_robot: bool` and
  `detected_robots_pose: list[(Pose, d)]` — the *de facto* robot neighbor
  channel.
- `models/wheel_encoder.py`: tick counter for odometry.
- Robot detection is implemented in `models/physics.py`
  `_update_proximity_sensors()` via
  `if solid.__str__() == "robot"` — a fragile string-based discrimination
  that works only because `MobileRobot.__str__` returns `"robot"`.

**Classification: partially working / tightly coupled.** The ray-casting and
read model work, but robot-vs-obstacle discrimination by `__str__` and the
neighbor information stored on per-sensor attributes is exactly the kind of
"overbuilt perception" the plan wants replaced by an explicit
`robot -> visible neighbors -> relative pose/distance` abstraction (M1.7).

---

## 6. Robot controller architecture

Two controller stacks exist; only one is live.

**Live stack (used by the default robot):**
`robot_control/custom_behavior/swarm_force_behavior.py`
(`SwarmForceBehavior`). It is a self-contained "supervisor" that:
1. `_update_state()`: reads proximity sensors, converts to meters, updates
   odometry from wheel encoders, and refreshes `robot_detection_sensor_array`;
2. routes to one of `execute_search` / `execute_wait_in_swarm` /
   `execute_avoid_obstacle` based on string state (`"search"`, `"wait"`,
   `"avoid obstacle"`); the separate `LEAVE` state is unused;
3. in WAIT mode computes a resultant force vector and converts it directly to
   `(v, omega)` via `f_to_velocities` (projection onto robot heading + gains
   `LINEAR_GAIN = 0.005`, `ANGULAR_GAIN = 0.06` from `utils/constants.py`);
4. `_send_robot_commands` clamps to `v_max = 0.3148 m/s`,
   `omega_max = 2.2763 rad/s` and converts unicycle -> wheel rates.

**Dead stack (classic Sobot Rimulator):**
`robot_control/supervisor.py` (`Supervisor`) with
`supervisor_state_machine.py`, `supervisor_controller_interface.py`, and the
controllers `go_to_goal`, `avoid_obstacles`, `follow_wall`, `gtg_and_ao`,
`go_to_angle`. This hybrid-automaton stack (GO_TO_GOAL / AVOID_OBSTACLES /
SLIDE_LEFT / SLIDE_RIGHT / AT_GOAL) is only instantiated by the unused base
`models/robot.py`, so it never executes in the running system. Also dead:
`controllers/forward_controller.py`, `wait_swarm_controller.py`,
`depart_swarm_controller.py` (empty stub), `reverse_controller.py`,
`search_robots_controller.py` (contains a latent bug — reads
`self.gtg_heading_vector`, which it never sets), and the whole
`custom_behavior/swarm_behavior.py` /
`custom_behavior/behavior_state_machine.py` /
`custom_behavior/bounce_state_machine.py` trio (superseded by
`swarm_force_behavior.py`).

**Classification:**
- Live stack: **working but tightly coupled** — sensing, odometry, state
  machine, LJ math, force->velocity conversion, and wheel commands all live
  in one class; the LJ "force" is used directly as a velocity command
  (body-frame projection), not as a heading-error controller (plan design
  correction #2).
- Dead stack: **dead/unused**; the unicycle<->differential conversions in
  `Supervisor`/`SwarmForceBehavior` are duplicated and are the only pieces
  worth carrying forward (M1.5).

---

## 7. Supervisor / state machine

- The plan/README terminology: a "supervisor" is the per-robot brain; it is
  added to `world.supervisors` by `World.add_robot` and stepped last in
  `World.step()`.
- **Active:** `SwarmForceBehavior` (see section 6). Its "state machine" is a
  hardcoded `if/elif` over string states in `execute()`; `control_state.py`
  constants (`SEARCH_ROBOTS`, `WAIT_IN_SWARM`, ...) are **not** used by it.
- **Dead:** `Supervisor` + `SupervisorStateMachine` (raises
  `GoalReachedException`), `BehaviorStateMachine`,
  `BounceStateMachine`. Only reachable through the unused base `Robot`.

**Classification:** active one is working/tightly coupled; the classic
machinery is dead/unused.

---

## 8. Swarm implementation (Lennard-Jones)

`SwarmForceBehavior.calculate_proximal_vector()` implements the standard LJ
force

```
F(r) = 24 * epsilon * (2*(sigma/r)^12 - (sigma/r)^6) / r
```

with `epsilon = 1.0`, `sigma = 0.40`, iterating over
`robot.read_robot_neighbors_pose()` (poses cached on the IR sensors by
`Physics`). Direction handling: the unit vector points *toward* the
neighbor; the sign is flipped (`-force * u`), so close neighbors
(positive magnitude) repel and distant neighbors (negative magnitude)
attract — the sign convention is internally consistent.

Known issues, relevant to later milestones:

1. **sigma vs equilibrium:** the code and README treat `sigma = 0.40 m` as
   the "equilibrium distance", but the zero-force equilibrium of this
   potential is `r_eq = 2^(1/6) * sigma ~= 0.449 m`. The plan (design
   correction #1) requires exposing `desired_spacing` and deriving
   `sigma = desired_spacing / 2^(1/6)`. The formula must therefore be
   re-parameterized, not copied.
2. **Force as velocity:** `f_to_velocities` treats the force vector as a
   direct velocity command in the robot frame with arbitrary gains; the plan
   wants a proper desired-heading + heading-error controller (M2.6).
3. **No numerical safety:** no min-distance clamp, force clamp, cutoff, or
   finite-value checks (M2.2 requirement).
4. **Non-determinism:** search behavior uses the global `random` module
   without seeding.
5. Alignment/goal/noise vectors are stubs (return zeros), but are still
   multiplied by weights (`a=1, b=0.1, c=0.1, d=0.3`) in
   `calculate_f_vector()`.
6. Dead code: `calculate_noise_vector` returns `[0,0]` before its actual
   implementation (unreachable).

**Classification: partially working.** It runs and produces
search/wander/avoid behavior, but is the primary candidate for replacement by
the clean `lennard_jones` -> `interaction` -> `control` pipeline (M2.1-M2.6).
The design corrections explicitly warn against copying this file's LJ
parameterization.

---

## 9. Configuration / constants

No centralized or validated configuration exists. Constants are scattered
module-level literals:

| Where | Constants |
|---|---|
| `utils/constants.py` | `LINEAR_GAIN=0.005`, `ANGULAR_GAIN=0.06` |
| `models/robot.py`, `layka.py`, `kheperaIII.py`, `test_robot.py` | wheel/sensor Khepera III constants (duplicated) |
| `robot_control/supervisor.py`, `swarm_force_behavior.py` | `K3_TRANS_VEL_LIMIT=0.3148`, `K3_ANG_VEL_LIMIT=2.2763` |
| `swarm_force_behavior.py` | `epsilon=1.0`, `sigma=0.40`, search wander interval `5.0 s`, turn angle `pi/3` |
| `models/map_manager.py` | obstacle/goal generation ranges |
| `simulator.py` | `REFRESH_RATE=20.0` |
| `supervisor_state_machine.py` / `bounce_state_machine.py` | `D_STOP/D_CAUTION/D_DANGER` thresholds (dead path) |

There is **no dependency manifest** (`requirements.txt`, `pyproject.toml`,
`setup.py` all absent); GTK3 + PyGObject are assumed system-installed.

**Classification: partially working / candidate for replacement.** Replace
with typed, validated config (M1.3) and a reproducible environment (M1.2).

---

## 10. Rendering / UI

GTK3 (PyGObject) stack:
- `gui/viewer.py`: `Gtk.Window` + `Gtk.DrawingArea`, control buttons
  (Play/Stop/Step/Reset, Save/Load/Random Map, Show Invisibles), alert
  label, file-chooser dialogs. Exits at import time with
  `"GUI code not find the display."` if no display is available
  (`Gdk.Display.get_default()` is None).
- `gui/frame.py`: accumulates draw primitives (circles, polygons, lines).
- `gui/painter.py` + `gui/color_palette.py`: Cairo rendering of the frame;
  metric -> pixel transform with y-flip.
- `views/world_view.py`: grid (1 m major / 20 cm minor), robots, obstacles.
- `views/robot_view.py`: robot body (blue/black), traverse path,
  "Show Invisibles" extras. Constructs `SwarmForceBehaviorView` for the
  supervisor and `ProximitySensorView` per sensor. Imports
  `SupervisorView` and `BounceBehaviorView` but never uses them.
- `views/swarm_force_behavior_view.py`: draws resultant + component force
  vectors and the estimated trajectory when invisibles are shown.
- Dead views: `views/supervisor_view.py`, `views/bounce_behavior_view.py`,
  `views/controllers/*` (render the dead classic controller stack).

**Classification: working** (verified launch, see section 12). Tightly
coupled in that `Viewer` mixes rendering, app control, and file dialogs, and
the app cannot start headless. Candidate for a separate debug renderer
(M1.8/M2.13) while keeping the legacy GUI untouched.

---

## 11. Tests

**There are no tests.** No `tests/` directory, no `unittest` files, no
`pytest` configuration, and `pytest` is not installed in this environment
(environment setup is M1.2). The only file matching `*test*` is
`models/custom_robots/test_robot.py`, which is a robot definition
(`Testbot`), not a test. The `test_poses` list in `simulator.py` is also not
a test — it is an optional hardcoded spawn configuration (currently disabled
in favor of `generate_random_robot_poses(10, 0.3)`).

---

## 12. Launchability

Verified in this environment (Python 3.14.6, PyGObject with GTK 3.24.52 /
GLib 2.88, interactive Wayland session, `DISPLAY=:0`):

- `python3 simulator.py` **launches successfully** and enters the GTK main
  loop. Verified by running it under `timeout 8`/`timeout 15`: the process
  was killed by the timeout (exit 124) with an empty stderr — i.e. no import
  error, no traceback, main loop running.
- Startup path detail: `Simulator.__init__` registers
  `GLib.idle_add(self.initialize_sim, True)`, which calls
  `initialize_sim(random=True)` (GLib passes the extra argument positionally,
  verified). This generates a random map with 10 `Layka` robots. Because of
  the `MapManager.current_goal = None` latent bug, the non-random startup
  path (`apply_to_world` before any map) would crash — the default entry
  point avoids it.
- **Headless blocker (exact):** without a display,
  `gui/viewer.py` line 10-12 executes
  `display = Gdk.Display.get_default(); if not display: exit("GUI code not find the display.")`
  — so the application exits at import time with that message. There is no
  headless mode and no CLI. (This environment has a display, so the sim
  launches here.)

Required system dependencies for launch: Python 3, GTK3, PyGObject
(gobject-introspection bindings). The README suggests
`conda create -n sobot-rimulator -c conda-forge python=3 gtk3 pygobject`.
No pip-installable dependency list exists.

---

## 13. Component classification summary

| Component | Files | Status |
|---|---|---|
| Simulation driving loop (GTK) | `simulator.py` | working |
| World + timestep | `models/world.py` | working |
| Physics (collision + sensor rays) | `models/physics.py` | working, tightly coupled (string-based robot detection) |
| Robot kinematics (differential drive) | `models/differential_drive_dynamics.py` | working, reusable |
| Default robot | `models/custom_robots/layka.py`, `mobile_robot.py` | working |
| Geometry / pose | `models/pose.py`, `polygon.py`, `line_segment.py`, `geometry.py`, `rectangle_obstacle.py` | working, reusable |
| Sensors | `models/proximity_sensor.py`, `sensor.py`, `wheel_encoder.py` | working; neighbor detection candidate for replacement |
| Maps | `models/map_manager.py`, `maps/` | working (latent ordering bug) |
| Active behavior/supervisor | `robot_control/custom_behavior/swarm_force_behavior.py` | partially working, tightly coupled, candidate for replacement |
| Robot-supervisor interface | `robot_control/robot_supervisor_interface.py` | working, reusable |
| Classic supervisor stack | `supervisor.py`, `supervisor_state_machine.py`, `supervisor_controller_interface.py`, `go_to_goal/avoid_obstacles/follow_wall/gtg_and_ao/go_to_angle` controllers | dead/unused (only reachable via unused `models/robot.py`); unicycle<->diff conversion reusable |
| Extra/dead controllers | `forward`, `wait_swarm`, `depart_swarm` (stub), `reverse`, `search_robots` (buggy) | dead/unused |
| Dead behavior trio | `swarm_behavior.py`, `behavior_state_machine.py`, `bounce_state_machine.py` | dead/unused |
| Control state constants | `robot_control/control_state.py` | partially used (classic states unused) |
| Rendering/UI | `gui/*`, `views/world_view.py`, `robot_view.py`, `swarm_force_behavior_view.py`, `obstacle_view.py`, `proximity_sensor_view.py` | working; `Viewer` tightly coupled; dead view imports present |
| Dead views | `views/supervisor_view.py`, `bounce_behavior_view.py`, `views/controllers/*` | dead/unused |
| Utils | `utils/linalg2_util.py`, `geometrics_util.py`, `math_util.py`, `constants.py` | working, reusable |
| Exceptions | `sim_exceptions/collision_exception.py` (live), `goal_reached_exception.py` (dead path) | working / dead |
| Config/constants | scattered module literals, no manifest | candidate for replacement (M1.3) |
| Tests | — | none |

Cross-cutting observations for later milestones:
- The single most tightly coupled artifact is `SwarmForceBehavior`: it merges
  sensing, odometry, state machine, LJ math, and control into one class —
  the exact fusion the plan forbids (Rule 5).
- Robot-neighbor information currently flows through mutable sensor
  attributes written by `Physics`; it should become an explicit query result.
- `sigma` is mis-documented as the equilibrium distance; re-parameterize with
  `desired_spacing -> sigma = desired_spacing / 2^(1/6)` (design correction #1).
- Nothing in the current code is Newtonian; the LJ result is already treated
  as a velocity/heading input, matching design correction #2 in spirit.

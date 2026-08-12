"""GTK simulator driven by the clean ``layka`` stack (simulator wiring).

This is the NEW simulator entry point: it runs a GTK window (reusing the
legacy ``gui.Viewer`` chrome: Play/Stop/Step/Reset/Show-Invisibles buttons,
cairo painter) but the simulation itself is built entirely from the clean
``layka`` modules -- no legacy robot/world/physics code is imported:

    World (M1.6) + SimulationClock (M1.4) + kinematics (M1.5)
    + NeighborSensor (M1.7) + LJ interaction (M2.1-M2.5)
    + SearchSwarmBehavior (M2.9) + ObstacleAvoidanceBehavior (M2.10)
    + BoundaryContainmentBehavior (wiring)

Robots run ``SearchSwarmBehavior`` wrapped in ``ObstacleAvoidanceBehavior``
and ``BoundaryContainmentBehavior`` (priority: boundary containment =
collision safety outermost, obstacle avoidance next, swarm innermost):
isolated robots patrol until they detect a neighbor, then aggregate via the
Lennard-Jones interaction; the avoidance wrapper steers robots away from the
world's static circular obstacles (M2.10) and the boundary wrapper keeps
robots from leaving the visible world.

Rendering is decoupled from the clock exactly as M1.4 intended: the GLib
timeout simply calls ``world.step()`` (advancing the explicit ``dt = 0.05``)
and then redraws; the simulation state never depends on rendering FPS.

Run with:  ``uv run python -m layka.sim``  (requires a display).

Buttons:
    Play / Stop / Step / Reset  -- standard transport controls.
    Show Invisibles             -- toggles the neighbor-link (and trail)
                                   debug overlay.
    Save Map / Load Map         -- not supported by the new World (no map
                                   system yet); clicking them shows a notice.
"""

from __future__ import annotations

import os
import random as _random
import sys

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib
from gi.repository import Gtk

# The new simulator reuses the generic legacy GUI chrome (buttons/window/cairo
# painter), which now lives at legacy_code/gui. Add legacy_code to sys.path so
# ``import gui.viewer`` resolves both from ``python simulator.py`` and from
# ``python -m layka.sim`` (the root launcher also adds it, idempotently).
_LEGACY_CODE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "legacy_code"
)
if os.path.isdir(_LEGACY_CODE) and _LEGACY_CODE not in sys.path:
    sys.path.insert(0, _LEGACY_CODE)

import gui.viewer  # noqa: E402  (reused generic GTK chrome; display required)
from layka.boundary import BoundaryContainmentBehavior
from layka.config import LennardJonesConfig, SimulationConfig
from layka.lj_controller import LJController, LJControllerConfig
from layka.obstacle import Obstacle
from layka.obstacle_avoidance import ObstacleAvoidanceBehavior
from layka.proximity_sensor import ProximitySensorConfig, compute_sensor_readings
from layka.renderer import TrajectoryRecorder
from layka.search_behavior import SearchSwarmBehavior, SearchSwarmConfig
from layka.sim_view import LaykaWorldView
from layka.vector import Vector2
from layka.world import World

REFRESH_RATE = 20.0  # hertz
PERIOD = 1.0 / REFRESH_RATE  # simulation timestep (s), also the frame interval

ROBOT_COUNT = 10
WORLD_WIDTH = 5.0
WORLD_HEIGHT = 5.0
DEFAULT_SEED = 42
BOUNDARY_MARGIN = 0.3  # m from each edge that triggers containment
#: Fraction of the IR sensor max_range at which an obstacle triggers avoidance.
#: 0.9 = a 0.18 m reaction distance: the robot veers around the obstacle
#: before committing to it, which keeps the swarm's aggregation intact.
OBSTACLE_TRIGGER_DELTA = 0.9


def _default_obstacles() -> list[Obstacle]:
    """Static circular obstacles placed around the 5 x 5 m demo world.

    Placement tuned empirically for the default seed 42 (M2.10): the plan's
    sketch placement ((1.5, 1.5, r=0.4), (3.5, 3.5, r=0.5), (2.5, 1.0, r=0.3))
    puts the seed-42 spawn at (2.725, 1.102) INSIDE the (2.5, 1.0) obstacle
    and splits the swarm into two non-aggregating clusters. These obstacles
    sit at the left/right mid-height and the top edge instead: they perturb
    search patrols (avoidance is exercised) without blocking aggregation.
    """
    return [
        Obstacle(Vector2(1.0, 2.5), 0.3),
        Obstacle(Vector2(4.2, 2.5), 0.3),
        Obstacle(Vector2(2.5, 4.2), 0.25),
    ]


class LaykaSimController:
    """Drives a ``layka`` World from the legacy GTK ``Viewer`` chrome.

    Exposes the callback interface ``gui.viewer.Viewer`` expects
    (``initialize_sim``, ``play_sim``, ``pause_sim``, ``step_sim_once``,
    ``reset_sim``, ``random_map``, ``draw_world``, ``save_map``,
    ``load_map``, ``end_sim``), so the existing window/buttons/painter are
    reused without any legacy simulation code.
    """

    def __init__(self) -> None:
        # create the GUI (reuses the legacy generic chrome)
        self.viewer = gui.viewer.Viewer(self)

        # timing control
        self.period = PERIOD

        # GTK simulation event source
        self.sim_event_source = GLib.idle_add(self.initialize_sim, True)

        # IR proximity sensors (ray-cast cone rendering)
        self._sensor_config = ProximitySensorConfig()

        # start the GTK main loop
        Gtk.main()

    # --- simulation lifecycle ---

    def initialize_sim(self, random: bool = False) -> None:
        """(Re)build the layka World, robots, and view.

        ``random=False`` keeps the current seed (so Reset reproduces the same
        world); ``random=True`` draws a fresh seed (Randomize gives a new
        initial configuration).
        """
        self.viewer.control_panel_state_init()
        if random:
            self._seed = _random.Random().randrange(0, 2**31 - 1)
        else:
            self._seed = getattr(self, "_seed", DEFAULT_SEED)

        config = SimulationConfig(
            timestep=self.period,
            robot_count=ROBOT_COUNT,
            random_seed=self._seed,
            world_width=WORLD_WIDTH,
            world_height=WORLD_HEIGHT,
        )
        self.world = World(config)
        controller_config = LJControllerConfig()
        search_config = SearchSwarmConfig()
        lj_config = LennardJonesConfig(desired_spacing=0.40)
        self._search_config = search_config
        for _ in range(ROBOT_COUNT):
            self.world.add_robot(
                behavior=self._make_behavior(
                    controller_config, search_config, lj_config
                )
            )
        for obstacle in _default_obstacles():
            self.world.add_obstacle(obstacle)

        self.trajectory = TrajectoryRecorder(max_points=200)
        self.trajectory.record(self.world)
        self.world_view = LaykaWorldView(
            self.world,
            self.viewer,
            debug=False,
            detection_range=self._search_config.detection_range,
            trail=self.trajectory,
        )
        self._fit_viewport()

        # render the initial world
        self.draw_world()

    def _fit_viewport(self) -> None:
        """Scale the painter so the whole world fits in the drawing area.

        The legacy painter uses a fixed 100 px/m zoom and maps the metric
        origin to the window center, so a world larger than half the view
        (or one offset from the origin) would be cut off. Recompute the zoom
        from the drawing-area size so every robot stays visible.
        """
        available_w = self.viewer.view_width_pixels
        available_h = self.viewer.view_height_pixels
        margin_px = 40.0
        zoom = min(
            (available_w - margin_px) / self.world.width,
            (available_h - margin_px) / self.world.height,
        )
        self.viewer.pixels_per_meter = zoom
        self.viewer.painter.pixels_per_meter = zoom

    def _make_behavior(
        self,
        controller_config: LJControllerConfig,
        search_config: SearchSwarmConfig,
        lj_config: LennardJonesConfig,
    ) -> BoundaryContainmentBehavior:
        """Fresh search+swarm behavior wrapped in obstacle avoidance (M2.10)
        and world-boundary containment.

        Priority (plan.md M2.10): collision safety > obstacle avoidance >
        swarm interaction, so the boundary wrapper (collision safety) is
        OUTERMOST, obstacle avoidance is next, and the swarm behavior is
        innermost. A fresh instance per robot: ``SearchSwarmBehavior`` keeps
        per-robot state (current state + patrol timer). The controller/config
        objects are stateless and shared.
        """
        inner = SearchSwarmBehavior.from_config(
            search_config=search_config,
            controller_config=controller_config,
            lj_config=lj_config,
        )
        avoidance = ObstacleAvoidanceBehavior(
            inner,
            LJController(controller_config),
            self._sensor_config,
            trigger_delta=OBSTACLE_TRIGGER_DELTA,
        )
        return BoundaryContainmentBehavior(
            avoidance,
            LJController(controller_config),
            self.world.width,
            self.world.height,
            margin=BOUNDARY_MARGIN,
        )

    # --- transport controls ---

    def play_sim(self) -> None:
        GLib.source_remove(self.sim_event_source)
        self._run_sim()
        self.viewer.control_panel_state_playing()

    def pause_sim(self) -> None:
        GLib.source_remove(self.sim_event_source)
        self.viewer.control_panel_state_paused()

    def step_sim_once(self) -> None:
        self.pause_sim()
        self._step_sim()

    def reset_sim(self) -> None:
        self.pause_sim()
        self.initialize_sim(random=False)

    def random_map(self) -> None:
        self.pause_sim()
        self.initialize_sim(random=True)

    def end_sim(self, alert_text: str = "") -> None:
        GLib.source_remove(self.sim_event_source)
        self.viewer.control_panel_state_finished(alert_text)

    # --- rendering ---

    def draw_world(self) -> None:
        self.viewer.new_frame()
        # The legacy "Show Invisibles" button doubles as the debug toggle.
        self.world_view.debug = self.viewer.show_invisibles
        # Fresh IR sensor readings so the red/green cones track the world.
        self.world_view.sensor_readings = compute_sensor_readings(
            self.world, self._sensor_config
        )
        self.world_view.draw_world_to_frame()
        self.viewer.draw_frame()

    def _run_sim(self) -> bool:
        self.sim_event_source = GLib.timeout_add(int(self.period * 1000), self._run_sim)
        self._step_sim()
        return False  # do not repeat automatically; the timeout re-registers

    def _step_sim(self) -> None:
        # advance exactly one explicit timestep, then record + redraw
        self.world.step()
        self.trajectory.record(self.world)
        self.draw_world()

    # --- maps (unsupported in the new World) ---

    def save_map(self, filename: str) -> None:
        self.viewer.alert_box.set_text(
            "Save Map is not available: the layka World has no map system yet."
        )

    def load_map(self, filename: str) -> None:
        self.viewer.alert_box.set_text(
            "Load Map is not available: the layka World has no map system yet."
        )


# RUN THE NEW SIMULATOR:
if __name__ == "__main__":
    LaykaSimController()

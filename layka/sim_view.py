"""GTK renderer glue for a :class:`layka.world.World` (simulator wiring).

The real drawing is a pure, headless-testable function,
:func:`build_frame_items`, which turns a ``World`` into a list of drawing
primitives in the exact schema used by the legacy ``gui.Frame`` renderer
(``{"type": "circle" | "lines" | "polygons", ...}``). This lets the new
simulator reuse the existing GTK window/buttons/cairo painter
(:mod:`gui.viewer`) without importing any legacy robot/world/physics code.

What gets drawn, in order (later items on top):

- a light-gray grid at ``grid_interval`` meter spacing;
- the world boundary rectangle;
- each static circular obstacle (M2.10) as a filled dark-gray circle
  (obstacles render BEFORE robots so robots draw on top);
- each robot's recent trajectory (when a :class:`TrajectoryRecorder` is
  supplied) as a steel-blue polyline;
- neighbor links (only in debug mode, via
  :class:`layka.neighbors.NeighborSensor`) as green segments;
- each robot's IR proximity-sensor cones (when ``sensor_readings`` from
  :func:`layka.proximity_sensor.compute_sensor_readings` is supplied): a
  sensor cone is GREEN when it detects another robot and RED when it detects
  an obstacle, fading from nearly opaque to nearly transparent with distance
  (mirroring the legacy ``ProximitySensorView``); idle sensors draw a faint
  red cone;
- each robot as a blue circle with a red heading line.

Debug mode toggles neighbor links (and, if a trajectory recorder is present,
trail polylines). ``detection_range`` is required for neighbor links and is
validated to be a finite positive number when provided.

:class:`LaykaWorldView` is the thin glue that pushes the primitives produced
by :func:`build_frame_items` into a legacy ``Viewer``'s current frame; it is
itself headless-importable (it only calls methods on the duck-typed viewer).
The GTK entry point (:mod:`layka.sim`) imports ``gi``, not this module.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from layka.neighbors import NeighborSensor
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.proximity_sensor import SensorReading
    from layka.renderer import TrajectoryRecorder
    from layka.world import World

#: Drawing constants (meters).
DEFAULT_GRID_INTERVAL = 1.0
DEFAULT_ROBOT_RADIUS = 0.06
DEFAULT_HEADING_LENGTH = 0.12

GRID_COLOR = "lightgray"
BOUNDARY_COLOR = "dimgray"
OBSTACLE_COLOR = "darkgray"
ROBOT_COLOR = "royal blue"
HEADING_COLOR = "red"
TRAIL_COLOR = "steel blue"
LINK_COLOR = "green"
SENSOR_ROBOT_COLOR = "green"
SENSOR_OBSTACLE_COLOR = "red"

GRID_LINEWIDTH = 0.005
BOUNDARY_LINEWIDTH = 0.02
ROBOT_LINEWIDTH = 0.015
TRAIL_LINEWIDTH = 0.008
LINK_LINEWIDTH = 0.01

#: Alpha for a sensor cone that detects nothing (idle, faint).
SENSOR_IDLE_ALPHA = 0.1
#: Alpha when the sensor detects something right at the minimum range
#: (``target_delta == min_range / max_range``).
SENSOR_NEAR_ALPHA = 0.9
#: Alpha when the detected object sits right at ``max_range`` (farthest hit).
SENSOR_FAR_ALPHA = 0.1


def _validate_positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number, got {value!r}")


def _sensor_cone(
    reading: SensorReading,
    offset_x: float,
    offset_y: float,
    max_range: float,
    fov: float,
) -> list[list[float]]:
    """Build a sensor cone polygon in world (offset) coordinates.

    The cone is a wedge from the sensor origin along its global heading, of
    length ``max_range`` and total angular width ``fov`` (mirroring the legacy
    ``ProximitySensorView`` cone).
    """
    cos_t = math.cos(reading.pose.theta)
    sin_t = math.sin(reading.pose.theta)
    half = fov / 2.0

    def _point(radius: float, angle: float) -> list[float]:
        x = radius * math.cos(angle) * cos_t - radius * math.sin(angle) * sin_t
        y = radius * math.cos(angle) * sin_t + radius * math.sin(angle) * cos_t
        return [reading.pose.x + x - offset_x, reading.pose.y + y - offset_y]

    return [
        [reading.pose.x - offset_x, reading.pose.y - offset_y],
        _point(max_range, -half),
        [reading.pose.x + max_range * cos_t - offset_x,
         reading.pose.y + max_range * sin_t - offset_y],
        _point(max_range, half),
    ]


def build_frame_items(
    world: World,
    *,
    debug: bool = False,
    detection_range: float | None = None,
    robot_radius: float = DEFAULT_ROBOT_RADIUS,
    heading_length: float = DEFAULT_HEADING_LENGTH,
    grid_interval: float = DEFAULT_GRID_INTERVAL,
    trail: TrajectoryRecorder | None = None,
    offset: tuple[float, float] = (0.0, 0.0),
    obstacle_color: str = OBSTACLE_COLOR,
    sensor_readings: dict[int, list[SensorReading]] | None = None,
    sensor_max_range: float = 0.2,
    sensor_fov: float = 0.6981317007977318,  # 40 degrees
) -> list[dict]:
    """Render a ``World`` into ``gui.Frame``-compatible drawing primitives.

    Pure and non-mutating: reads robot poses and the neighbor graph only. The
    returned list contains plain dicts with the keys the legacy ``Frame`` and
    ``Painter`` expect (``type``/``pos``/``radius``/``color``/``alpha`` for
    circles, ``type``/``lines``/``linewidth``/``color``/``alpha`` for lines,
    ``type``/``polygons``/``color``/``alpha`` for polygons).

    ``sensor_readings`` (per-robot list of :class:`SensorReading`) adds the IR
    proximity-sensor cones; see the module docstring for the color/alpha rule.
    Cones render below the robots so the robot bodies stay visible.

    ``offset`` is subtracted from every emitted coordinate. The legacy painter
    maps the metric origin ``(0, 0)`` to the center of the window, while a
    ``World`` keeps its robots in the positive quadrant ``[0, width] x
    [0, height]``; passing ``offset=(width/2, height/2)`` re-centers the world
    in the window instead of drawing it in the top-right quadrant.
    """
    _validate_positive("robot_radius", robot_radius)
    _validate_positive("heading_length", heading_length)
    _validate_positive("grid_interval", grid_interval)
    if len(offset) != 2 or not all(math.isfinite(v) for v in offset):
        raise ValueError(f"offset must be two finite numbers, got {offset!r}")
    if detection_range is not None and (
        not math.isfinite(detection_range) or detection_range <= 0
    ):
        raise ValueError(
            "detection_range must be a finite positive number, "
            f"got {detection_range!r}"
        )

    offset_x, offset_y = offset

    def _p(x: float, y: float) -> list[float]:
        return [x - offset_x, y - offset_y]

    width = world.width
    height = world.height
    items: list[dict] = []

    grid_lines: list[list[list[float]]] = []
    x = 0.0
    while x <= width + 1e-12:
        grid_lines.append([_p(x, 0.0), _p(x, height)])
        x += grid_interval
    y = 0.0
    while y <= height + 1e-12:
        grid_lines.append([_p(0.0, y), _p(width, y)])
        y += grid_interval
    if grid_lines:
        items.append(
            {
                "type": "lines",
                "lines": grid_lines,
                "linewidth": GRID_LINEWIDTH,
                "color": GRID_COLOR,
                "alpha": None,
            }
        )

    boundary = [
        [_p(0.0, 0.0), _p(width, 0.0)],
        [_p(width, 0.0), _p(width, height)],
        [_p(width, height), _p(0.0, height)],
        [_p(0.0, height), _p(0.0, 0.0)],
    ]
    items.append(
        {
            "type": "lines",
            "lines": boundary,
            "linewidth": BOUNDARY_LINEWIDTH,
            "color": BOUNDARY_COLOR,
            "alpha": None,
        }
    )

    for obstacle in world.obstacles:
        items.append(
            {
                "type": "circle",
                "pos": _p(obstacle.center.x, obstacle.center.y),
                "radius": obstacle.radius,
                "color": obstacle_color,
                "alpha": None,
            }
        )

    if trail is not None:
        trail_lines: list[list[list[float]]] = []
        for robot in world.robots:
            points = trail.positions(robot.robot_id)
            if len(points) >= 2:
                trail_lines.append([_p(p.x, p.y) for p in points])
        if trail_lines:
            items.append(
                {
                    "type": "lines",
                    "lines": trail_lines,
                    "linewidth": TRAIL_LINEWIDTH,
                    "color": TRAIL_COLOR,
                    "alpha": 0.6,
                }
            )

    if debug and detection_range is not None:
        sensor = NeighborSensor(world, detection_range)
        link_lines: list[list[list[float]]] = []
        for robot in world.robots:
            rid = robot.robot_id
            for neighbor in sensor.neighbors_of(rid):
                if neighbor.neighbor_id > rid:
                    other = world.robot_by_id(neighbor.neighbor_id)
                    link_lines.append(
                        [
                            _p(robot.pose.x, robot.pose.y),
                            _p(other.pose.x, other.pose.y),
                        ]
                    )
        if link_lines:
            items.append(
                {
                    "type": "lines",
                    "lines": link_lines,
                    "linewidth": LINK_LINEWIDTH,
                    "color": LINK_COLOR,
                    "alpha": 0.7,
                }
            )

    if sensor_readings:
        sensor_cones: list[dict] = []
        for readings in sensor_readings.values():
            for reading in readings:
                cone = _sensor_cone(
                    reading, offset_x, offset_y, sensor_max_range, sensor_fov
                )
                if reading.target_delta is None:
                    color = SENSOR_OBSTACLE_COLOR
                    alpha = SENSOR_IDLE_ALPHA
                else:
                    if reading.detecting_robot:
                        color = SENSOR_ROBOT_COLOR
                    else:
                        color = SENSOR_OBSTACLE_COLOR
                    alpha = SENSOR_NEAR_ALPHA - (
                        SENSOR_NEAR_ALPHA - SENSOR_FAR_ALPHA
                    ) * reading.target_delta
                sensor_cones.append(
                    {
                        "type": "polygons",
                        "polygons": [cone],
                        "color": color,
                        "alpha": alpha,
                    }
                )
        items.extend(sensor_cones)

    for robot in world.robots:
        pos = _p(robot.pose.x, robot.pose.y)
        items.append(
            {
                "type": "circle",
                "pos": pos,
                "radius": robot_radius,
                "color": ROBOT_COLOR,
                "alpha": None,
            }
        )
        heading_end = _p(
            robot.pose.x + heading_length * math.cos(robot.pose.theta),
            robot.pose.y + heading_length * math.sin(robot.pose.theta),
        )
        items.append(
            {
                "type": "lines",
                "lines": [[pos, heading_end]],
                "linewidth": ROBOT_LINEWIDTH,
                "color": HEADING_COLOR,
                "alpha": None,
            }
        )

    return items


class LaykaWorldView:
    """Pushes :func:`build_frame_items` output into a legacy ``Viewer`` frame.

    Wraps a ``World`` and a duck-typed viewer (any object exposing
    ``current_frame`` with ``add_circle``/``add_lines``/``add_polygons``).
    ``debug`` toggles neighbor links and trails; ``sensor_readings`` adds the
    IR proximity-sensor cones; ``draw_world_to_frame`` must be called between
    the viewer's ``new_frame()`` and ``draw_frame()`` calls.

    With ``centered=True`` (default) the world is shifted so its center lands
    on the painter's origin (the window center) rather than drawing the
    positive-quadrant world in the top-right corner.
    """

    __slots__ = (
        "_world",
        "_viewer",
        "_debug",
        "_detection_range",
        "_robot_radius",
        "_heading_length",
        "_trail",
        "_centered",
        "_sensor_readings",
        "_sensor_max_range",
        "_sensor_fov",
    )

    def __init__(
        self,
        world: World,
        viewer,
        *,
        debug: bool = False,
        detection_range: float | None = None,
        robot_radius: float = DEFAULT_ROBOT_RADIUS,
        heading_length: float = DEFAULT_HEADING_LENGTH,
        trail: TrajectoryRecorder | None = None,
        centered: bool = True,
        sensor_readings: dict[int, list[SensorReading]] | None = None,
        sensor_max_range: float = 0.2,
        sensor_fov: float = 0.6981317007977318,
    ) -> None:
        self._world = world
        self._viewer = viewer
        self._debug = debug
        self._detection_range = detection_range
        self._robot_radius = robot_radius
        self._heading_length = heading_length
        self._trail = trail
        self._centered = centered
        self._sensor_readings = sensor_readings
        self._sensor_max_range = sensor_max_range
        self._sensor_fov = sensor_fov

    @property
    def world(self) -> World:
        """The wrapped world (read-only)."""
        return self._world

    @property
    def debug(self) -> bool:
        """Whether neighbor links / trails are shown (toggle with the setter)."""
        return self._debug

    @debug.setter
    def debug(self, value: bool) -> None:
        self._debug = bool(value)

    @property
    def detection_range(self) -> float | None:
        """Detection radius used for neighbor-link rendering."""
        return self._detection_range

    @property
    def trail(self) -> TrajectoryRecorder | None:
        """Trajectory recorder used for trail rendering (read-only)."""
        return self._trail

    @property
    def centered(self) -> bool:
        """Whether the world is re-centered on the painter origin."""
        return self._centered

    @property
    def sensor_readings(self) -> dict[int, list[SensorReading]] | None:
        """Per-robot IR sensor readings to render (settable)."""
        return self._sensor_readings

    @sensor_readings.setter
    def sensor_readings(self, value: dict[int, list[SensorReading]] | None) -> None:
        self._sensor_readings = value

    def draw_world_to_frame(self) -> None:
        """Add all primitives for the current world state to the viewer frame."""
        frame = self._viewer.current_frame
        if self._centered:
            offset = (self._world.width / 2.0, self._world.height / 2.0)
        else:
            offset = (0.0, 0.0)
        for item in build_frame_items(
            self._world,
            debug=self._debug,
            detection_range=self._detection_range,
            robot_radius=self._robot_radius,
            heading_length=self._heading_length,
            trail=self._trail,
            offset=offset,
            sensor_readings=self._sensor_readings,
            sensor_max_range=self._sensor_max_range,
            sensor_fov=self._sensor_fov,
        ):
            if item["type"] == "circle":
                frame.add_circle(
                    item["pos"], item["radius"], item["color"], item["alpha"]
                )
            elif item["type"] == "lines":
                frame.add_lines(
                    item["lines"],
                    item["linewidth"],
                    item["color"],
                    item["alpha"],
                )
            elif item["type"] == "polygons":
                frame.add_polygons(
                    item["polygons"], item["color"], item["alpha"]
                )


__all__ = [
    "LaykaWorldView",
    "build_frame_items",
]

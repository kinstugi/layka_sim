"""Minimal text-based debug visualization (M1.8).

This is intentionally NOT a GUI: it renders a :class:`layka.world.World` to
deterministic, human-readable text so a developer can visually answer "where
is each robot, which way is it facing, and which robots are neighbors?" The
rendered output is a pure, non-mutating read of the world, so it is safe to
call between simulation steps and produces identical output for an identical
world state. Richer force/overlay visualization is deferred to M2.13.

Debug mode toggle
    ``debug=False`` (default) shows the header, an ASCII grid of robot
    positions/headings, and a per-robot legend. ``debug=True`` additionally
    shows neighbor links (on the grid and in the text report) and, when a
    :class:`TrajectoryRecorder` is supplied, each robot's recent trajectory.
    Neighbor links are computed with :class:`layka.neighbors.NeighborSensor`
    from an optional ``detection_range``; links are only shown in debug mode.

Grid orientation
    The world is projected onto a ``grid_height`` x ``grid_width`` character
    grid. Row 0 is the TOP of the grid and maps to the world's maximum y
    (``world.height``); the last row maps to ``y = 0``. Columns map left to
    right from ``x = 0`` to ``world.width``. Each cell represents the world
    region around its center point. A robot outside the world bounds is
    clamped to the nearest edge cell. A cell holding exactly one robot shows
    that robot's heading arrow; a cell holding two or more robots shows ``#``
    (the text report is the authoritative disambiguation).

Heading-arrow octant mapping
    The heading angle (radians, math convention: x right, y up,
    counter-clockwise positive) is mapped to one of 8 compass arrows by
    rounding ``theta / (pi/4)`` to the nearest octant::

        0 -> right (→)       pi/2 -> up (↑)       pi -> left (←)
        -pi/2 -> down (↓)    pi/4 -> up-right (↗)  etc.

    So ``theta`` in [-pi/8, pi/8] maps to ``→``, ``(pi/8, 3pi/8]`` to ``↗``,
    ``(3pi/8, 5pi/8]`` to ``↑``, and so on around the circle. Angles exactly
    on an octant boundary (an odd multiple of ``pi/8``) round to the even
    (cardinal) octant via Python's banker's rounding; they never map to a
    distant octant.

Neighbor links on the grid
    When debug mode is on and ``detection_range`` is set, every neighbor pair
    whose robot cells share a row or column has the cells strictly between the
    two robots filled with ``-`` (horizontal) or ``|`` (vertical) link
    characters (``+`` where a horizontal and vertical link cross). Links are
    skipped if another robot's cell lies between the pair, so a link never
    overwrites a robot marker. The text report is the source of truth; the
    grid is a visual aid.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from layka.neighbors import Neighbor, NeighborSensor
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.robot import RobotState
    from layka.world import World

#: Compass arrows indexed by octant: 0=right, 1=up-right, 2=up, 3=up-left,
#: 4=left, 5=down-left, 6=down, 7=down-right.
ARROWS: tuple[str, ...] = ("→", "↗", "↑", "↖", "←", "↙", "↓", "↘")


def heading_arrow(theta: float) -> str:
    """Map a heading angle (radians, math convention) to a compass arrow.

    The circle is divided into eight octants of width ``pi/4`` centered on the
    cardinal/diagonal directions; the nearest octant is chosen (see the module
    docstring for the exact boundary rule). Raises ``ValueError`` for a
    non-finite angle.
    """
    if not math.isfinite(theta):
        raise ValueError(f"theta must be a finite number, got {theta!r}")
    octant = int(round(theta / (math.pi / 4.0))) % 8
    return ARROWS[octant]


class TrajectoryRecorder:
    """Rolling per-robot position history for debug trajectory display.

    ``record(world)`` appends each robot's current position to a per-robot
    buffer kept to the last ``max_points`` entries; it is a read-only
    operation on the world (never mutates robots, poses, or the clock).
    ``positions(robot_id)`` returns a copy of the buffered positions in
    chronological order (oldest first), or ``[]`` for an unknown/unrecorded
    robot id.
    """

    __slots__ = ("_max_points", "_buffers")

    def __init__(self, max_points: int = 10) -> None:
        if isinstance(max_points, bool) or not isinstance(max_points, int):
            raise ValueError(f"max_points must be an int, got {max_points!r}")
        if max_points <= 0:
            raise ValueError(f"max_points must be > 0, got {max_points!r}")
        self._max_points = max_points
        self._buffers: dict[int, list[Vector2]] = {}

    @property
    def max_points(self) -> int:
        """Maximum number of positions kept per robot."""
        return self._max_points

    def record(self, world: World) -> None:
        """Append each robot's current position to its rolling buffer."""
        for robot in world.robots:
            buffer = self._buffers.setdefault(robot.robot_id, [])
            buffer.append(robot.pose.position())
            if len(buffer) > self._max_points:
                del buffer[: len(buffer) - self._max_points]

    def positions(self, robot_id: int) -> list[Vector2]:
        """Last recorded positions for ``robot_id``, oldest first (a copy)."""
        return list(self._buffers.get(robot_id, []))

    def clear(self) -> None:
        """Drop all recorded positions."""
        self._buffers.clear()


def _validate_grid_dimensions(grid_width: int, grid_height: int) -> None:
    for name, value in (("grid_width", grid_width), ("grid_height", grid_height)):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} must be an int, got {value!r}")
        if value <= 0:
            raise ValueError(f"{name} must be > 0, got {value!r}")


def _robot_line(robot: RobotState) -> str:
    theta_deg = math.degrees(robot.pose.theta)
    return (
        f"R{robot.robot_id}: pos=({robot.pose.x:.2f}, {robot.pose.y:.2f}) "
        f"heading={robot.pose.theta:.2f} rad ({theta_deg:.1f} deg) "
        f"{heading_arrow(robot.pose.theta)}"
    )


def _neighbor_line(robot_id: int, neighbors: list[Neighbor]) -> str:
    if not neighbors:
        return f"R{robot_id} neighbors: none"
    parts = [f"R{n.neighbor_id} (d={n.distance:.3f} m)" for n in neighbors]
    return f"R{robot_id} neighbors: " + ", ".join(parts)


def _trajectory_line(robot_id: int, points: list[Vector2]) -> str:
    if not points:
        return f"R{robot_id} trajectory: (no points yet)"
    rendered = " -> ".join(f"({p.x:.2f}, {p.y:.2f})" for p in points)
    return f"R{robot_id} trajectory: {rendered}"


class DebugRenderer:
    """Renders a :class:`layka.world.World` to deterministic text (no GUI).

    Pure read: neither :meth:`render` nor :meth:`report` mutates the world,
    any robot, or the clock. ``debug`` toggles neighbor links and trajectory
    output. ``detection_range`` (required for neighbor links) is validated to
    be a finite positive number when provided; grid dimensions and
    ``trajectory_points`` are validated as positive/zero ints.
    """

    __slots__ = (
        "_world",
        "_debug",
        "_grid_width",
        "_grid_height",
        "_trajectory_points",
        "_trajectory",
        "_sensor",
    )

    def __init__(
        self,
        world: World,
        *,
        debug: bool = False,
        grid_width: int = 60,
        grid_height: int = 24,
        trajectory_points: int = 0,
        detection_range: float | None = None,
        trajectory: TrajectoryRecorder | None = None,
    ) -> None:
        _validate_grid_dimensions(grid_width, grid_height)
        if isinstance(trajectory_points, bool) or not isinstance(
            trajectory_points, int
        ):
            raise ValueError(
                f"trajectory_points must be an int, got {trajectory_points!r}"
            )
        if trajectory_points < 0:
            raise ValueError(
                f"trajectory_points must be >= 0, got {trajectory_points!r}"
            )
        if detection_range is not None and (
            not math.isfinite(detection_range) or detection_range <= 0
        ):
            raise ValueError(
                f"detection_range must be a finite positive number, "
                f"got {detection_range!r}"
            )
        self._world = world
        self._debug = debug
        self._grid_width = grid_width
        self._grid_height = grid_height
        self._trajectory_points = trajectory_points
        self._trajectory = trajectory
        self._sensor: NeighborSensor | None = (
            NeighborSensor(world, detection_range)
            if debug and detection_range is not None
            else None
        )

    # --- public API ---

    def render(self) -> str:
        """Header + ASCII grid + per-robot legend, as one string."""
        lines = [self._header("renderer"), self._border()]
        for row in self._grid_rows():
            lines.append("|" + row + "|")
        lines.append(self._border())
        lines.append("")
        lines.append("Robot legend (grid marker = heading arrow):")
        lines.extend(_robot_line(robot) for robot in self._world.robots)
        return "\n".join(lines)

    def report(self) -> str:
        """Authoritative structured text report, as one string.

        Always lists every robot's id, position, heading (radians and
        degrees) and heading arrow. In debug mode it additionally lists each
        robot's neighbors (id and exact distance) and, when a trajectory
        recorder was provided and ``trajectory_points > 0``, the last
        recorded positions.
        """
        lines = [self._header("report")]
        lines.extend(_robot_line(robot) for robot in self._world.robots)
        if self._sensor is not None:
            lines.append("")
            lines.append(
                f"-- debug: neighbor links "
                f"(range={self._sensor.detection_range:.2f} m) --"
            )
            for robot in self._world.robots:
                lines.append(
                    _neighbor_line(
                        robot.robot_id, self._sensor.neighbors_of(robot.robot_id)
                    )
                )
        if (
            self._debug
            and self._trajectory is not None
            and self._trajectory_points > 0
        ):
            lines.append("")
            lines.append(
                f"-- debug: trajectories (last {self._trajectory_points} points) --"
            )
            for robot in self._world.robots:
                points = self._trajectory.positions(robot.robot_id)
                points = points[-self._trajectory_points :]
                lines.append(_trajectory_line(robot.robot_id, points))
        return "\n".join(lines)

    # --- internals ---

    def _header(self, kind: str) -> str:
        debug = "ON" if self._debug else "OFF"
        return (
            f"=== Layka debug {kind}: world {self._world.width:.2f} x "
            f"{self._world.height:.2f} m | {self._world.robot_count} robots | "
            f"t={self._world.time:.2f} s | step={self._world.step_count} | "
            f"debug {debug} ==="
        )

    def _border(self) -> str:
        return "+" + "-" * self._grid_width + "+"

    def _cell_of(self, x: float, y: float) -> tuple[int, int]:
        """Project world coordinates onto a grid cell ``(row, col)``.

        Row 0 is the top of the grid and maps to ``y = height``; the last row
        maps to ``y = 0``. A robot outside the world bounds is clamped to the
        nearest edge cell.
        """
        col = int(x / self._world.width * self._grid_width)
        row = int((1.0 - y / self._world.height) * self._grid_height)
        return (
            max(0, min(row, self._grid_height - 1)),
            max(0, min(col, self._grid_width - 1)),
        )

    def _grid_rows(self) -> list[str]:
        grid: list[list[str]] = [[" "] * self._grid_width for _ in range(self._grid_height)]
        cell_robots: dict[tuple[int, int], list[RobotState]] = {}
        for robot in self._world.robots:
            cell = self._cell_of(robot.pose.x, robot.pose.y)
            cell_robots.setdefault(cell, []).append(robot)
        for cell, robots in cell_robots.items():
            row, col = cell
            if len(robots) == 1:
                grid[row][col] = heading_arrow(robots[0].pose.theta)
            else:
                grid[row][col] = "#"
        if self._sensor is not None:
            for a, b in self._neighbor_pairs():
                robot_a = self._world.robot_by_id(a)
                robot_b = self._world.robot_by_id(b)
                self._draw_link(
                    grid,
                    self._cell_of(robot_a.pose.x, robot_a.pose.y),
                    self._cell_of(robot_b.pose.x, robot_b.pose.y),
                    cell_robots,
                )
        return ["".join(row) for row in grid]

    def _neighbor_pairs(self) -> list[tuple[int, int]]:
        """All unordered neighbor pairs ``(a, b)`` with ``a < b``, ascending."""
        sensor = self._sensor
        if sensor is None:
            return []
        pairs: list[tuple[int, int]] = []
        for robot in self._world.robots:
            rid = robot.robot_id
            for neighbor in sensor.neighbors_of(rid):
                if neighbor.neighbor_id > rid:
                    pairs.append((rid, neighbor.neighbor_id))
        return pairs

    def _draw_link(
        self,
        grid: list[list[str]],
        cell_a: tuple[int, int],
        cell_b: tuple[int, int],
        cell_robots: dict[tuple[int, int], list[RobotState]],
    ) -> None:
        """Fill grid cells between two neighbor robots on a shared row/column.

        ``cell_a``/``cell_b`` are ``(row, col)`` grid cells. ``-`` fills
        horizontal links, ``|`` vertical links, ``+`` where a link crosses an
        existing perpendicular one. A link is skipped entirely if another
        robot's cell lies between the pair, and a robot marker is never
        overwritten.
        """
        row_a, col_a = cell_a
        row_b, col_b = cell_b
        if row_a == row_b and abs(col_b - col_a) > 1:
            lo, hi = sorted((col_a, col_b))
            for col in range(lo + 1, hi):
                if (row_a, col) in cell_robots:
                    return
                if grid[row_a][col] == " ":
                    grid[row_a][col] = "-"
                elif grid[row_a][col] == "|":
                    grid[row_a][col] = "+"
        elif col_a == col_b and abs(row_b - row_a) > 1:
            lo, hi = sorted((row_a, row_b))
            for row in range(lo + 1, hi):
                if (row, col_a) in cell_robots:
                    return
                if grid[row][col_a] == " ":
                    grid[row][col_a] = "|"
                elif grid[row][col_a] == "-":
                    grid[row][col_a] = "+"


__all__ = ["DebugRenderer", "TrajectoryRecorder", "heading_arrow"]

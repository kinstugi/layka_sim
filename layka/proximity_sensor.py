"""Infrared proximity sensors around a robot (ported from the legacy model).

This is a clean port of the legacy ``models/proximity_sensor.py`` +
``models/physics.py`` ray-casting behaviour into the new architecture:

- Each robot carries ``sensor_poses`` -- a ring of sensors placed at fixed
  poses *relative* to the robot (robot at the origin, facing +x). The legacy
  Layka uses 9 sensors around its outline.
- Each sensor casts a ray of length ``max_range`` along its (global) heading.
  The ray is tested against every static obstacle and every *other* robot
  (treated as a disc of ``robot_radius``). The closest intersection wins, and
  ``detecting_robot`` records whether that closest hit was another robot or an
  obstacle.
- The sensor model itself is PURE: :func:`compute_sensor_readings` reads a
  ``World`` and returns per-robot readings; it never mutates robots, the world
  or the clock, and it knows nothing about rendering. The red/green coloring
  lives in the renderer (:mod:`layka.sim_view`).

An obstacle is never treated as a robot and a sensor never detects its own
robot (M2.10 design rule). ``target_delta`` follows the legacy convention:
``distance / max_range``, clamped so a hit closer than ``min_range`` reads as
``min_range / max_range``; ``None`` means nothing was detected in range.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from pydantic import BaseModel, Field, field_validator, model_validator

from layka.pose import Pose2D
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.world import World

#: Legacy Layka sensor layout (relative [x, y, theta_radians]) -- 9 IR sensors
#: around the robot outline, facing outward.
DEFAULT_SENSOR_POSES: tuple[tuple[float, float, float], ...] = (
    (-0.038, 0.048, 2.356194490192345),  # 135 deg
    (0.019, 0.064, 1.5707963267948966),  # 90 deg
    (0.050, 0.050, 0.7853981633974483),  # 45 deg
    (0.070, 0.000, 0.0),  # 0 deg
    (0.070, -0.017, -0.0),  # ~0 deg
    (0.050, -0.050, -0.7853981633974483),  # -45 deg
    (0.019, -0.064, -1.5707963267948966),  # -90 deg
    (-0.038, -0.048, -2.356194490192345),  # -135 deg
    (-0.048, 0.000, 3.141592653589793),  # 180 deg
)


class ProximitySensorConfig(BaseModel):
    """Validated configuration for a robot's ring of IR proximity sensors."""

    sensor_poses: list[tuple[float, float, float]] = Field(
        default_factory=lambda: list(DEFAULT_SENSOR_POSES),
        description="Relative sensor placements (x, y, theta in radians) "
        "with the robot at the origin facing +x.",
    )
    min_range: float = Field(
        default=0.02, gt=0, description="Sensors saturate below this distance (m)."
    )
    max_range: float = Field(
        default=0.2, gt=0, description="Maximum sensing distance (m)."
    )
    fov: float = Field(
        default=0.6981317007977318,  # 40 degrees
        gt=0,
        description="Sensor cone angle (radians); used for the visual cone only.",
    )
    robot_radius: float = Field(
        default=0.07, gt=0, description="Disc radius used to detect other robots (m)."
    )

    @field_validator(
        "min_range", "max_range", "fov", "robot_radius"
    )
    @classmethod
    def _finite_positive(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("must be a finite number")
        return value

    @model_validator(mode="after")
    def _sensible_ranges(self) -> ProximitySensorConfig:
        if self.max_range <= self.min_range:
            raise ValueError(
                "max_range must be > min_range, got "
                f"{self.max_range!r} <= {self.min_range!r}"
            )
        return self


@dataclass(frozen=True, slots=True)
class SensorReading:
    """One sensor's reading at a given simulation instant."""

    sensor_index: int
    pose: Pose2D  # global pose of the sensor in the world
    target_delta: float | None  # distance / max_range in [0, 1], or None
    distance: float | None  # closest detected distance along the ray (m), or None
    detecting_robot: bool  # True if the closest hit was another robot


def _ray_circle_hit(
    origin: Vector2,
    direction: Vector2,
    center: Vector2,
    radius: float,
    max_range: float,
) -> float | None:
    """Distance ``t`` in ``[0, max_range]`` to the first hit of a ray vs disc.

    ``direction`` must be a unit vector. Returns ``None`` if the ray misses the
    disc entirely or the first hit lies beyond ``max_range``.
    """
    f = origin - center
    b = 2.0 * f.dot(direction)
    c = f.dot(f) - radius * radius
    if c < 0.0:
        return 0.0  # ray origin is already inside the disc -> immediate hit
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    sqrt_disc = math.sqrt(disc)
    t = (-b - sqrt_disc) / 2.0
    if t < 0.0:
        t = (-b + sqrt_disc) / 2.0
    if t < 0.0:
        return None  # origin beyond the disc, both intersections behind
    if t > max_range:
        return None
    return t


def _sensor_global_pose(
    robot_pose: Pose2D, relative: tuple[float, float, float]
) -> Pose2D:
    """Global pose of a sensor given the robot pose and a relative placement."""
    rel_pos = Vector2(relative[0], relative[1]).rotate(robot_pose.theta)
    return Pose2D(
        x=robot_pose.x + rel_pos.x,
        y=robot_pose.y + rel_pos.y,
        theta=robot_pose.theta + relative[2],
    )


def compute_sensor_readings(
    world: World, config: ProximitySensorConfig
) -> dict[int, list[SensorReading]]:
    """Ray-cast every sensor of every robot and return per-robot readings.

    For each robot, each sensor's ray is tested against all static obstacles
    (circle radius = obstacle radius) and all *other* robots (disc radius =
    ``config.robot_radius``). The closest hit wins and determines whether the
    sensor is detecting a robot (green) or an obstacle (red). A sensor never
    detects its own robot. Pure read: does not mutate the world or robots.
    """
    readings: dict[int, list[SensorReading]] = {}
    for robot in world.robots:
        robot_readings: list[SensorReading] = []
        for index, relative in enumerate(config.sensor_poses):
            sensor_pose = _sensor_global_pose(robot.pose, relative)
            direction = Vector2(
                math.cos(sensor_pose.theta), math.sin(sensor_pose.theta)
            )
            origin = sensor_pose.position()

            best_t: float | None = None
            detecting_robot = False

            for obstacle in world.obstacles:
                t = _ray_circle_hit(
                    origin, direction, obstacle.center, obstacle.radius, config.max_range
                )
                if t is not None and (best_t is None or t < best_t):
                    best_t = t
                    detecting_robot = False

            for other in world.robots:
                if other.robot_id == robot.robot_id:
                    continue
                t = _ray_circle_hit(
                    origin,
                    direction,
                    other.pose.position(),
                    config.robot_radius,
                    config.max_range,
                )
                if t is not None and (best_t is None or t < best_t):
                    best_t = t
                    detecting_robot = True

            if best_t is not None:
                if best_t <= config.min_range:
                    target_delta = config.min_range / config.max_range
                else:
                    target_delta = best_t / config.max_range
                robot_readings.append(
                    SensorReading(
                        sensor_index=index,
                        pose=sensor_pose,
                        target_delta=target_delta,
                        distance=best_t,
                        detecting_robot=detecting_robot,
                    )
                )
            else:
                robot_readings.append(
                    SensorReading(
                        sensor_index=index,
                        pose=sensor_pose,
                        target_delta=None,
                        distance=None,
                        detecting_robot=False,
                    )
                )
        readings[robot.robot_id] = robot_readings
    return readings


__all__ = [
    "ProximitySensorConfig",
    "SensorReading",
    "compute_sensor_readings",
]

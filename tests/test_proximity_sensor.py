"""Unit tests for the IR proximity sensors (ported from the legacy model)."""

from __future__ import annotations

import math

import pytest

from layka.obstacle import Obstacle
from layka.pose import Pose2D
from layka.proximity_sensor import (
    DEFAULT_SENSOR_POSES,
    ProximitySensorConfig,
    SensorReading,
    compute_sensor_readings,
)
from layka.sim_view import SENSOR_OBSTACLE_COLOR, SENSOR_ROBOT_COLOR, build_frame_items
from layka.vector import Vector2
from layka.world import World


def _config(**overrides) -> ProximitySensorConfig:
    return ProximitySensorConfig(**overrides)


def _single_robot(x: float = 0.0, y: float = 0.0, theta: float = 0.0):
    world = World(timestep=0.05)
    world.add_robot(pose=Pose2D(x, y, theta))
    return world


def _robot(world: World, x: float, y: float, theta: float = 0.0):
    robot_id = world.add_robot(pose=Pose2D(x, y, theta))
    return world.robot_by_id(robot_id)


class TestConfig:
    def test_defaults_match_legacy_layout(self):
        config = _config()
        assert len(config.sensor_poses) == 9
        assert config.sensor_poses == list(DEFAULT_SENSOR_POSES)
        assert config.min_range == 0.02
        assert config.max_range == 0.2
        assert math.isclose(config.fov, math.radians(40))

    def test_max_range_must_exceed_min_range(self):
        with pytest.raises(Exception):
            _config(min_range=0.3, max_range=0.2)

    def test_invalid_ranges(self):
        for bad in (0.0, -1.0, math.nan, math.inf):
            with pytest.raises(Exception):
                _config(max_range=bad)
            with pytest.raises(Exception):
                _config(robot_radius=bad)


class TestGlobalPose:
    def test_translation_only(self):
        world = _single_robot(x=1.0, y=2.0, theta=0.0)
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        readings = compute_sensor_readings(world, config)[0]
        sensor = readings[0].pose
        assert sensor.x == pytest.approx(1.07)
        assert sensor.y == pytest.approx(2.0)
        assert sensor.theta == pytest.approx(0.0)

    def test_rotation(self):
        world = _single_robot(x=0.0, y=0.0, theta=math.pi / 2.0)
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        readings = compute_sensor_readings(world, config)[0]
        sensor = readings[0].pose
        # sensor mounted +x on the robot, robot turned +90 deg -> sensor at +y
        assert sensor.x == pytest.approx(0.0)
        assert sensor.y == pytest.approx(0.07)
        assert sensor.theta == pytest.approx(math.pi / 2.0)


class TestReadings:
    def test_no_detection_when_empty(self):
        world = _single_robot()
        readings = compute_sensor_readings(world, _config())[0]
        assert len(readings) == 9
        for reading in readings:
            assert reading.target_delta is None
            assert reading.distance is None
            assert reading.detecting_robot is False

    def test_detects_obstacle_red(self):
        world = _single_robot()
        world.add_obstacle(Obstacle(center=Vector2(0.1, 0.0), radius=0.02))
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        reading = compute_sensor_readings(world, config)[0][0]
        assert reading.target_delta is not None
        assert reading.detecting_robot is False
        # front intersection at x=0.08, sensor origin at x=0.07
        assert reading.distance == pytest.approx(0.01)
        # inside min_range -> clamped to min_range/max_range
        assert reading.target_delta == pytest.approx(0.02 / 0.2)

    def test_detects_robot_green(self):
        world = _single_robot()
        _robot(world, 0.15, 0.0)
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        reading = compute_sensor_readings(world, config)[0][0]
        assert reading.target_delta is not None
        assert reading.detecting_robot is True
        # robot disc front at 0.15 - 0.07 = 0.08, sensor origin at 0.07
        assert reading.distance == pytest.approx(0.01)

    def test_closest_solid_wins_obstacle(self):
        world = _single_robot()
        world.add_obstacle(Obstacle(center=Vector2(0.1, 0.0), radius=0.02))
        _robot(world, 0.2, 0.0)
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        reading = compute_sensor_readings(world, config)[0][0]
        assert reading.detecting_robot is False  # obstacle closer (t~0.01 < t~0.06)

    def test_closest_solid_wins_robot(self):
        world = _single_robot()
        world.add_obstacle(Obstacle(center=Vector2(0.2, 0.0), radius=0.02))
        _robot(world, 0.12, 0.0)
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        reading = compute_sensor_readings(world, config)[0][0]
        assert reading.detecting_robot is True  # robot closer

    def test_does_not_detect_own_robot(self):
        # A single robot with a huge max range still sees nothing (skip-self).
        world = _single_robot()
        config = _config(max_range=10.0, sensor_poses=[(0.0, 0.0, 0.0)])
        readings = compute_sensor_readings(world, config)[0]
        assert readings[0].target_delta is None

    def test_misses_beside_obstacle(self):
        world = _single_robot()
        world.add_obstacle(Obstacle(center=Vector2(0.15, 0.05), radius=0.02))
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        reading = compute_sensor_readings(world, config)[0][0]
        assert reading.target_delta is None  # ray along +x passes below the disc

    def test_sensor_reading_fields(self):
        world = _single_robot()
        world.add_obstacle(Obstacle(center=Vector2(0.15, 0.0), radius=0.02))
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        reading = compute_sensor_readings(world, config)[0][0]
        assert reading.sensor_index == 0
        assert isinstance(reading.pose, Pose2D)
        assert reading.target_delta == pytest.approx((0.15 - 0.02 - 0.07) / 0.2)


class TestPurity:
    def test_deterministic_and_non_mutating(self):
        world = _single_robot()
        world.add_obstacle(Obstacle(center=Vector2(0.1, 0.0), radius=0.02))
        time_before = world.time
        a = compute_sensor_readings(world, _config())[0]
        b = compute_sensor_readings(world, _config())[0]
        assert a == b
        assert world.time == time_before
        assert world.robot_count == 1

    def test_robot_count_of_readings(self):
        world = World(timestep=0.05)
        _robot(world, 0.0, 0.0)
        _robot(world, 0.3, 0.0)
        readings = compute_sensor_readings(world, _config())
        assert set(readings.keys()) == {0, 1}
        assert len(readings[0]) == len(readings[1]) == 9


class TestRendering:
    def _cones(self, items: list[dict]) -> list[dict]:
        return [i for i in items if i["type"] == "polygons"]

    def test_idle_cones_are_faint_red(self):
        world = _single_robot()
        readings = compute_sensor_readings(world, _config())
        items = build_frame_items(world, sensor_readings=readings)
        cones = self._cones(items)
        assert len(cones) == 9
        for cone in cones:
            assert cone["color"] == SENSOR_OBSTACLE_COLOR
            assert cone["alpha"] == pytest.approx(0.1)

    def test_robot_detection_cone_is_green(self):
        world = _single_robot()
        _robot(world, 0.15, 0.0)
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        readings = compute_sensor_readings(world, config)
        items = build_frame_items(world, sensor_readings=readings, sensor_max_range=config.max_range, sensor_fov=config.fov)
        cones = self._cones(items)
        assert cones[0]["color"] == SENSOR_ROBOT_COLOR
        assert 0.1 < cones[0]["alpha"] <= 0.9

    def test_obstacle_detection_cone_is_red(self):
        world = _single_robot()
        world.add_obstacle(Obstacle(center=Vector2(0.1, 0.0), radius=0.02))
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        readings = compute_sensor_readings(world, config)
        items = build_frame_items(world, sensor_readings=readings, sensor_max_range=config.max_range, sensor_fov=config.fov)
        cones = self._cones(items)
        assert cones[0]["color"] == SENSOR_OBSTACLE_COLOR
        assert 0.1 < cones[0]["alpha"] <= 0.9

    def test_cone_apex_at_sensor_origin(self):
        world = _single_robot()
        world.add_obstacle(Obstacle(center=Vector2(0.1, 0.0), radius=0.02))
        config = _config(sensor_poses=[(0.07, 0.0, 0.0)])
        readings = compute_sensor_readings(world, config)
        items = build_frame_items(world, sensor_readings=readings)
        cone = self._cones(items)[0]["polygons"][0]
        assert cone[0] == [0.07, 0.0]  # apex at the sensor origin

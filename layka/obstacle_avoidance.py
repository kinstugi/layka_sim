"""IR-sensor-driven obstacle-avoidance behavior wrapper (M2.10, sensor port).

Priority (plan.md M2.10): collision safety > obstacle avoidance > swarm
interaction. :class:`ObstacleAvoidanceBehavior` implements the middle layer,
driven by the robot's ring of infrared proximity sensors -- mirroring how a
real differential-drive robot with IR sensors avoids obstacles.

How it works (per robot, per step), a clean port of the legacy controller's
``detect_obstacle`` + ``turn_to_avoid_obstacle`` logic:

1. Ray-cast the robot's IR sensors via
   :func:`layka.proximity_sensor.compute_sensor_readings` (the same readings
   the renderer draws as red/green cones).
2. Consider only the FORWARD hemisphere of sensors (relative angle within
   +/- 90 deg of the heading). Robot-detecting (GREEN) sensors are ignored --
   another robot is never treated as an obstacle; the swarm behavior handles
   robot-robot spacing via LJ repulsion. If no forward sensor detects an
   obstacle within ``trigger_delta * max_range``, the behavior delegates to
   the inner swarm behavior unchanged.
3. Otherwise, steer toward the forward sensor with the MOST CLEARANCE (the
   largest distance: an idle sensor counts as ``max_range``, a blocked sensor
   counts as its hit distance). This turns the robot toward the open
   direction -- around the obstacle, not into it -- instead of pushing it in
   a coarse sensor direction (which makes robots bounce/scatter). The chosen
   direction is fed to the shared :class:`layka.lj_controller.LJController`
   (proportional heading error, bounded velocities), the same control math
   used for boundary containment; no new control code.

The inner behavior is NOT called while an override is active, so a wrapped
stateful behavior (e.g. search) pauses its patrol timing until clear.

Rule 5 separation: obstacles are WORLD-level static geometry, never robots --
they never enter the neighbor/LJ pipeline (:class:`NeighborSensor` iterates
only ``world.robots``) and never carry a behavior or velocity. This behavior
only READS world state via the pure sensor function; it never mutates the
robot, the world, or other robots (``World.step`` is the only mutation
driver).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from layka.kinematics import BodyVelocity
from layka.lj_controller import LJController
from layka.pose import normalize_angle
from layka.proximity_sensor import ProximitySensorConfig, compute_sensor_readings
from layka.vector import Vector2

if TYPE_CHECKING:
    from layka.behavior import Behavior
    from layka.robot import RobotState
    from layka.world import World

#: A summed avoidance vector with norm below this is treated as "no obstacle
#: in range": exact zeros plus the residue of canceling sensor contributions.
ZERO_AVOIDANCE_TOLERANCE = 1e-9


def _validate_trigger_delta(trigger_delta: float) -> None:
    if not math.isfinite(trigger_delta) or not (0.0 < trigger_delta <= 1.0):
        raise ValueError(
            "trigger_delta must be finite and in (0, 1], "
            f"got {trigger_delta!r}"
        )


class ObstacleAvoidanceBehavior:
    """``Behavior`` wrapper that steers robots away using their IR sensors.

    Wraps ``inner`` and delegates to it while no forward IR sensor detects an
    obstacle within ``trigger_delta * max_range``. When an obstacle-detecting
    sensor fires, the command is overridden with a turn toward the forward
    sensor with the most clearance (the open direction), computed by the
    supplied :class:`LJController`.

    Deterministic and side-effect free; reads world state only through the
    pure sensor function.
    """

    __slots__ = ("_inner", "_controller", "_sensor_config", "_trigger_delta")

    def __init__(
        self,
        inner: Behavior,
        controller: LJController,
        sensor_config: ProximitySensorConfig,
        *,
        trigger_delta: float = 0.9,
    ) -> None:
        """Wrap ``inner``.

        ``sensor_config`` is the ring of IR sensors used to detect obstacles;
        ``trigger_delta`` (in ``(0, 1]``) is the fraction of ``max_range`` at
        which an obstacle-detecting sensor activates avoidance. The default
        ``0.9`` (obstacle within 0.18 m of a sensor) makes the robot react
        before it commits to the obstacle, so it veers smoothly around it; a
        smaller value (e.g. the legacy 0.75 = 0.15 m danger distance) fires
        later and disrupts the swarm's aggregation.
        """
        if not isinstance(sensor_config, ProximitySensorConfig):
            raise TypeError(
                "sensor_config must be a ProximitySensorConfig, "
                f"got {type(sensor_config).__name__}"
            )
        _validate_trigger_delta(trigger_delta)
        self._inner = inner
        self._controller = controller
        self._sensor_config = sensor_config
        self._trigger_delta = trigger_delta

    @property
    def inner(self) -> Behavior:
        """The wrapped behavior (read-only)."""
        return self._inner

    @property
    def controller(self) -> LJController:
        """The controller used for the avoidance override (read-only)."""
        return self._controller

    @property
    def sensor_config(self) -> ProximitySensorConfig:
        """The IR sensor ring used for obstacle detection (read-only)."""
        return self._sensor_config

    @property
    def trigger_delta(self) -> float:
        """Fraction of ``max_range`` at which avoidance activates (read-only)."""
        return self._trigger_delta

    def _avoidance_vector(self, robot: RobotState, world: World) -> Vector2 | None:
        """Summed repulsive vector from obstacle-detecting IR sensors.

        Returns ``None`` when no FORWARD sensor detects an obstacle within
        ``trigger_delta``. Only RED (obstacle) readings contribute -- GREEN
        (robot) readings are ignored so another robot is never treated as an
        obstacle (the swarm behavior handles robot-robot spacing via LJ
        repulsion), and rear sensors are ignored so an obstacle behind the
        robot does not push it back into a limit cycle.

        Each blocked sensor contributes a vector from the detected obstacle
        SURFACE (the sensor's ray hit point) back toward the robot, weighted
        by urgency (``trigger_delta - target_delta``). Summing these gives a
        smooth, near-radial repulsion -- the same behaviour that made the
        analytic M2.10 avoidance aggregate cleanly -- but derived entirely
        from the sensor readings.
        """
        readings = compute_sensor_readings(
            world, self._sensor_config, robot_id=robot.robot_id
        ).get(robot.robot_id, [])
        robot_heading = robot.pose.theta
        total_x = 0.0
        total_y = 0.0
        for reading in readings:
            if reading.detecting_robot or reading.target_delta is None:
                continue  # a robot (swarm handles it) or nothing detected
            if reading.target_delta >= self._trigger_delta:
                continue  # obstacle too far to demand action
            relative = abs(normalize_angle(reading.pose.theta - robot_heading))
            if relative > math.pi / 2.0:
                continue  # rear hemisphere: the robot is moving away anyway
            weight = self._trigger_delta - reading.target_delta
            hit_x = reading.pose.x + math.cos(reading.pose.theta) * reading.distance
            hit_y = reading.pose.y + math.sin(reading.pose.theta) * reading.distance
            total_x += weight * (robot.pose.x - hit_x)
            total_y += weight * (robot.pose.y - hit_y)
        if math.hypot(total_x, total_y) < ZERO_AVOIDANCE_TOLERANCE:
            return None
        return Vector2(total_x, total_y)

    def compute_command(
        self, robot: RobotState, world: World, dt: float
    ) -> BodyVelocity:
        """Return the body velocity commanded for ``robot``.

        If a forward IR sensor detects an obstacle within
        ``trigger_delta * max_range``, steer away from the detected obstacle
        surface via the LJController; otherwise delegate to the inner
        behavior. The inner behavior is not invoked while an override is
        active.
        """
        avoidance = self._avoidance_vector(robot, world)
        if avoidance is not None:
            return self._controller.compute(avoidance, robot.pose.theta)
        return self._inner.compute_command(robot, world, dt)


__all__ = ["ObstacleAvoidanceBehavior"]

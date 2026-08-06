#!/usr/bin/env python3
"""Simulator adapters for the UN R171 scenario harness.

The harness (r171_stationary_target.py) is written against `SimAdapter`
and never imports `carla` itself. Everything simulator-specific — actor
spawning, ground-truth geometry, control application, camera frames —
lives behind this interface, so adding MORAI later is a matter of
implementing `MoraiAdapter` rather than rewriting the scenario logic.

Coordinate / sign conventions used across the interface:

    * `gap_m`      bumper-to-bumper LONGITUDINAL gap, projected onto the
                   ego's forward axis. Matches the semantics the
                   perception node publishes on /ACC/lead_vehicle_distance
                   (see DEBUG §22), so ground truth and perception are
                   directly comparable.
    * `lateral_offset_m`  target displacement from the lane centreline,
                   POSITIVE = to the right of the direction of travel.
    * `steer`      normalised [-1, 1], positive = right. Same as
                   /Car_1/cmd_steer.
"""

from __future__ import annotations

import abc
import math
import queue
import time
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Value types
# ---------------------------------------------------------------------------
@dataclass
class EgoState:
    """Ground-truth ego state at one instant."""
    x: float
    y: float
    yaw_rad: float
    speed: float          # [m/s], magnitude of the velocity vector
    accel: float          # [m/s^2], longitudinal (projected on forward axis)


@dataclass
class LaneError:
    """Ego deviation from the lane centreline, for the locked-lateral mode."""
    cross_track_m: float      # +ve = ego is right of centre
    heading_err_rad: float    # +ve = lane heads right of the ego's heading


@dataclass
class CollisionEvent:
    sim_time: float
    other_id: str
    impact_speed: float       # ego speed [m/s] at the moment of impact


@dataclass
class ScenarioGeometry:
    """What the adapter actually built, for the run record."""
    town: str
    spawn_index: int
    straight_runway_m: float
    placement_distance_m: float   # ego spawn -> target, bumper-to-bumper
    lane_width_m: float
    ego_blueprint: str
    target_blueprint: str


# ---------------------------------------------------------------------------
# Interface
# ---------------------------------------------------------------------------
class SimAdapter(abc.ABC):
    """One live simulator connection, reused across every scenario point."""

    # -- lifecycle ----------------------------------------------------------
    @abc.abstractmethod
    def connect(self) -> None:
        """Open the connection and load the map. Called once."""

    @abc.abstractmethod
    def spawn_ego(self) -> None:
        """Spawn the VUT plus its camera and collision sensor. Called once —
        the ego survives the whole matrix so the perception nodes see an
        unbroken camera stream."""

    @abc.abstractmethod
    def arm(self, placement_distance_m: float,
            lateral_offset_m: float) -> ScenarioGeometry:
        """Reset for one scenario point: teleport the ego back to the start
        line at rest, then place a stationary target `placement_distance_m`
        ahead (bumper-to-bumper) offset `lateral_offset_m` from the lane
        centre. Called once per run."""

    @abc.abstractmethod
    def disarm(self) -> None:
        """Tear down the per-run target. Ego stays alive."""

    @abc.abstractmethod
    def close(self) -> None:
        """Destroy everything and restore the simulator's original settings."""

    # -- per-tick -----------------------------------------------------------
    @abc.abstractmethod
    def wait_for_tick(self) -> float:
        """Block until the simulator advances. Returns sim time [s]."""

    @abc.abstractmethod
    def ego_state(self) -> EgoState: ...

    @abc.abstractmethod
    def gap_m(self) -> float:
        """Ground-truth bumper-to-bumper longitudinal gap to the target."""

    @abc.abstractmethod
    def lane_error(self) -> LaneError: ...

    @abc.abstractmethod
    def apply_control(self, throttle: float, brake: float,
                      steer: float) -> None: ...

    @abc.abstractmethod
    def force_speed(self, speed_mps: float) -> None:
        """Kinematically set the ego's velocity along its heading. Used to
        snap to the approach speed at the start of a run, bypassing the
        acceleration phase (the ASAM OpenSCENARIO 'init absolute speed'
        pattern)."""

    @abc.abstractmethod
    def poll_camera(self) -> bytes | None:
        """Latest camera frame as JPEG bytes, or None if no new frame."""

    @abc.abstractmethod
    def take_collision(self) -> CollisionEvent | None:
        """Pop the first collision recorded since `arm`, if any."""

    @abc.abstractmethod
    def straight_runway_m(self, max_m: float = 1200.0,
                          lat_tol_m: float = 1.0) -> float:
        """How much dead-straight road lies ahead of the start line."""


# ---------------------------------------------------------------------------
# CARLA
# ---------------------------------------------------------------------------
class CarlaAdapter(SimAdapter):
    """CARLA 0.9.16 implementation.

    Mirrors carlaaccsim/carlaAccSimTown.py's setup (same camera pose and
    resolution, same static-vehicle culling, same async-world default) so
    perception sees the imagery it was tuned on — the only differences are
    scenario-owned: no traffic, no junction monitor, no pure-pursuit
    fallback, and a scripted stationary target.
    """

    # Minimum window for the speed-differentiated acceleration estimate,
    # and the physical bound any single sample is clamped to.
    ACCEL_MIN_DT = 0.05
    ACCEL_CLAMP = 12.0

    def __init__(self, *, host='localhost', port=2000, town='Town06',
                 spawn_index=80, ego_bp='vehicle.dodge.charger_2020',
                 target_bp='vehicle.dodge.charger_2020',
                 cam_width=1280, cam_height=720, cam_fov=90, cam_tick=0.05,
                 weather='ClearNoon', sync=False, fixed_delta=0.05,
                 clean_start=True):
        import carla  # imported lazily so the harness can list the matrix
        self._carla = carla  # without a CARLA install present
        self.host, self.port = host, port
        self.town, self.spawn_index = town, spawn_index
        self.ego_bp_id, self.target_bp_id = ego_bp, target_bp
        self.cam_width, self.cam_height = cam_width, cam_height
        self.cam_fov, self.cam_tick = cam_fov, cam_tick
        self.weather = weather
        self.sync, self.fixed_delta = sync, fixed_delta
        self.clean_start = clean_start

        self.client = None
        self.world = None
        self.map = None
        self._original_settings = None
        self._start_tf = None

        self.ego = None
        self.target = None
        self.camera = None
        self.collision_sensor = None
        self._image_queue: queue.Queue = queue.Queue()
        self._collisions: list[CollisionEvent] = []
        self._ego_half_len = 0.0
        self._target_half_len = 0.0
        self._prev_speed = 0.0
        self._prev_t = None
        self._accel = 0.0
        self._encode_jpeg = None
        self._tick_time = 0.0
        self._ego_tf_cache = None
        self._target_loc = None

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> None:
        carla = self._carla
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(120.0)
        version = self.client.get_server_version()

        loaded = self.client.get_world().get_map().name
        if not loaded.endswith(self.town):
            self.world = self.client.load_world(self.town)
        else:
            self.world = self.client.get_world()
        self.map = self.world.get_map()
        self._original_settings = self.world.get_settings()

        if self.weather:
            preset = getattr(carla.WeatherParameters, self.weather, None)
            if preset is None:
                raise ValueError(f"unknown weather preset {self.weather!r}")
            self.world.set_weather(preset)

        # Same static-vehicle cull as the production bridge — parked map
        # geometry otherwise gives YOLO a second "lead" to lock onto and
        # would contaminate the single-target premise of this scenario.
        env_objs = []
        for label in (carla.CityObjectLabel.Car,
                      carla.CityObjectLabel.Truck,
                      carla.CityObjectLabel.Bus):
            env_objs.extend(self.world.get_environment_objects(label))
        self.world.enable_environment_objects({o.id for o in env_objs}, False)

        if self.clean_start:
            self._clear_stale_actors()

        spawn_points = self.map.get_spawn_points()
        if not (0 <= self.spawn_index < len(spawn_points)):
            raise RuntimeError(
                f"spawn index {self.spawn_index} out of range for "
                f"{self.map.name} (0..{len(spawn_points) - 1})")
        self._start_tf = spawn_points[self.spawn_index]
        self._start_tf.location.z += 0.5

        if self.sync:
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = self.fixed_delta
            self.world.apply_settings(settings)
        else:
            # TM state persists on the server across processes; a prior
            # session Ctrl-C'd in sync mode leaves it waiting for ticks
            # that never come. Always set it explicitly. See DEBUG §4.
            self.client.get_trafficmanager().set_synchronous_mode(False)

        return version

    def _clear_stale_actors(self) -> int:
        """Destroy every vehicle and sensor left in the world.

        A Ctrl-C or a crash mid-matrix leaves the ego, its camera and the
        current target behind, and the next session then fails to spawn
        because something is already parked on the start line. This
        scenario owns the world outright — it already culls the map's
        static vehicles so YOLO has exactly one target — so clearing
        anything left over is the correct behaviour, not a heuristic.
        Disable with clean_start=False if the world is shared.
        """
        doomed = list(self.world.get_actors().filter('vehicle.*'))
        doomed += list(self.world.get_actors().filter('sensor.*'))
        killed = 0
        for actor in doomed:
            try:
                if hasattr(actor, 'stop'):
                    actor.stop()
            except Exception:
                pass
            try:
                if actor.destroy():
                    killed += 1
            except Exception:
                pass
        if killed:
            print(f"[sim] cleared {killed} stale actor(s) from a previous "
                  f"session", flush=True)
            for _ in range(5):
                self.wait_for_tick()
        return killed

    def spawn_ego(self) -> None:
        carla = self._carla
        bp_lib = self.world.get_blueprint_library()

        matches = bp_lib.filter(self.ego_bp_id)
        if not matches:
            raise RuntimeError(f"no blueprint matching {self.ego_bp_id!r}")
        ego_bp = matches[0]
        ego_bp.set_attribute('ros_name', 'hero')
        ego_bp.set_attribute('role_name', 'hero')

        self.ego = self.world.try_spawn_actor(ego_bp, self._start_tf)
        if self.ego is None:
            # Almost always something still parked on the start line.
            # Report what is in the way rather than just "could not spawn".
            blockers = [
                f"{a.type_id}#{a.id}"
                for a in self.world.get_actors().filter('vehicle.*')
                if a.get_transform().location.distance(
                    self._start_tf.location) < 10.0]
            raise RuntimeError(
                f"ego could not be spawned at {self.map.name} spawn "
                f"{self.spawn_index}"
                + (f" — blocked by: {', '.join(blockers)}" if blockers else
                   " — start line is clear, so the blueprint or the spawn "
                   "transform is at fault"))
        self._ego_half_len = self.ego.bounding_box.extent.x

        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(self.cam_width))
        cam_bp.set_attribute('image_size_y', str(self.cam_height))
        cam_bp.set_attribute('fov', str(self.cam_fov))
        cam_bp.set_attribute('sensor_tick', str(self.cam_tick))
        cam_bp.set_attribute('ros_name', 'front_camera')
        self.camera = self.world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=0.6, y=0.0, z=1.35)),
            attach_to=self.ego)
        self.camera.listen(self._image_queue.put)

        col_bp = bp_lib.find('sensor.other.collision')
        self.collision_sensor = self.world.spawn_actor(
            col_bp, carla.Transform(), attach_to=self.ego)
        self.collision_sensor.listen(self._on_collision)

    def _on_collision(self, event) -> None:
        v = self.ego.get_velocity()
        self._collisions.append(CollisionEvent(
            sim_time=self.world.get_snapshot().timestamp.elapsed_seconds,
            other_id=event.other_actor.type_id,
            impact_speed=math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)))

    def arm(self, placement_distance_m: float,
            lateral_offset_m: float) -> ScenarioGeometry:
        carla = self._carla
        self.disarm()
        self._collisions.clear()

        # Park the ego back on the start line at a dead stop. Teleporting
        # rather than respawning keeps the camera actor (and therefore the
        # perception nodes' image stream) alive across the whole matrix.
        self.ego.set_target_velocity(carla.Vector3D(0, 0, 0))
        self.ego.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        self.ego.set_transform(self._start_tf)
        self.ego.apply_control(carla.VehicleControl(
            throttle=0.0, brake=1.0, steer=0.0, hand_brake=True))
        for _ in range(10):          # let the physics settle on the ground
            self.wait_for_tick()
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))

        start_wp = self.map.get_waypoint(self._start_tf.location,
                                         project_to_road=True,
                                         lane_type=carla.LaneType.Driving)
        # placement is a bumper gap; convert to a centre-to-centre arc length
        target_bp = self.world.get_blueprint_library().filter(
            self.target_bp_id)
        if not target_bp:
            raise RuntimeError(f"no blueprint matching {self.target_bp_id!r}")
        target_bp = target_bp[0]

        arc = placement_distance_m + self._ego_half_len
        nexts = start_wp.next(arc)
        if not nexts:
            raise RuntimeError(
                f"no road {arc:.0f} m ahead of the start line — "
                f"the requested speed/TTC combination does not fit on "
                f"{self.map.name} spawn {self.spawn_index}")
        cyaw = start_wp.transform.rotation.yaw
        target_wp = min(nexts, key=lambda w: abs(
            (w.transform.rotation.yaw - cyaw + 540.0) % 360.0 - 180.0))

        tf = target_wp.transform
        # The target's own half-length pushes it further out so the
        # *bumper* gap is what the caller asked for.
        right = tf.get_right_vector()
        forward = tf.get_forward_vector()
        loc = carla.Location(
            x=tf.location.x + right.x * lateral_offset_m,
            y=tf.location.y + right.y * lateral_offset_m,
            z=tf.location.z + 0.5)
        spawn_tf = carla.Transform(loc, tf.rotation)

        self.target = self.world.try_spawn_actor(target_bp, spawn_tf)
        if self.target is None:
            raise RuntimeError(
                f"target could not be spawned {arc:.0f} m ahead "
                f"(offset {lateral_offset_m:+.1f} m)")
        self._target_half_len = self.target.bounding_box.extent.x

        # Nudge the target forward by its own half-length so the requested
        # figure is a true bumper-to-bumper gap, then pin it in place.
        self.target.set_transform(carla.Transform(
            carla.Location(x=loc.x + forward.x * self._target_half_len,
                           y=loc.y + forward.y * self._target_half_len,
                           z=loc.z),
            tf.rotation))
        self.target.apply_control(carla.VehicleControl(
            throttle=0.0, brake=1.0, hand_brake=True))
        self.target.set_simulate_physics(True)

        for _ in range(5):
            self.wait_for_tick()

        # Cache the target pose only AFTER ticking. set_transform is applied
        # server-side on the next tick, so reading it back immediately
        # returns the pre-nudge pose — which silently under-reported every
        # gap by the target's half-length (2.5 m), biasing gap_at_trigger,
        # a_req and min_gap in every run. The target is stationary by
        # definition, so one read here saves an RPC per tick.
        self._target_loc = self.target.get_transform().location

        self._prev_speed = 0.0
        self._prev_t = None
        self._accel = 0.0
        while not self._image_queue.empty():
            self._image_queue.get()

        # Verify what was actually built matches what was asked for. A
        # stale-pose bug here is invisible in the results — it just shifts
        # every gap by a constant and quietly biases a_req — so check it
        # rather than trusting the placement maths.
        self._ego_tf_cache = None
        actual = self.gap_m()
        if abs(actual - placement_distance_m) > 1.0:
            raise RuntimeError(
                f"target placed at {actual:.2f} m but {placement_distance_m:.2f} m "
                f"was requested ({actual - placement_distance_m:+.2f} m off). "
                f"Scenario geometry is wrong — refusing to record a run "
                f"whose gaps would all be biased.")

        return ScenarioGeometry(
            town=self.map.name,
            spawn_index=self.spawn_index,
            straight_runway_m=self.straight_runway_m(),
            placement_distance_m=actual,
            lane_width_m=start_wp.lane_width,
            ego_blueprint=self.ego.type_id,
            target_blueprint=self.target.type_id)

    def disarm(self) -> None:
        if self.target is not None:
            try:
                if self.target.is_alive:
                    self.target.destroy()
            except Exception:
                pass
            self.target = None
        self._target_loc = None

    def close(self) -> None:
        self.disarm()
        for actor in (self.collision_sensor, self.camera, self.ego):
            try:
                if actor is not None and actor.is_alive:
                    if hasattr(actor, 'stop'):
                        actor.stop()
                    actor.destroy()
            except Exception:
                pass
        self.collision_sensor = self.camera = self.ego = None
        try:
            if self._original_settings is not None:
                self.world.apply_settings(self._original_settings)
        except Exception:
            pass

    # -- per-tick -----------------------------------------------------------
    def wait_for_tick(self) -> float:
        if self.sync:
            self.world.tick()
            snap = self.world.get_snapshot()
        else:
            snap = self.world.wait_for_tick()
        # Every CARLA getter is a blocking RPC. ego_state/gap_m/lane_error
        # were each re-fetching the ego transform, costing three extra
        # round-trips per iteration and holding the director to ~15 Hz.
        # Cache per tick instead — at 130 km/h the loop period is worth
        # 2.4 m of closing distance, so this is measurement fidelity, not
        # micro-optimisation.
        self._tick_time = snap.timestamp.elapsed_seconds
        self._ego_tf_cache = None
        return self._tick_time

    def _ego_tf(self):
        if self._ego_tf_cache is None:
            self._ego_tf_cache = self.ego.get_transform()
        return self._ego_tf_cache

    def ego_state(self) -> EgoState:
        tf = self._ego_tf()
        v = self.ego.get_velocity()
        speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

        # CARLA's IMU-free accel (get_acceleration) is noisy at the tick
        # rate; differentiate speed against sim time instead, which is what
        # the run record needs for the R171 deceleration figure.
        # Differentiate over a minimum window, not tick-to-tick. In async
        # mode CARLA's ticks are irregular, and a 2 ms gap turns a 0.03 m/s
        # rounding wobble into 15 m/s^2 — which is how a 30 km/h stop that
        # physically peaks near 6 m/s^2 reported 13.9. Holding the previous
        # sample until at least ACCEL_MIN_DT has elapsed bounds that error,
        # and the clamp rejects whatever still gets through.
        now = self._tick_time
        if self._prev_t is None:
            self._prev_t, self._prev_speed = now, speed
        else:
            dt = now - self._prev_t
            if dt >= self.ACCEL_MIN_DT:
                raw = (speed - self._prev_speed) / dt
                raw = max(min(raw, self.ACCEL_CLAMP), -self.ACCEL_CLAMP)
                self._accel = 0.3 * raw + 0.7 * self._accel
                self._prev_t, self._prev_speed = now, speed

        return EgoState(x=tf.location.x, y=tf.location.y,
                        yaw_rad=math.radians(tf.rotation.yaw),
                        speed=speed, accel=self._accel)

    def gap_m(self) -> float:
        if self.target is None:
            return float('inf')
        ego_tf = self._ego_tf()
        fwd = ego_tf.get_forward_vector()
        d = self._target_loc - ego_tf.location
        longitudinal = d.x * fwd.x + d.y * fwd.y
        return longitudinal - self._ego_half_len - self._target_half_len

    def lane_error(self) -> LaneError:
        carla = self._carla
        ego_tf = self._ego_tf()

        # Cross-track is measured against the START-LINE RAY, not against a
        # per-tick waypoint lookup.
        #
        # get_waypoint() snaps to whatever lane is nearest, so the instant
        # the vehicle crosses a marking the reference jumps to the next
        # lane and the reported error flips sign. One 70 km/h run showed
        # cte going -1.66 -> +1.47 m in a single 50 ms sample while the
        # vehicle's world y moved a steady +0.26 m per sample: a straight
        # lane departure, reported as a 3.13 m oscillation. The metric hid
        # exactly the failure it existed to reveal.
        #
        # This scenario runs on a dead-straight road by construction, so
        # the start ray IS the intended path and the perpendicular offset
        # from it is the honest lateral error, unbounded by lane width.
        p0 = self._start_tf.location
        start_right = self._start_tf.get_right_vector()
        d0x = ego_tf.location.x - p0.x
        d0y = ego_tf.location.y - p0.y
        cte_ray = d0x * start_right.x + d0y * start_right.y

        wp = self.map.get_waypoint(ego_tf.location, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        lane_tf = wp.transform
        right = lane_tf.get_right_vector()
        d = ego_tf.location - lane_tf.location
        # Heading error still comes from the local lane (it is continuous
        # across lane boundaries and is what the locked-lateral controller
        # needs); only the cross-track term uses the start ray.
        heading_err = math.radians(
            (lane_tf.rotation.yaw - ego_tf.rotation.yaw + 540.0) % 360.0
            - 180.0)
        return LaneError(cross_track_m=cte_ray, heading_err_rad=heading_err)

    def apply_control(self, throttle: float, brake: float,
                      steer: float) -> None:
        self.ego.apply_control(self._carla.VehicleControl(
            throttle=float(min(max(throttle, 0.0), 1.0)),
            brake=float(min(max(brake, 0.0), 1.0)),
            steer=float(min(max(steer, -1.0), 1.0))))

    def force_speed(self, speed_mps: float) -> None:
        fwd = self.ego.get_transform().get_forward_vector()
        self.ego.set_target_velocity(self._carla.Vector3D(
            x=fwd.x * speed_mps, y=fwd.y * speed_mps, z=0.0))

    def poll_camera(self) -> bytes | None:
        if self._encode_jpeg is None:
            import cv2
            import numpy as np

            def _enc(img):
                arr = np.frombuffer(img.raw_data, dtype=np.uint8)
                arr = arr.reshape((img.height, img.width, 4))
                ok, buf = cv2.imencode('.jpg', arr[:, :, :3],
                                       [cv2.IMWRITE_JPEG_QUALITY, 95])
                return buf.tobytes() if ok else None

            self._encode_jpeg = _enc

        latest = None
        while not self._image_queue.empty():
            latest = self._image_queue.get()
        return self._encode_jpeg(latest) if latest is not None else None

    def take_collision(self) -> CollisionEvent | None:
        return self._collisions[0] if self._collisions else None

    def straight_runway_m(self, max_m: float = 1200.0,
                          lat_tol_m: float = 1.0) -> float:
        """Walk the lane forward from the start line until it deviates
        laterally from the initial heading ray by more than `lat_tol_m`.

        Junctions are deliberately NOT a stop condition: Town06's highway
        merges are flagged `is_junction` but stay dead straight, and this
        scenario has no junction monitor to care.
        """
        carla = self._carla
        wp = self.map.get_waypoint(self._start_tf.location,
                                   project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        if wp is None:
            return 0.0
        p0 = wp.transform.location
        right = wp.transform.get_right_vector()
        dist, cur, step = 0.0, wp, 2.0
        while dist < max_m:
            nexts = cur.next(step)
            if not nexts:
                break
            cyaw = cur.transform.rotation.yaw
            cur = min(nexts, key=lambda w: abs(
                (w.transform.rotation.yaw - cyaw + 540.0) % 360.0 - 180.0))
            dist += step
            d = cur.transform.location
            if abs((d.x - p0.x) * right.x + (d.y - p0.y) * right.y) > lat_tol_m:
                break
        return dist


# ---------------------------------------------------------------------------
# MORAI — seam only
# ---------------------------------------------------------------------------
class MoraiAdapter(SimAdapter):
    """Placeholder for the MORAI side of the CARLA/MORAI comparison.

    MORAI has no Python actor API equivalent to CARLA's, so this adapter
    will be topic-driven rather than RPC-driven. When implementing it:

    ``ego_state``    /Ego/vehicleinfo (morai_v2_1_ros2_msgs/VehicleInfo),
                     already consumed by morai_bridge/state_adapter_node.py.
                     Pose comes from the same message; speed is what that
                     node republishes on /Car_1/vehicle/speed.
    ``apply_control`` publish to /Car_1/control via the existing
                     morai_bridge/control_adapter_node.py rather than
                     writing a second control path — the two would race
                     over the same topic (see start_adas.sh's duplicate-
                     adapter guard and DEBUG.md).
    ``arm``          MORAI has no runtime spawn API. The target vehicle
                     and its lateral offset have to come from a scenario
                     file (MORAI/Test1.xosc is the existing example) —
                     expect one .xosc per matrix point, generated ahead of
                     time, with this method selecting and loading it.
    ``gap_m``        needs a ground-truth object list from MORAI; the
                     ROS2 Interface's object-info topic is the candidate.
    ``force_speed``  no kinematic velocity override exists. The approach
                     phase has to be flown closed-loop with the speed-hold
                     PI (``--approach-mode physical``), which is why that
                     mode exists at all.
    ``straight_runway_m`` no map API — hardcode the surveyed runway of
                     whichever MORAI map is chosen as the Town06 analogue.

    Until then every entry point raises, loudly, rather than silently
    producing a half-run.
    """

    _MSG = ("MORAI adapter is not implemented yet — this pass is CARLA-only. "
            "See MoraiAdapter's docstring for the topic-by-topic plan.")

    def __init__(self, **kwargs):
        self._kwargs = kwargs

    def connect(self): raise NotImplementedError(self._MSG)
    def spawn_ego(self): raise NotImplementedError(self._MSG)
    def arm(self, placement_distance_m, lateral_offset_m):
        raise NotImplementedError(self._MSG)
    def disarm(self): pass
    def close(self): pass
    def wait_for_tick(self): raise NotImplementedError(self._MSG)
    def ego_state(self): raise NotImplementedError(self._MSG)
    def gap_m(self): raise NotImplementedError(self._MSG)
    def lane_error(self): raise NotImplementedError(self._MSG)
    def apply_control(self, throttle, brake, steer):
        raise NotImplementedError(self._MSG)
    def force_speed(self, speed_mps): raise NotImplementedError(self._MSG)
    def poll_camera(self): raise NotImplementedError(self._MSG)
    def take_collision(self): raise NotImplementedError(self._MSG)
    def straight_runway_m(self, max_m=1200.0, lat_tol_m=1.0):
        raise NotImplementedError(self._MSG)


ADAPTERS = {'carla': CarlaAdapter, 'morai': MoraiAdapter}

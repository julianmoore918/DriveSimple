#!/usr/bin/env python3
"""CARLA adapter for curved-road scenarios, plus the surveyed site table.

`CarlaAdapter` (sim_adapter.py) assumes a dead-straight road: it measures
cross-track against the start-line ray and the gap as a projection onto
the ego's forward axis. Both are correct there and wrong on a curve —
the ray diverges from the road, and a projected gap under-reads the real
distance-to-go by the sagitta of the arc (7 m at 200 m of separation on a
400 m radius). This subclass replaces those two measurements with ones
taken against the lane's own centreline, and adds what the curved R171
test and the R79 lane-keeping test need on top: arc placement, lateral
clearance to the lane markings, and a high-rate IMU.

Everything else — spawning, camera, collisions, powertrain fix, control
application — is inherited unchanged, so the curved scenarios are running
the same plant as the validated straight one.

Site selection
--------------
Sites come from `curve_survey.py`, which walks every stock map and
extracts constant-radius arcs. See SITES below for the surveyed set and
scenarios/README.md for how the radii map onto R171's reference geometry.
"""

from __future__ import annotations

import math
import queue
from dataclasses import dataclass, field

from sim_adapter import CarlaAdapter, LaneError, ScenarioGeometry


# ---------------------------------------------------------------------------
# Surveyed sites
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CurveSite:
    """One constant-radius arc in a CARLA map, as measured by curve_survey.

    `entry_s_m` is the OpenDRIVE arc coordinate of the arc's start along
    its own road, so `map.get_waypoint_xodr(road_id, lane_id, entry_s_m)`
    reproduces the survey's entry point exactly. Radius and lead-in are
    what the survey measured; the adapter re-measures both at connect()
    and refuses to run if the map has moved under the table.
    """
    name: str
    town: str
    road_id: int
    lane_id: int
    entry_s_m: float
    radius_m: float
    direction: str          # 'left' | 'right', as driven
    arc_m: float            # length of the constant-radius section
    lead_in_m: float        # dead-straight road before the arc
    lane_width_m: float
    lanes_left: int         # driving lanes beside it, same direction
    lanes_right: int
    note: str = ''

    @property
    def ay_at(self):
        """Lateral demand [m/s^2] this site imposes at a given speed."""
        return lambda v_mps: v_mps ** 2 / self.radius_m

    def speed_for_ay(self, ay: float) -> float:
        """Speed [m/s] whose steady-state lateral demand is `ay`."""
        return math.sqrt(max(ay, 0.0) * self.radius_m)


# Measured 2026-08-12 against CARLA 0.9.16:
#   curve_survey.py --maps Town03 Town04 Town05 Town06 Town07 Town10HD \
#                   --min-curve-m 25 --min-lead-in-m 40
#   curve_survey.py --maps Town12 --min-curve-m 40 --min-lead-in-m 100
#
# Two findings shape this table.
#
# 1. The small stock maps have no highway-grade radii. The largest
#    constant arc in Town01-Town10 is 386 m (Town07, 38 m long, 14 m of
#    lead-in); the largest with room to place a target on is 199 m. R171's
#    reference geometry — 787 m and 374 m — is not reproducible there, and
#    the stock sites are all MORE severe than the reference at the same
#    speed, which is the direction §4.2.4.1's substitution clause permits.
#    The cost is a speed ceiling: 199 m reaches the 3 m/s^2 M1 limit at
#    88 km/h, so no stock site can carry the 110 or 130 km/h rows.
#
# 2. Town12 does have highway geometry — 1185 m over a 1026 m arc, 500 m
#    radii with 500 m of straight lead-in, and 345-417 m arcs long enough
#    for the whole manoeuvre. It is the only map where the curved matrix
#    reaches 130 km/h. It is also a Large Map: it takes minutes to load
#    and costs noticeably more GPU, so the stock sites remain the ones to
#    iterate on and Town12 is where a reportable matrix is flown.
#
# R_spread (survey column, quoted per site) is the variation of the
# windowed radius inside the arc, as a fraction of the mean. Below ~0.05
# it is a true constant-radius arc; 0.15-0.2 means the radius is still
# opening or closing and the figure is a mean.
SITES: dict[str, CurveSite] = {
    # --- Town12: the only R171-grade geometry in stock CARLA ------------
    't12_r1185': CurveSite(
        't12_r1185', 'Town12', 738, 4, 1314.0, 1184.9, 'left', 1026.0, 500.0,
        3.5, 1, 2,
        '1026 m of dead-constant arc (R_spread 0.00) after 500 m of '
        'straight, three lanes wide. The only site where 130 km/h fits, '
        'and the only one where the whole approach AND the whole braking '
        'manoeuvre happen inside the bend. Larger than R171\'s 787 m '
        'first turn, so it is LESS severe than the reference: quote it as '
        'a high-speed curved-road case, not as a substitute for §4.2.4.1.'),
    't12_r500': CurveSite(
        't12_r500', 'Town12', 820, -3, 684.3, 500.0, 'right', 276.0, 500.0,
        3.5, 1, 1,
        'First-turn analogue: 500 m against the reference 787 m, so more '
        'severe (2.60 m/s^2 at 130 km/h against the reference\'s 1.66). '
        'R_spread 0.00, 276 m of arc, 500 m of lead-in, traffic lane '
        'either side. The site to fly the high-speed rows on.'),
    't12_r417': CurveSite(
        't12_r417', 'Town12', 924, -2, 56.9, 417.1, 'right', 460.0, 500.0,
        3.5, 0, 2,
        '460 m of arc and 500 m of lead-in — the most room of any site '
        'near the reference second turn (374 m), at +11 % so marginally '
        'less severe. R_spread 0.10.'),
    't12_r345': CurveSite(
        't12_r345', 'Town12', 701, -2, 21.6, 345.2, 'right', 314.0, 210.0,
        3.5, 0, 1,
        'Second-turn analogue on the severe side: 345 m against the '
        'reference 374 m (-8 %), with 314 m of arc. 210 m of lead-in caps '
        'the approach at about 110 km/h with a 1.5 s settle. '
        'R_spread 0.17.'),
    # --- Stock maps: quicker to load, tighter than the reference --------
    't07_r364': CurveSite(
        't07_r364', 'Town07', 44, -1, 26.4, 363.6, 'right', 34.0, 92.0,
        3.5, 0, 0,
        'The regulation second turn to within 2.7 %, and near-perfectly '
        'constant (R_spread 0.02) — the only small-map site that '
        'reproduces an R171 reference radius. Paid for with 34 m of arc '
        'and 92 m of lead-in, and it is a single-lane rural road, so both '
        'lane edges are road boundaries rather than traffic-lane '
        'markings. Use it to check the radius the regulation names; use '
        't04_r199 or t12_r417 to actually exercise the system.'),
    # --- the workhorses: long arc, long lead-in, multi-lane -------------
    't04_r199': CurveSite(
        't04_r199', 'Town04', 46, 4, 60.6, 199.3, 'right', 296.0, 462.0,
        3.5, 1, 2,
        'Best site in stock CARLA: 296 m of constant arc (R_spread 0.03) '
        'after 462 m of straight, three lanes wide with traffic lanes on '
        'both sides. Everything fits — placement, settle, and the whole '
        'braking manoeuvre inside the bend. Reaches 3 m/s^2 at 88 km/h, '
        'which is the speed ceiling for the curved matrix.'),
    't04_r182': CurveSite(
        't04_r182', 'Town04', 45, 4, 285.4, 181.6, 'right', 270.0, 126.0,
        3.5, 1, 2,
        'Second-longest arc, on the other side of the same Town04 ring. '
        '126 m of lead-in caps the approach speed at about 90 km/h with '
        'a 1.5 s settle. R_spread 0.16.'),
    't04_r162': CurveSite(
        't04_r162', 'Town04', 49, -2, 33.5, 162.2, 'right', 240.0, 466.0,
        3.5, 1, 2,
        '240 m of arc after 466 m of straight, mid-lane of three. The '
        'other sign of Town04 ring geometry (lane_id negative), so it '
        'covers the opposite direction of travel. R_spread 0.17.'),
    't06_r144': CurveSite(
        't06_r144', 'Town06', 5, 4, 2.4, 143.6, 'left', 36.0, 132.0,
        3.5, 1, 3,
        'Same map as the straight R171 matrix, so perception runs on '
        'ground it has already been characterised on and no world reload '
        'is needed between the two scripts. Perfectly constant '
        '(R_spread 0.00) but only 36 m of arc — enough for placement at '
        'up to about 70 km/h, not for a long manoeuvre.'),
    # --- tighter: the low-speed R79 bands -------------------------------
    't04_r076': CurveSite(
        't04_r076', 'Town04', 41, -2, 8.8, 76.4, 'left', 102.0, 500.0,
        3.5, 1, 2,
        '500 m of straight into a 102 m arc. 2.55 m/s^2 at 50 km/h, so it '
        'is the site that covers the 10-60 km/h band at a 3.0 m/s^2 '
        'declaration. R_spread 0.16.'),
    't03_r060': CurveSite(
        't03_r060', 'Town03', 67, -1, 55.4, 60.3, 'right', 48.0, 122.0,
        3.5, 0, 1,
        'Urban radius: 2.55 m/s^2 at 45 km/h, 1.5 m/s^2 at 34 km/h. '
        'R_spread 0.16.'),
    't10_r042': CurveSite(
        't10_r042', 'Town10HD', 10, 1, 65.5, 41.9, 'right', 50.0, 132.0,
        3.5, 0, 1,
        'Tightest surveyed arc with real lead-in — the bottom of the '
        'speed range, 2.55 m/s^2 at 37 km/h. R_spread 0.17.'),
}

# The site that carries the most of the matrix, and the one to reach for
# when a run has to fit: 296 m of arc after 462 m of straight.
DEFAULT_SITE = 't04_r199'


@dataclass
class CurveGeometry(ScenarioGeometry):
    """ScenarioGeometry plus what only a curve has."""
    site: str = ''
    radius_m: float = 0.0
    direction: str = ''
    arc_m: float = 0.0
    # Where the target ended up relative to the arc entry, and the lateral
    # deviation of the lane from the entry tangent there. The placement
    # rule (R171 §4.2.5.2.2.1.1) is defined by that deviation, so both
    # halves belong in the run record.
    target_into_curve_m: float = 0.0
    target_tangent_offset_m: float = 0.0
    # How much of the approach falls before the arc entry, and how much
    # of THAT is on the straight lead-in. §4.2.5.2.2.1.2 wants the lateral
    # controller settled on the straight, so the second figure is the one
    # the scenario gates on: a start line 300 m before a curve with only
    # 124 m of straight ahead of it spends the first 176 m in whatever
    # precedes that straight, which is usually another bend.
    before_entry_m: float = 0.0
    straight_before_entry_m: float = 0.0
    ay_demand_mps2: float = 0.0
    imu: bool = False


# ---------------------------------------------------------------------------
# Adapter
# ---------------------------------------------------------------------------
class CarlaCurveAdapter(CarlaAdapter):
    """CARLA adapter whose reference is a lane centreline, not a ray."""

    PATH_STEP_M = 1.0
    # Tyre tread half-width. R79 Annex 8 §3.2.1.2 judges lane keeping on
    # "the outside edge of the tyre tread of the vehicle's front wheel",
    # and CARLA exposes wheel centres but not tread width. 0.125 m is half
    # a 245-section tyre; with the charger_2020's measured 0.812 m front
    # track half-width that puts the tread edge at 0.937 m, within 4 mm of
    # the vehicle's own bounding-box half-width (0.941 m), so the two
    # candidate definitions agree and the choice is not load-bearing.
    TYRE_HALF_WIDTH_M = 0.125

    # CARLA's Large Maps stream tiles and are an order of magnitude
    # heavier to load than the stock towns.
    LARGE_MAPS = ('Town11', 'Town12', 'Town13', 'Town15')
    LARGE_MAP_TIMEOUT_S = 600.0
    # Free memory a Large Map needs. Measured the hard way: a sweep that
    # had already loaded Town03, Town04 and Town10HD into one server
    # session took the server down when it reached Town12 — the run died
    # between two cells with nine of twenty-seven done (DEBUG §58).
    LARGE_MAP_MIN_FREE_GB = 8.0

    def __init__(self, *, site: CurveSite, with_imu: bool = False,
                 imu_hz: float = 200.0, path_back_m: float = 700.0,
                 path_fwd_m: float = 400.0, **kwargs):
        kwargs.setdefault('town', site.town)
        kwargs.setdefault('spawn_index', 0)   # unused; the site sets the start
        super().__init__(**kwargs)
        self.site = site
        self.with_imu = with_imu
        self.imu_hz = imu_hz
        self.path_back_m = path_back_m
        self.path_fwd_m = path_fwd_m

        self.imu = None
        self._imu_queue: queue.Queue = queue.Queue()
        self._path = []          # [waypoint], PATH_STEP_M apart
        self._px = self._py = self._pyaw = self._prx = self._pry = None
        self._ps = None
        self._entry_idx = 0
        self._target_s = None
        self._ego_s = 0.0
        self._track_half_m = 0.0
        self._front_axle_m = 0.0
        # Overwritten from the spawned actor's physics control; the
        # defaults are the charger_2020's measured values so anything
        # reading them before spawn_ego() gets the right order of
        # magnitude rather than a zero.
        self.wheelbase_m = 3.048
        self.max_steer_deg = 70.0

    # -- lifecycle ----------------------------------------------------------
    @staticmethod
    def free_memory_gb() -> float:
        """MemAvailable, i.e. what a new allocation can actually get."""
        try:
            with open('/proc/meminfo') as fh:
                for line in fh:
                    if line.startswith('MemAvailable:'):
                        return int(line.split()[1]) / 1024.0 / 1024.0
        except Exception:
            pass
        return float('inf')

    def preflight(self) -> None:
        """Refuse a Large Map the machine cannot hold.

        Loading one into a server that has already cycled through several
        towns is how the first full sweep died: the server went away
        mid-load and took eighteen unrun cells with it. Failing here costs
        a second and says what to do; failing there costs the run.
        """
        if self.site.town not in self.LARGE_MAPS:
            return
        free = self.free_memory_gb()
        if free < self.LARGE_MAP_MIN_FREE_GB:
            raise RuntimeError(
                f"{self.site.town} is a CARLA Large Map and needs roughly "
                f"{self.LARGE_MAP_MIN_FREE_GB:.0f} GB free; "
                f"{free:.1f} GB is available. Restart the CARLA server "
                f"(the UI's Restart CARLA) and run the Town12 sites in "
                f"their own session — a server that has already loaded "
                f"several towns will not survive this load.")

    def connect(self):
        self.preflight()
        if self.site.town in self.LARGE_MAPS:
            self.RPC_TIMEOUT_S = self.LARGE_MAP_TIMEOUT_S
        version = super().connect()
        import numpy as np

        entry = self.map.get_waypoint_xodr(
            self.site.road_id, self.site.lane_id, self.site.entry_s_m)
        if entry is None:
            raise RuntimeError(
                f"site {self.site.name}: no waypoint at road "
                f"{self.site.road_id} lane {self.site.lane_id} "
                f"s={self.site.entry_s_m:.1f} on {self.map.name}. The site "
                f"table was surveyed on CARLA 0.9.16 stock maps — a "
                f"different map version needs a fresh curve_survey.py run.")

        back = self._march(entry, self.path_back_m, backwards=True)[1:]
        fwd = self._march(entry, self.path_fwd_m)
        self._path = list(reversed(back)) + fwd
        self._entry_idx = len(back)

        self._px = np.array([w.transform.location.x for w in self._path])
        self._py = np.array([w.transform.location.y for w in self._path])
        self._pyaw = np.array([math.radians(w.transform.rotation.yaw)
                               for w in self._path])
        rights = [w.transform.get_right_vector() for w in self._path]
        self._prx = np.array([r.x for r in rights])
        self._pry = np.array([r.y for r in rights])
        self._ps = np.arange(len(self._path)) * self.PATH_STEP_M

        # The start line is only provisional here — arm() moves the ego to
        # wherever the requested approach distance puts it. It exists so
        # spawn_ego() has somewhere to put the car.
        self._start_tf = self._path[0].transform
        self._start_tf.location.z += 0.5

        measured = self._measure_radius()
        if abs(measured - self.site.radius_m) / self.site.radius_m > 0.25:
            raise RuntimeError(
                f"site {self.site.name}: table says R = "
                f"{self.site.radius_m:.0f} m, the map measures "
                f"{measured:.0f} m. Re-run curve_survey.py — a scenario "
                f"whose lateral demand is not what the site claims is "
                f"worse than no scenario.")
        self.measured_radius_m = measured
        return version

    def _march(self, wp, length_m: float, backwards: bool = False) -> list:
        """Waypoints every PATH_STEP_M along the lane, straightest branch."""
        out, cur, s = [wp], wp, 0.0
        while s < length_m:
            cand = (cur.previous(self.PATH_STEP_M) if backwards
                    else cur.next(self.PATH_STEP_M))
            if not cand:
                break
            cyaw = cur.transform.rotation.yaw
            cur = min(cand, key=lambda w: abs(
                (w.transform.rotation.yaw - cyaw + 540.0) % 360.0 - 180.0))
            out.append(cur)
            s += self.PATH_STEP_M
        return out

    def _measure_radius(self) -> float:
        """Radius of the arc as it exists in the loaded map."""
        i0 = self._entry_idx
        i1 = min(i0 + int(self.site.arc_m / self.PATH_STEP_M),
                 len(self._path) - 1)
        total = 0.0
        for a, b in zip(self._path[i0:i1], self._path[i0 + 1:i1 + 1]):
            total += ((b.transform.rotation.yaw - a.transform.rotation.yaw
                       + 540.0) % 360.0 - 180.0)
        arc = (i1 - i0) * self.PATH_STEP_M
        return arc / math.radians(abs(total)) if abs(total) > 1e-6 else 1e9

    def spawn_ego(self) -> None:
        super().spawn_ego()
        # Wheel geometry, read once. Reading physics control per tick would
        # be an extra blocking RPC in the control loop; the numbers are
        # constant for the run, and body roll (~1 cm of contact-patch shift
        # at 3 m/s^2) is below the resolution the lane-crossing check needs.
        pc = self.ego.get_physics_control()
        tf = self.ego.get_transform()
        fwd, right = tf.get_forward_vector(), tf.get_right_vector()

        def axle(wheels):
            lat, lon = [], []
            for wheel in wheels:
                dx = wheel.position.x / 100.0 - tf.location.x
                dy = wheel.position.y / 100.0 - tf.location.y
                lat.append(dx * right.x + dy * right.y)
                lon.append(dx * fwd.x + dy * fwd.y)
            return lat, lon

        lat, lon = axle(pc.wheels[:2])          # front
        _, lon_rear = axle(pc.wheels[2:4])      # rear
        self._track_half_m = (max(lat) - min(lat)) / 2.0 if lat else 0.81
        self._front_axle_m = sum(lon) / len(lon) if lon else 1.4
        # Wheelbase from the wheel positions rather than the 3.048 m
        # figure the Stanley node carries: the two should agree, and if
        # they ever stop agreeing the steering ground truth computed here
        # would silently drift away from the controller's own model.
        rear = sum(lon_rear) / len(lon_rear) if lon_rear else -1.6
        self.wheelbase_m = abs(self._front_axle_m - rear)
        # Normalised steer [-1, 1] maps linearly onto this, so it is what
        # converts a cmd_steer value back into degrees at the wheel.
        self.max_steer_deg = (pc.wheels[0].max_steer_angle
                              if pc.wheels else 70.0)

        if self.with_imu:
            # R79 Annex 8 §2.4 requires the lateral acceleration to be
            # sampled at >= 100 Hz. The director loop runs at ~20 Hz, so
            # the compliance signal comes from a dedicated IMU rather than
            # from differentiating pose in the loop.
            carla = self._carla
            bp = self.world.get_blueprint_library().find('sensor.other.imu')
            bp.set_attribute('sensor_tick', str(1.0 / self.imu_hz))
            for axis in 'xyz':
                bp.set_attribute(f'noise_accel_stddev_{axis}', '0.0')
                bp.set_attribute(f'noise_gyro_stddev_{axis}', '0.0')
            self.imu = self.world.spawn_actor(
                bp, carla.Transform(), attach_to=self.ego)
            self.imu.listen(self._imu_queue.put)

    def close(self) -> None:
        if self.imu is not None:
            try:
                if self.imu.is_alive:
                    self.imu.stop()
                    self.imu.destroy()
            except Exception:
                pass
            self.imu = None
        super().close()

    # -- path geometry ------------------------------------------------------
    def _project(self, x: float, y: float) -> tuple[float, float, float]:
        """(arc length along the path, cross-track, path yaw) at (x, y).

        Nearest-point projection onto the whole polyline rather than a
        CARLA waypoint lookup. get_waypoint() snaps to whichever lane is
        nearest, so the instant a wheel crosses a marking the reference
        jumps to the next lane and the reported error flips sign — a
        straight lane departure came out as a 3.13 m oscillation (see
        CarlaAdapter.lane_error). The polyline is the intended path by
        construction and cannot re-snap, so the error it reports is
        unbounded by lane width, which is exactly what a departure test
        needs.
        """
        import numpy as np
        i = int(np.argmin((self._px - x) ** 2 + (self._py - y) ** 2))
        dx, dy = x - self._px[i], y - self._py[i]
        cte = dx * self._prx[i] + dy * self._pry[i]
        # Longitudinal residual within the segment, so s is continuous
        # rather than quantised to PATH_STEP_M.
        yaw = self._pyaw[i]
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        return float(self._ps[i] + along), float(cte), float(yaw)

    def lane_error(self) -> LaneError:
        tf = self._ego_tf()
        s, cte, yaw = self._project(tf.location.x, tf.location.y)
        self._ego_s = s
        heading_err = math.radians(
            (math.degrees(yaw) - tf.rotation.yaw + 540.0) % 360.0 - 180.0)
        return LaneError(cross_track_m=cte, heading_err_rad=heading_err)

    def tangent_touch_offset(self, half_width_m: float) -> tuple[float, float]:
        """Where R171 §4.2.5.2.2.1.1 puts the target, measured from the
        arc entry.

        The regulation places the stationary target in the curved lane
        "so that the rear corner is touching the extrapolated lane line if
        the straight were to continue". Extrapolating the entry tangent,
        the lane centre has moved sideways by lane_width/2 + the target's
        half-width by the time the target's near rear corner sits on the
        far marking's extrapolation — so that deviation is the placement
        condition, and this walks the real polyline to find where it is
        met rather than assuming the parabolic s^2/2R approximation.

        Returns (distance past the arc entry [m], deviation there [m]).
        """
        need = self.site.lane_width_m / 2.0 + half_width_m
        i0 = self._entry_idx
        x0, y0 = self._px[i0], self._py[i0]
        rx, ry = self._prx[i0], self._pry[i0]
        for i in range(i0 + 1, len(self._path)):
            dev = abs((self._px[i] - x0) * rx + (self._py[i] - y0) * ry)
            if dev >= need:
                return float(self._ps[i] - self._ps[i0]), float(dev)
        raise RuntimeError(
            f"site {self.site.name}: the lane never deviates {need:.2f} m "
            f"from the entry tangent within {self.path_fwd_m:.0f} m, so the "
            f"R171 placement condition cannot be met here. Either the arc "
            f"is too short or the radius is too large for this lane width.")

    # -- per-run ------------------------------------------------------------
    def arm(self, placement_distance_m: float,
            lateral_offset_m: float) -> CurveGeometry:
        """Place the ego and the stationary target on the arc.

        `placement_distance_m` is measured ALONG THE LANE, not as a
        straight line: on a 400 m radius a 200 m separation has a 12.5 m
        sagitta, so a chord-based placement would put the ego 12 m closer
        than the scenario asked for and bias every a_req in the run.
        """
        carla = self._carla
        self.disarm()
        self._collisions.clear()

        target_bp = self.world.get_blueprint_library().filter(
            self.target_bp_id)
        if not target_bp:
            raise RuntimeError(f"no blueprint matching {self.target_bp_id!r}")
        target_bp = target_bp[0]
        # A blueprint's extent is only readable from a spawned actor, and
        # the placement distance has to be known before the target exists.
        # Ego and target are the same blueprint by default, so the ego's
        # half-width stands in.
        half_w = self.ego.bounding_box.extent.y

        into, dev = self.tangent_touch_offset(half_w)
        s_target = self._ps[self._entry_idx] + into
        # `placement_distance_m` is a BUMPER-to-bumper gap; s_target and
        # s_start are both centre positions on the path, and the target's
        # own half-length is added back by the nudge below. So the ego's
        # half-length is what stands between the two conventions — leaving
        # it out put every target 2.5 m closer than asked for, which the
        # placement assertion at the end of this method caught.
        s_start = s_target - placement_distance_m - self._ego_half_len
        if s_start < self._ps[0] + 5.0:
            raise RuntimeError(
                f"site {self.site.name}: an approach of "
                f"{placement_distance_m:.0f} m would start "
                f"{self._ps[0] - s_start:.0f} m before the surveyed path "
                f"begins. Reduce the speed or the TTC, or pick a site with "
                f"a longer lead-in (this one has "
                f"{self.site.lead_in_m:.0f} m).")

        start_wp = self._wp_at(s_start)
        start_tf = start_wp.transform
        start_tf.location.z += 0.5
        self._start_tf = start_tf

        self.ego.set_target_velocity(carla.Vector3D(0, 0, 0))
        self.ego.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        self.ego.set_transform(start_tf)
        self.ego.apply_control(carla.VehicleControl(
            throttle=0.0, brake=1.0, steer=0.0, hand_brake=True))
        for _ in range(10):
            self.wait_for_tick()
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))

        target_wp = self._wp_at(s_target)
        tf = target_wp.transform
        right, forward = tf.get_right_vector(), tf.get_forward_vector()
        loc = carla.Location(
            x=tf.location.x + right.x * lateral_offset_m,
            y=tf.location.y + right.y * lateral_offset_m,
            z=tf.location.z + 0.5)
        self.target = self.world.try_spawn_actor(
            target_bp, carla.Transform(loc, tf.rotation))
        if self.target is None:
            raise RuntimeError(
                f"target could not be spawned {placement_distance_m:.0f} m "
                f"along the lane (offset {lateral_offset_m:+.1f} m)")
        self._target_half_len = self.target.bounding_box.extent.x

        # Nudge forward by the target's own half-length so the requested
        # figure is a bumper-to-bumper gap, as in the straight scenario.
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

        # Cache only after ticking — set_transform lands server-side on the
        # next tick, and reading it back immediately returns the pre-nudge
        # pose, which silently shortened every gap by 2.5 m in the straight
        # scenario before it was caught.
        self._target_loc = self.target.get_transform().location
        self._target_s, _, _ = self._project(self._target_loc.x,
                                             self._target_loc.y)
        self._ego_tf_cache = None
        self._prev_speed, self._prev_t, self._accel = 0.0, None, 0.0
        while not self._image_queue.empty():
            self._image_queue.get()
        while not self._imu_queue.empty():
            self._imu_queue.get()

        actual = self.gap_m()
        if abs(actual - placement_distance_m) > 1.5:
            raise RuntimeError(
                f"target placed at {actual:.2f} m along the lane but "
                f"{placement_distance_m:.2f} m was requested "
                f"({actual - placement_distance_m:+.2f} m off). Refusing to "
                f"record a run whose gaps would all be biased.")

        entry_s = float(self._ps[self._entry_idx])
        return CurveGeometry(
            town=self.map.name,
            spawn_index=-1,
            straight_runway_m=self.site.lead_in_m,
            placement_distance_m=actual,
            lane_width_m=start_wp.lane_width,
            ego_blueprint=self.ego.type_id,
            target_blueprint=self.target.type_id,
            site=self.site.name,
            radius_m=self.measured_radius_m,
            direction=self.site.direction,
            arc_m=self.site.arc_m,
            target_into_curve_m=into,
            target_tangent_offset_m=dev,
            before_entry_m=entry_s - s_start,
            straight_before_entry_m=min(entry_s - s_start,
                                        self.site.lead_in_m),
            imu=self.with_imu)

    def arm_lane_keeping(self, before_entry_m: float) -> CurveGeometry:
        """Put the ego on the lane `before_entry_m` ahead of the arc, at
        rest, with no target. The R79 lane-keeping tests have no lead
        vehicle — the only thing under test is where the car ends up
        laterally, so a target would just give YOLO something to slow for.
        """
        carla = self._carla
        self.disarm()
        self._collisions.clear()

        entry_s = float(self._ps[self._entry_idx])
        s_start = entry_s - before_entry_m
        if s_start < self._ps[0] + 5.0:
            raise RuntimeError(
                f"site {self.site.name}: a {before_entry_m:.0f} m run-up "
                f"starts before the surveyed path ({self._ps[0] - s_start:.0f} "
                f"m short). Reduce the settle time or the speed.")

        start_wp = self._wp_at(s_start)
        start_tf = start_wp.transform
        start_tf.location.z += 0.5
        self._start_tf = start_tf

        self.ego.set_target_velocity(carla.Vector3D(0, 0, 0))
        self.ego.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        self.ego.set_transform(start_tf)
        self.ego.apply_control(carla.VehicleControl(
            throttle=0.0, brake=1.0, steer=0.0, hand_brake=True))
        for _ in range(10):
            self.wait_for_tick()
        self.ego.apply_control(carla.VehicleControl(hand_brake=False))

        self._target_s = None
        self._ego_tf_cache = None
        self._prev_speed, self._prev_t, self._accel = 0.0, None, 0.0
        while not self._image_queue.empty():
            self._image_queue.get()
        while not self._imu_queue.empty():
            self._imu_queue.get()

        return CurveGeometry(
            town=self.map.name,
            spawn_index=-1,
            straight_runway_m=self.site.lead_in_m,
            placement_distance_m=0.0,
            lane_width_m=start_wp.lane_width,
            ego_blueprint=self.ego.type_id,
            target_blueprint='',
            site=self.site.name,
            radius_m=self.measured_radius_m,
            direction=self.site.direction,
            arc_m=self.site.arc_m,
            before_entry_m=before_entry_m,
            straight_before_entry_m=min(before_entry_m, self.site.lead_in_m),
            imu=self.with_imu)

    def path_s(self) -> float:
        """Arc length of the ego along the reference path [m]. Valid after
        lane_error() or gap_m() has run this tick."""
        return self._ego_s

    def entry_s(self) -> float:
        """Arc length of the curve entry on the same scale as path_s()."""
        return float(self._ps[self._entry_idx])

    def path_end_s(self) -> float:
        return float(self._ps[-1])

    def _wp_at(self, s: float):
        idx = int(round((s - self._ps[0]) / self.PATH_STEP_M))
        return self._path[max(0, min(idx, len(self._path) - 1))]

    def gap_m(self) -> float:
        """Bumper-to-bumper gap ALONG THE LANE.

        The straight scenario projects onto the ego's forward axis, which
        on a curve reads short by the arc's sagitta. Distance-to-go is
        what a_req and the handover trigger mean, so this is arc length;
        `gap_projected_m` keeps the chord figure for comparison with what
        the camera measures.
        """
        if self.target is None or self._target_s is None:
            return float('inf')
        tf = self._ego_tf()
        s, _, _ = self._project(tf.location.x, tf.location.y)
        self._ego_s = s
        return (self._target_s - s
                - self._ego_half_len - self._target_half_len)

    def gap_projected_m(self) -> float:
        """Straight-line gap projected on the ego's forward axis — the
        quantity a monocular range estimate approximates, so this is the
        fair comparison for /ACC/lead_vehicle_distance."""
        if self.target is None:
            return float('inf')
        return super().gap_m()

    def target_bearing_rad(self) -> float:
        """Angle of the target from the ego's heading, +ve to the right.

        The substance of the curved-road test: a target 200 m ahead round
        a 400 m bend sits ~14 degrees off axis, and whether perception
        associates it at all is the question the scenario is asking.
        """
        if self.target is None or self._target_loc is None:
            return float('nan')
        tf = self._ego_tf()
        fwd, right = tf.get_forward_vector(), tf.get_right_vector()
        dx = self._target_loc.x - tf.location.x
        dy = self._target_loc.y - tf.location.y
        return math.atan2(dx * right.x + dy * right.y,
                          dx * fwd.x + dy * fwd.y)

    def marking_clearance_m(self, cte: float,
                            marking_width_m: float = 0.125) -> tuple:
        """(left, right) clearance from the front tyre tread to the outer
        edge of the lane marking [m]. Negative = the tread has crossed.

        R79 Annex 8 §3.2.1.2 fails the lane-keeping test if "the outside
        edge of the tyre tread of the vehicle's front wheel crosses the
        outside edge of any lane marking". Markings in OpenDRIVE are
        centred on the lane boundary, so their outer edge is at
        lane_width/2 + marking_width/2 from the centreline.
        """
        edge = (self.site.lane_width_m / 2.0 + marking_width_m / 2.0)
        tread = self._track_half_m + self.TYRE_HALF_WIDTH_M
        return (edge - (tread - cte), edge - (tread + cte))

    # -- steering ground truth ---------------------------------------------
    def curvature_at(self, s: float, window_m: float = 6.0) -> float:
        """Signed curvature of the reference path at arc position `s`
        [1/m], +ve to the right.

        Differentiated over a window rather than between adjacent path
        points: the polyline's per-point heading carries ~0.01 deg of
        quantisation, which over a 1 m step is 1.7e-4 rad/m of noise — a
        30 % error on a 200 m radius. Over 6 m it is negligible.
        """
        n = max(int(window_m / self.PATH_STEP_M / 2), 1)
        i = int(round((s - self._ps[0]) / self.PATH_STEP_M))
        i0 = max(i - n, 0)
        i1 = min(i + n, len(self._path) - 1)
        if i1 <= i0:
            return 0.0
        dyaw = math.degrees(self._pyaw[i1] - self._pyaw[i0])
        dyaw = (dyaw + 540.0) % 360.0 - 180.0
        ds = float(self._ps[i1] - self._ps[i0])
        return math.radians(dyaw) / ds if ds > 0 else 0.0

    def feedforward_steer_rad(self) -> float:
        """Front-wheel angle that holds the lane centreline FROM the
        centreline: the Ackermann angle for the lane's own curvature,
        delta = atan(L * kappa).

        This is the steer a perfect controller would settle at once the
        error is nulled — the curve's demand with no correction in it. Any
        difference between it and the commanded angle is either correction
        or error.
        """
        return math.atan(self.wheelbase_m * self.curvature_at(self._ego_s))

    def reference_steer_rad(self, lookahead_m: float = 10.0) -> float:
        """Front-wheel angle needed RIGHT NOW to be back on the centreline
        and stay there, from where the vehicle actually is.

        Pure pursuit to the point `lookahead_m` ahead on the lane centre:
        delta = atan(2 L sin(alpha) / ld). Geometric, so unlike "what
        Stanley should have commanded" it does not depend on a control
        law, a gain, or a tuning choice — it is the angle that puts the
        vehicle on the centreline at the lookahead point.

        The lookahead is what makes it well posed. There is no unique
        instantaneous "correct" steer for a vehicle that is off-centre:
        recovering in 1 m and recovering in 50 m are both correct, and
        they need different angles. `lookahead_m` names the horizon, and
        the harness records it with the run.
        """
        tf = self._ego_tf()
        s, _, _ = self._project(tf.location.x, tf.location.y)
        target = self._wp_at(s + lookahead_m).transform.location
        fwd, right = tf.get_forward_vector(), tf.get_right_vector()
        dx = target.x - tf.location.x
        dy = target.y - tf.location.y
        x = dx * fwd.x + dy * fwd.y          # forward
        y = dx * right.x + dy * right.y      # right, +ve = target is right
        ld = math.hypot(x, y)
        if ld < 1e-3:
            return 0.0
        return math.atan2(2.0 * self.wheelbase_m * y, ld * ld)

    def wheel_steer_rad(self) -> float:
        """Realised front-wheel angle, averaged across the axle.

        The commanded steer is a normalised position request; CARLA's
        steering has its own dynamics, so what the tyres actually do is a
        third trace and not a redundant one.
        """
        try:
            carla = self._carla
            fl = self.ego.get_wheel_steer_angle(
                carla.VehicleWheelLocation.FL_Wheel)
            fr = self.ego.get_wheel_steer_angle(
                carla.VehicleWheelLocation.FR_Wheel)
            return math.radians((fl + fr) / 2.0)
        except Exception:
            return float('nan')

    def poll_imu(self) -> list:
        """Drain the IMU queue: [(t, ay, yaw_rate, ax), ...].

        CARLA's IMU reports in the sensor frame, which is the vehicle's:
        +y is right, so ay is already the lateral acceleration R79 asks
        for, at the sensor's mounting point rather than the centre of
        gravity. The sensor is attached at the actor origin, which for
        CARLA vehicles is the bounding-box centre at ground level — about
        0.6 m below the CoG, which adds a roll-rate term the trace records
        but the compliance figure does not correct for. Documented rather
        than silently ignored; a roll-corrected figure would need CARLA to
        expose the CoG height, which it does not.
        """
        out = []
        while not self._imu_queue.empty():
            m = self._imu_queue.get()
            out.append((m.timestamp, float(m.accelerometer.y),
                        float(m.gyroscope.z), float(m.accelerometer.x)))
        return out

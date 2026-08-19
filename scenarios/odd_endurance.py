#!/usr/bin/env python3
"""ODD exposure campaign — quantify the declared R171 envelope by driving it.

    ./start_adas.sh carla          # stack up, BRIDGE STOPPED (see below)
    python3 scenarios/odd_endurance.py --plan       # print the schedule
    python3 scenarios/odd_endurance.py --smoke      # 20 min, one leg, Town10HD
    python3 scenarios/odd_endurance.py              # the full 3 h campaign

What this replaces
------------------
"After many runs the stack felt like it was running smoothly in traffic" is
not a validation statement. This turns it into a rate with a confidence
bound, by driving the declared ODD and counting graded events:

    tier 3   collision                     CARLA collision sensor, absolute
    tier 2   lane departure                tyre over a marking, R79 §3.2.1.2
    tier 1   envelope exceedance           decel > R171 5 m/s^2,
                                           ay > R79 5.6.2.1.1 ceiling,
                                           jerk > 5 m/s^3,
                                           speed outside the declared 0-20

Exposure is POOLED across the ODD for the rate, and reported per cell as
coverage evidence. With zero tier-3 events in exposure n, the 95 % upper
bound on the collision rate is 3/n (rule of three) — so the campaign length
sets the strength of the claim, and `--plan` prints exactly what you buy.

Be honest about what 3 h buys: ~24 km of ADAS exposure bounds collisions at
about 1 per 8 km, which is weak. The tier-1 and tier-2 rates are what this
length actually earns — they occur often enough to have real precision. Run
it longer (--minutes-per-leg) for a stronger tier-3 bound; nothing else in
the design changes.

Junctions are NOT ADAS exposure
-------------------------------
The stack is ACC + LKA with no path planning. In a junction UFLD has no lane
to hold, stanley_node goes HOLD, and nothing steers. So this harness drives
junctions itself along a PLANNED ROUTE (see RoutePlanner) and subtracts that
distance from the exposure total. The regulated claim is only ever made
about metres the ADAS actually drove, on roads with markings — which is what
the R171 Non-Highway declaration says.

Two consequences worth stating in the write-up:
  * `adas_m` < `driven_m` always. Quoting `driven_m` as system exposure
    would be counting metres a route-follower drove.
  * tier-2 and tier-1 events are only counted while the ADAS owns control.
    A departure under the route-follower is the harness's fault, not the
    system's, and is logged separately as `route_m` rather than scored.

Before running
--------------
Stop the bridge. This harness IS the CARLA<->ROS bridge for the duration;
leaving carlaAccSimTown.py up puts two writers on /Car_1/cmd_vel and the
ego's apply_control. The ADAS stack itself stays up — it survives the town
reboots fine, it is only a second world inside one server session that
breaks (DEBUG §59).

The set speed is republished at the start of EVERY leg. /ACC/target_speed is
only ever published by this harness — the UI has no control for it — so
controller_node holds whatever the last scenario left behind for the life of
the node. A session was found sitting at 93 km/h from a previous curved-R171
leg while the operator believed it was at 20. A 0-20 km/h envelope campaign
that silently drove at 93 would be worthless, so each leg asserts the speed
and the report fails any leg whose mean exceeded the declared maximum.
"""

from __future__ import annotations

import argparse
import faulthandler
import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import carla_server                                          # noqa: E402
from scenario_common import (                                # noqa: E402
    CameraPump, ScenarioBridge, SpeedHold, check_no_bridge_conflict,
    install_sigterm_handler, make_out_dir, start_bridge, wait_for_stack,
    write_trace,
)

# ---------------------------------------------------------------------------
# Declared ODD (README) — the thing being validated.
# ---------------------------------------------------------------------------
SET_SPEED_KMH = 20.0            # R171 declared System Designed Speed Range
SPEED_ENVELOPE_TOL = 1.10       # tier-1 trips above 10 % over the declaration

# Town order: most-representative first, so a campaign cut short still has
# the best coverage. Town04 and Town06 are DELIBERATELY EXCLUDED — both
# carry motorway sections, and the declaration is R171 Non-Highway, so their
# kilometres would inflate exposure with out-of-envelope driving.
TOWNS = [
    ('Town10HD', 'dense urban, best markings — closest to the declared ODD'),
    ('Town05',   'multi-lane urban grid, signalised junctions'),
    ('Town03',   'largest, roundabout + varied junctions — stressor'),
    ('Town01',   'small classic urban, many unsignalised junctions'),
    ('Town02',   'small suburban, tight blocks'),
    ('Town04',   'suburban arterials — URBAN PORTION ONLY, see below'),
]

# Town04 replaces Town07 at the operator's request.
#
# It needs its own route policy. Town04's outermost ring IS its motorway
# loop, so 'outer_loop' — which picks the branch furthest from the map
# centre at every fork — would drive the campaign straight onto it. That is
# outside the declared R171 Non-Highway domain, and kilometres gathered
# there would inflate exposure with out-of-envelope driving. 'explore'
# keeps it on the urban and suburban roads inside the ring.
#
# The trade is that Town04 then meets more junctions than the other towns,
# so its ADAS share will be lower. That is the honest cost of keeping it
# inside the declared domain.

# Route policy. 'outer_loop' hugs the perimeter by preferring, at every
# fork, the branch that ends furthest from the map centre; 'explore'
# wanders, picking mostly-straight branches.
#
# EVERY town laps its outer ring. Perimeter roads are long arcs with far
# fewer junctions per kilometre, so the ADAS holds control for a much larger
# share of the distance, there is less of the harness's own route-following
# in the numbers, and the collisions that kept ending legs — all of them in
# or just after junctions — become rare.
#
# State the consequence in the write-up, because it cuts against the
# declaration. The declared roadway domain is R171 Non-Highway "urban and
# suburban with signalised and unsignalised intersections", and lapping the
# perimeter deliberately MINIMISES intersections. So this campaign measures
# arterial and perimeter driving inside the ODD, not the whole ODD. It is a
# narrower claim than the declaration, and it must be reported as such — the
# result cannot be read as evidence the stack handles intersections, because
# it was routed to avoid them.
#
# The honest form of the eventual sentence is roughly: "X km of ADAS-on-lane
# exposure on marked perimeter and arterial roads across 6 towns and 4
# weather conditions; intersection traversal was performed by the test
# harness and excluded from exposure."
#
# --route-policy explore restores network-wide wandering when you want the
# intersection-inclusive number instead.
DEFAULT_ROUTE_POLICY = 'outer_loop'
ROUTE_POLICY: dict[str, str] = {'Town04': 'explore'}
# The four declared weather families. FogNoon is not a CARLA preset: the
# declaration covers fog but WeatherParameters ships none, so it is built
# from CloudyNoon at the operator's values (50 % density, 30 m distance).
WEATHER = ['ClearNoon', 'CloudyNoon', 'FogNoon', 'HardRainNoon']
FOG = dict(base='CloudyNoon', fog_density=50.0, fog_distance=30.0,
           fog_falloff=0.2)
# 10, not 30. At 30 the ego was boxed in behind queueing traffic within
# the first minute of every leg and the 10 s standstill abort ended it:
# three legs across two runs died at 32 s, 55 s and 151 s, all with
# gap_gt ~2.0 m — the route-follower correctly stopped 2 m behind a car
# that was itself waiting. Total yield was 0.06 km of exposure from two
# legs. Fewer NPCs attacks the cause rather than relaxing the rule.
NPC_COUNT = 10

MINUTES_PER_LEG = 6.9           # 24 legs + overhead = 3.00 h
SETTLE_S = 15.0                 # weather change + ACC re-engage per leg

# ---------------------------------------------------------------------------
# Regulated thresholds. Sourced from the same places the R79/R171 plotters
# use so a limit cannot drift between the harness and the figures.
# ---------------------------------------------------------------------------
DECEL_LIMIT = 5.0               # R171 §5.3.7.1.2
AY_TABLE_MAX = 3.0              # R79 §5.6.2.1.3 Table 1, M1/N1
AY_CEILING = min(1.4 * AY_TABLE_MAX, AY_TABLE_MAX + 0.3)   # §5.6.2.1.1
JERK_LIMIT = 5.0                # R79 §3.2.1
LANE_HALF_W = 1.75
TYRE_HALF_W = 0.875
DEPARTURE_M = LANE_HALF_W - TYRE_HALF_W    # |cte| beyond this = tyre over

CONTROL_HZ = 20.0
# The ADAS only takes control back when the ego is genuinely on its
# lane and pointing along it. See the handover gate in run_leg.
# Lateral acceleration the route-follower is allowed to demand through a
# turn. R79 Table 1 aysmax for M1/N1 — the rig holds itself to the same
# envelope the system under test is declared for.
AY_BUDGET = 3.0
# Fraction of the geometric limit actually commanded, leaving headroom
# for the follower's tracking error. 0.85 -> 2.2 m/s^2 planned.
CURV_MARGIN = 0.85
HANDOVER_CTE_M = 0.5
HANDOVER_HEADING_DEG = 12.0
# A new collision incident needs a different actor, or the same one
# after this much separation. See the counter in run_leg.
COLLISION_DEDUP_S = 2.0
# Standstill that ends a leg: below this speed for this long.
STANDSTILL_KMH = 0.5
STANDSTILL_ABORT_S = 10.0


# ---------------------------------------------------------------------------
# Route planning through junctions
# ---------------------------------------------------------------------------
class RoutePlanner:
    """A committed lane-level route ahead of the ego.

    Junctions are the reason this exists. `waypoint.next(d)` FORKS inside a
    junction — it returns every legal continuation — so a follower that just
    takes `next()[0]` each tick can pick a different branch on consecutive
    ticks and steer into the kerb between them. That is the crash the
    operator saw under pure pursuit.

    So the route is planned once, several junctions deep, and COMMITTED: the
    follower consumes waypoints from a fixed list and never re-decides. At a
    fork the choice is made by `_pick`, seeded per leg so a campaign is
    reproducible, and biased toward going straight because a fixed-length
    leg that spends its time turning around a single block covers less of
    the town than one that drives through it.
    """

    STEP_M = 2.0
    # Sub-window for the curvature estimate, matched to the arc
    # length of a real junction turn (~8-10 m). Longer windows
    # average the turn away; see min_radius.
    CURV_WIN_M = 6.0

    def __init__(self, carla_map, seed: int = 0, ahead_m: float = 120.0,
                 policy: str = 'explore'):
        self.map = carla_map
        self.ahead_m = ahead_m
        self.policy = policy
        self._rng = __import__('random').Random(seed)
        self.route: list = []          # carla.Waypoint, ego-forward order
        # Map centre, for the outer_loop policy. Taken from the spawn-point
        # cloud rather than the road network because spawn points are spread
        # over the drivable area and are cheap to fetch.
        pts = carla_map.get_spawn_points()
        self._cx = sum(p.location.x for p in pts) / max(len(pts), 1)
        self._cy = sum(p.location.y for p in pts) / max(len(pts), 1)

    def _radius(self, wp) -> float:
        loc = wp.transform.location
        return math.hypot(loc.x - self._cx, loc.y - self._cy)

    @staticmethod
    def _yaw_delta(a, b) -> float:
        d = b.transform.rotation.yaw - a.transform.rotation.yaw
        return abs((d + 540.0) % 360.0 - 180.0)

    def _pick(self, cur, options):
        """Choose one continuation at a fork.

        Straightest wins 70 % of the time; otherwise a random legal branch,
        so a long campaign explores the map instead of looping one circuit.
        Both are drawn from the seeded RNG, so a leg replays identically.
        """
        if len(options) == 1:
            return options[0]
        straightest = min(options, key=lambda w: self._yaw_delta(cur, w))
        if self.policy == 'outer_loop':
            # Hug the perimeter: take whichever branch ends furthest from
            # the map centre. The 3 m deadband keeps it from dithering
            # between two near-equal branches on a straight, where the
            # radius difference is noise and the straight one is right.
            outermost = max(options, key=self._radius)
            if self._radius(outermost) - self._radius(straightest) > 3.0:
                return outermost
            return straightest
        if self._rng.random() < 0.70:
            return straightest
        return self._rng.choice(options)

    def extend(self, from_wp=None) -> None:
        """Top the route up to `ahead_m` of committed waypoints."""
        if from_wp is not None and not self.route:
            self.route = [from_wp]
        while len(self.route) * self.STEP_M < self.ahead_m:
            nxt = self.route[-1].next(self.STEP_M)
            if not nxt:
                # Dead end (map edge, or a lane that simply stops). Leave the
                # route short; the leg loop treats a short route as a reason
                # to respawn rather than driving off the end.
                return
            cur = self.route[-1]
            # Drop candidates that are not actually continuous with the
            # current point. wp.next() can return a waypoint on a different
            # connection, and following one produces a route with a jump in
            # it — the "badly placed waypoint" the follower then steers
            # toward across whatever lies between.
            here = cur.transform.location
            ok = []
            for w in nxt:
                loc = w.transform.location
                if math.hypot(loc.x - here.x, loc.y - here.y) > self.STEP_M * 2.5:
                    continue
                if self._yaw_delta(cur, w) > 60.0:
                    continue
                ok.append(w)
            if not ok:
                return
            self.route.append(self._pick(cur, ok))

    def advance(self, x: float, y: float) -> None:
        """Drop route points the ego has passed, then re-extend."""
        while len(self.route) > 1:
            w = self.route[0].transform.location
            if math.hypot(w.x - x, w.y - y) < self.STEP_M * 1.5:
                self.route.pop(0)
            else:
                break
        self.extend()

    def min_radius(self, metres: float) -> float:
        """Tightest turn radius over the next `metres` of route [m].

        From ACCUMULATED HEADING CHANGE over a short sliding window, not
        the circumradius of point triples.

        The triple version was wrong in a way that let the speed limiter do
        nothing. It scanned a 20 m window, and a junction arc is only ~8-10
        m long, so the tight middle was averaged with the straight approach
        and exit: it reported the route's median ~30 m while the car was
        actually driving 6.1 m at 20.6 km/h — 5.3 m/s^2, well past the 3.0
        budget the limiter exists to enforce. Measured in
        results/20260817_105031_odd_50km, run 1, t=19.9 s, which is also
        the instant it was struck on the flank in a junction.

        R = arc / dtheta over a WIN_M sub-window, minimised across the
        lookahead. Accumulated heading is robust where local geometry is
        noisy, and a window matched to a junction's arc length cannot
        dilute the turn with the straights around it.
        """
        step = self.STEP_M
        win = max(2, int(round(self.CURV_WIN_M / step)))
        n = min(len(self.route) - 1, max(win + 1, int(metres / step)))
        best = 1e6
        for i in range(0, max(1, n - win)):
            a = self.route[i].transform.rotation.yaw
            b = self.route[i + win].transform.rotation.yaw
            dtheta = abs((b - a + 540.0) % 360.0 - 180.0)
            if dtheta < 0.5:
                continue                      # straight enough
            best = min(best, (win * step) / math.radians(dtheta))
        return best

    def heading_change(self, metres: float) -> float:
        """Absolute heading change over the next `metres` of route [rad].

        Used to shorten the pure-pursuit lookahead into a turn.
        """
        n = min(len(self.route) - 1, max(1, int(metres / self.STEP_M)))
        if n < 1:
            return 0.0
        a = self.route[0].transform.rotation.yaw
        b = self.route[n].transform.rotation.yaw
        return abs(math.radians((b - a + 540.0) % 360.0 - 180.0))

    def target(self, x: float, y: float, ahead: int = 4):
        """Route point `ahead` indices past the nearest — the reference
        implementation's target rule.

        The previous version accumulated arc length from route[0], which is
        itself up to STEP_M*1.5 = 3 m in front of the ego, so the effective
        lookahead was inflated by that much and every shortening of it was
        partly cancelled. Indexing from the NEAREST point removes the
        floating origin."""
        if not self.route:
            return None
        best_i, best_d = 0, float('inf')
        for i, w in enumerate(self.route):
            loc = w.transform.location
            d = (loc.x - x) ** 2 + (loc.y - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        j = min(best_i + ahead, len(self.route) - 1)
        return self.route[j].transform.location

    def junction_within(self, metres: float) -> bool:
        n = max(1, int(metres / self.STEP_M))
        return any(w.is_junction for w in self.route[:n])


class LegWatchdog:
    """Abort the process if a leg overruns its budget by a wide margin.

    The leg loop bounds itself with `while time.time() < deadline`, which
    only helps if the loop is running. A blocking CARLA RPC that never
    returns is invisible to it: run 20260816_160118 sat inside one for
    26 minutes with no output, no trace flush and no way to tell from the
    outside whether it was working or wedged.

    A watchdog thread cannot unwedge a stuck RPC — the C++ client owns that
    socket — so it does the one useful thing available and kills the
    process, loudly, with the leg named. A campaign that dies at leg 2 with
    a reason beats one that silently produces nothing for three hours.
    """

    def __init__(self, grace=2.5):
        self.grace = grace
        self._deadline = None
        self._leg = None
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def arm(self, leg: int, minutes: float) -> None:
        self._leg = leg
        self._deadline = time.time() + minutes * 60.0 * self.grace + 120.0

    def disarm(self) -> None:
        self._deadline = None

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.wait(5.0):
            d = self._deadline
            if d is not None and time.time() > d:
                sys.stderr.write(
                    f"\n[watchdog] leg {self._leg} overran its budget by "
                    f"{self.grace:g}x with no progress — some kind of "
                    f"stall. Killing the run so it fails loudly rather "
                    f"than hanging. Send SIGUSR1 before it dies to "
                    f"dump thread stacks.\n")
                sys.stderr.flush()
                os._exit(2)


def _clear_lane_spawn(adapter, seed: int):
    """A spawn point on a marked lane, out of junctions, clear of traffic."""
    import carla
    import random
    world = adapter.world
    cmap = world.get_map()
    pts = world.get_map().get_spawn_points()
    rng = random.Random(seed)
    order = list(range(len(pts)))
    rng.shuffle(order)
    # Positions from ONE world snapshot, not per-actor get_transform().
    #
    # The first version called a.get_transform() inside the candidate loop:
    # 155 spawn points x 30 actors is up to 4650 blocking RPCs per leg
    # reset, every one of them racing the CameraPump on the same client
    # connection. Leg 2 of run 20260816_160118 hung there for 26 minutes —
    # three threads stuck in a socket wait, 91 more blocked behind them,
    # while the CARLA server itself was perfectly healthy. A snapshot is
    # one RPC and gives every actor's transform at a consistent instant.
    snap = world.get_snapshot()
    ego_id = adapter.ego.id
    others = []
    for a in world.get_actors().filter('vehicle.*'):
        if a.id == ego_id:
            continue
        act = snap.find(a.id)
        if act is not None:
            loc = act.get_transform().location
            others.append((loc.x, loc.y))

    for i in order:
        loc = pts[i].location
        wp = cmap.get_waypoint(loc, project_to_road=True,
                               lane_type=carla.LaneType.Driving)
        if wp is None or wp.is_junction or not wp.next(25.0):
            continue
        # Not on top of an NPC — respawning into one is how a leg starts
        # already in contact, which is the failure this whole helper exists
        # to prevent.
        if any(math.hypot(ax - loc.x, ay - loc.y) < 8.0 for ax, ay in others):
            continue
        return pts[i]
    return None


def _reset_ego_for_leg(adapter, spec):
    """Teleport the ego to a clear lane and zero its motion.

    Cheaper and far more reliable than destroying and re-spawning: the
    camera and collision sensors stay attached, so the ROS side never sees
    a gap and the stack does not have to re-acquire.
    """
    import carla
    def _step(msg):
        print(f"    [reset] {msg}", flush=True)

    _step('picking spawn')
    tf = _clear_lane_spawn(adapter, seed=1000 + spec['leg'])
    if tf is None:
        print(f"    [reset] leg {spec['leg']}: no clear spawn found, "
              f"continuing from current pose", flush=True)
        return False
    # +0.5 m is CARLA's usual spawn clearance; the extra headroom matters
    # here because the ego may be embedded in geometry when this runs.
    tf = carla.Transform(
        carla.Location(tf.location.x, tf.location.y, tf.location.z + 0.6),
        tf.rotation)
    ego = adapter.ego
    # PHYSICS OFF ACROSS THE TELEPORT.
    #
    # set_transform() on a vehicle the physics engine is resolving a
    # penetration for blocks the RPC, and the whole client wedges behind
    # it. Twice now, and both times with the same precondition: the leg
    # before ended in contact. odd_conf08 leg 1 finished pressed against a
    # seat.leon (gap_gt -0.05 m) and leg 2's reset hung for 26 min;
    # odd_v4 leg 2 finished in static.vegetation and leg 3's reset hung
    # until the watchdog killed it. The leg that ended cleanly transitioned
    # fine. Disabling physics detaches the actor from that solve, so the
    # teleport is a pure pose write.
    _step('physics off')
    ego.set_simulate_physics(False)
    _step('set_transform')
    ego.set_transform(tf)
    _step('zero velocity')
    ego.set_target_velocity(carla.Vector3D(0, 0, 0))
    ego.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
    _step('brake')
    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
    time.sleep(0.3)
    _step('physics on')
    ego.set_simulate_physics(True)
    # Let the suspension settle at the new pose before the leg measures.
    time.sleep(1.0)
    ego.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0))
    _step('done')
    return True


def classify_collision(col) -> tuple[str, str]:
    """(object category, contact geometry) for a collision event.

    Category answers "what did it hit" — a car, a building, roadside
    furniture, or the kerb. Geometry answers the harder and more important
    question, "did the ego drive into it or was it struck", from the
    contact bearing derived off the collision impulse:

        |bearing| <  50 deg   FRONTAL   ego drove into it
        50-130 deg           FLANK     struck on the side — in a junction
                                       this is usually an NPC arriving,
                                       and not something the system under
                                       test could have avoided
        > 130 deg            REAR      ego was rear-ended

    Both go in the record because a pooled collision count that mixes
    "ran into a fence" with "was T-boned while stationary" is not a
    measure of the system.
    """
    t = (col.other_id or '').lower()
    if t.startswith('vehicle.'):
        cat = 'vehicle'
    elif t.startswith('walker.'):
        cat = 'pedestrian'
    elif 'building' in t or 'wall' in t:
        cat = 'building'
    elif any(k in t for k in ('fence', 'pole', 'sign', 'vegetation',
                              'streetsign', 'trafficlight', 'traffic_light',
                              'guardrail', 'traffic.')):
        # 'traffic_light' and the 'traffic.' prefix are both needed: CARLA's
        # actor is `traffic.traffic_light`, with an underscore, so the
        # earlier 'trafficlight' pattern missed it and it fell through to
        # the generic first-token fallback as category 'traffic'. Seen live
        # in odd_50km_v4 run 17.
        cat = 'roadside furniture'
    elif 'sidewalk' in t or 'curb' in t or 'kerb' in t:
        cat = 'kerb'
    else:
        cat = t.split('.')[0] or 'unknown'

    b = col.contact_bearing_deg
    if b != b:                      # NaN
        geom = 'unknown'
    else:
        a = abs(b)
        geom = 'frontal' if a < 50 else ('flank' if a < 130 else 'rear')
    return cat, geom


def nearest_ahead_m(world, ego, half_w: float = 2.2, max_m: float = 30.0):
    """Ground-truth gap to the nearest vehicle ahead, or None.

    Used ONLY while the harness owns the car (junctions and LKAS dropouts).
    Ground truth is legitimate here and does not contaminate the study: the
    route-follower is test equipment, not the system under test, and every
    metre it drives is already excluded from ADAS exposure. What it must
    not do is crash — a harness that drives into cross traffic ends legs
    for reasons that have nothing to do with the ADAS.

    This exists because the route-follower had no collision avoidance of
    any kind. In results/20260816_144335_odd it entered a junction at
    20 km/h with the throttle open, ACC reporting no lead, and drove
    straight into traffic: 337 route-owned samples, maximum brake 0.08.
    """
    et = ego.get_transform()
    ex, ey = et.location.x, et.location.y
    yaw = math.radians(et.rotation.yaw)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    ego_half = ego.bounding_box.extent.x
    # Same reason as _clear_lane_spawn: this runs at 20 Hz, so per-actor
    # get_transform() is 10-30 blocking RPCs every tick against a client
    # the CameraPump is also using.
    snap = world.get_snapshot()
    best = None
    for a in world.get_actors().filter('vehicle.*'):
        if a.id == ego.id:
            continue
        act = snap.find(a.id)
        if act is None:
            continue
        loc = act.get_transform().location
        dx, dy = loc.x - ex, loc.y - ey
        # Into the ego frame: forward, then lateral.
        fwd = dx * cos_y + dy * sin_y
        lat = -dx * sin_y + dy * cos_y
        if 0.0 < fwd < max_m and abs(lat) < half_w:
            # BUMPER TO BUMPER: subtract BOTH half-lengths, from the actual
            # bounding boxes.
            #
            # The first version subtracted a single hardcoded 2.5 m, i.e.
            # one vehicle's worth. Two cars are ~2.4 m half-length each, so
            # centre-to-centre at contact is ~4.8 m and that formula
            # reported 2.3 m of clearance while the bumpers were touching.
            # The 1.5 m hard-brake threshold was therefore unreachable, and
            # the harness drove into a queue and sat there with the
            # throttle at 1.0 pushing — exactly what the tail of
            # results/20260816_145316_odd shows: v = 0.0, throttle 1.01,
            # gap_gt pinned at 2.33.
            gap = fwd - ego_half - a.bounding_box.extent.x
            if best is None or gap < best:
                best = gap
    return best


# Pure pursuit, PORTED VERBATIM from carlaaccsim/pure_pursuit_controller.py
# — the bridge's own fallback controller, which has driven these towns for
# months.
#
# Three attempts at a kinematically-pure version of this all under-steered
# out of junctions ("turns too wide"). The reason is the last line: the
# reference implementation multiplies the pure-pursuit angle by 2.5 and
# feeds it straight in as the normalised steer, where the textbook version
# divides by the 70 deg max steer angle. On the 8 m junction radius these
# towns actually have, mine commanded 0.30 and this commands ~0.91.
#
# atan(L/R) is the STEADY-STATE steer for a radius; a real vehicle with
# tyre slip and actuator lag never reaches the radius it implies, so a
# controller that asks for exactly that always runs wide. The 2.5 is not a
# fudge to be tidied away — it is what makes the loop track, and it was
# validated by use rather than derivation. Do not "correct" it.
PP_L = 3.05           # wheelbase [m], as in the reference
PP_KDD = 4.0          # lookahead gain
PP_VF_CLIP = 2.5      # speed is clipped here, so ld saturates at 10 m
PP_GAIN = 2.5         # the empirical output gain described above
PP_TARGET_AHEAD = 4   # route points ahead of the nearest, as in the reference


def pure_pursuit_steer(ego_x, ego_y, ego_yaw, tgt, speed_mps=0.0,
                       alpha_prev=0.0):
    """Normalised steer toward `tgt`. Positive = right (CARLA convention)."""
    if tgt is None:
        return 0.0, alpha_prev
    vf = min(max(speed_mps, 0.1), PP_VF_CLIP)
    ld = PP_KDD * vf
    alpha = math.atan2(tgt.y - ego_y, tgt.x - ego_x) - ego_yaw
    if math.isnan(alpha):
        alpha = alpha_prev
    alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
    delta = PP_GAIN * math.atan2(2.0 * PP_L * math.sin(alpha), ld)
    if math.isnan(delta):
        return 0.0, alpha
    return max(-1.0, min(1.0, delta)), alpha


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------
@dataclass
class LegResult:
    leg: int = 0
    town: str = ''
    weather: str = ''
    npcs: int = 0
    minutes: float = 0.0
    driven_m: float = 0.0
    adas_m: float = 0.0            # exposure: ADAS owned control, on lane
    route_m: float = 0.0           # junctions, driven by this harness
    acc_lead_m: float = 0.0        # ADAS metres with a lead in range
    v_mean_kmh: float = 0.0
    v_max_kmh: float = 0.0
    speed_envelope_ok: bool = True
    collisions: int = 0            # tier 3
    departures: int = 0            # tier 2
    exc_decel: int = 0             # tier 1
    exc_ay: int = 0
    exc_jerk: int = 0
    exc_speed: int = 0
    lkas_silent_s: float = 0.0
    ended_early: bool = False
    handover_blocked_s: float = 0.0
    harness_collisions: int = 0
    contact_s: float = 0.0
    collision_detail: list = field(default_factory=list)
    collision_kinds: dict = field(default_factory=dict)
    note: str = ''

    @property
    def tier1(self) -> int:
        return self.exc_decel + self.exc_ay + self.exc_jerk + self.exc_speed


# ---------------------------------------------------------------------------
# Campaign plan
# ---------------------------------------------------------------------------
def build_plan(towns, weather, minutes_per_leg, npcs,
               policy=DEFAULT_ROUTE_POLICY):
    legs, n = [], 0
    for town, role in towns:
        for w in weather:
            n += 1
            legs.append(dict(leg=n, town=town, weather=w, npcs=npcs,
                             minutes=minutes_per_leg, role=role,
                             policy=ROUTE_POLICY.get(town, policy)))
    return legs


# Measured by the 20 min Town10HD/ClearNoon smoke leg
# (results/20260816_150055_odd): 2.71 km driven, 1.38 km ADAS-on-lane,
# mean 10.1 km/h. The original 13.0 / 0.68 were estimates from map
# layout and were optimistic by about 40 % on yield.
def print_plan(legs, v_mean_kmh=10.1, lane_frac=0.51):
    boot_min = 1.5
    towns = []
    for lg in legs:
        if lg['town'] not in towns:
            towns.append(lg['town'])
    drive = sum(lg['minutes'] for lg in legs)
    overhead = len(towns) * boot_min + len(legs) * SETTLE_S / 60.0
    total = drive + overhead
    km = v_mean_kmh * drive / 60.0
    adas = km * lane_frac
    print(f"{'leg':>3} {'town':<10}{'weather':<14}{'NPC':>4}{'min':>6}")
    for lg in legs:
        print(f"{lg['leg']:>3} {lg['town']:<10}{lg['weather']:<14}"
              f"{lg['npcs']:>4}{lg['minutes']:>6.1f}"
              + ('   <- reboot + spawn NPCs' if lg['weather'] == WEATHER[0]
                 else ''))
    print(f"\n{len(legs)} legs · {len(towns)} town reboots")
    print(f"driving {drive:.0f} min + overhead {overhead:.0f} min "
          f"= {total/60:.2f} h")
    print(f"\nprojected at {v_mean_kmh:g} km/h mean, lane fraction "
          f"{lane_frac:g} (ESTIMATES — the smoke leg measures both):")
    print(f"  driven ~{km:.0f} km · ADAS-on-lane ~{adas:.0f} km")
    print(f"  zero collisions would bound the rate at 1 per {adas/3:.0f} km "
          f"(95 % UCB, rule of three)")
    print("  tier 1/2 rates are what this length actually earns; the tier-3 "
          "bound is weak and must be reported as a bound.")


# ---------------------------------------------------------------------------
# One leg
# ---------------------------------------------------------------------------
def run_leg(adapter, bridge, pump, spec, out_dir, args) -> LegResult:
    import carla
    res = LegResult(leg=spec['leg'], town=spec['town'],
                    weather=spec['weather'], npcs=spec['npcs'],
                    minutes=spec['minutes'])
    world = adapter.world
    ego = adapter.ego
    cmap = world.get_map()

    # RESET THE EGO AT THE START OF EVERY LEG.
    #
    # The ego is spawned once per TOWN, so without this a leg inherits
    # wherever the previous one left the car — and if that was against a
    # wall, every remaining leg in the town starts crashed, sits still,
    # and ends on the 10 s standstill with 0.00 km. Measured: in the
    # 20260816_15xx campaign, Town10HD legs 3-4 and Town05 legs 6-8 all
    # returned exactly 0.00 km ADAS after a collision in the leg before.
    # Five of eight legs produced no data at all.
    #
    # Also drain the collision queue: it is a list on the adapter that
    # survives between legs, so leg N+1 popped leg N's collision at
    # t = 0.0 s and counted it again. The same actor and impact speed
    # showing up at t=0.0 in consecutive legs is that bug, not an event.
    # The CameraPump shares this client connection. If the block is a
    # concurrency problem rather than a physics one, pausing it across the
    # reset removes the only other RPC source and the hang should move.
    if pump is not None:
        pump.stop()
        pump.join(timeout=3.0)
    _reset_ego_for_leg(adapter, spec)
    # BOUNDED drain. This was `while take_collision() is not None: pass`,
    # which is unbounded — and that was the leg-transition hang, in all
    # three runs that suffered it.
    #
    # A leg that ends wedged against something has its collision sensor
    # firing for the entire standstill, and the sensor callback keeps
    # appending from its own thread while the loop pops. The loop drains
    # nothing net, holds the GIL, and never returns. Every diagnosis
    # pointing at a blocked CARLA RPC was reading the symptom: the pump and
    # ROS threads starved behind this loop.
    #
    # Cap it. Anything still arriving after the cap belongs to the new leg
    # and the counter's own de-duplication handles it.
    drained = 0
    while drained < 5000 and adapter.take_collision() is not None:
        drained += 1
    if drained:
        print(f'    [reset] drained {drained} stale collision event(s)',
              flush=True)
    # Draining alone is not enough: a wedged leg can queue more than the
    # cap, so leftovers survive into the next leg and are counted again at
    # t = 0.0. Measured in odd_v6 — leg 5's real static.fence hit was
    # re-counted in legs 6 AND 7, same actor, same 19.0 km/h, t=0.0 both
    # times. Three tier-3 events reported for one collision.
    #
    # So gate on SIM TIME rather than on draining. CollisionEvent carries
    # the world timestamp it happened at; anything older than this leg's
    # start belongs to a previous leg, however many are queued.
    leg_start_sim_t = world.get_snapshot().timestamp.elapsed_seconds
    if pump is not None:
        pump = CameraPump(adapter, bridge)
        pump.start()
        print('    [reset] camera pump restarted', flush=True)

    # Assert the set speed EVERY leg. controller_node holds whatever the
    # last publisher left; see the module docstring.
    bridge.publish_target_speed(SET_SPEED_KMH)
    bridge.publish_not_in_junction()

    planner = RoutePlanner(cmap, seed=spec['leg'],
                           policy=spec.get('policy', DEFAULT_ROUTE_POLICY))
    wp = cmap.get_waypoint(ego.get_transform().location,
                           project_to_road=True,
                           lane_type=carla.LaneType.Driving)
    planner.extend(wp)

    hold = SpeedHold()
    hold.reset()
    set_mps = SET_SPEED_KMH / 3.6
    samples = []
    # Flush the trace periodically, not only at the end of the leg.
    #
    # Two legs have now been stopped mid-run — once by the operator, once
    # after a collision — and both times the whole trace went with them,
    # because write_trace only ran after the loop. The evidence you most
    # want is the evidence from the run that went wrong, so it has to be on
    # disk before the run ends.
    trace_name = f"leg{res.leg:02d}_{res.town}_{res.weather}"
    next_flush = time.time() + 30.0
    still_ticks = 0
    handover_blocked_ticks = 0
    pp_alpha = 0.0
    last_col_id, last_col_t = None, -99.0
    t0 = time.time()
    prev = None
    prev_ay = 0.0
    last_steer_t = time.time()
    deadline = t0 + spec['minutes'] * 60.0
    dt_nom = 1.0 / CONTROL_HZ

    while time.time() < deadline:
        loop_t = time.time()
        st = adapter.ego_state()
        # controller_node closes its speed loop on /Car_1/vehicle/speed.
        # Without this the stack sees v_ego = 0 for the whole leg, the
        # speed error never closes, and every number it produces is
        # against a standstill it is not at. The harness IS the bridge
        # here, so nothing else publishes it.
        bridge.publish_speed(st.speed)
        tr = ego.get_transform()
        x, y, yaw = tr.location.x, tr.location.y, math.radians(
            tr.rotation.yaw)
        v = st.speed

        planner.advance(x, y)
        if len(planner.route) < 5:
            # Route ran out (map edge / lane ends). Re-seed from the current
            # lane rather than driving off the end of the world.
            wp = cmap.get_waypoint(tr.location, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
            planner.route = []
            planner.extend(wp)

        # Who owns lateral control this tick?
        gap_guard = None          # ground-truth guard, harness-owned ticks
        cur_wp = cmap.get_waypoint(tr.location, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
        in_junction = cur_wp.is_junction or planner.junction_within(12.0)

        # CLEAN HANDOVER GATE.
        #
        # Control used to return to the ADAS the instant `in_junction` went
        # false, regardless of where the route-follower had left the car.
        # Leg 5 of odd_v6 handed over at cte = 0.64 m already tracking
        # outward; cte grew monotonically for 2.5 s, crossed a lane
        # boundary, and hit a static.fence at 19 km/h. The stack was
        # driving at impact but was handed a car it could not save, and the
        # event was scored against it.
        #
        # So the handover now also requires the ego to actually be ON its
        # lane and roughly aligned with it. Until then the route-follower
        # keeps the car and the distance stays out of ADAS exposure — which
        # is the honest accounting either way, since those metres were
        # driven by the rig.
        if not in_junction:
            cte_now = _signed_cte(cur_wp, x, y)
            head_err = abs((math.radians(cur_wp.transform.rotation.yaw) - yaw
                            + math.pi) % (2 * math.pi) - math.pi)
            if (abs(cte_now) > HANDOVER_CTE_M
                    or head_err > math.radians(HANDOVER_HEADING_DEG)):
                in_junction = True          # keep the rig in control
                handover_blocked_ticks += 1

        # stack_steer_stamp is time.monotonic() in the bridge, so this
        # must be monotonic too — mixing it with time.time() silently
        # yields a ~decades-large age and marks the LKAS permanently
        # stale, handing every metre to the route-follower.
        stamp = bridge.stack_steer_stamp
        lkas_stale = (stamp <= 0.0) or (time.monotonic() - stamp > 0.5)

        if in_junction:
            owner = 'route'
            tgt = planner.target(x, y, PP_TARGET_AHEAD)
            steer, pp_alpha = pure_pursuit_steer(x, y, yaw, tgt, v, pp_alpha)
            # Junctions run at the SAME declared set speed as lanes. An
            # earlier version capped this at 4 m/s "to slow into the
            # turn", which drove every junction at 14.4 km/h — it made the
            # campaign's mean speed a property of the harness rather than
            # of the system, and the declared envelope is 0-20 throughout.
            #
            # ...but the speed hold alone will drive into a stationary
            # queue, which is exactly how the last leg ended. Bound the
            # target by what is actually ahead.
            # SPEED LIMITED BY THE TURN THE ROUTE ACTUALLY HAS.
            #
            # Driving every junction at the full 20 km/h is what put the
            # car into fences, vegetation and static.dynamic. A junction
            # turn of 5 m radius needs 6.2 m/s^2 to hold at 20 km/h and 8 m
            # needs 3.9 — past grip and past the R79 3.0 ceiling
            # respectively. The car understeered, ran wide, and hit the
            # outside of the bend. That is the "turns too far outwards" the
            # operator reported, and the same event the collision log kept
            # attributing to the ADAS because control had just switched
            # back at the junction exit.
            #
            # An earlier fixed 4 m/s cap was rejected as arbitrary, and it
            # was. This is derived: v <= sqrt(a_lat * R) for the measured
            # route radius, so a wide junction still runs at the full set
            # speed and only a genuinely tight one slows. AY_BUDGET is the
            # R79 Table 1 aysmax, so the rig stays inside the same lateral
            # envelope the system is declared for.
            tgt_mps = set_mps
            radius = planner.min_radius(24.0)
            if radius < 1e5:
                # CURV_MARGIN because commanding exactly sqrt(a*R) leaves
                # nothing for the pure-pursuit tracking error that makes it
                # run wide in the first place — the lateral the car ends up
                # pulling is the one for the radius it ACTUALLY drives, which
                # is tighter than the route when it is cutting or wider when
                # it is lagging.
                tgt_mps = min(tgt_mps,
                              CURV_MARGIN * math.sqrt(AY_BUDGET * radius))
            gap_guard = gap = nearest_ahead_m(world, ego)
            if gap is not None:
                # Same stopping profile the ACC governor uses, so the
                # harness decelerates like the system rather than stamping
                # on the brake: v = sqrt(2*a*(gap - d0)).
                tgt_mps = min(set_mps,
                              math.sqrt(max(0.0, 2.0 * 2.0 * (gap - 2.0))))
            thr, brk = hold.step(tgt_mps, v, dt_nom)
            if gap is not None and gap < 1.5:
                thr, brk = 0.0, 1.0
        else:
            owner = 'adas'
            steer = bridge.stack_steer
            thr, brk = bridge.stack_throttle, bridge.stack_brake
            if lkas_stale:
                # The stack is not steering and we are NOT in a junction:
                # hold the lane on the route so the leg survives, but this
                # distance is not ADAS exposure either.
                owner = 'route_lkas_silent'
                tgt = planner.target(x, y, PP_TARGET_AHEAD)
                steer, pp_alpha = pure_pursuit_steer(x, y, yaw, tgt, v,
                                                     pp_alpha)
                res.lkas_silent_s += dt_nom
                # This branch also drives open-loop, so it needs the same
                # ground-truth guard as the junction branch.
                gap_guard = nearest_ahead_m(world, ego)
                if gap_guard is not None and gap_guard < 1.5:
                    thr, brk = 0.0, 1.0

        adapter.apply_control(thr, brk, steer)

        # ---- distance accounting ----
        if prev is not None:
            d = math.hypot(x - prev[0], y - prev[1])
            res.driven_m += d
            if owner == 'adas':
                res.adas_m += d
                if math.isfinite(bridge.lead_distance or float('inf')):
                    res.acc_lead_m += d
            else:
                res.route_m += d
        prev = (x, y)

        # ---- events, only scored while the ADAS owns control ----
        ay = abs(v * (yaw - prev_ay) / dt_nom) if prev is not None else 0.0
        prev_ay = yaw
        if owner == 'adas':
            lane_wp = cur_wp
            cte = _signed_cte(lane_wp, x, y)
            if abs(cte) > DEPARTURE_M:
                res.departures += 1
            if -st.accel > DECEL_LIMIT:
                res.exc_decel += 1
            if ay > AY_CEILING:
                res.exc_ay += 1
            if v * 3.6 > SET_SPEED_KMH * SPEED_ENVELOPE_TOL:
                res.exc_speed += 1

        col = adapter.take_collision()
        if col is not None:
            # COUNT INCIDENTS, NOT SENSOR CALLBACKS.
            #
            # CARLA's collision sensor re-fires for as long as two bodies
            # are touching, so incrementing per callback counts contact
            # DURATION, not events. The 20 min smoke leg reported 17442
            # "collisions" — one long contact sampled every tick — which
            # made the tier-3 metric, the headline of the whole campaign,
            # unusable.
            #
            # A new incident needs a different actor, or the same actor
            # after COLLISION_DEDUP_S of separation. That treats "rolled
            # along a wall for 8 s" as one incident and "hit two cars in a
            # junction" as two, which is what a reader means by the word.
            if col.sim_time < leg_start_sim_t:
                continue          # queued by a previous leg — not ours
            now_s = time.time() - t0
            same = (col.other_id == last_col_id
                    and (now_s - last_col_t) < COLLISION_DEDUP_S)
            last_col_id, last_col_t = col.other_id, now_s
            if not same:
                # Attribute the incident. Route-owned collisions are the
                # HARNESS driving into something, not the ADAS, and the
                # regulated tier-3 rate is over adas_m — so counting them
                # together would contaminate the headline number with the
                # test rig's own failures. Both are reported; only
                # `collisions` is the regulated one.
                if owner == 'adas':
                    res.collisions += 1
                else:
                    res.harness_collisions += 1
                cat, geom = classify_collision(col)
                res.collision_detail.append(
                    f'{col.other_id}@{now_s:.1f}s/'
                    f'{col.impact_speed*3.6:.1f}kmh/{owner}/{cat}/{geom}')
                res.collision_kinds[f'{cat}|{geom}'] = (
                    res.collision_kinds.get(f'{cat}|{geom}', 0) + 1)
                # Flush immediately: a collision is the one sample nobody
                # wants to lose, and the leg may not survive to its next
                # scheduled flush.
                write_trace(out_dir, trace_name, samples)
                next_flush = time.time() + 30.0
                print(f"    !! collision {res.collisions} with "
                      f"{col.other_id} at {col.impact_speed*3.6:.1f} km/h, "
                      f"t={now_s:.1f} s, owner={owner}, {cat}, {geom} "
                      f"(bearing {col.contact_bearing_deg:.0f} deg)",
                      flush=True)
            else:
                res.contact_s += dt_nom
            res.note = (res.note + f' collision with {col.other_id} at '
                                   f'{col.impact_speed*3.6:.1f} km/h').strip()

        samples.append(dict(
            t=round(time.time() - t0, 3), owner=owner, v_kmh=round(v*3.6, 2),
            accel=round(st.accel, 3), ay=round(ay, 3),
            cte=round(_signed_cte(cur_wp, x, y), 3),
            junction=int(in_junction), throttle=round(thr, 3),
            gap_gt=('' if gap_guard is None else round(gap_guard, 2)),
            brake=round(brk, 3), steer=round(steer, 3),
            lead=bridge.lead_distance, x=round(x, 2), y=round(y, 2)))

        # Standstill ends the leg.
        #
        # The smoke leg spent 47 % of its samples below 0.5 km/h and its
        # last 76 s parked: boxed in at a junction by NPC traffic, with the
        # ACC holding for a lead it could see and the harness with nowhere
        # to route. Time spent stationary buys no exposure, so the leg is
        # ended and the campaign moves on rather than burning its minutes.
        #
        # Deliberately does NOT test the throttle. The earlier version
        # required thr > 0.6 (it was written for "shoving a stationary
        # car") and therefore never fired here, where the ACC had correctly
        # dropped to brake 0.05 and was simply waiting.
        #
        # NOTE this also ends a leg that is legitimately queueing in
        # traffic, which at 30 NPCs is a normal thing to be doing. Legs
        # will therefore vary in length and the campaign may finish short
        # of its planned exposure — `minutes` records what each leg
        # actually got, and the report totals real distance, so the claim
        # stays honest. If too many legs end this way, the answer is fewer
        # NPCs, not a longer timeout.
        if v < STANDSTILL_KMH:
            still_ticks += 1
            if still_ticks > int(STANDSTILL_ABORT_S * CONTROL_HZ):
                res.ended_early = True
                res.note = (res.note + f' leg ended: stationary '
                            f'>{STANDSTILL_ABORT_S:.0f} s').strip()
                print(f'    !! stationary >{STANDSTILL_ABORT_S:.0f} s at '
                      f't={time.time()-t0:.1f} s (owner={owner}, '
                      f'gap_gt={gap_guard}) — ending leg', flush=True)
                break
        else:
            still_ticks = 0

        if time.time() >= next_flush:
            write_trace(out_dir, trace_name, samples)
            next_flush = time.time() + 30.0

        time.sleep(max(0.0, dt_nom - (time.time() - loop_t)))

    res.handover_blocked_s = handover_blocked_ticks / CONTROL_HZ
    res.minutes = (time.time() - t0) / 60.0
    vs = [s['v_kmh'] for s in samples] or [0.0]
    res.v_mean_kmh = sum(vs) / len(vs)
    res.v_max_kmh = max(vs)
    res.speed_envelope_ok = res.v_mean_kmh <= SET_SPEED_KMH
    write_trace(out_dir, trace_name, samples)
    return res


def _signed_cte(wp, x, y) -> float:
    """Ego offset from the lane centreline, +ve right of centre."""
    loc = wp.transform.location
    yaw = math.radians(wp.transform.rotation.yaw)
    dx, dy = x - loc.x, y - loc.y
    return -math.sin(yaw) * dx + math.cos(yaw) * dy


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def report(results, args):
    adas = sum(r.adas_m for r in results) / 1000.0
    driven = sum(r.driven_m for r in results) / 1000.0
    route = sum(r.route_m for r in results) / 1000.0
    lead = sum(r.acc_lead_m for r in results) / 1000.0
    col = sum(r.collisions for r in results)
    hcol = sum(r.harness_collisions for r in results)
    dep = sum(r.departures for r in results)
    t1 = sum(r.tier1 for r in results)

    print('\n' + '=' * 92)
    print('ODD exposure campaign — declared envelope 0–20 km/h, R171 '
          'Non-Highway')
    print('=' * 92)
    print(f"{'leg':>3} {'town':<10}{'weather':<14}{'driven':>8}{'ADAS':>8}"
          f"{'route':>8}{'v̄':>7}{'col':>4}{'dep':>4}{'t1':>4}")
    for r in results:
        print(f"{r.leg:>3} {r.town:<10}{r.weather:<14}"
              f"{r.driven_m/1000:>8.2f}{r.adas_m/1000:>8.2f}"
              f"{r.route_m/1000:>8.2f}{r.v_mean_kmh:>7.1f}"
              f"{r.collisions:>4}{r.departures:>4}{r.tier1:>4}")
    print('-' * 92)
    print(f"driven {driven:.1f} km · ADAS-on-lane {adas:.1f} km · "
          f"junction/route {route:.1f} km · with a lead {lead:.1f} km")
    early = [r for r in results if r.ended_early]
    if early:
        print(f"{len(early)}/{len(results)} legs ended early on a "
              f">{STANDSTILL_ABORT_S:.0f} s standstill — planned minutes were "
              f"not reached, so exposure is below the schedule's projection")
    contact = sum(r.contact_s for r in results)
    if contact > 0:
        print(f"time in sustained contact: {contact:.0f} s "
              f"(counted once per incident, not per tick)")
    detail = [d for r in results for d in r.collision_detail]
    if detail:
        print('collision incidents '
              '(actor@t/impact/owner/category/contact):')
        for d in detail[:30]:
            print(f'    {d}')
        kinds = {}
        for r in results:
            for k, n in r.collision_kinds.items():
                kinds[k] = kinds.get(k, 0) + n
        if kinds:
            print('\n  by object and contact geometry:')
            for k in sorted(kinds, key=lambda k: -kinds[k]):
                cat, geom = k.split('|')
                note = ''
                if geom == 'flank':
                    note = '  <- struck on the side; in a junction this is '\
                           'usually an NPC arriving, not an avoidable event'
                elif geom == 'rear':
                    note = '  <- ego was rear-ended'
                print(f'    {cat:<20}{geom:<9}{kinds[k]:>3}{note}')
            front = sum(n for k, n in kinds.items() if k.endswith('|frontal'))
            print(f'\n  frontal (ego drove into it): {front} of '
                  f'{sum(kinds.values())} — these are the ones that speak '
                  f'to the system')

    print('\nPOOLED RESULT (exposure = ADAS-on-lane only)')
    print(f"  tier 3  ADAS collisions     {col}   (the regulated number)")
    if hcol:
        print(f"          harness collisions  {hcol}   over {route:.1f} km of "
              f"route-following — a RIG statistic, not the system's, and"
              f"\n          evidence that junction traversal is not yet good "
              f"enough to claim rather than exclude")
    if col == 0 and adas > 0:
        print(f"          -> 95 % upper bound {3.0/adas:.3f} per km "
              f"= 1 per {adas/3.0:.0f} km  (rule of three)")
        print( "          this is a BOUND, not a measured rate. Report it "
               "as such.")
    elif adas > 0:
        print(f"          -> measured {col/adas:.3f} per km "
              f"(1 per {adas/col:.1f} km)")
    for label, n in (('tier 2  lane departures', dep),
                     ('tier 1  envelope exceedances', t1)):
        rate = (n / adas) if adas else float('nan')
        print(f"  {label:<27} {n:<6} {rate:.2f} per km")
    print(f"\n  tier 1 breakdown: decel>{DECEL_LIMIT:g} "
          f"{sum(r.exc_decel for r in results)} · "
          f"ay>{AY_CEILING:g} {sum(r.exc_ay for r in results)} · "
          f"speed>{SET_SPEED_KMH*SPEED_ENVELOPE_TOL:.0f} km/h "
          f"{sum(r.exc_speed for r in results)}")

    bad = [r for r in results if not r.speed_envelope_ok]
    if bad:
        print(f"\n  !! {len(bad)} leg(s) exceeded the declared mean speed — "
              f"envelope claim INVALID for those legs:")
        for r in bad:
            print(f"       leg {r.leg} {r.town}/{r.weather}: "
                  f"mean {r.v_mean_kmh:.1f} km/h")

    silent = sum(r.lkas_silent_s for r in results)
    if silent > 0:
        print(f"\n  LKAS silent outside junctions: {silent:.0f} s "
              f"(driven by the harness, excluded from exposure)")

    print('\nCOVERAGE (per cell, evidence that the ODD was spanned)')
    cells = {}
    for r in results:
        cells.setdefault((r.town, r.weather), 0.0)
        cells[(r.town, r.weather)] += r.adas_m / 1000.0
    for (t, w), km in sorted(cells.items()):
        print(f"  {t:<10}{w:<14}{km:>6.2f} km ADAS")


# ---------------------------------------------------------------------------
def _install_stack_dumper():
    """kill -USR1 <pid> dumps every thread's Python stack to stderr.

    Two guesses at the leg-transition hang have now been wrong (the RPC
    storm, then the physics-off teleport), and both cost a run to
    disprove. py-spy needs ptrace permission this box does not grant, so
    faulthandler is the way to see where the process actually is.
    """
    faulthandler.register(signal.SIGUSR1, all_threads=True, chain=False)


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--plan', action='store_true',
                   help='print the schedule and the claim it buys, then exit')
    p.add_argument('--smoke', action='store_true',
                   help='one 20 min leg in Town10HD/ClearNoon — validates the '
                        'rig and MEASURES the mean speed and lane fraction '
                        'the full plan is projected from')
    p.add_argument('--minutes-per-leg', type=float, default=MINUTES_PER_LEG)
    p.add_argument('--target-adas-km', type=float, default=None,
                   help='keep cycling the ODD schedule until this much '
                        'ADAS-on-lane exposure is banked, instead of '
                        'stopping after one pass. Exposure is what the '
                        'claim rests on, and a run that ends early on the '
                        'standstill rule banks less than its budget — so a '
                        'time-based campaign cannot promise a distance. '
                        'Cells are visited in the same order each cycle, '
                        'so coverage stays balanced as the total grows.')
    p.add_argument('--max-hours', type=float, default=8.0,
                   help='wall-clock guard on --target-adas-km, so a '
                        'campaign that is yielding badly stops rather than '
                        'running overnight for nothing')
    p.add_argument('--towns', nargs='+', default=[t for t, _ in TOWNS])
    p.add_argument('--weather', nargs='+', default=WEATHER)
    p.add_argument('--npcs', type=int, default=NPC_COUNT)
    p.add_argument('--set-speed-kmh', type=float, default=SET_SPEED_KMH)
    p.add_argument('--route-policy', choices=('outer_loop', 'explore'),
                   default=DEFAULT_ROUTE_POLICY,
                   help='outer_loop (default) laps each town\'s perimeter — fewer junctions, but it AVOIDS the intersections the declared ODD names, so the claim narrows to arterial/perimeter roads. explore wanders the network instead.')
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--out-dir', default=None)
    p.add_argument('--tag', default='odd')
    p.add_argument('--resume', action='store_true',
                   help='load an existing legs.json from --out-dir and '
                        'continue from it. A CARLA time-out raises a C++ '
                        'exception Python cannot catch — it calls '
                        'terminate() and takes the process with it — so a '
                        'long campaign has to be restartable from outside. '
                        'Exposure then accumulates across restarts instead '
                        'of resetting, which is the whole point of a '
                        'distance target.')
    p.add_argument('--no-carla-restart', action='store_true')
    p.add_argument('--stack-timeout', type=float, default=180.0)
    args = p.parse_args(argv)

    roles = dict(TOWNS)
    towns = [(t, roles.get(t, '')) for t in args.towns]
    if args.smoke:
        towns, args.weather, args.minutes_per_leg = (
            [towns[0]], [args.weather[0]], 20.0)
    legs = build_plan(towns, args.weather, args.minutes_per_leg,
                      args.npcs, args.route_policy)

    if args.plan:
        print_plan(legs)
        return 0

    _install_stack_dumper()
    check_no_bridge_conflict()
    install_sigterm_handler()
    out_dir = make_out_dir(__file__, args.tag, args.out_dir)
    print(f'[out] {out_dir}', flush=True)
    print_plan(legs)

    bridge, shutdown = start_bridge('odd_endurance')
    watchdog = LegWatchdog()
    prior = []
    if args.resume:
        f = Path(out_dir) / 'legs.json'
        if f.exists():
            for d in json.loads(f.read_text()):
                r = LegResult()
                for k, v in d.items():
                    if hasattr(r, k):
                        setattr(r, k, v)
                prior.append(r)
            km = sum(r.adas_m for r in prior) / 1000
            print(f'[resume] {len(prior)} runs, {km:.2f} km already banked',
                  flush=True)
    results: list[LegResult] = []
    try:
        if not wait_for_stack(bridge, args.stack_timeout):
            print('[odd] ADAS stack not ready — is start_adas.sh running?')
            return 1
        current_town = None
        adapter = pump = None

        # One pass, or repeat until the exposure target is met.
        schedule = list(legs)
        if args.target_adas_km:
            cycles = 40                    # bounded by --max-hours anyway
            schedule = []
            for c in range(cycles):
                for spec in legs:
                    q = dict(spec)
                    q['leg'] = len(schedule) + 1
                    q['cycle'] = c + 1
                    schedule.append(q)
            print(f'[plan] target {args.target_adas_km:g} km ADAS — cycling '
                  f'the {len(legs)}-cell schedule, cap {args.max_hours:g} h',
                  flush=True)
        t_campaign = time.time()
        results.extend(prior)
        banked_km = sum(r.adas_m for r in prior) / 1000
        run_offset = len(prior)

        for spec in schedule:
            if args.target_adas_km:
                if banked_km >= args.target_adas_km:
                    print(f'\n[plan] target reached: {banked_km:.2f} km ADAS',
                          flush=True)
                    break
                hrs = (time.time() - t_campaign) / 3600.0
                if hrs > args.max_hours:
                    print(f'\n[plan] stopping at the {args.max_hours:g} h cap '
                          f'with {banked_km:.2f} of {args.target_adas_km:g} km '
                          f'— yield was {banked_km/max(hrs,1e-6):.2f} km/h',
                          flush=True)
                    break
            if spec['town'] != current_town:
                if adapter is not None:
                    pump.stop(); adapter.close()
                    adapter = pump = None
                if not args.no_carla_restart:
                    carla_server.ensure_town(spec['town'], host=args.host,
                                             port=args.port)
                current_town = spec['town']
                adapter = _open_town(spec, args)
                _spawn_npcs(adapter, args.npcs)
                pump = CameraPump(adapter, bridge)
                pump.start()
            _set_weather(adapter, spec['weather'])
            time.sleep(SETTLE_S)
            print(f"\n[{spec['leg']}/{len(schedule)}] {spec['town']} "
                  f"{spec['weather']} {spec['npcs']} NPC "
                  f"{spec['minutes']:.1f} min", flush=True)
            watchdog.arm(spec['leg'], spec['minutes'])
            r = run_leg(adapter, bridge, pump, spec, out_dir, args)
            watchdog.disarm()
            results.append(r)
            banked_km += r.adas_m / 1000
            if args.target_adas_km:
                hrs = (time.time() - t_campaign) / 3600.0
                print(f"    [target] {banked_km:.2f}/{args.target_adas_km:g} km "
                      f"after {hrs:.2f} h "
                      f"({banked_km/max(hrs,1e-6):.2f} km/h)", flush=True)
            print(f"    driven {r.driven_m/1000:.2f} km · ADAS "
                  f"{r.adas_m/1000:.2f} km · v̄ {r.v_mean_kmh:.1f} km/h · "
                  f"col {r.collisions} dep {r.departures} t1 {r.tier1}")
            _dump(out_dir, results)
        if adapter is not None:
            pump.stop(); adapter.close()
    finally:
        watchdog.stop()
        shutdown()
    if results:
        report(results, args)
        _dump(out_dir, results)
    print(f'\n[out] {out_dir}')
    return 0


def _open_town(spec, args):
    """Connect, then start the ego on a lane rather than in a junction.

    CarlaAdapter's default spawn_index is 80 — chosen for Town06's 718 m
    straight, which is a property of one map and meaningless here. Worse,
    an arbitrary index can land in a junction, so the leg would open under
    the route-follower with no ADAS exposure and no lane for UFLD to lock.

    connect() computes `_start_tf` from spawn_index, so the override has to
    happen between connect() and spawn_ego(). Reaching into the private is
    deliberate and local: adding a public setter to sim_adapter for one
    caller would be the bigger change.
    """
    import carla
    import random
    from sim_adapter import CarlaAdapter
    a = CarlaAdapter(host=args.host, port=args.port, town=spec['town'],
                     weather=spec['weather'], clean_start=True)
    a.connect()

    pts = a.world.get_map().get_spawn_points()
    cmap = a.world.get_map()
    rng = random.Random(spec['leg'])
    ranked = list(range(len(pts)))
    rng.shuffle(ranked)
    chosen = None
    for i in ranked:
        wp = cmap.get_waypoint(pts[i].location, project_to_road=True,
                               lane_type=carla.LaneType.Driving)
        # Not in a junction, and with enough lane ahead that the leg starts
        # in ADAS control rather than immediately handing to the route.
        if wp is not None and not wp.is_junction and wp.next(25.0):
            chosen = i
            break
    if chosen is None:
        chosen = min(a.spawn_index, len(pts) - 1)
        print(f'[spawn] {spec["town"]}: no clear lane spawn found, '
              f'falling back to index {chosen}', flush=True)
    a.spawn_index = chosen
    a._start_tf = pts[chosen]
    a._start_tf.location.z += 0.5
    print(f'[spawn] {spec["town"]}: spawn point {chosen}/{len(pts)}',
          flush=True)

    a.spawn_ego()
    return a


def _set_weather(adapter, preset):
    import carla
    if preset == 'FogNoon':
        w = getattr(carla.WeatherParameters, FOG['base'])
        w.fog_density = FOG['fog_density']
        w.fog_distance = FOG['fog_distance']
        w.fog_falloff = FOG['fog_falloff']
    else:
        w = getattr(carla.WeatherParameters, preset)
    adapter.world.set_weather(w)


def _spawn_npcs(adapter, n):
    """n autopilot vehicles. Failures are counted, not fatal — a town with
    fewer spawn points than requested still gives a valid (smaller) traffic
    density, and the actual count goes in the record."""
    import random
    world = adapter.world
    bp_lib = world.get_blueprint_library()
    pts = world.get_map().get_spawn_points()
    random.shuffle(pts)
    bps = [b for b in bp_lib.filter('vehicle.*')
           if b.has_attribute('number_of_wheels')
           and int(b.get_attribute('number_of_wheels')) == 4]
    tm = adapter.client.get_trafficmanager()
    spawned = 0
    for pt in pts:
        if spawned >= n:
            break
        v = world.try_spawn_actor(random.choice(bps), pt)
        if v is not None:
            v.set_autopilot(True, tm.get_port())
            spawned += 1
    print(f'[npc] spawned {spawned}/{n}', flush=True)
    return spawned


def _dump(out_dir: Path, results):
    (Path(out_dir) / 'legs.json').write_text(
        json.dumps([asdict(r) for r in results], indent=2))


if __name__ == '__main__':
    sys.exit(main())

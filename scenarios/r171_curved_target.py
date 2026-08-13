#!/usr/bin/env python3
"""UN R171 (DCAS) Annex 4 §4.2.5.2.2 — stationary vehicle ahead, curved road.

The curved sibling of r171_stationary_target.py. Same plant, same ROS
bridge, same headline KPI (a_req at brake onset vs 5 m/s^2); what changes
is that the target sits round a bend, so the system has to associate a
lead that is not straight ahead — and has to keep braking while the
lateral controller works.

    ./start_adas.sh carla
    python3 scenarios/r171_curved_target.py --site t04_r199 --speed-kmh 70
    python3 scenarios/r171_curved_target.py --matrix

Geometry: why these radii
-------------------------
§4.2.4.1 specifies an S-bend of clothoids and arcs — first turn R = 787 m,
second turn R = 374 m — and then says:

    "At the request of the manufacturer and with the agreement of the Type
     Approval Authority, tests may be conducted on a road of different
     curvature, provided this does not change the intention or lower the
     severity of the test."

What carries the severity is (a) the lateral demand v^2/R the curve
imposes and (b) how far outside a straight-ahead corridor the target sits
— not the radius as such. The site table (curve_adapter.SITES, measured by
curve_survey.py) is therefore chosen to bracket the reference radii from
the severe side, and every run records the radius the map actually has.

The survey's finding is that only Town12 has highway geometry:

    t12_r1185  1185 m  the only site that fits 130 km/h; LESS severe than
                       the 787 m reference, so a case in its own right
    t12_r500    500 m  first-turn analogue, more severe than 787 m
    t12_r417    417 m  second-turn analogue, +11 % on 374 m, 460 m of arc
    t12_r345    345 m  second-turn analogue, -8 %, i.e. more severe
    t07_r364    364 m  the reference radius to 2.7 %, but 34 m of arc
    t04_r199    199 m  best small-map site: 296 m arc, 462 m lead-in,
                       and the one to iterate on — Town12 takes minutes
                       to load. Caps the matrix at 88 km/h.

Where the target goes
---------------------
§4.2.5.2.2.1.1 puts the target in the curved lane, within 0.5 m of its
centreline, "so that the rear corner is touching the extrapolated lane
line if the straight were to continue". Extrapolate the tangent at the arc
entry and that condition is met where the lane has moved sideways by
lane_width/2 + the target's half-width — 2.7 m for a 3.5 m lane and a
charger_2020, which is 33 m into a 199 m bend, 47 m into a 417 m one and
80 m into the 1185 m site. The adapter walks the real lane polyline to
find that point rather than using the s^2/2R approximation.

The consequence is the point of the test: at the handover the target is
*outside* the corridor a straight-line path prediction would sweep. A
system that extrapolates the ego's current heading has no reason to treat
it as a lead at all.

Where the VUT starts
--------------------
§4.2.5.2.2.1.2 requires the VUT to have been driven along the straight
"for enough time for the lateral control to take up a constant position
within the lane, prior to the start of the curved section". The start line
is therefore derived, not chosen: walk back from the target by
(settle + ttc) * v along the lane. `straight_before_entry_m` in the run
record says how much of that landed on the straight; the run is refused
if it is less than --min-settle-straight seconds' worth.

Reading the results against the straight matrix
-----------------------------------------------
The two scripts share `a_req_at_brake_onset_mps2`, so a curved run at
70 km/h is directly comparable with the straight run at 70 km/h. Three
extra columns explain any difference:

    gap_at_first_detection_m    how much later perception acquires a
                                target that is off-axis
    bearing_at_first_detection_deg   how far off-axis it was
    target_in_straight_corridor_at_trigger   whether a straight-line
                                path predictor would have counted it at
                                the handover point

Gaps are measured ALONG THE LANE. On a 400 m radius, a 200 m separation
has a 12.5 m sagitta, so the chord under-reads distance-to-go by 6 % —
enough to move a_req by the same margin. `gap_projected_m` keeps the chord
in the trace because that is what a monocular range estimate approximates.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, asdict, replace
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import carla_server  # noqa: E402
from curve_adapter import SITES, DEFAULT_SITE, CarlaCurveAdapter  # noqa: E402
from scenario_common import (  # noqa: E402
    LOG_HZ, DECEL_VALID_GAP_M, DECEL_VALID_SPEED_MPS, DECEL_PLAUSIBLE_MAX,
    DECEL_PHYSICAL_MAX, CameraPump, LaneHold, SpeedHold,
    check_no_bridge_conflict, fmt_decel, install_sigterm_handler,
    make_out_dir, peak_decel_from_trace, required_decel, start_bridge,
    wait_for_stack, write_outputs, write_trace)


# Run-end conditions, identical to the straight scenario so the two sets
# of results are comparable.
STOP_SPEED_MPS = 0.3
STOP_HOLD_S = 1.0
STOP_CREEP_VREF_KMH = 0.5
STOP_CREEP_TIMEOUT_S = 6.0
PASS_MARGIN_M = -5.0
TIMEOUT_MARGIN_S = 25.0
BRAKE_ONSET_THRESHOLD = 0.10
DECEL_LIMIT_MPS2 = 5.0

# The lateral demand at which a run stops being a longitudinal test.
# R171 §5.3.7.1.2 caps what an M1 DCAS may induce at 3 m/s^2, and above
# that the ego cannot hold the lane anyway, so a speed/site combination
# beyond it would measure the lateral controller's failure rather than the
# ACC's reaction. Each site's speed ceiling falls out of this.
AY_CEILING_MPS2 = 3.0


@dataclass
class ScenarioPoint:
    speed_kmh: float
    offset_m: float
    ttc_s: float
    site: str = DEFAULT_SITE
    block: str = 'custom'
    rep: int = 0

    @property
    def speed_mps(self) -> float:
        return self.speed_kmh / 3.6

    @property
    def trigger_gap_m(self) -> float:
        """Handover distance. TTC against a stationary target is
        gap / v_ego, and gap here is arc length."""
        return self.ttc_s * self.speed_mps

    @property
    def ay_mps2(self) -> float:
        return self.speed_mps ** 2 / SITES[self.site].radius_m

    @property
    def run_id(self) -> str:
        base = (f"{self.block}_{self.site}_v{self.speed_kmh:g}"
                f"_off{self.offset_m:g}_ttc{self.ttc_s:g}")
        return f"{base}_r{self.rep}" if self.rep else base


# The straight matrix's speeds, minus whatever a site cannot carry. R171
# constrains the lateral offset to 0.5 m of the lane centre here
# (§4.2.5.2.2.1.1), so the offset axis of the straight matrix — which went
# out to 1.0 m — collapses to two points.
SPEEDS_KMH = (30, 50, 70, 90, 110, 130)
OFFSETS_M = (0.0, 0.5)
TTCS_S = (4.5, 6.0, 10.0)
NOMINAL_TTC_S = 6.0
NOMINAL_OFFSET_M = 0.0
# Sites for --matrix: one analogue of each R171 reference turn. Both are
# in Town12, so the matrix costs a single (slow) world load and then runs
# straight through. `--matrix-sites t04_r199` is the fast local
# alternative at the cost of an 88 km/h ceiling.
MATRIX_SITES = ('t12_r412', 't12_r500')


def site_max_speed_kmh(site_name: str) -> float:
    """Fastest approach this site can carry inside the 3 m/s^2 ceiling."""
    return math.sqrt(AY_CEILING_MPS2 * SITES[site_name].radius_m) * 3.6


def build_matrix(sites=MATRIX_SITES) -> list[ScenarioPoint]:
    """Block A: speed x offset @ TTC 6 s;  Block B: speed x TTC @ offset 0.

    Per site, and with any point whose lateral demand would exceed
    AY_CEILING_MPS2 dropped rather than silently run — at 130 km/h on the
    400 m site the curve alone asks for 3.26 m/s^2, which is over what
    R171 §5.3.7.1.2 permits the system to induce.
    """
    points: list[ScenarioPoint] = []
    for site in sites:
        v_cap = site_max_speed_kmh(site)
        for v in SPEEDS_KMH:
            if v > v_cap:
                continue
            for off in OFFSETS_M:
                points.append(ScenarioPoint(v, off, NOMINAL_TTC_S, site, 'A'))
        for v in SPEEDS_KMH:
            if v > v_cap:
                continue
            for ttc in TTCS_S:
                if ttc == NOMINAL_TTC_S:
                    continue          # already in Block A at offset 0
                points.append(
                    ScenarioPoint(v, NOMINAL_OFFSET_M, ttc, site, 'B'))
    return points


# ---------------------------------------------------------------------------
# Run record
# ---------------------------------------------------------------------------
@dataclass
class RunMetrics:
    run_id: str = ''
    block: str = ''
    site: str = ''
    radius_m: float = 0.0
    direction: str = ''
    speed_kmh: float = 0.0
    offset_m: float = 0.0
    ttc_s: float = 0.0
    trigger_gap_m: float = 0.0
    # What the curve itself demands at the test speed. Logged per run
    # because it, not the radius, is the severity the site delivers.
    ay_demand_mps2: float = 0.0
    target_into_curve_m: float = 0.0
    straight_before_entry_m: float = 0.0

    v_at_trigger_kmh: float = 0.0
    gap_at_trigger_m: float = 0.0

    # ---- headline KPI, identical to the straight scenario ----
    a_req_at_trigger_mps2: float = float('nan')
    a_req_at_brake_onset_mps2: float = float('nan')
    a_req_peak_mps2: float = float('nan')
    peak_decel_achieved_mps2: float = 0.0
    decel_limit_mps2: float = DECEL_LIMIT_MPS2
    exceeded_decel_limit: bool = False
    unavoidable_at_onset: bool = False
    decel_margin_mps2: float = float('nan')
    verdict: str = ''

    # ---- what the curve adds ----
    gap_at_first_detection_m: float = float('nan')
    bearing_at_first_detection_deg: float = float('nan')
    detected_before_trigger: bool = False
    # Whether the target was still inside the corridor swept by the ego's
    # current heading when the scenario handed over. False means a
    # straight-line path predictor had no reason to call it a lead.
    target_in_straight_corridor_at_trigger: bool = False
    bearing_at_trigger_deg: float = float('nan')
    gap_at_acc_engage_m: float = float('nan')
    # Lane keeping while the ACC brakes in the bend. The straight
    # scenario's equivalent column is uninteresting; here it is the
    # coupling between the two controllers.
    max_abs_cte_m: float = 0.0
    cte_at_min_gap_m: float = float('nan')
    handover_in_curve: bool = False

    brake_onset_gap_m: float = float('nan')
    brake_onset_ttc_s: float = float('nan')
    reaction_latency_s: float = float('nan')
    min_gap_m: float = float('inf')
    final_gap_m: float = float('nan')
    final_speed_kmh: float = 0.0

    collision: bool = False
    impact_speed_kmh: float = 0.0
    speed_reduction_kmh: float = 0.0
    outcome: str = ''
    duration_s: float = 0.0
    note: str = ''


# ---------------------------------------------------------------------------
# The director
# ---------------------------------------------------------------------------
class CurvedRunner:

    def __init__(self, adapter, bridge, args):
        self.adapter = adapter
        self.bridge = bridge
        self.args = args
        self.speed_hold = SpeedHold()
        self.lane_hold = LaneHold()

    def run(self, point: ScenarioPoint, out_dir: Path):
        args = self.args
        v_set = point.speed_mps
        trigger_gap = point.trigger_gap_m
        placement = (args.settle_time + point.ttc_s) * v_set
        site = SITES[point.site]

        if point.ay_mps2 > AY_CEILING_MPS2 + 1e-6:
            raise RuntimeError(
                f"{point.run_id}: {point.speed_kmh:g} km/h on a "
                f"{site.radius_m:.0f} m radius demands "
                f"{point.ay_mps2:.2f} m/s^2 of lateral acceleration, above "
                f"the {AY_CEILING_MPS2:g} m/s^2 R171 §5.3.7.1.2 permits an "
                f"M1 system to induce. Cap for this site is "
                f"{site_max_speed_kmh(point.site):.0f} km/h.")

        geometry = self.adapter.arm(placement, point.offset_m)

        # §4.2.5.2.2.1.2: settled in-lane on the straight before the bend.
        need_straight = args.min_settle_straight * v_set
        if geometry.straight_before_entry_m < need_straight:
            raise RuntimeError(
                f"{point.run_id}: only "
                f"{geometry.straight_before_entry_m:.0f} m of the approach "
                f"is on the straight, and settling the lateral controller "
                f"needs {need_straight:.0f} m "
                f"({args.min_settle_straight:g} s at {point.speed_kmh:g} "
                f"km/h). §4.2.5.2.2.1.2 wants the VUT in a constant "
                f"in-lane position before the curve — pick a site with a "
                f"longer lead-in ({point.site} has "
                f"{site.lead_in_m:.0f} m).")

        self.speed_hold.reset(SpeedHold.drag_throttle_estimate(v_set))
        self.lane_hold.reset()
        self.bridge.publish_not_in_junction()
        for _ in range(5):
            self.bridge.publish_target_speed(point.speed_kmh)
            time.sleep(0.05)

        if args.approach_mode in ('physical', 'kinematic'):
            self.adapter.force_speed(v_set)

        m = RunMetrics(
            run_id=point.run_id, block=point.block, site=point.site,
            radius_m=geometry.radius_m, direction=geometry.direction,
            speed_kmh=point.speed_kmh, offset_m=point.offset_m,
            ttc_s=point.ttc_s, trigger_gap_m=trigger_gap,
            ay_demand_mps2=point.ay_mps2,
            target_into_curve_m=geometry.target_into_curve_m,
            straight_before_entry_m=geometry.straight_before_entry_m)
        m.decel_limit_mps2 = DECEL_LIMIT_MPS2

        samples: list[dict] = []
        handed_over = False
        t0 = self.adapter.wait_for_tick()
        t_trigger = None
        t_brake_onset = None
        stopped_since = None
        last_log = -1.0
        prev_t = t0
        timeout = (args.settle_time + point.ttc_s
                   + TIMEOUT_MARGIN_S + args.timeout_extra)
        # Half the corridor a straight-line path prediction would sweep:
        # the ego's own width plus the lane's shoulder either side.
        corridor_half = site.lane_width_m / 2.0

        while True:
            t = self.adapter.wait_for_tick()
            dt = max(t - prev_t, 1e-3)
            prev_t = t
            elapsed = t - t0

            ego = self.adapter.ego_state()
            gap = self.adapter.gap_m()                  # arc length
            gap_proj = self.adapter.gap_projected_m()   # chord
            lane = self.adapter.lane_error()
            bearing = self.adapter.target_bearing_rad()
            self.bridge.publish_speed(ego.speed)

            perceived = self.bridge.lead_distance
            acc_mode = self.bridge.acc_mode
            tracked_gap = self.bridge.tracked_gap
            speed_ref = self.bridge.speed_ref
            pinhole_gap = self.bridge.pinhole_gap

            # Lateral offset of the target from the ego's heading ray. The
            # test's premise is that this exceeds the corridor before the
            # system reacts.
            lat_offset = (abs(math.sin(bearing)) * max(gap_proj, 0.0)
                          if math.isfinite(bearing) else float('nan'))
            in_corridor = bool(math.isfinite(lat_offset)
                               and lat_offset <= corridor_half)

            if (perceived is not None
                    and math.isnan(m.gap_at_first_detection_m)):
                m.gap_at_first_detection_m = gap
                m.bearing_at_first_detection_deg = math.degrees(bearing)
                m.detected_before_trigger = not handed_over
            if acc_mode == 'ACC' and math.isnan(m.gap_at_acc_engage_m):
                m.gap_at_acc_engage_m = gap

            a_req = required_decel(ego.speed, gap)

            # ---- handover: the DCAS trigger point ----
            if not handed_over and gap <= trigger_gap:
                handed_over = True
                t_trigger = t
                m.v_at_trigger_kmh = ego.speed * 3.6
                m.gap_at_trigger_m = gap
                m.a_req_at_trigger_mps2 = a_req
                m.bearing_at_trigger_deg = math.degrees(bearing)
                m.target_in_straight_corridor_at_trigger = in_corridor
                m.handover_in_curve = abs(lane.heading_err_rad) > 0.02
                if args.verbose:
                    print(f"    [handover] gap={gap:.1f} m (arc)  "
                          f"chord={gap_proj:.1f} m  "
                          f"bearing={math.degrees(bearing):+.1f} deg  "
                          f"corridor={'in' if in_corridor else 'OUT'}  "
                          f"a_req={a_req:.2f} m/s^2", flush=True)

            # ---- longitudinal authority ----
            if handed_over:
                throttle, brake = (self.bridge.stack_throttle,
                                   self.bridge.stack_brake)
                if brake > BRAKE_ONSET_THRESHOLD and t_brake_onset is None:
                    t_brake_onset = t
                    m.brake_onset_gap_m = gap
                    m.brake_onset_ttc_s = (gap / ego.speed
                                           if ego.speed > 0.1 else float('inf'))
                    m.reaction_latency_s = t - t_trigger
                    m.a_req_at_brake_onset_mps2 = a_req
                    m.decel_margin_mps2 = DECEL_LIMIT_MPS2 - a_req
            elif args.approach_mode == 'kinematic':
                self.adapter.force_speed(v_set)
                throttle, brake = 0.0, 0.0
            else:
                throttle, brake = self.speed_hold.step(v_set, ego.speed, dt)

            # ---- lateral authority ----
            if args.lateral_mode == 'lkas':
                steer = self.bridge.stack_steer
            else:
                steer = self.lane_hold.step(lane.cross_track_m,
                                            lane.heading_err_rad, ego.speed)

            self.bridge.set_longitudinal(throttle, brake)
            with self.bridge._sink_lock:
                self.adapter.apply_control(throttle, brake, steer)

            # ---- metrics ----
            if gap < m.min_gap_m:
                m.min_gap_m = gap
                m.cte_at_min_gap_m = lane.cross_track_m
            m.max_abs_cte_m = max(m.max_abs_cte_m, abs(lane.cross_track_m))
            if (handed_over and gap > DECEL_VALID_GAP_M
                    and ego.speed > DECEL_VALID_SPEED_MPS):
                achieved = min(-ego.accel, DECEL_PLAUSIBLE_MAX)
                m.peak_decel_achieved_mps2 = max(
                    m.peak_decel_achieved_mps2, achieved)
                m.a_req_peak_mps2 = (
                    a_req if math.isnan(m.a_req_peak_mps2)
                    else max(m.a_req_peak_mps2, a_req))

            if elapsed - last_log >= 1.0 / LOG_HZ:
                last_log = elapsed
                samples.append(dict(
                    t=round(elapsed, 3),
                    phase='measure' if handed_over else 'approach',
                    v_kmh=round(ego.speed * 3.6, 3),
                    accel_mps2=round(ego.accel, 3),
                    # Arc length — the distance-to-go a_req is computed on.
                    gap_gt_m=round(gap, 3),
                    # Chord — what a monocular range estimate approximates,
                    # so gap_perceived_m should be compared against this.
                    gap_projected_m=round(gap_proj, 3),
                    gap_perceived_m=(round(perceived, 3)
                                     if perceived is not None else ''),
                    bearing_deg=(round(math.degrees(bearing), 2)
                                 if math.isfinite(bearing) else ''),
                    lat_offset_m=(round(lat_offset, 3)
                                  if math.isfinite(lat_offset) else ''),
                    in_straight_corridor=in_corridor,
                    ttc_s=round(gap / ego.speed, 3) if ego.speed > 0.1 else '',
                    a_req_mps2=(round(a_req, 3) if math.isfinite(a_req)
                                else ''),
                    acc_mode=acc_mode,
                    acc_pd_active=(acc_mode == 'ACC'),
                    gap_tracked_m=(round(tracked_gap, 3)
                                   if tracked_gap is not None else ''),
                    v_ref_kmh=(round(speed_ref * 3.6, 2)
                               if speed_ref is not None else ''),
                    gap_pinhole_m=(round(pinhole_gap, 3)
                                   if pinhole_gap is not None else ''),
                    throttle=round(throttle, 3),
                    brake=round(brake, 3),
                    steer=round(steer, 4),
                    cte_m=round(lane.cross_track_m, 3),
                    heading_err_deg=round(
                        math.degrees(lane.heading_err_rad), 3),
                    x=round(ego.x, 2), y=round(ego.y, 2)))

            # ---- stop conditions ----
            collision = self.adapter.take_collision()
            if collision is not None:
                m.collision = True
                m.impact_speed_kmh = collision.impact_speed * 3.6
                m.outcome = 'collision'
                break
            creeping = (speed_ref is not None
                        and speed_ref * 3.6 > STOP_CREEP_VREF_KMH
                        and acc_mode != 'STANDSTILL')
            if ego.speed < STOP_SPEED_MPS and handed_over:
                stopped_since = stopped_since if stopped_since else t
                held = t - stopped_since
                if held >= STOP_HOLD_S and (not creeping
                                            or held >= STOP_CREEP_TIMEOUT_S):
                    m.outcome = 'stopped'
                    if creeping:
                        m.note = ((m.note or '') + ' (creep did not settle '
                                  f'within {STOP_CREEP_TIMEOUT_S:.0f} s)')
                    break
            else:
                stopped_since = None
            if gap < PASS_MARGIN_M:
                m.outcome = 'passed'
                m.note = 'VUT passed the target without contact'
                break
            if elapsed > timeout:
                m.outcome = 'timeout'
                break

        peak = peak_decel_from_trace(samples)
        if peak is not None:
            m.peak_decel_achieved_mps2 = peak

        m.final_gap_m = self.adapter.gap_m()
        m.final_speed_kmh = self.adapter.ego_state().speed * 3.6
        m.speed_reduction_kmh = point.speed_kmh - (
            m.impact_speed_kmh if m.collision else m.final_speed_kmh)
        m.duration_s = round(prev_t - t0, 2)

        onset = m.a_req_at_brake_onset_mps2
        m.exceeded_decel_limit = bool(
            math.isfinite(onset) and onset > DECEL_LIMIT_MPS2)
        m.unavoidable_at_onset = bool(
            math.isfinite(onset) and onset > DECEL_PHYSICAL_MAX)
        if m.collision:
            m.verdict = 'fail_collision'
        elif math.isnan(onset):
            m.verdict = 'no_reaction'
            m.note = (m.note or
                      'target never braked — cleared without an intervention')
        elif m.exceeded_decel_limit:
            m.verdict = 'pass_over_limit'
        else:
            m.verdict = 'pass'

        write_trace(out_dir, point.run_id, samples)
        self.adapter.disarm()
        return m, asdict(geometry)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    g = p.add_argument_group('scenario point (single run)')
    g.add_argument('--site', default=DEFAULT_SITE, choices=sorted(SITES),
                   help=f'Curve site. Default {DEFAULT_SITE}. '
                        f'--list-sites prints the table.')
    g.add_argument('--speed-kmh', type=float,
                   help='VUT approach speed [km/h].')
    g.add_argument('--offset-m', type=float, default=0.0,
                   help='Target offset from the lane centreline [m], +ve = '
                        'right. R171 §4.2.5.2.2.1.1 allows 0.5 m; larger '
                        'values are outside the regulation and are flagged.')
    g.add_argument('--ttc-s', type=float, default=6.0,
                   help='Initial TTC margin [s]; handover at gap = ttc * v, '
                        'measured along the lane. Default 6.')

    g = p.add_argument_group('matrix')
    g.add_argument('--matrix', action='store_true',
                   help='Speed x offset @ TTC 6 s and speed x TTC @ offset 0, '
                        'on each site in MATRIX_SITES, minus points over the '
                        'lateral-acceleration ceiling.')
    g.add_argument('--block', choices=['A', 'B'])
    g.add_argument('--matrix-sites', nargs='+', default=list(MATRIX_SITES),
                   choices=sorted(SITES),
                   help=f'Sites for --matrix. Default {" ".join(MATRIX_SITES)}.')
    g.add_argument('--list', action='store_true',
                   help='Print the matrix and exit. Needs no simulator.')
    g.add_argument('--list-sites', action='store_true',
                   help='Print the surveyed curve sites and exit.')
    g.add_argument('--repeats', type=int, default=1)

    g = p.add_argument_group('simulator')
    g.add_argument('--host', default='localhost')
    g.add_argument('--port', type=int, default=2000)
    g.add_argument('--vehicle', default='vehicle.dodge.charger_2020',
                   help='Blueprint for BOTH the VUT and the target.')
    g.add_argument('--weather', default='ClearNoon')
    g.add_argument('--keep-existing-actors', action='store_true')
    g.add_argument('--no-carla-restart', action='store_true',
                   help='Do not reboot the CARLA server when a site needs a '
                        'different town, and use client.load_world() '
                        'instead. That is what the harness used to do, and '
                        'it is why every site group after the first '
                        'produced no steering at all — in-band load_world '
                        'is not safe on this install (DEBUG §59). Only pass '
                        'this if CARLA is managed elsewhere.')
    g.add_argument('--carla-quality', default='Epic',
                   choices=['Low', 'Epic'],
                   help='Quality level for a harness-started CARLA.')
    g.add_argument('--sync', action='store_true',
                   help='Synchronous world. Off by default — sync mode '
                        'regressed forward motion in this multi-process '
                        'setup (DEBUG.md §4).')

    g = p.add_argument_group('run shaping')
    g.add_argument('--settle-time', type=float, default=3.0,
                   help='Seconds of steady-state approach before the trigger '
                        'point; placement = (settle + ttc) * v along the '
                        'lane. Default 3.')
    g.add_argument('--min-settle-straight', type=float, default=1.5,
                   help='Seconds of the approach that must fall on the '
                        'straight before the arc (§4.2.5.2.2.1.2). Default '
                        '1.5. Runs that cannot meet it are refused, not '
                        'quietly run on a shorter lead-in.')
    g.add_argument('--approach-mode', choices=['physical', 'kinematic'],
                   default='physical')
    g.add_argument('--lateral-mode', choices=['locked', 'lkas'],
                   default='locked',
                   help='locked (default): the scenario holds the lane '
                        'centreline through the bend, isolating the '
                        'longitudinal result. lkas: Stanley steers, so the '
                        'run measures both loops at once.')
    g.add_argument('--decel-limit', type=float, default=DECEL_LIMIT_MPS2)
    g.add_argument('--timeout-extra', type=float, default=0.0)
    g.add_argument('--settle-between-runs', type=float, default=2.0)

    g = p.add_argument_group('output')
    g.add_argument('--out-dir', default=None)
    g.add_argument('--tag', default='')
    g.add_argument('--verbose', action='store_true')
    g.add_argument('--no-stack-check', action='store_true')
    g.add_argument('--stack-timeout', type=float, default=120.0)
    return p.parse_args(argv)


def resolve_points(args) -> list[ScenarioPoint]:
    if args.matrix or args.list:
        points = build_matrix(tuple(args.matrix_sites))
        if args.block:
            points = [p for p in points if p.block == args.block]
        return points
    if args.speed_kmh is None:
        raise SystemExit(
            "give --speed-kmh (with optional --site / --offset-m / --ttc-s) "
            "for a single point, or --matrix. --list prints the matrix and "
            "--list-sites the surveyed curves; neither needs a simulator.")
    return [ScenarioPoint(args.speed_kmh, args.offset_m, args.ttc_s,
                          args.site, block='single')]


def print_sites() -> None:
    print(f"{'site':<10} {'town':<9} {'R[m]':>7} {'dir':>6} {'arc[m]':>7} "
          f"{'lead[m]':>8} {'v_max[km/h]':>12}")
    print('-' * 62)
    for name, s in sorted(SITES.items(), key=lambda kv: -kv[1].radius_m):
        print(f"{name:<10} {s.town:<9} {s.radius_m:>7.1f} {s.direction:>6} "
              f"{s.arc_m:>7.1f} {s.lead_in_m:>8.1f} "
              f"{site_max_speed_kmh(name):>12.0f}")
    print(f"\nv_max is where the curve alone reaches "
          f"{AY_CEILING_MPS2:g} m/s^2, the R171 §5.3.7.1.2 ceiling for M1.\n")
    for name, s in sorted(SITES.items(), key=lambda kv: -kv[1].radius_m):
        print(f"  {name}: {s.note}")


def print_matrix(points: list[ScenarioPoint], settle_time: float) -> None:
    print(f"{'#':>3} {'block':>5} {'site':>10} {'v[km/h]':>8} "
          f"{'offset[m]':>10} {'TTC[s]':>7} {'trigger[m]':>11} "
          f"{'placement[m]':>13} {'ay[m/s2]':>9}")
    for i, p in enumerate(points, 1):
        placement = (settle_time + p.ttc_s) * p.speed_mps
        print(f"{i:>3} {p.block:>5} {p.site:>10} {p.speed_kmh:>8g} "
              f"{p.offset_m:>10g} {p.ttc_s:>7g} {p.trigger_gap_m:>11.1f} "
              f"{placement:>13.1f} {p.ay_mps2:>9.2f}")
    if not points:
        print("  (no points — every speed is over the site's ay ceiling)")
        return
    worst = max((settle_time + p.ttc_s) * p.speed_mps for p in points)
    print(f"\n{len(points)} scenario points; longest approach {worst:.0f} m "
          f"along the lane (settle {settle_time:g} s + TTC).")


SUMMARY_FIELDS = list(RunMetrics().__dict__.keys())


def main(argv=None) -> int:
    global DECEL_LIMIT_MPS2
    install_sigterm_handler()
    args = parse_args(argv)
    DECEL_LIMIT_MPS2 = args.decel_limit

    if args.list_sites:
        print_sites()
        return 0

    points = resolve_points(args)
    if args.list:
        print_matrix(points, args.settle_time)
        return 0
    if args.repeats > 1:
        points = [replace(p, rep=r + 1)
                  for p in points for r in range(args.repeats)]
    check_no_bridge_conflict()
    # Sites live on different maps, and load_world destroys every actor —
    # including the ego and the camera the perception nodes are locked
    # onto. So the matrix is flown one site at a time, with the ego
    # rebuilt per site and the ROS bridge (and therefore the stack's view
    # of the world) held across the whole session. Points are grouped
    # rather than sorted, so a hand-built order is preserved.
    groups: list[tuple[str, list[ScenarioPoint]]] = []
    for p in points:
        if not groups or groups[-1][0] != p.site:
            groups.append((p.site, []))
        groups[-1][1].append(p)

    tag = args.tag or ('curve_' + '_'.join(s for s, _ in groups))
    out_dir = make_out_dir(__file__, tag, args.out_dir)
    print(f"[out] {out_dir}", flush=True)

    bridge, shutdown_ros = start_bridge('r171_curve_scenario_bridge')

    results: list[RunMetrics] = []
    geometry: dict = {}
    exit_code = 0
    done = 0
    stack_checked = args.no_stack_check
    try:
        for site_name, group in groups:
            site = SITES[site_name]
            if not args.no_carla_restart:
                # Reboot into the town rather than load_world() into it.
                # The stack stays up across this: a fresh server plus a
                # long-running ADAS stack is the combination that works.
                carla_server.ensure_town(site.town, host=args.host,
                                         port=args.port,
                                         quality=args.carla_quality)
            adapter = CarlaCurveAdapter(
                site=site, host=args.host, port=args.port,
                ego_bp=args.vehicle, target_bp=args.vehicle,
                weather=args.weather, sync=args.sync,
                clean_start=not args.keep_existing_actors)
            camera_pump: CameraPump | None = None
            try:
                version = adapter.connect()
                print(f"\n[sim] carla {version or ''} — {site.town} site "
                      f"{site.name}: R={adapter.measured_radius_m:.0f} m "
                      f"{site.direction}, arc {site.arc_m:.0f} m, lead-in "
                      f"{site.lead_in_m:.0f} m", flush=True)
                adapter.spawn_ego()

                if args.lateral_mode == 'lkas':
                    bridge.set_control_sink(adapter.apply_control)

                camera_pump = CameraPump(adapter, bridge)
                camera_pump.start()

                for _ in range(40):
                    adapter.wait_for_tick()
                    bridge.publish_speed(adapter.ego_state().speed)

                if not stack_checked:
                    if not wait_for_stack(bridge, args.stack_timeout):
                        raise SystemExit(
                            f"the ADAS stack never reported ready within "
                            f"{args.stack_timeout:g} s. Start it with "
                            f"`./start_adas.sh carla`, or pass "
                            f"--no-stack-check.")
                    stack_checked = True

                for point in group:
                    done += 1
                    print(f"\n[{done}/{len(points)}] {point.run_id}: "
                          f"{point.speed_kmh:g} km/h, offset "
                          f"{point.offset_m:+g} m, TTC {point.ttc_s:g} s -> "
                          f"handover at {point.trigger_gap_m:.1f} m (arc), "
                          f"curve demands {point.ay_mps2:.2f} m/s^2",
                          flush=True)
                    try:
                        m, geometry = CurvedRunner(adapter, bridge, args).run(
                            point, out_dir)
                    except KeyboardInterrupt:
                        raise
                    except Exception as exc:
                        m = RunMetrics(
                            run_id=point.run_id, block=point.block,
                            site=point.site, radius_m=site.radius_m,
                            speed_kmh=point.speed_kmh,
                            offset_m=point.offset_m, ttc_s=point.ttc_s,
                            trigger_gap_m=point.trigger_gap_m,
                            ay_demand_mps2=point.ay_mps2,
                            outcome='error', note=str(exc))
                        print(f"    ERROR: {exc}", flush=True)
                        exit_code = 1
                    results.append(m)
                    _report_point(m)
                    _write(out_dir, results, geometry, args)

                    if done < len(points):
                        deadline = time.time() + args.settle_between_runs
                        while time.time() < deadline:
                            adapter.wait_for_tick()
                            bridge.publish_speed(adapter.ego_state().speed)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                # One site failing must not take the rest of the run with
                # it. The first full sweep died exactly here: nine cells
                # in, the CARLA server went away while loading Town12, and
                # the remaining eighteen were never attempted — nor
                # recorded, so the summary silently ended early rather
                # than saying why (DEBUG §58).
                print(f"\n[site] {site_name} FAILED: {exc}\n"
                      f"[site] skipping its remaining "
                      f"{len(group) - sum(1 for m in results if m.site == site_name)}"
                      f" point(s) and continuing", flush=True)
                for point in group:
                    if any(m.run_id == point.run_id for m in results):
                        continue
                    done += 1
                    results.append(RunMetrics(
                        run_id=point.run_id, block=point.block,
                        site=point.site, radius_m=site.radius_m,
                        speed_kmh=point.speed_kmh,
                        offset_m=point.offset_m, ttc_s=point.ttc_s,
                        trigger_gap_m=point.trigger_gap_m,
                        ay_demand_mps2=point.ay_mps2,
                        outcome='error',
                        note=f'site {site_name} unavailable: {exc}'))
                exit_code = 1
                _write(out_dir, results, geometry, args)
            finally:
                # Order matters: detach the sink first so no late
                # cmd_steer can reach a dying adapter, then stop the pump
                # (generously — it may be mid-encode), then close.
                bridge.set_control_sink(None)
                if camera_pump is not None:
                    camera_pump.stop()
                    camera_pump.join(timeout=5.0)
                    if camera_pump.is_alive():
                        print('[warn] camera pump did not stop; leaving the '
                              'adapter open so its thread cannot RPC a dead '
                              'server', flush=True)
                        camera_pump = None
                        continue
                adapter.close()

    except KeyboardInterrupt:
        print("\n[abort] interrupted — writing what completed", flush=True)
        exit_code = 130
    finally:
        _write(out_dir, results, geometry, args)
        shutdown_ros()

    _print_report(results)
    print(f"\n[out] {out_dir}", flush=True)
    return exit_code


def _report_point(m: RunMetrics) -> None:
    det = ('none' if math.isnan(m.gap_at_first_detection_m)
           else f"{m.gap_at_first_detection_m:.0f} m @ "
                f"{m.bearing_at_first_detection_deg:+.1f} deg")
    onset = ('never' if math.isnan(m.brake_onset_gap_m)
             else f"{m.brake_onset_gap_m:.0f} m")
    flag = ('  <-- UNAVOIDABLE BY THEN' if m.unavoidable_at_onset
            else '  <-- OVER LIMIT' if m.exceeded_decel_limit else '')
    print(f"    -> {m.verdict.upper():<15} a_req: "
          f"{fmt_decel(m.a_req_at_trigger_mps2)} -> "
          f"{fmt_decel(m.a_req_at_brake_onset_mps2)} m/s^2 at brake onset "
          f"(limit {DECEL_LIMIT_MPS2:g}){flag}", flush=True)
    print(f"       first_detect={det}  onset={onset}  "
          f"min_gap={m.min_gap_m:.1f} m  |cte|max={m.max_abs_cte_m:.2f} m  "
          f"corridor_at_trigger="
          f"{'in' if m.target_in_straight_corridor_at_trigger else 'OUT'}",
          flush=True)


def _write(out_dir: Path, results: list[RunMetrics], geometry: dict,
           args) -> None:
    write_outputs(out_dir, results, SUMMARY_FIELDS,
                  'UN R171 Annex 4 4.2.5.2.2 — stationary vehicle ahead, '
                  'curved road', args,
                  extra=dict(geometry=geometry,
                             sites={k: asdict(v) for k, v in SITES.items()}))


def _print_report(results: list[RunMetrics]) -> None:
    if not results:
        return
    w = 104
    print("\n" + "=" * w)
    print(f"UN R171 Annex 4 §4.2.5.2.2 — curved road, deceleration demand "
          f"vs the {DECEL_LIMIT_MPS2:g} m/s^2 limit")
    print("=" * w)
    print(f"{'run':<34} {'verdict':<16} {'a_req@trig':>11} "
          f"{'a_req@brake':>12} {'detect[m]':>10} {'bearing':>8} "
          f"{'|cte|max':>9}")
    print("-" * w)
    for m in results:
        det = ('-' if math.isnan(m.gap_at_first_detection_m)
               else f"{m.gap_at_first_detection_m:.0f}")
        brg = ('-' if math.isnan(m.bearing_at_first_detection_deg)
               else f"{m.bearing_at_first_detection_deg:+.1f}")
        star = '*' if m.exceeded_decel_limit else ' '
        print(f"{m.run_id:<34} {m.verdict:<16} "
              f"{fmt_decel(m.a_req_at_trigger_mps2, 11)} "
              f"{fmt_decel(m.a_req_at_brake_onset_mps2, 11)}{star} "
              f"{det:>10} {brg:>8} {m.max_abs_cte_m:>9.2f}")
    print("-" * w)

    tally: dict = {}
    for m in results:
        tally[m.verdict] = tally.get(m.verdict, 0) + 1
    print(f"{len(results)} runs — " + ", ".join(
        f"{v}: {c}" for v, c in sorted(tally.items())))
    out = sum(1 for m in results
              if not m.target_in_straight_corridor_at_trigger)
    print(f"{out}/{len(results)} handed over with the target already "
          f"outside the straight-ahead corridor")
    seen = [m.gap_at_first_detection_m for m in results
            if math.isfinite(m.gap_at_first_detection_m)]
    if seen:
        print(f"first detection — min {min(seen):.0f} m, "
              f"median {sorted(seen)[len(seen) // 2]:.0f} m, "
              f"max {max(seen):.0f} m (compare the straight matrix at the "
              f"same speed to separate range from bearing)")


if __name__ == '__main__':
    sys.exit(main())

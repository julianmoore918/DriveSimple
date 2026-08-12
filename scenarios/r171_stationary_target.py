#!/usr/bin/env python3
"""UN R171 (DCAS) Annex 4 §4.2.5.2.1 — stationary vehicle ahead, straight road.

Longitudinal-only test against a single stationary target. This script
REPLACES carlaaccsim/carlaAccSimTown.py for the duration of a scenario
run: it is the ROS bridge (camera + speed out, cmd_vel + cmd_steer in)
*and* the scenario director. Do not run both at once — they would fight
over /Car_1/cmd_vel and the ego's apply_control.

The ADAS stack itself must already be up:

    ./start_adas.sh carla          # perception, controller, lane, stanley
    python3 scenarios/r171_stationary_target.py --matrix

Three parameters define a scenario point
----------------------------------------
    --speed-kmh        VUT approach speed
    --offset-m         target lateral offset from the lane centreline
                       (+ve = right of the direction of travel)
    --ttc-s            initial TTC margin — sets the handover distance

How a run is flown
------------------
    ARMING     ego parked on the start line, stationary target placed
               (settle_time + ttc) * v ahead, bumper-to-bumper.
    APPROACH   the scenario owns the throttle and holds the VUT at
               exactly `speed_kmh`. The stack's ACC is running and
               publishing, but its cmd_vel is ignored.
    HANDOVER   the instant the ground-truth gap falls to `ttc * v`, the
               scenario releases the longitudinal channel and the
               project's own ACC (controller/controller_node.py) owns
               throttle and brake for the rest of the run. This is the
               DCAS trigger point.
    MEASURE    log at 20 Hz until the VUT stops, hits the target, passes
               it, or the run times out.

Lateral is held by the scenario on the lane centreline by default
(`--lateral-mode locked`). §4.2.5.2.1 is a longitudinal test, and letting
Stanley wander at 130 km/h over a 400 m approach would corrupt the gap
measurement and the YOLO detection geometry. Use `--lateral-mode lkas` to
put Stanley in the loop instead.

Outputs, under --out-dir (default scenarios/results/<timestamp>/):
    <run_id>.csv      20 Hz trace of the run
    summary.csv       one row per scenario point
    manifest.json     full parameters + metrics + geometry

Why /ACC/target_speed is published before every run
---------------------------------------------------
controller_node's ACC mode is not purely a distance controller. Its final
command is

    throttle = min(acc_throttle, cruise_throttle)
    brake    = max(acc_brake,    cruise_brake)

so the cruise law (which tracks `self.target_speed`, default 25 km/h)
caps everything. Left at the default, an 130 km/h approach would sit far
above set speed, cruise would command brake continuously, and the run
would measure the cruise controller rather than the ACC PD law. The
scenario therefore publishes the test speed on /ACC/target_speed as part
of arming, which puts cruise in its ±0.5 m/s deadband and lets the PD law
own the braking decision.

That min/max combination is also load-bearing in the other direction: the
PD law's distance term saturates to a_max with a stationary target far
ahead (see below), and it is only the cruise cap that stops the VUT
accelerating into it.

What the PD law implies for this test
-------------------------------------
With a = k_p*(d - d_desired) + k_d*d_dot, d_desired = d0 + T_gap*v, and a
stationary target (d_dot = -v), the ACC first commands a <= 0 at

    d_brake = d0 + (T_gap + k_d/k_p) * v = 3.0 + 0.967*v      [CARLA gains]

That is a fixed *distance* schedule, not a TTC one, so the deceleration it
implies grows linearly with speed:

    v [km/h]    d_brake [m]    a_req at that point [m/s^2]
        30          11.1              3.1     under the 5 limit
        50          16.4              5.9     over
        70          21.8              8.7     over
        90          27.2             11.5     beyond tyre capability
       110          32.5             14.4     beyond tyre capability
       130          37.9             17.2     beyond tyre capability

So the controller's own gains — before perception range is considered at
all — bound where this matrix can pass. The harness measures the real
figure rather than assuming this one, but it is the expected shape of the
result, and `a_req_at_brake_onset_mps2` is the column that confirms it.
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
from sim_adapter import ADAPTERS, CollisionEvent  # noqa: E402,F401
from scenario_common import (  # noqa: E402
    LOG_HZ, DECEL_VALID_GAP_M, DECEL_VALID_SPEED_MPS, DECEL_PLAUSIBLE_MAX,
    DECEL_PHYSICAL_MAX, CameraPump, LaneHold, ScenarioBridge, SpeedHold,
    check_no_bridge_conflict, fmt_decel, install_sigterm_handler,
    make_out_dir, peak_decel_from_trace, required_decel, start_bridge,
    wait_for_stack, write_outputs, write_trace)


# A run ends when one of these trips.
STOP_SPEED_MPS = 0.3          # "stopped" threshold
STOP_HOLD_S = 1.0             # ...held this long
# While the controller is still asking for forward speed after the stop it
# is closing the last metres to d0, not finished. See the stop test below.
STOP_CREEP_VREF_KMH = 0.5
STOP_CREEP_TIMEOUT_S = 6.0    # hard bound so a stuck creep cannot hang a run
PASS_MARGIN_M = -5.0          # gap this negative = VUT has passed the target
TIMEOUT_MARGIN_S = 25.0       # on top of the nominal approach duration

BRAKE_ONSET_THRESHOLD = 0.10  # cmd_vel brake above this = the stack reacted

# Headline KPI. R171 treats ~5 m/s^2 as the ceiling for system-commanded
# DCAS braking; above it the manoeuvre is no longer a comfort-domain
# intervention. A run is only a clean pass if the collision was avoided
# AND the deceleration demanded to do so stayed under this.
DECEL_LIMIT_MPS2 = 5.0

# ---------------------------------------------------------------------------
# Scenario definition
# ---------------------------------------------------------------------------
@dataclass
class ScenarioPoint:
    speed_kmh: float
    offset_m: float
    ttc_s: float
    block: str = 'custom'
    rep: int = 0          # repeat index; keeps --repeats traces distinct

    @property
    def speed_mps(self) -> float:
        return self.speed_kmh / 3.6

    @property
    def trigger_gap_m(self) -> float:
        """Handover distance. TTC against a stationary target is just
        gap / v_ego, so gap = ttc * v."""
        return self.ttc_s * self.speed_mps

    @property
    def run_id(self) -> str:
        base = (f"{self.block}_v{self.speed_kmh:g}"
                f"_off{self.offset_m:g}_ttc{self.ttc_s:g}")
        # Without the suffix, --repeats writes every repeat's trace to the
        # same <run_id>.csv and only the last one survives.
        return f"{base}_r{self.rep}" if self.rep else base


SPEEDS_KMH = (30, 50, 70, 90, 110, 130)
OFFSETS_M = (0.0, 0.5, 1.0)
TTCS_S = (4.5, 6.0, 10.0)
NOMINAL_TTC_S = 6.0
NOMINAL_OFFSET_M = 0.0


def build_matrix() -> list[ScenarioPoint]:
    """Block A: speed x offset @ TTC=6 s          -> 18 points
       Block B: speed x TTC   @ offset=0 m        -> 12 further points

    Block B's TTC=6 s column is already in Block A (offset 0 is one of
    Block A's three offsets), so it contributes 12 new points, not 18.
    Total 30, matching the test plan.
    """
    points: list[ScenarioPoint] = []
    for v in SPEEDS_KMH:
        for off in OFFSETS_M:
            points.append(ScenarioPoint(v, off, NOMINAL_TTC_S, block='A'))
    for v in SPEEDS_KMH:
        for ttc in TTCS_S:
            if ttc == NOMINAL_TTC_S:
                continue          # already covered by Block A at offset 0
            points.append(ScenarioPoint(v, NOMINAL_OFFSET_M, ttc, block='B'))
    return points


# ---------------------------------------------------------------------------
# Run record
# ---------------------------------------------------------------------------
@dataclass
class RunMetrics:
    run_id: str = ''
    block: str = ''
    speed_kmh: float = 0.0
    offset_m: float = 0.0
    ttc_s: float = 0.0
    trigger_gap_m: float = 0.0

    v_at_trigger_kmh: float = 0.0
    gap_at_trigger_m: float = 0.0

    # ---- headline KPI: deceleration demand vs the 5 m/s^2 limit ----
    # What the scenario hands the system at the trigger point. Depends only
    # on the scenario parameters (v / 2*ttc), so it is the test's own
    # statement of difficulty.
    a_req_at_trigger_mps2: float = float('nan')
    # What the system actually needed by the time it started braking. The
    # gap between this and a_req_at_trigger is the cost of its reaction
    # delay, and it is the number that gets compared to 5 m/s^2.
    a_req_at_brake_onset_mps2: float = float('nan')
    # Worst demand seen at any point in the run.
    a_req_peak_mps2: float = float('nan')
    # What the vehicle actually achieved (impact impulse excluded).
    peak_decel_achieved_mps2: float = 0.0
    decel_limit_mps2: float = DECEL_LIMIT_MPS2
    exceeded_decel_limit: bool = False
    # a_req at brake onset already past what any tyre can deliver — the
    # impact was unavoidable the moment the system finally reacted.
    unavoidable_at_onset: bool = False
    decel_margin_mps2: float = float('nan')   # limit - a_req_at_brake_onset
    verdict: str = ''   # pass | pass_over_limit | fail_collision | no_reaction
    # Perception reach: the gap at which /ACC/lead_vehicle_distance first
    # went valid. At 130 km/h with a 10 s margin the trigger sits 361 m
    # out — far beyond what YOLO can resolve — so this column is what
    # explains a late or absent reaction.
    gap_at_first_detection_m: float = float('nan')
    detected_before_trigger: bool = False
    # Where controller_node first handed the command to its ACC PD branch
    # (as opposed to CRUISE). Distinct from brake onset: the PD law takes
    # authority as soon as a lead is tracked, but only commands braking
    # below d_brake = d0 + (T_gap + k_d/k_p)*v.
    gap_at_acc_engage_m: float = float('nan')

    brake_onset_gap_m: float = float('nan')
    brake_onset_ttc_s: float = float('nan')
    reaction_latency_s: float = float('nan')   # trigger -> first brake
    min_gap_m: float = float('inf')
    final_gap_m: float = float('nan')
    final_speed_kmh: float = 0.0

    collision: bool = False
    impact_speed_kmh: float = 0.0
    speed_reduction_kmh: float = 0.0
    outcome: str = ''            # stopped | collision | passed | timeout | error
    duration_s: float = 0.0
    note: str = ''


# ---------------------------------------------------------------------------
# The director
# ---------------------------------------------------------------------------
class ScenarioRunner:

    def __init__(self, adapter, bridge: ScenarioBridge, args):
        self.adapter = adapter
        self.bridge = bridge
        self.args = args
        self.speed_hold = SpeedHold()
        self.lane_hold = LaneHold()

    # -- one scenario point -------------------------------------------------
    def run(self, point: ScenarioPoint, out_dir: Path) -> tuple[RunMetrics, dict]:
        args = self.args
        v_set = point.speed_mps
        trigger_gap = point.trigger_gap_m
        placement = (args.settle_time + point.ttc_s) * v_set

        runway = self.adapter.straight_runway_m()
        if placement > runway - args.runway_reserve:
            raise RuntimeError(
                f"{point.run_id}: needs {placement:.0f} m of approach "
                f"(settle {args.settle_time:g} s + TTC {point.ttc_s:g} s at "
                f"{point.speed_kmh:g} km/h) but only {runway:.0f} m of "
                f"straight road is available (reserving "
                f"{args.runway_reserve:.0f} m). Pick a longer spawn.")

        geometry = self.adapter.arm(placement, point.offset_m)
        self.speed_hold.reset(SpeedHold.drag_throttle_estimate(v_set))
        self.lane_hold.reset()
        self.bridge.publish_not_in_junction()
        # Set the ACC's cruise cap to the test speed before the run, not
        # during it: controller_node logs at INFO on every /ACC/target_speed
        # message, so a periodic republish buries its own diagnostics. A
        # short burst covers the volatile QoS without the spam.
        for _ in range(5):
            self.bridge.publish_target_speed(point.speed_kmh)
            time.sleep(0.05)

        if args.approach_mode in ('physical', 'kinematic'):
            self.adapter.force_speed(v_set)   # snap to the test speed

        m = RunMetrics(run_id=point.run_id, block=point.block,
                       speed_kmh=point.speed_kmh, offset_m=point.offset_m,
                       ttc_s=point.ttc_s, trigger_gap_m=trigger_gap)
        # The dataclass default was bound at import; --decel-limit rebinds
        # the module global, so stamp the live value onto the record.
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

        while True:
            t = self.adapter.wait_for_tick()
            dt = max(t - prev_t, 1e-3)
            prev_t = t
            elapsed = t - t0

            ego = self.adapter.ego_state()
            gap = self.adapter.gap_m()
            lane = self.adapter.lane_error()   # one RPC, used by both
                                               # the lateral hold and the log
            self.bridge.publish_speed(ego.speed)
            perceived = self.bridge.lead_distance
            acc_mode = self.bridge.acc_mode
            tracked_gap = self.bridge.tracked_gap
            speed_ref = self.bridge.speed_ref
            pinhole_gap = self.bridge.pinhole_gap
            if (perceived is not None
                    and math.isnan(m.gap_at_first_detection_m)):
                m.gap_at_first_detection_m = gap
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
                if args.verbose:
                    print(f"    [handover] gap={gap:.1f} m  "
                          f"v={ego.speed * 3.6:.1f} km/h  "
                          f"a_req={a_req:.2f} m/s^2  "
                          f"perceived="
                          f"{'%.1f m' % perceived if perceived else 'none'}",
                          flush=True)

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
                    # The headline number: how much deceleration the system
                    # had left itself by the moment it decided to act.
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

            # Hand the latest longitudinal command to the bridge so a
            # steer callback arriving between iterations applies the right
            # throttle/brake alongside it.
            self.bridge.set_longitudinal(throttle, brake)
            with self.bridge._sink_lock:
                self.adapter.apply_control(throttle, brake, steer)

            # ---- metrics ----
            m.min_gap_m = min(m.min_gap_m, gap)
            if (handed_over and gap > DECEL_VALID_GAP_M
                    and ego.speed > DECEL_VALID_SPEED_MPS):
                # Gate on gap so the collision impulse — which the
                # speed-differentiated accel reads as hundreds of m/s^2 —
                # never lands in the achieved-deceleration figure, and on
                # speed so the standstill snap does not either.
                achieved = min(-ego.accel, DECEL_PLAUSIBLE_MAX)
                m.peak_decel_achieved_mps2 = max(
                    m.peak_decel_achieved_mps2, achieved)
                if math.isnan(m.a_req_peak_mps2):
                    m.a_req_peak_mps2 = a_req
                else:
                    m.a_req_peak_mps2 = max(m.a_req_peak_mps2, a_req)

            if elapsed - last_log >= 1.0 / LOG_HZ:
                last_log = elapsed
                samples.append(dict(
                    t=round(elapsed, 3),
                    phase='measure' if handed_over else 'approach',
                    v_kmh=round(ego.speed * 3.6, 3),
                    accel_mps2=round(ego.accel, 3),
                    gap_gt_m=round(gap, 3),
                    gap_perceived_m=(round(perceived, 3)
                                     if perceived is not None else ''),
                    ttc_s=round(gap / ego.speed, 3) if ego.speed > 0.1 else '',
                    a_req_mps2=(round(a_req, 3) if math.isfinite(a_req)
                                else ''),
                    # Which controller_node branch is driving, straight
                    # from the node. acc_pd_active is the boolean form:
                    # True only while the ACC PD distance law owns the
                    # command, so a glance at the column separates "the PD
                    # law is holding station" from "CRUISE is holding set
                    # speed" — the two look identical in throttle/brake.
                    acc_mode=acc_mode,
                    acc_pd_active=(acc_mode == 'ACC'),
                    # What the controller believes, vs gap_gt_m which is
                    # truth and gap_perceived_m which is the raw topic.
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
                    x=round(ego.x, 2), y=round(ego.y, 2)))

            # ---- stop conditions ----
            collision = self.adapter.take_collision()
            if collision is not None:
                m.collision = True
                m.impact_speed_kmh = collision.impact_speed * 3.6
                m.outcome = 'collision'
                break
            # "Stopped" means the vehicle is at rest AND the controller has
            # stopped asking it to move.
            #
            # Speed alone was not enough. CARLA snaps a vehicle's velocity
            # to zero from ~5 km/h, so a stop finishes wherever that snap
            # lands — measured, 3.46 m against a d0 of 2.0 — and the
            # controller then creeps the remainder. Ending the run 1.0 s
            # after the speed threshold cut that creep off at 2.75 m and
            # recorded a final gap the controller had not finished
            # producing. The creep needs ~2.0 s from 3.5 m.
            #
            # Gating on the reference as well ends the run when the
            # manoeuvre is actually over, rather than at a fixed timeout
            # after the wheels first stop. STOP_CREEP_TIMEOUT_S bounds it
            # so a controller that never settles cannot hang the matrix.
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
                        m.note = (m.note or '') + ' (creep did not settle '
                        m.note += f'within {STOP_CREEP_TIMEOUT_S:.0f} s)'
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

        # Recompute peak deceleration from the trace with a centred-window
        # slope of v(t), and take that as the reported figure.
        #
        # The online estimate (sim_adapter's EMA, alpha=0.3) both lags and
        # attenuates: on one 50 km/h run it reported a 7.29 m/s^2 peak
        # where the true value was 9.11. Every peak_decel figure produced
        # before this fix is understated by roughly that margin. A centred
        # window cannot be used online — it needs samples from after the
        # instant it describes — but the run record is offline by
        # definition, so the KPI has no reason to inherit the online
        # estimator's lag. Raw sample-to-sample differencing is not an
        # option either: it peaks at 23 m/s^2 on single-sample noise.
        peak = peak_decel_from_trace(samples)
        if peak is not None:
            m.peak_decel_achieved_mps2 = peak

        m.final_gap_m = self.adapter.gap_m()
        m.final_speed_kmh = self.adapter.ego_state().speed * 3.6
        m.speed_reduction_kmh = point.speed_kmh - (
            m.impact_speed_kmh if m.collision else m.final_speed_kmh)
        m.duration_s = round(prev_t - t0, 2)

        # ---- verdict against the 5 m/s^2 limit ----
        # Judged on the demand at brake onset, not on what the vehicle
        # managed to achieve: a system that reacts so late that stopping
        # *requires* more than the limit has already failed the DCAS
        # authority constraint, whether or not the tyres could deliver it.
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
    g.add_argument('--speed-kmh', type=float,
                   help='VUT approach speed [km/h].')
    g.add_argument('--offset-m', type=float, default=0.0,
                   help='Target lateral offset from the lane centreline [m]. '
                        '+ve = right of the direction of travel. Default 0.')
    g.add_argument('--ttc-s', type=float, default=6.0,
                   help='Initial TTC margin [s]. Handover to the ADAS stack '
                        'happens at gap = ttc * v. Default 6.')

    g = p.add_argument_group('matrix')
    g.add_argument('--matrix', action='store_true',
                   help='Run the full 30-point Annex 4 matrix '
                        '(Block A: speed x offset @ TTC 6 s; '
                        'Block B: speed x TTC @ offset 0 m).')
    g.add_argument('--block', choices=['A', 'B'],
                   help='Restrict --matrix to one block.')
    g.add_argument('--list', action='store_true',
                   help='Print the matrix and exit. Needs no simulator.')
    g.add_argument('--repeats', type=int, default=1,
                   help='Repeat each scenario point N times. Default 1.')

    g = p.add_argument_group('simulator')
    g.add_argument('--sim', choices=list(ADAPTERS), default='carla',
                   help='Target simulator. MORAI is a stub for now.')
    g.add_argument('--host', default='localhost')
    g.add_argument('--port', type=int, default=2000)
    g.add_argument('--town', default='Town06',
                   help='Default Town06 — 718 m of dead-straight highway at '
                        'spawn 80, the only stock map that fits the matrix. '
                        'Town03 spawn 5 has 0 m (junction at 2 m).')
    g.add_argument('--spawn-index', type=int, default=80,
                   help='Start line. Default 80 = Town06 road 48, centre '
                        'lane of five, 3.5 m wide, 718 m straight.')
    g.add_argument('--vehicle', default='vehicle.dodge.charger_2020',
                   help='Blueprint for BOTH the VUT and the target.')
    g.add_argument('--weather', default='ClearNoon')
    g.add_argument('--keep-existing-actors', action='store_true',
                   help='Do not clear vehicles/sensors left over from a '
                        'previous session before starting. Off by default: '
                        'a Ctrl-C mid-matrix leaves the ego parked on the '
                        'start line and the next run then cannot spawn.')
    g.add_argument('--sync', action='store_true',
                   help='Run the world in synchronous mode. Off by default — '
                        'sync mode regressed forward motion in this '
                        'multi-process setup (DEBUG.md §4).')

    g = p.add_argument_group('run shaping')
    g.add_argument('--settle-time', type=float, default=3.0,
                   help='Seconds of steady-state approach before the trigger '
                        'point. Scales with speed, so the placement distance '
                        'is (settle + ttc) * v. Default 3.')
    g.add_argument('--approach-mode', choices=['physical', 'kinematic'],
                   default='physical',
                   help='physical (default): snap to the test speed, then '
                        'hold it with a PI on throttle so the vehicle is '
                        'physically settled at handover. kinematic: force '
                        'the velocity every tick for an exact approach '
                        'speed, at the cost of an unloaded tyre model.')
    g.add_argument('--lateral-mode', choices=['locked', 'lkas'],
                   default='locked',
                   help='locked (default): the scenario holds the lane '
                        'centreline, isolating the longitudinal result. '
                        'lkas: Stanley owns steer via /Car_1/cmd_steer.')
    g.add_argument('--decel-limit', type=float, default=DECEL_LIMIT_MPS2,
                   help=f'Deceleration ceiling for the pass/fail verdict '
                        f'[m/s^2]. Default {DECEL_LIMIT_MPS2:g} — a run only '
                        f'passes cleanly if the collision was avoided AND '
                        f'the deceleration required at brake onset stayed '
                        f'under this.')
    g.add_argument('--runway-reserve', type=float, default=60.0,
                   help='Straight road kept in hand beyond the target [m]. '
                        'Default 60.')
    g.add_argument('--timeout-extra', type=float, default=0.0,
                   help='Extra seconds on the per-run timeout.')
    g.add_argument('--settle-between-runs', type=float, default=2.0,
                   help='Pause between scenario points [s], so perception '
                        'clears the previous target. Default 2.')

    g = p.add_argument_group('output')
    g.add_argument('--out-dir', default=None,
                   help='Default scenarios/results/<timestamp>/.')
    g.add_argument('--tag', default='',
                   help='Label folded into the output directory name.')
    g.add_argument('--verbose', action='store_true')
    g.add_argument('--no-stack-check', action='store_true',
                   help='Skip waiting for the perception model_ready flags.')
    g.add_argument('--stack-timeout', type=float, default=120.0,
                   help='How long to wait for YOLO + UFLD [s]. Default 120.')
    return p.parse_args(argv)


def resolve_points(args) -> list[ScenarioPoint]:
    if args.matrix or args.list:
        points = build_matrix()
        if args.block:
            points = [p for p in points if p.block == args.block]
        return points
    if args.speed_kmh is None:
        raise SystemExit(
            "give --speed-kmh (with optional --offset-m / --ttc-s) for a "
            "single point, or --matrix for the full 30-point run. "
            "--list prints the matrix without touching a simulator.")
    return [ScenarioPoint(args.speed_kmh, args.offset_m, args.ttc_s,
                          block='single')]


def print_matrix(points: list[ScenarioPoint], settle_time: float) -> None:
    print(f"{'#':>3} {'block':>5} {'v[km/h]':>8} {'offset[m]':>10} "
          f"{'TTC[s]':>7} {'trigger_gap[m]':>15} {'placement[m]':>13}")
    for i, p in enumerate(points, 1):
        placement = (settle_time + p.ttc_s) * p.speed_mps
        print(f"{i:>3} {p.block:>5} {p.speed_kmh:>8g} {p.offset_m:>10g} "
              f"{p.ttc_s:>7g} {p.trigger_gap_m:>15.1f} {placement:>13.1f}")
    worst = max((settle_time + p.ttc_s) * p.speed_mps for p in points)
    print(f"\n{len(points)} scenario points; "
          f"longest approach {worst:.0f} m "
          f"(settle {settle_time:g} s + TTC).")


SUMMARY_FIELDS = list(RunMetrics().__dict__.keys())


def main(argv=None) -> int:
    global DECEL_LIMIT_MPS2
    install_sigterm_handler()
    args = parse_args(argv)
    DECEL_LIMIT_MPS2 = args.decel_limit
    points = resolve_points(args)

    if args.list:
        print_matrix(points, args.settle_time)
        return 0

    if args.repeats > 1:
        points = [replace(p, rep=r + 1)
                  for p in points for r in range(args.repeats)]

    if args.sim == 'carla':
        check_no_bridge_conflict()

    out_dir = make_out_dir(__file__, args.tag, args.out_dir)
    print(f"[out] {out_dir}", flush=True)

    adapter_cls = ADAPTERS[args.sim]
    adapter = adapter_cls(host=args.host, port=args.port, town=args.town,
                          spawn_index=args.spawn_index,
                          ego_bp=args.vehicle, target_bp=args.vehicle,
                          weather=args.weather, sync=args.sync,
                          clean_start=not args.keep_existing_actors)

    bridge, shutdown_ros = start_bridge('r171_scenario_bridge')

    results: list[RunMetrics] = []
    geometry: dict = {}
    camera_pump: CameraPump | None = None
    exit_code = 0
    try:
        version = adapter.connect()
        print(f"[sim] {args.sim} {version or ''} — {args.town} "
              f"spawn {args.spawn_index}", flush=True)
        adapter.spawn_ego()
        runway = adapter.straight_runway_m()
        print(f"[sim] straight runway ahead of the start line: "
              f"{runway:.0f} m", flush=True)

        if args.lateral_mode == 'lkas':
            # Zero-latency steer path — see ScenarioBridge._on_cmd_steer.
            bridge.set_control_sink(adapter.apply_control)

        camera_pump = CameraPump(adapter, bridge)
        camera_pump.start()

        # Let perception load and lock before the first run is armed.
        for _ in range(40):
            adapter.wait_for_tick()
            bridge.publish_speed(adapter.ego_state().speed)

        if not args.no_stack_check:
            if not wait_for_stack(bridge, args.stack_timeout):
                raise SystemExit(
                    f"the ADAS stack never reported ready within "
                    f"{args.stack_timeout:g} s. Start it with "
                    f"`./start_adas.sh carla`, or pass --no-stack-check to "
                    f"run the scenario against a silent stack.")

        for i, point in enumerate(points, 1):
            print(f"\n[{i}/{len(points)}] {point.run_id}: "
                  f"{point.speed_kmh:g} km/h, offset {point.offset_m:+g} m, "
                  f"TTC {point.ttc_s:g} s -> handover at "
                  f"{point.trigger_gap_m:.1f} m", flush=True)
            try:
                m, geometry = ScenarioRunner(adapter, bridge, args).run(
                    point, out_dir)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                m = RunMetrics(run_id=point.run_id, block=point.block,
                               speed_kmh=point.speed_kmh,
                               offset_m=point.offset_m, ttc_s=point.ttc_s,
                               trigger_gap_m=point.trigger_gap_m,
                               outcome='error', note=str(exc))
                print(f"    ERROR: {exc}", flush=True)
                exit_code = 1
            results.append(m)

            det = ('none' if math.isnan(m.gap_at_first_detection_m)
                   else f"{m.gap_at_first_detection_m:.0f} m")
            onset = ('never' if math.isnan(m.brake_onset_gap_m)
                     else f"{m.brake_onset_gap_m:.0f} m")
            a_trig = fmt_decel(m.a_req_at_trigger_mps2)
            a_onset = fmt_decel(m.a_req_at_brake_onset_mps2)
            flag = ('  <-- UNAVOIDABLE BY THEN' if m.unavoidable_at_onset
                    else '  <-- OVER LIMIT' if m.exceeded_decel_limit else '')
            print(f"    -> {m.verdict.upper():<15} "
                  f"a_req: {a_trig} m/s^2 at trigger -> {a_onset} m/s^2 at "
                  f"brake onset (limit {DECEL_LIMIT_MPS2:g}){flag}",
                  flush=True)
            print(f"       achieved {m.peak_decel_achieved_mps2:.2f} m/s^2  |  "
                  f"min_gap={m.min_gap_m:.1f} m  first_detect={det}  "
                  f"onset={onset}  impact={m.impact_speed_kmh:.1f} km/h",
                  flush=True)

            _write_r171_outputs(out_dir, results, geometry, args)
            if i < len(points):
                deadline = time.time() + args.settle_between_runs
                while time.time() < deadline:
                    adapter.wait_for_tick()
                    bridge.publish_speed(adapter.ego_state().speed)

    except KeyboardInterrupt:
        print("\n[abort] interrupted — writing what completed", flush=True)
        exit_code = 130
    finally:
        _write_r171_outputs(out_dir, results, geometry, args)
        if camera_pump is not None:
            camera_pump.stop()
            camera_pump.join(timeout=2.0)
        adapter.close()
        shutdown_ros()

    _print_report(results)
    print(f"\n[out] {out_dir}", flush=True)
    return exit_code


def _write_r171_outputs(out_dir: Path, results: list[RunMetrics],
                        geometry: dict, args) -> None:
    write_outputs(out_dir, results, SUMMARY_FIELDS,
                  'UN R171 Annex 4 4.2.5.2.1 — stationary vehicle ahead, '
                  'straight road', args, extra=dict(geometry=geometry))


def _print_report(results: list[RunMetrics]) -> None:
    if not results:
        return
    w = 96
    print("\n" + "=" * w)
    print(f"UN R171 Annex 4 §4.2.5.2.1 — deceleration demand vs the "
          f"{DECEL_LIMIT_MPS2:g} m/s^2 limit")
    print("=" * w)
    print(f"{'run':<26} {'verdict':<16} {'a_req@trig':>11} "
          f"{'a_req@brake':>12} {'achieved':>9} {'min_gap':>8} {'detect':>7}")
    print("-" * w)
    for m in results:
        det = ('-' if math.isnan(m.gap_at_first_detection_m)
               else f"{m.gap_at_first_detection_m:.0f}")
        star = '*' if m.exceeded_decel_limit else ' '
        print(f"{m.run_id:<26} {m.verdict:<16} "
              f"{fmt_decel(m.a_req_at_trigger_mps2, 11)} "
              f"{fmt_decel(m.a_req_at_brake_onset_mps2, 11)}{star} "
              f"{m.peak_decel_achieved_mps2:>9.2f} "
              f"{m.min_gap_m:>8.1f} {det:>7}")
    print("-" * w)

    n = len(results)
    tally = {}
    for m in results:
        tally[m.verdict] = tally.get(m.verdict, 0) + 1
    over = sum(1 for m in results if m.exceeded_decel_limit)
    print(f"{n} runs — " + ", ".join(
        f"{v}: {c}" for v, c in sorted(tally.items())))
    print(f"* {over}/{n} needed more than {DECEL_LIMIT_MPS2:g} m/s^2 "
          f"by the time the system braked")

    reacted = [m.a_req_at_brake_onset_mps2 for m in results
               if math.isfinite(m.a_req_at_brake_onset_mps2)]
    if reacted:
        print(f"a_req at brake onset — min {min(reacted):.2f}, "
              f"median {sorted(reacted)[len(reacted) // 2]:.2f}, "
              f"max {max(reacted):.2f} m/s^2")


if __name__ == '__main__':
    sys.exit(main())

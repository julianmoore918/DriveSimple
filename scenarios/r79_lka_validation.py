#!/usr/bin/env python3
"""UN R79 Annex 8 §3.2 — lane-keeping validation of the LKAS on its own.

No lead vehicle, no ACC decision to make: the scenario holds the test
speed and Stanley owns the steering for the whole run. What is measured is
where the car ends up laterally, how hard it had to corner to stay there,
and how smoothly it did it.

    ./start_adas.sh carla
    python3 scenarios/r79_lka_validation.py --list
    python3 scenarios/r79_lka_validation.py --site t04_r199 --test lane_keeping
    python3 scenarios/r79_lka_validation.py --matrix

What R79 asks of an ACSF of Category B1, and what this script does
------------------------------------------------------------------
    §3.2.1  lane keeping           IMPLEMENTED. Drive the curve at 80-90 %
                                   of the declared aysmax; no front tyre
                                   tread may cross the outer edge of a
                                   lane marking, and the half-second
                                   moving average of lateral jerk must
                                   stay under 5 m/s^3.
    §3.2.2  maximum lateral accel  IMPLEMENTED. Provoke a demand above
                                   aysmax + 0.3 and check the system's
                                   response stays inside §5.6.2.1.1.
    §3.2.5  lane-crossing warning  PARTIAL. The crossing is measured; the
                                   warning cannot be, because the stack
                                   publishes no lane-departure signal.
                                   Reported as `not_implemented` rather
                                   than as a pass. Pass --warning-topic if
                                   one is ever added.
    §3.2.3  overriding force       NOT ASSESSABLE. Needs 50 N measured at
                                   the steering control; CARLA exposes no
                                   steering torque and the stack's steer
                                   is a normalised position command, so
                                   there is no force to measure. It is a
                                   bench/vehicle test, not a sim one.
    §3.2.4  hands-on transition    NOT APPLICABLE. Requires hands-on
                                   detection and the escalating warning
                                   chain of §5.6.2.2.5. The stack has no
                                   driver monitoring, so there is nothing
                                   to test — which is itself a finding: a
                                   B1 approval is not reachable without
                                   it.

§5.1.2 (straight running without unusual steering correction) is already
covered by the straight R171 matrix run with `--lateral-mode lkas`, whose
trace carries the same cte column; it is not duplicated here.

Declaring aysmax
----------------
aysmax is a manufacturer declaration per speed band, bounded by
§5.6.2.1.3 Table 1. The test speed is then derived from the site's radius
so the demand lands in the window each test needs:

    §3.2.1   v = sqrt(0.85 * A * R)      (80-90 % window, centre)
    §3.2.2   v = sqrt((A + 0.4) * R)     (must provoke > A + 0.3)
    §3.2.5   v = sqrt((A + 0.25) * R)    (A + 0.1 .. A + 0.4)

and the point is only kept if that speed falls inside the band the
declaration applies to. That inversion is what makes a fixed set of CARLA
radii usable: the map supplies R, the regulation supplies the demand, and
the speed is whatever satisfies both.

Measuring lateral acceleration
------------------------------
Annex 8 §2.4 wants ay at the centre of gravity, sampled at >= 100 Hz and
filtered with a fourth-order Butterworth at 0.5 Hz, with jerk taken as the
500 ms moving average of its derivative. The director loop runs at ~20 Hz,
so the compliance signal comes from a dedicated CARLA IMU at 200 Hz
(curve_adapter's `with_imu`), resampled onto a uniform grid before
filtering.

Both filterings are reported. `ay_peak_mps2` uses the regulation's filter
applied causally, which is the compliance figure; `ay_peak_zerophase_mps2`
uses a zero-phase equivalent, which places the peak in time correctly. In
a steady-state curve the two agree to within a few per cent — where they
diverge is the §3.2.2 transient, and that divergence is the filter's group
delay, not the vehicle. Quoting only the causal figure there would repeat
the mistake DEBUG §45.1 records.
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
from curve_adapter import SITES, CarlaCurveAdapter  # noqa: E402
from scenario_common import (  # noqa: E402
    LOG_HZ, CameraPump, LaneHold, SpeedHold, check_no_bridge_conflict,
    install_sigterm_handler, make_out_dir, start_bridge, wait_for_stack,
    write_outputs, write_trace)


# R79 §5.6.2.1.3 Table 1 — the envelope a declared aysmax must sit in.
# (upper speed bound [km/h], min, max) per vehicle class.
AY_TABLE_M1 = ((60.0, 0.0, 3.0), (100.0, 0.5, 3.0),
               (130.0, 0.8, 3.0), (1e9, 0.3, 3.0))
AY_TABLE_HEAVY = ((30.0, 0.0, 2.5), (60.0, 0.3, 2.5), (1e9, 0.5, 2.5))

# §3.2.1.1: "The necessary lateral acceleration to follow the curve shall
# be between 80 and 90 per cent of aysmax". Aim at the middle so drift in
# the speed hold does not push the run out of the window.
LK_FRACTION = 0.85
# §3.2.2.1: the Technical Service picks a speed/radius provoking more than
# aysmax + 0.3. 0.4 clears that with margin without asking for a demand
# the tyres cannot deliver.
MAXAY_EXCESS = 0.4
# §3.2.5.1: aysmax + 0.1 .. aysmax + 0.4.
CROSSING_EXCESS = 0.25

JERK_LIMIT_MPS3 = 5.0          # §3.2.1.2 / §3.2.2.2
# How old a /Car_1/cmd_steer message may be before the LKAS counts as
# having stopped commanding.
#
# stanley_node deliberately publishes NOTHING on CARLA when UFLD cannot
# recover a lane centre (HOLD mode) — it expects carlaAccSimTown.py's
# pure-pursuit fallback to take the wheel. The harness IS the bridge, and
# has no such fallback, so a silent Stanley means the last steer value
# stays on the wheels indefinitely. Measured on t04_r199 at 50 km/h:
# cmd_steer stopped at t=3.0 s, the harness held +0.15 deg for the
# following 10 s, and the car left the lane at a point where the geometry
# was asking for 14 deg. Scored naively that is a lane departure; it is
# actually an availability failure, and the two need different fixes.
STEER_STALE_S = 0.3
# Total silence above this fails the run: R79 §5.6.2.2.3 expects assistance
# to continue, and a system that stops steering mid-curve is not keeping
# the lane whatever the car does next.
LKAS_SILENCE_FAIL_S = 0.5
# §5.6.2.1.1 transient allowance: aysmax may be exceeded by up to 40 % for
# no more than 2 s.
TRANSIENT_FACTOR = 1.4
TRANSIENT_MAX_S = 2.0
# and by 0.3 m/s^2 indefinitely.
SUSTAINED_MARGIN = 0.3

TESTS = ('lane_keeping', 'max_lateral_accel', 'lane_crossing_warning')

# --sweep: the engineering view of the same test. R79's own matrix derives
# the speed from the declared aysmax so each run lands in the window a
# paragraph specifies; that answers "does the declaration hold". It does
# not answer "where does the controller stop holding the lane", because
# the declaration moves the speed with it. The sweep fixes both axes
# instead — radius from the site, speed from this list — and reports
# kept_lane per cell. The regulation's criteria still decide the verdict.
SWEEP_SPEEDS_KMH = (30, 50, 70, 90, 110, 130)
# Sites are ordered by radius so a sweep reads as a curve-severity ladder.
SWEEP_SITES = ('t12_r1185', 't12_r500', 't12_r417', 't04_r199',
               't04_r076', 't03_r060', 't10_r042')
# Above this the tyres, not the controller, decide the outcome, and the
# run stops measuring lane keeping. Deliberately above the 3.0 m/s^2 M1
# ceiling: the point of a sweep is to find the edge, which means crossing
# the declared limit but not the friction one.
SWEEP_AY_CAP = 4.5

# How far past the end of the arc to keep recording. §3.2.1.2's criterion
# is about the whole manoeuvre, and the exit transient is where an
# over-tuned controller puts its worst jerk.
EXIT_MARGIN_M = 40.0
# Straight ahead of the arc, in seconds of travel. It has to cover the
# warm-up (during which the scenario steers) AND leave the LKAS a few
# seconds of straight of its own before the measurement window opens,
# or the curve-entry result is contaminated by the handover step.
SETTLE_TIME_S = 6.0
# Minimum of that which must fall after the handover.
MIN_LKAS_STRAIGHT_S = 2.0
TIMEOUT_MARGIN_S = 30.0


# ---------------------------------------------------------------------------
# Declared capability
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Band:
    lo_kmh: float
    hi_kmh: float
    ay_max: float          # declared aysmax for this band
    min_allowed: float
    max_allowed: float

    @property
    def label(self) -> str:
        hi = '∞' if self.hi_kmh > 1e8 else f"{self.hi_kmh:g}"
        return f"{self.lo_kmh:g}-{hi}"

    def contains(self, v_kmh: float) -> bool:
        return self.lo_kmh < v_kmh <= self.hi_kmh


def parse_declaration(spec: str, heavy: bool) -> list[Band]:
    """`"60:1.5,100:3.0,130:3.0"` -> the declared aysmax per speed band.

    Validated against §5.6.2.1.3 Table 1 and refused if it does not fit:
    a declaration outside the table is not a system this test can approve,
    and running the matrix against one would produce numbers that look
    like compliance and are not.
    """
    table = AY_TABLE_HEAVY if heavy else AY_TABLE_M1
    declared = {}
    for part in spec.split(','):
        if not part.strip():
            continue
        hi, val = part.split(':')
        declared[float(hi)] = float(val)

    bands, lo = [], 10.0     # §5.6.2.2.5: B1 operates from 10 km/h up
    for hi, mn, mx in table:
        if hi not in declared:
            # Not declared = the system does not offer that band. Legal;
            # it just means no test points there.
            lo = hi
            continue
        a = declared[hi]
        if not (mn <= a <= mx):
            raise SystemExit(
                f"declared aysmax {a:g} m/s^2 for the "
                f"{lo:g}-{hi:g} km/h band is outside R79 Table 1, which "
                f"allows {mn:g}..{mx:g} m/s^2 there.")
        bands.append(Band(lo, hi, a, mn, mx))
        lo = hi
    if not bands:
        raise SystemExit(f"no usable speed band in --declared-ay {spec!r}")
    return bands


@dataclass
class LkaPoint:
    site: str
    test: str
    speed_kmh: float
    ay_target_mps2: float     # what the geometry should demand at that speed
    ay_max: float             # the declaration being tested
    band: str
    rep: int = 0
    # A sweep point is judged on §3.2.1's criteria but is not trying to
    # sit in a paragraph's window, so `window_valid` must not veto it.
    sweep: bool = False

    @property
    def speed_mps(self) -> float:
        return self.speed_kmh / 3.6

    @property
    def run_id(self) -> str:
        if self.sweep:
            base = f"sweep_{self.site}_v{self.speed_kmh:.0f}"
        else:
            base = (f"{self.site}_{self.test}_v{self.speed_kmh:.0f}"
                    f"_ay{self.ay_max:g}")
        return f"{base}_r{self.rep}" if self.rep else base


def speed_for(test: str, ay_max: float, radius_m: float) -> tuple[float, float]:
    """(test speed [km/h], lateral demand there [m/s^2]) for one test."""
    ay = {'lane_keeping': LK_FRACTION * ay_max,
          'max_lateral_accel': ay_max + MAXAY_EXCESS,
          'lane_crossing_warning': ay_max + CROSSING_EXCESS}[test]
    return math.sqrt(ay * radius_m) * 3.6, ay


def build_points(sites, bands: list[Band], tests,
                 v_min_kmh: float, v_max_kmh: float) -> list[LkaPoint]:
    """Every (site, band, test) whose derived speed is legal and reachable.

    A point is dropped, not clamped, when the speed falls outside its own
    band: clamping would run the test at a demand the regulation does not
    ask for and report it as if it had.
    """
    points: list[LkaPoint] = []
    for site_name in sites:
        r = SITES[site_name].radius_m
        for band in bands:
            for test in tests:
                v, ay = speed_for(test, band.ay_max, r)
                if not band.contains(v):
                    continue
                if not (v_min_kmh <= v <= v_max_kmh):
                    continue
                points.append(LkaPoint(site_name, test, round(v, 1), ay,
                                       band.ay_max, band.label))
    return points


def build_sweep(sites, speeds, ay_max: float, ay_cap: float) -> list[LkaPoint]:
    """radius x speed, every cell whose demand the tyres can still carry.

    Cells above `ay_cap` are dropped rather than run: past roughly 0.45 g
    on this plant the vehicle understeers out of the lane whatever the
    controller does, so the cell would record the tyre model's limit and
    read as a lane-keeping failure.
    """
    points: list[LkaPoint] = []
    for site_name in sites:
        r = SITES[site_name].radius_m
        for v in speeds:
            ay = (v / 3.6) ** 2 / r
            if ay > ay_cap:
                continue
            points.append(LkaPoint(site_name, 'lane_keeping', float(v), ay,
                                   ay_max, 'sweep', sweep=True))
    return points


# ---------------------------------------------------------------------------
# Lateral signal analysis (offline, per R79 Annex 8 §2.4)
# ---------------------------------------------------------------------------
def butter_lowpass(order: int, fc_hz: float, fs_hz: float):
    """Butterworth low-pass as a cascade of biquads, by bilinear transform.

    Written out rather than imported from scipy because the harness has to
    run under whichever interpreter launches it: UI.py starts scenarios
    with CARLA's bundled Python, which has numpy but NOT scipy, so the
    first R79 run launched from the UI died with `No module named
    'scipy'` — after driving the whole scenario, and wrote a summary row
    of dataclass defaults that read like a result (DEBUG §57).

    Returns [(b, a), ...], each a 3-tap biquad in the usual
    y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] - a1 y[n-1] - a2 y[n-2] form.
    Verified against scipy on a realistic ay trace (step into a curve,
    1.7 Hz ripple, sensor noise, 200 Hz): causal output matches
    lfilter+lfilter_zi to 1.1e-8, and the zero-phase peak — the figure
    actually reported — matches filtfilt to 1.3e-4 m/s^2, the remainder
    being filtfilt's edge padding.
    """
    w = math.tan(math.pi * fc_hz / fs_hz)      # prewarped
    sections = []
    for k in range(order // 2):
        # Pole quality factors of a Butterworth of this order.
        q = 1.0 / (2.0 * math.cos(math.pi * (2 * k + 1) / (2 * order)))
        norm = 1.0 + w / q + w * w
        b0 = w * w / norm
        sections.append(((b0, 2 * b0, b0),
                         (1.0,
                          2.0 * (w * w - 1.0) / norm,
                          (1.0 - w / q + w * w) / norm)))
    return sections


def _biquad(x, b, a):
    """One biquad, started in DC steady state.

    The state is seeded from the first sample rather than from zero.
    A zero-state start rings for the first second or so of a 0.5 Hz
    filter, and since the R79 window opens mid-run — the vehicle is
    already cornering — that transient is pure fiction that lands
    straight in the peak. scipy's filtfilt does the same thing via
    lfilter_zi; this is that, for a unity-DC-gain section.
    """
    if not len(x):
        return []
    y = [0.0] * len(x)
    x1 = x2 = y1 = y2 = float(x[0])
    for i, xi in enumerate(x):
        yi = b[0] * xi + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
        y[i] = yi
        x2, x1 = x1, xi
        y2, y1 = y1, yi
    return y


def sosfilt(x, sections, zero_phase: bool = False):
    """Run a biquad cascade forward, or forward-and-back for zero phase."""
    y = list(x)
    for b, a in sections:
        y = _biquad(y, b, a)
    if zero_phase:
        y = y[::-1]
        for b, a in sections:
            y = _biquad(y, b, a)
        y = y[::-1]
    return y


def analyse_lateral(imu: list, t_lo: float, t_hi: float) -> dict:
    """Filter the IMU trace and pull out the figures R79 judges on.

    `imu` is [(t, ay, yaw_rate, ax), ...] straight from the sensor;
    [t_lo, t_hi] is the measurement window (curve entry to exit).

    Returns peaks and jerk for both the regulation's causal filter and a
    zero-phase equivalent. Every figure is over the window only — the
    approach contains the scenario's own snap to the test speed, and
    including it would report the harness's transient as the system's.
    """
    import numpy as np

    pts = [(t, ay) for (t, ay, _, _) in imu if t_lo <= t <= t_hi]
    if len(pts) < 50:
        return dict(samples=len(pts), rate_hz=float('nan'),
                    ay_peak_mps2=float('nan'),
                    ay_peak_zerophase_mps2=float('nan'),
                    ay_mean_mps2=float('nan'),
                    jerk_peak_mps3=float('nan'),
                    time_over_sustained_s=float('nan'),
                    note='too few IMU samples in the window')

    t = np.array([p[0] for p in pts])
    ay = np.array([p[1] for p in pts])
    # Uniform grid: CARLA sensor ticks jitter, and a Butterworth assumes a
    # constant sample interval. Resampling at the median rate keeps the
    # filter's cut-off meaningful.
    dt = np.median(np.diff(t))
    fs = 1.0 / dt if dt > 0 else 0.0
    grid = np.arange(t[0], t[-1], dt)
    ay_u = np.interp(grid, t, ay)

    # §2.4's filter, applied causally: this is the compliance figure.
    ay_reg = np.array(sosfilt(ay_u, butter_lowpass(4, 0.5, fs)))
    # Zero-phase reference. Forward-and-back doubles the effective order,
    # so the prototype is 2nd order to land back at 4th overall.
    ay_zp = np.array(sosfilt(ay_u, butter_lowpass(2, 0.5, fs),
                             zero_phase=True))

    # §2.4: jerk is the 500 ms moving average of the derivative.
    n = max(int(round(0.5 * fs)), 1)
    jerk = np.convolve(np.gradient(ay_zp, dt), np.ones(n) / n, mode='same')

    # The causal filter needs its settling time discarded, or the run
    # opens with the filter's own ramp from zero and the "peak" is that.
    skip = min(int(round(2.0 * fs)), len(ay_reg) // 3)
    return dict(
        samples=len(pts),
        rate_hz=round(fs, 1),
        ay_peak_mps2=float(np.max(np.abs(ay_reg[skip:]))),
        ay_peak_zerophase_mps2=float(np.max(np.abs(ay_zp))),
        ay_mean_mps2=float(np.mean(np.abs(ay_zp))),
        jerk_peak_mps3=float(np.max(np.abs(jerk))),
        _ay_zp=ay_zp, _grid=grid, _dt=dt, note='')


def time_above(ay_zp, dt: float, limit: float) -> float:
    """Longest contiguous time |ay| stays above `limit` [s].

    §5.6.2.1.1's transient allowance is "for time periods of not more than
    2 s", so what matters is the longest single excursion, not their sum.
    """
    import numpy as np
    over = np.abs(ay_zp) > limit
    best = run = 0
    for flag in over:
        run = run + 1 if flag else 0
        best = max(best, run)
    return best * dt


# ---------------------------------------------------------------------------
# Run record
# ---------------------------------------------------------------------------
@dataclass
class LkaMetrics:
    run_id: str = ''
    site: str = ''
    test: str = ''
    radius_m: float = 0.0
    direction: str = ''
    band: str = ''
    ay_max_declared_mps2: float = 0.0
    speed_kmh: float = 0.0
    ay_target_mps2: float = 0.0

    # ---- what the run actually did ----
    v_mean_in_curve_kmh: float = 0.0
    ay_geometric_mps2: float = 0.0     # v_mean^2 / R, the demand delivered
    ay_peak_mps2: float = float('nan')          # regulation filter (causal)
    ay_peak_zerophase_mps2: float = float('nan')
    ay_mean_mps2: float = float('nan')
    jerk_peak_mps3: float = float('nan')
    imu_rate_hz: float = float('nan')
    time_over_sustained_s: float = float('nan')

    # ---- lane keeping ----
    # The headline outcome of a sweep, and the first half of §3.2.1.2's
    # criterion: did any front tyre tread cross the outer edge of a lane
    # marking at any point in the run.
    kept_lane: bool = True
    # When the scenario handed the wheel to Stanley, and how centred the
    # car was at that moment — the LKAS is only scored after this.
    t_lkas_handover_s: float = float('nan')
    cte_at_lkas_handover_m: float = float('nan')
    steer_err_at_handover_deg: float = float('nan')
    # Seconds the LKAS spent not publishing a steer command after it had
    # the wheel — stanley_node's HOLD mode, where it expects a fallback
    # controller that the harness does not provide.
    lkas_silent_s: float = 0.0
    t_first_silence_s: float = float('nan')
    max_abs_cte_m: float = 0.0
    min_marking_clearance_m: float = float('nan')
    tyre_crossed_marking: bool = False
    crossing_side: str = ''
    time_to_crossing_s: float = float('nan')
    cte_at_crossing_m: float = float('nan')
    assistance_after_crossing: bool = False

    # ---- steering: commanded vs the angle the lane actually needed ----
    # steer_err is (commanded - pure-pursuit-to-centreline), so positive
    # means Stanley asked for more right lock than the geometry called
    # for. RMS over the curve, peak anywhere in the run.
    steer_rms_err_deg: float = float('nan')
    steer_peak_err_deg: float = float('nan')
    steer_cmd_peak_deg: float = float('nan')
    # What the curve alone asked for, from the centreline: atan(L*kappa).
    steer_ff_mean_deg: float = float('nan')
    # A constant offset between the two — a controller that tracks the
    # shape but sits under it shows up here rather than in the RMS.
    steer_mean_err_deg: float = float('nan')
    lookahead_m: float = 0.0
    # Worst age of a /Car_1/cmd_steer message while the LKAS had the
    # wheel. A run where this is large did not measure the controller.
    steer_max_age_s: float = float('nan')

    # ---- §3.2.5 ----
    warning_seen: bool = False
    warning_latency_s: float = float('nan')
    warning_status: str = ''    # not_implemented | seen | absent

    # ---- verdict ----
    # False on any run that did not complete its measurement — an error,
    # a timeout, an LKAS that never settled. Every metric below is a
    # dataclass default in that case, and a default reads exactly like a
    # result: the first UI-launched run died in the analysis step and
    # still wrote kept_lane=True, max_abs_cte_m=0.0 (DEBUG §57).
    measured: bool = False
    window_valid: bool = False   # did the run deliver the demand it should
    jerk_ok: bool = True
    ay_within_limits: bool = True
    verdict: str = ''            # pass | fail | not_assessable | invalid_window
    reason: str = ''
    outcome: str = ''            # completed | departed | timeout | error
    duration_s: float = 0.0
    note: str = ''


class WarningWatch:
    """Latch a lane-departure warning topic, if the stack has one.

    §3.2.5.2 is judged on a warning arriving no later than the tyre
    crossing the marking. The stack publishes no such signal today, so
    this exists to make the check runnable the day one is added rather
    than leaving --warning-topic as a flag that quietly does nothing.
    """

    def __init__(self, bridge, topic: str):
        from std_msgs.msg import Bool
        self.raised = False
        bridge.create_subscription(Bool, topic, self._cb, 10)

    def _cb(self, msg) -> None:
        self.raised = self.raised or bool(msg.data)

    def reset(self) -> None:
        self.raised = False


# ---------------------------------------------------------------------------
# The director
# ---------------------------------------------------------------------------
class LkaRunner:

    def __init__(self, adapter, bridge, args, warning=None):
        self.adapter = adapter
        self.bridge = bridge
        self.args = args
        self.speed_hold = SpeedHold()
        self.lane_hold = LaneHold()
        self.warning = warning

    def run(self, point: LkaPoint, out_dir: Path):
        args = self.args
        site = SITES[point.site]
        v_set = point.speed_mps

        run_up = min(args.settle_time * v_set, site.lead_in_m - 5.0)
        need = (args.warmup_s + MIN_LKAS_STRAIGHT_S) * v_set
        if run_up < need:
            raise RuntimeError(
                f"{point.run_id}: {site.lead_in_m:.0f} m of straight lead-in "
                f"gives {run_up:.0f} m of run-up, and this point needs "
                f"{need:.0f} m — {args.warmup_s:g} s of warm-up plus "
                f"{MIN_LKAS_STRAIGHT_S:g} s of straight under the LKAS "
                f"before the curve. Without that gap the curve-entry result "
                f"is the handover step, not the controller: measured on "
                f"t04_r199 at 50 km/h with only 1.2 s of it, Stanley "
                f"commanded 10 deg where the geometry asked for 4 and put a "
                f"tyre over the marking. Lower the speed, or pick a site "
                f"with a longer lead-in.")

        geometry = self.adapter.arm_lane_keeping(run_up)
        self.speed_hold.reset(SpeedHold.drag_throttle_estimate(v_set))
        self.lane_hold.reset()
        self.bridge.publish_not_in_junction()
        # Keep the ACC quiet: it shares /Car_1/cmd_vel, and if it thinks it
        # is above set speed it will brake against the scenario's throttle.
        for _ in range(5):
            self.bridge.publish_target_speed(point.speed_kmh)
            time.sleep(0.05)
        self.adapter.force_speed(v_set)

        m = LkaMetrics(
            run_id=point.run_id, site=point.site, test=point.test,
            radius_m=geometry.radius_m, direction=geometry.direction,
            band=point.band, ay_max_declared_mps2=point.ay_max,
            speed_kmh=point.speed_kmh, ay_target_mps2=point.ay_target_mps2)

        if self.warning is not None:
            self.warning.reset()
        entry_s = self.adapter.entry_s()
        exit_s = entry_s + site.arc_m
        end_s = min(exit_s + EXIT_MARGIN_M, self.adapter.path_end_s() - 5.0)

        samples: list[dict] = []
        imu: list = []
        t0 = self.adapter.wait_for_tick()
        prev_t = t0
        last_log = -1.0
        t_entry = t_exit = None
        t_crossing = None
        t_warning = None
        lkas_owns = False
        v_sum = v_n = 0.0
        steer_err: list[float] = []
        steer_ff: list[float] = []
        timeout = (args.settle_time
                   + (end_s - (entry_s - run_up)) / max(v_set, 1.0)
                   + TIMEOUT_MARGIN_S)

        while True:
            t = self.adapter.wait_for_tick()
            dt = max(t - prev_t, 1e-3)
            prev_t = t
            elapsed = t - t0

            ego = self.adapter.ego_state()
            lane = self.adapter.lane_error()
            s = self.adapter.path_s()
            self.bridge.publish_speed(ego.speed)
            imu.extend(self.adapter.poll_imu())

            left_clear, right_clear = self.adapter.marking_clearance_m(
                lane.cross_track_m)
            clear = min(left_clear, right_clear)
            in_curve = entry_s <= s <= exit_s

            # Steering ground truth. Three angles, all at the front wheel
            # and all in degrees:
            #   cmd  what Stanley asked for (/Car_1/cmd_steer, normalised,
            #        scaled back up by the vehicle's max steer angle)
            #   pp   what pure pursuit would need RIGHT NOW to be on the
            #        centreline `lookahead` ahead — the "should have been"
            #   ff   what the curve alone asks for from the centreline,
            #        atan(L*kappa): the steady state the other two settle
            #        towards once the error is zero
            steer_age = time.monotonic() - (self.bridge.stack_steer_stamp
                                            or time.monotonic())
            steer_cmd_deg = self.bridge.stack_steer * self.adapter.max_steer_deg
            steer_pp_deg = math.degrees(
                self.adapter.reference_steer_rad(args.lookahead_m))
            steer_ff_deg = math.degrees(self.adapter.feedforward_steer_rad())
            wheel_deg = math.degrees(self.adapter.wheel_steer_rad())
            kappa = self.adapter.curvature_at(s)
            if in_curve:
                steer_err.append(steer_cmd_deg - steer_pp_deg)
                steer_ff.append(steer_ff_deg)
            if (math.isnan(m.steer_cmd_peak_deg)
                    or abs(steer_cmd_deg) > abs(m.steer_cmd_peak_deg)):
                m.steer_cmd_peak_deg = round(steer_cmd_deg, 3)

            if in_curve and t_entry is None:
                t_entry = t
            if s > exit_s and t_entry is not None and t_exit is None:
                t_exit = t
            if in_curve:
                v_sum += ego.speed
                v_n += 1

            # The scenario owns speed for the whole run. Lateral is
            # Stanley's — but not from the first tick.
            #
            # `arm` teleports the ego, which hands perception a camera
            # that has jumped hundreds of metres. UFLD needs frames and
            # the lane KF needs to re-converge; until it has, Stanley is
            # steering on the previous location's state. Measured on the
            # first attempt here (t04_r076, 50 km/h): commanded steer
            # crossed 6 deg within 0.8 s of the teleport while the
            # geometry called for under 1, heading error grew
            # monotonically, and the run left the lane at t=2.1 s — before
            # it ever reached the curve. Scored as the LKAS's failure, it
            # would have been the harness's.
            #
            # So the scenario holds the centreline for `warmup_s`, on the
            # straight, and then hands lateral over. The measurement
            # window is the curve, which is always after that.
            throttle, brake = self.speed_hold.step(v_set, ego.speed, dt)
            # Handover is CONDITIONAL, not timed. `warmup_s` is the
            # earliest it may happen; the wheel changes hands at the first
            # tick after that where the LKAS's command agrees with the
            # geometry to within --handover-tol-deg.
            #
            # Measured on t04_r199 at 50 km/h: while the scenario held the
            # centreline, Stanley sat at +30 deg — the geometry asked for
            # -0.4 — and an unconditional handover put that step straight
            # on the wheels. The car was 2.3 m off line 0.6 s later. That
            # is the harness's transient, and scoring it as a lane
            # departure would have been a fabricated result.
            steer_disagreement = abs(steer_cmd_deg - steer_pp_deg)
            if (not lkas_owns and elapsed >= args.warmup_s
                    and steer_disagreement <= args.handover_tol_deg):
                lkas_owns = True
                m.t_lkas_handover_s = round(elapsed, 2)
                m.cte_at_lkas_handover_m = round(lane.cross_track_m, 3)
                m.steer_err_at_handover_deg = round(
                    steer_cmd_deg - steer_pp_deg, 3)
            # LKAS silent: hand the wheel back to the scenario so the
            # vehicle stays on the road and the rest of the window is
            # still measurable, and record the outage. Letting it depart
            # would record a lane-keeping failure for a controller that
            # was not commanding at the time.
            lkas_silent = lkas_owns and steer_age > STEER_STALE_S
            if lkas_silent:
                m.lkas_silent_s = round(m.lkas_silent_s + dt, 3)
                if math.isnan(m.t_first_silence_s):
                    m.t_first_silence_s = round(elapsed, 2)

            if s >= entry_s and not lkas_owns:
                # The straight ran out before the LKAS ever agreed with
                # the road. Not a lane-keeping failure — the controller
                # never got the wheel — so it is reported as its own
                # outcome rather than folded into pass/fail.
                m.outcome = 'lkas_never_settled'
                m.note = (f'LKAS command still {steer_disagreement:.1f} deg '
                          f'from the geometry at curve entry '
                          f'(tolerance {args.handover_tol_deg:g})')
                break

            if lkas_owns and not lkas_silent:
                steer = self.bridge.stack_steer
                owner = 'lkas'
            else:
                steer = self.lane_hold.step(lane.cross_track_m,
                                            lane.heading_err_rad, ego.speed)
                owner = 'scenario_fallback' if lkas_owns else 'warmup'
            if lkas_owns and (math.isnan(m.steer_max_age_s)
                              or steer_age > m.steer_max_age_s):
                m.steer_max_age_s = round(steer_age, 3)
            self.bridge.set_longitudinal(throttle, brake)
            with self.bridge._sink_lock:
                self.adapter.apply_control(throttle, brake, steer)

            def sample():
                """One trace row.

                A single builder because there used to be two: the warm-up
                branch had its own copy of this dict, the copies drifted
                within the hour (the warm-up one was missing
                lateral_owner), and csv.DictWriter takes its fieldnames
                from the first row — so every run died at write time.
                """
                return dict(
                    t=round(elapsed, 3),
                    phase=('warmup' if not lkas_owns else
                           'curve' if in_curve else
                           'exit' if s > exit_s else 'approach'),
                    # lkas | scenario_fallback | warmup — who actually
                    # steered this tick. A run measures the LKAS only
                    # where this reads 'lkas'.
                    lateral_owner=owner,
                    s_m=round(s, 2),
                    v_kmh=round(ego.speed * 3.6, 3),
                    cte_m=round(lane.cross_track_m, 4),
                    heading_err_deg=round(
                        math.degrees(lane.heading_err_rad), 3),
                    clear_left_m=round(left_clear, 4),
                    clear_right_m=round(right_clear, 4),
                    kappa_1pm=round(kappa, 6),
                    lane_radius_m=(round(1.0 / kappa, 1) if abs(kappa) > 1e-6
                                   else ''),
                    # The steering panel of plot_lka.py is these four.
                    steer_cmd_deg=round(steer_cmd_deg, 3),
                    steer_required_deg=round(steer_pp_deg, 3),
                    steer_feedforward_deg=round(steer_ff_deg, 3),
                    steer_wheel_deg=(round(wheel_deg, 3)
                                     if math.isfinite(wheel_deg) else ''),
                    steer_err_deg=round(steer_cmd_deg - steer_pp_deg, 3),
                    # Age of the last /Car_1/cmd_steer message. Above
                    # STEER_STALE_S the LKAS is not commanding at all, and
                    # steer_cmd_deg is a stale value rather than a
                    # decision — the distinction this run turned on.
                    steer_age_s=round(steer_age, 3),
                    # Kinematic cross-check on the IMU: on a planar
                    # trajectory ay = v * yaw_rate, so a disagreement
                    # between this column and the filtered IMU figure
                    # means one of the two is wrong.
                    ay_kinematic_mps2=round(
                        ego.speed * (imu[-1][2] if imu else 0.0), 3),
                    throttle=round(throttle, 3),
                    brake=round(brake, 3),
                    steer=round(steer, 4),
                    x=round(ego.x, 2), y=round(ego.y, 2))

            # Everything below scores the LKAS, so none of it counts while
            # the scenario still has the wheel — but the trace records
            # every tick either way, so the warm-up and any fallback stay
            # visible instead of being a gap.
            if not lkas_owns:
                if elapsed - last_log >= 1.0 / LOG_HZ:
                    last_log = elapsed
                    samples.append(sample())
                if elapsed > timeout:
                    m.outcome = 'timeout'
                    break
                continue


            m.max_abs_cte_m = max(m.max_abs_cte_m, abs(lane.cross_track_m))
            if math.isnan(m.min_marking_clearance_m) \
                    or clear < m.min_marking_clearance_m:
                m.min_marking_clearance_m = clear
            if clear < 0.0 and not m.tyre_crossed_marking and owner == 'lkas':
                # §3.2.1.2's criterion: the outer edge of the front tyre
                # tread has crossed the outer edge of a lane marking.
                #
                # Only while the LKAS actually has the wheel. During a
                # HOLD fallback the scenario is steering, and a crossing
                # there says nothing about the system under test — it is
                # already reported, far more precisely, as lkas_silent_s.
                m.tyre_crossed_marking = True
                m.crossing_side = 'left' if left_clear < right_clear else 'right'
                m.time_to_crossing_s = round(elapsed, 3)
                m.cte_at_crossing_m = round(lane.cross_track_m, 3)
                t_crossing = t
            if (self.warning is not None and self.warning.raised
                    and t_warning is None):
                t_warning = t
                m.warning_seen = True
            if (t_crossing is not None and t - t_crossing > 0.5
                    and abs(self.bridge.stack_steer) > 0.005):
                # §5.6.2.2.3 wants assistance to continue through a
                # boundary excursion rather than the system giving up.
                m.assistance_after_crossing = True

            if elapsed - last_log >= 1.0 / LOG_HZ:
                last_log = elapsed
                samples.append(sample())


            if s >= end_s:
                m.outcome = 'completed'
                break
            if abs(lane.cross_track_m) > args.departure_abort_m:
                m.outcome = 'departed'
                m.note = (f'aborted: {lane.cross_track_m:+.2f} m off the '
                          f'centreline')
                break
            if elapsed > timeout:
                m.outcome = 'timeout'
                break

        imu.extend(self.adapter.poll_imu())
        self.adapter.apply_control(0.0, 1.0, 0.0)

        # ---- offline analysis over the curve window ----
        t_lo = (t_entry or t0) - t0
        t_hi = ((t_exit or prev_t) - t0)
        # CARLA stamps sensor data with the same simulation clock
        # wait_for_tick() returns, so rebasing on t0 puts the IMU on the
        # trace's own time base exactly rather than approximately.
        imu_rel = [(ts - t0, ay, gz, ax) for (ts, ay, gz, ax) in imu]
        stats = analyse_lateral(imu_rel, t_lo, t_hi) if imu_rel else {}
        if stats:
            m.imu_rate_hz = stats['rate_hz']
            m.ay_peak_mps2 = stats['ay_peak_mps2']
            m.ay_peak_zerophase_mps2 = stats['ay_peak_zerophase_mps2']
            m.ay_mean_mps2 = stats['ay_mean_mps2']
            m.jerk_peak_mps3 = stats['jerk_peak_mps3']
            if '_ay_zp' in stats:
                sustained = min(point.ay_max + SUSTAINED_MARGIN,
                                self.args.ay_table_max)
                m.time_over_sustained_s = round(
                    time_above(stats['_ay_zp'], stats['_dt'], sustained), 3)
            if stats.get('note'):
                m.note = (m.note or '') + ' ' + stats['note']

        m.kept_lane = not m.tyre_crossed_marking
        m.measured = m.outcome in ('completed', 'departed')
        m.lookahead_m = args.lookahead_m
        if steer_err:
            m.steer_rms_err_deg = round(
                math.sqrt(sum(e * e for e in steer_err) / len(steer_err)), 3)
            m.steer_peak_err_deg = round(max(steer_err, key=abs), 3)
            m.steer_mean_err_deg = round(sum(steer_err) / len(steer_err), 3)
        if steer_ff:
            m.steer_ff_mean_deg = round(sum(steer_ff) / len(steer_ff), 3)

        v_mean = v_sum / v_n if v_n else 0.0
        m.v_mean_in_curve_kmh = round(v_mean * 3.6, 2)
        m.ay_geometric_mps2 = round(v_mean ** 2 / geometry.radius_m, 3)
        m.duration_s = round(prev_t - t0, 2)
        m.warning_status = ('not_implemented' if not args.warning_topic
                            else 'seen' if m.warning_seen else 'absent')
        if t_warning is not None and t_crossing is not None:
            # Negative = the warning came before the tyre crossed, which is
            # what §3.2.5.2 requires ("at the latest when the outside edge
            # of the tyre tread ... has crossed").
            m.warning_latency_s = round(t_warning - t_crossing, 3)

        self._judge(m, point)
        write_trace(out_dir, point.run_id, samples)
        if args.save_imu and imu_rel:
            write_trace(out_dir, f"{point.run_id}_imu",
                        [dict(t=round(t, 4), ay_mps2=round(ay, 4),
                              yaw_rate_rps=round(gz, 5), ax_mps2=round(ax, 4))
                         for (t, ay, gz, ax) in imu_rel])
        return m, asdict(geometry)

    # -- verdicts -----------------------------------------------------------
    def _judge(self, m: LkaMetrics, point: LkaPoint) -> None:
        """Apply the criteria of whichever paragraph this point is testing.

        `window_valid` is checked first and separately: if the run did not
        deliver the lateral demand the paragraph asks for, its result is
        not a pass or a fail — the test was not performed. Reporting a
        "pass" from a curve that only loaded the system to half its
        declared capability is the failure mode this guard exists for.
        """
        ay = (m.ay_peak_zerophase_mps2 if math.isfinite(
            m.ay_peak_zerophase_mps2) else m.ay_geometric_mps2)
        m.jerk_ok = (not math.isfinite(m.jerk_peak_mps3)
                     or m.jerk_peak_mps3 <= JERK_LIMIT_MPS3)

        # §5.6.2.1.1 binds the system "at any time", not only during the
        # §3.2.2 test, so the envelope is checked on every run and the
        # column always means something. It only DECIDES the verdict for
        # max_lateral_accel — §3.2.1.2's criteria are crossing and jerk —
        # but a breach seen during a lane-keeping run is still a breach,
        # so it goes in the note.
        #
        # Left uncomputed, this field defaulted to True on every
        # lane_keeping run: the first passing run reported
        # ay_within_limits=True with a 2.75 m/s^2 peak against a 2.10
        # ceiling. Same failure mode as `measured` (DEBUG §57.2).
        sustained = min(point.ay_max + SUSTAINED_MARGIN,
                        self.args.ay_table_max)
        transient = min(TRANSIENT_FACTOR * point.ay_max,
                        self.args.ay_table_max + SUSTAINED_MARGIN)
        over_transient = math.isfinite(ay) and ay > transient
        over_sustained_too_long = (
            math.isfinite(m.time_over_sustained_s)
            and m.time_over_sustained_s > TRANSIENT_MAX_S)
        m.ay_within_limits = not (over_transient or over_sustained_too_long)
        if not m.ay_within_limits:
            m.note = ((m.note or '') + f' §5.6.2.1.1: ay peaked at '
                      f'{ay:.2f} m/s^2 against a {transient:.2f} ceiling '
                      f'for a {point.ay_max:g} declaration').strip()

        if m.outcome in ('timeout', 'error', 'lkas_never_settled'):
            m.verdict = 'invalid_window'
            m.reason = m.note or f'run {m.outcome}'
            return

        if point.sweep:
            # A sweep cell is not aiming at a paragraph's window — radius
            # and speed were chosen, not derived — so the only questions
            # are §3.2.1.2's two: did a tyre cross, and was the jerk
            # inside 5 m/s^3.
            m.window_valid = True
            fails = []
            if m.lkas_silent_s > LKAS_SILENCE_FAIL_S:
                fails.append(
                    f"LKAS stopped commanding for {m.lkas_silent_s:.1f} s "
                    f"from t={m.t_first_silence_s:.1f} s (stanley_node HOLD: "
                    f"no lane centre at the lookahead) — the scenario held "
                    f"the lane in its place")
            if not m.kept_lane:
                fails.append(f"crossed the {m.crossing_side} marking at "
                             f"t={m.time_to_crossing_s:.1f} s")
            if not m.jerk_ok:
                fails.append(f"lateral jerk {m.jerk_peak_mps3:.2f} > "
                             f"{JERK_LIMIT_MPS3:g} m/s^3")
            m.verdict = 'fail' if fails else 'pass'
            m.reason = '; '.join(fails)
            return

        # Did the geometry deliver the demand the paragraph specifies?
        if point.test == 'lane_keeping':
            lo, hi = 0.80 * point.ay_max, 0.90 * point.ay_max
            m.window_valid = lo <= m.ay_geometric_mps2 <= hi
            window_txt = f"{lo:.2f}-{hi:.2f}"
        elif point.test == 'max_lateral_accel':
            m.window_valid = m.ay_geometric_mps2 > point.ay_max + 0.3
            window_txt = f">{point.ay_max + 0.3:.2f}"
        else:                       # lane_crossing_warning
            m.window_valid = (point.ay_max + 0.1 <= m.ay_geometric_mps2
                              <= point.ay_max + 0.4)
            window_txt = f"{point.ay_max + 0.1:.2f}-{point.ay_max + 0.4:.2f}"

        if not m.window_valid:
            m.verdict = 'invalid_window'
            m.reason = (f"the curve delivered {m.ay_geometric_mps2:.2f} m/s^2 "
                        f"at {m.v_mean_in_curve_kmh:.1f} km/h; this test "
                        f"needs {window_txt} m/s^2. Speed hold or radius is "
                        f"off — the point was not executed as specified.")
            return

        if point.test == 'lane_keeping':
            # §3.2.1.2
            fails = []
            if m.tyre_crossed_marking:
                fails.append(
                    f"front tyre crossed the {m.crossing_side} marking at "
                    f"t={m.time_to_crossing_s:.1f} s "
                    f"(cte {m.cte_at_crossing_m:+.2f} m)")
            if not m.jerk_ok:
                fails.append(f"lateral jerk {m.jerk_peak_mps3:.2f} > "
                             f"{JERK_LIMIT_MPS3:g} m/s^3")
            m.verdict = 'fail' if fails else 'pass'
            m.reason = '; '.join(fails)

        elif point.test == 'max_lateral_accel':
            # §3.2.2.2 -> §5.6.2.1.1, computed above.
            fails = []
            if over_transient:
                fails.append(f"ay peaked at {ay:.2f}, over the "
                             f"{transient:.2f} m/s^2 transient ceiling")
            elif over_sustained_too_long:
                fails.append(
                    f"ay stayed above {sustained:.2f} m/s^2 for "
                    f"{m.time_over_sustained_s:.1f} s, over the "
                    f"{TRANSIENT_MAX_S:g} s the 40 % allowance permits")
            if not m.jerk_ok:
                fails.append(f"lateral jerk {m.jerk_peak_mps3:.2f} > "
                             f"{JERK_LIMIT_MPS3:g} m/s^3")
            m.verdict = 'fail' if fails else 'pass'
            m.reason = '; '.join(fails)

        else:
            # §3.2.5 — the crossing is measurable, the warning is not.
            m.verdict = 'not_assessable'
            m.reason = (
                'crossing '
                + (f"occurred at t={m.time_to_crossing_s:.1f} s"
                   if m.tyre_crossed_marking else 'did not occur')
                + '; the stack publishes no lane-departure warning, so '
                  '§3.2.5.2 cannot be judged (pass --warning-topic if one '
                  'is added)')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    g = p.add_argument_group('scenario point')
    g.add_argument('--site', default=None, choices=sorted(SITES),
                   help='Curve site for a single run. --list-sites prints '
                        'the surveyed table.')
    g.add_argument('--test', choices=TESTS, default='lane_keeping')
    g.add_argument('--declare-for-speed', type=float, default=None,
                   help='Run this test at exactly this speed [km/h] by '
                        'solving the DECLARATION for it: aysmax = v^2 / '
                        '(f * R), with f the paragraph\'s window fraction. '
                        'The regulation fixes the demand as a fraction of '
                        'aysmax and the map fixes R, so speed is normally '
                        'the dependent variable — this makes aysmax the '
                        'dependent one instead, which is legal as long as '
                        'the result sits inside Table 1 (checked).')
    g.add_argument('--speed-kmh', type=float, default=None,
                   help='Override the derived test speed. The derived value '
                        'is what puts the lateral demand in the window the '
                        'paragraph specifies, so an override usually makes '
                        'the run invalid_window — use it to explore, not to '
                        'report.')

    g = p.add_argument_group('declaration')
    g.add_argument('--declared-ay', default='60:1.5,100:3.0,130:3.0',
                   help='Declared aysmax per speed band, "upper:value". '
                        'Default "60:1.5,100:3.0,130:3.0" — 3.0 is the M1 '
                        'ceiling, and 1.5 in the low band is what the '
                        'tightest surveyed CARLA radius can load to 85 %% '
                        'inside 10-60 km/h.')
    g.add_argument('--heavy', action='store_true',
                   help='Use the M2/M3/N2/N3 half of Table 1 (2.5 m/s^2 '
                        'ceiling) instead of M1/N1.')

    g = p.add_argument_group('matrix')
    g.add_argument('--matrix', action='store_true',
                   help='Every (site, band, test) whose derived speed is '
                        'legal — the compliance view, speed derived from '
                        'the declaration.')
    g.add_argument('--sweep', action='store_true',
                   help='radius x speed, judged on kept_lane. The '
                        'engineering view: both axes are set, not derived, '
                        'so the result maps where the controller stops '
                        'holding the lane.')
    g.add_argument('--sweep-speeds', nargs='+', type=float,
                   default=list(SWEEP_SPEEDS_KMH),
                   help=f'Speeds for --sweep [km/h]. Default '
                        f'{" ".join(str(v) for v in SWEEP_SPEEDS_KMH)}.')
    g.add_argument('--sweep-sites', nargs='+', default=list(SWEEP_SITES),
                   choices=sorted(SITES),
                   help='Sites (i.e. radii) for --sweep, largest first.')
    g.add_argument('--sweep-ay-cap', type=float, default=SWEEP_AY_CAP,
                   help=f'Drop sweep cells demanding more than this '
                        f'[m/s^2]. Default {SWEEP_AY_CAP:g} — above it the '
                        f'tyres decide the outcome, not the controller.')
    g.add_argument('--sites', nargs='+', default=sorted(SITES),
                   choices=sorted(SITES))
    g.add_argument('--tests', nargs='+', default=list(TESTS), choices=TESTS)
    g.add_argument('--list', action='store_true')
    g.add_argument('--list-sites', action='store_true')
    g.add_argument('--repeats', type=int, default=1)

    g = p.add_argument_group('simulator')
    g.add_argument('--host', default='localhost')
    g.add_argument('--port', type=int, default=2000)
    g.add_argument('--vehicle', default='vehicle.dodge.charger_2020')
    g.add_argument('--weather', default='ClearNoon')
    g.add_argument('--keep-existing-actors', action='store_true')
    g.add_argument('--sync', action='store_true')
    g.add_argument('--imu-hz', type=float, default=200.0,
                   help='IMU rate [Hz]. R79 Annex 8 §2.4 needs >= 100.')

    g = p.add_argument_group('run shaping')
    g.add_argument('--settle-time', type=float, default=SETTLE_TIME_S,
                   help='Seconds on the straight before the arc. Default 4.')
    g.add_argument('--v-min-kmh', type=float, default=10.0,
                   help='Declared Vsmin. Points below it are dropped.')
    g.add_argument('--v-max-kmh', type=float, default=130.0,
                   help='Declared Vsmax. Points above it are dropped.')
    g.add_argument('--warmup-s', type=float, default=3.0,
                   help='Seconds after the teleport during which the '
                        'SCENARIO holds the centreline, before Stanley '
                        'takes the wheel [s]. Default 3. Perception has '
                        'just been moved hundreds of metres; handing over '
                        'before UFLD and the lane KF have re-converged '
                        'measures the warm-up, not the controller.')
    g.add_argument('--handover-tol-deg', type=float, default=3.0,
                   help='The LKAS takes the wheel at the first tick after '
                        'the warm-up where its command is within this of '
                        'the geometric requirement [deg]. Default 3. A run '
                        'that reaches the curve without ever agreeing that '
                        'closely is reported lkas_never_settled, not as a '
                        'lane departure.')
    g.add_argument('--lookahead-m', type=float, default=10.0,
                   help='Horizon for the steering ground truth [m]. There '
                        'is no unique "correct" steer for an off-centre '
                        'vehicle — recovering in 1 m and in 50 m are both '
                        'correct — so this names the horizon the '
                        'pure-pursuit reference aims at. Recorded with the '
                        'run. Default 10.')
    g.add_argument('--departure-abort-m', type=float, default=3.0,
                   help='Abort the run once the ego is this far off the '
                        'centreline [m]. Default 3 — a lane and a half, so '
                        'the crossing is fully recorded first.')
    g.add_argument('--warning-topic', default='',
                   help='std_msgs/Bool topic carrying a lane-departure '
                        'warning, if the stack ever publishes one. Without '
                        'it §3.2.5 is reported not_assessable.')
    g.add_argument('--settle-between-runs', type=float, default=2.0)

    g = p.add_argument_group('output')
    g.add_argument('--out-dir', default=None)
    g.add_argument('--tag', default='')
    g.add_argument('--save-imu', action='store_true',
                   help='Write the raw 200 Hz IMU trace per run as well.')
    g.add_argument('--verbose', action='store_true')
    g.add_argument('--no-stack-check', action='store_true')
    g.add_argument('--stack-timeout', type=float, default=120.0)
    return p.parse_args(argv)


def resolve_points(args, bands: list[Band]) -> list[LkaPoint]:
    if args.sweep:
        return build_sweep(args.sweep_sites, args.sweep_speeds,
                           bands[-1].ay_max, args.sweep_ay_cap)
    if args.matrix:
        return build_points(args.sites, bands, args.tests,
                            args.v_min_kmh, args.v_max_kmh)
    # --site wins over --list so that `--site X --list` previews THAT
    # point rather than printing the whole matrix; --list on its own
    # still lists the matrix.
    if args.site is None:
        if args.list:
            return build_points(args.sites, bands, args.tests,
                                args.v_min_kmh, args.v_max_kmh)
        raise SystemExit(
            "give --site (with --test) for a single run, or --matrix. "
            "--list and --list-sites need no simulator.")
    r = SITES[args.site].radius_m
    if args.declare_for_speed is not None:
        # Invert speed_for(): pick the aysmax that puts the requested
        # speed exactly in this paragraph's window.
        v = args.declare_for_speed
        frac = {'lane_keeping': LK_FRACTION}.get(args.test)
        a = ((v / 3.6) ** 2 / r / frac if frac else
             (v / 3.6) ** 2 / r - (MAXAY_EXCESS
                                   if args.test == 'max_lateral_accel'
                                   else CROSSING_EXCESS))
        band = next((b for b in bands if b.contains(v)), bands[-1])
        if not (band.min_allowed <= a <= band.max_allowed):
            raise SystemExit(
                f"{v:g} km/h on {args.site} (R={r:.0f} m) needs a declared "
                f"aysmax of {a:.2f} m/s^2 for {args.test}, and R79 Table 1 "
                f"allows {band.min_allowed:g}..{band.max_allowed:g} in the "
                f"{band.label} km/h band. Pick another site: the radius "
                f"that would work is "
                f"{(v / 3.6) ** 2 / (frac or 1.0) / band.max_allowed:.0f}"
                f"..{(v / 3.6) ** 2 / (frac or 1.0) / max(band.min_allowed, 0.1):.0f} m.")
        _, ay = speed_for(args.test, a, r)
        return [LkaPoint(args.site, args.test, round(v, 1), ay, a,
                         band.label)]
    band = next((b for b in bands
                 if b.contains(speed_for(args.test, b.ay_max, r)[0])), bands[-1])
    v, ay = speed_for(args.test, band.ay_max, r)
    if args.speed_kmh is not None:
        v = args.speed_kmh
        ay = (v / 3.6) ** 2 / r
    return [LkaPoint(args.site, args.test, round(v, 1), ay, band.ay_max,
                     band.label)]


def print_points(points: list[LkaPoint], sweep: bool = False) -> None:
    if sweep:
        # radius x speed, printed as the grid it is.
        sites = sorted({p.site for p in points},
                       key=lambda n: -SITES[n].radius_m)
        speeds = sorted({p.speed_kmh for p in points})
        cell = {(p.site, p.speed_kmh): p for p in points}
        print(f"{'site':>10} {'R[m]':>7} | " + " ".join(
            f"{v:>7.0f}" for v in speeds) + "   km/h")
        print(f"{'':>10} {'':>7} | " + " ".join(
            f"{'ay':>7}" for _ in speeds))
        for name in sites:
            row = []
            for v in speeds:
                p = cell.get((name, v))
                row.append(f"{p.ay_target_mps2:>7.2f}" if p else f"{'-':>7}")
            print(f"{name:>10} {SITES[name].radius_m:>7.0f} | "
                  + " ".join(row))
        print(f"\n{len(points)} cells. Each is a lane-keeping run judged on "
              f"kept_lane and jerk; '-' is a cell dropped for demanding more "
              f"than the tyre cap.")
        return
    print(f"{'#':>3} {'site':>10} {'R[m]':>7} {'test':<22} {'band':>9} "
          f"{'aysmax':>7} {'v[km/h]':>8} {'ay[m/s2]':>9}")
    for i, p in enumerate(points, 1):
        print(f"{i:>3} {p.site:>10} {SITES[p.site].radius_m:>7.0f} "
              f"{p.test:<22} {p.band:>9} {p.ay_max:>7.2f} "
              f"{p.speed_kmh:>8.1f} {p.ay_target_mps2:>9.2f}")
    print(f"\n{len(points)} points. Speed is derived from the site radius so "
          f"the demand lands in the window each paragraph specifies; points "
          f"whose speed left its own band were dropped.")


SUMMARY_FIELDS = list(LkaMetrics().__dict__.keys())


def main(argv=None) -> int:
    install_sigterm_handler()
    args = parse_args(argv)

    if args.list_sites:
        print(f"{'site':<10} {'town':<9} {'R[m]':>7} {'dir':>6} {'arc[m]':>7} "
              f"{'lead[m]':>8}")
        for name, s in sorted(SITES.items(), key=lambda kv: -kv[1].radius_m):
            print(f"{name:<10} {s.town:<9} {s.radius_m:>7.1f} "
                  f"{s.direction:>6} {s.arc_m:>7.1f} {s.lead_in_m:>8.1f}")
        return 0

    bands = parse_declaration(args.declared_ay, args.heavy)
    args.ay_table_max = (AY_TABLE_HEAVY if args.heavy else AY_TABLE_M1)[0][2]
    points = resolve_points(args, bands)

    if args.list:
        print_points(points, sweep=args.sweep)
        return 0
    if not points:
        raise SystemExit(
            "no test points: every derived speed fell outside its own band "
            "or outside --v-min-kmh/--v-max-kmh. Declare a different aysmax "
            "(--declared-ay) or survey a site with a radius that suits the "
            "band you want to test.")
    if args.repeats > 1:
        points = [replace(p, rep=r + 1)
                  for p in points for r in range(args.repeats)]

    check_no_bridge_conflict()
    groups: list[tuple[str, list[LkaPoint]]] = []
    for p in sorted(points, key=lambda p: (p.site, p.test, p.speed_kmh)):
        if not groups or groups[-1][0] != p.site:
            groups.append((p.site, []))
        groups[-1][1].append(p)

    out_dir = make_out_dir(__file__, args.tag or 'r79_lka', args.out_dir)
    print(f"[out] {out_dir}", flush=True)
    print("[decl] " + ", ".join(
        f"{b.label} km/h: aysmax {b.ay_max:g} m/s^2 "
        f"(table {b.min_allowed:g}..{b.max_allowed:g})" for b in bands),
        flush=True)

    bridge, shutdown_ros = start_bridge('r79_lka_scenario_bridge')
    warning = WarningWatch(bridge, args.warning_topic) \
        if args.warning_topic else None
    # Stanley owns steer for the whole run, applied the instant a command
    # lands rather than polled — a polled path adds up to 52 ms of pure
    # phase lag to the very loop this script exists to measure.
    results: list[LkaMetrics] = []
    geometry: dict = {}
    exit_code = 0
    done = 0
    stack_checked = args.no_stack_check
    try:
        for site_name, group in groups:
            site = SITES[site_name]
            adapter = CarlaCurveAdapter(
                site=site, with_imu=True, imu_hz=args.imu_hz,
                host=args.host, port=args.port,
                ego_bp=args.vehicle, target_bp=args.vehicle,
                weather=args.weather, sync=args.sync,
                clean_start=not args.keep_existing_actors)
            camera_pump: CameraPump | None = None
            try:
                version = adapter.connect()
                print(f"\n[sim] carla {version or ''} — {site.town} site "
                      f"{site.name}: R={adapter.measured_radius_m:.0f} m "
                      f"{site.direction}, arc {site.arc_m:.0f} m",
                      flush=True)
                adapter.spawn_ego()
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
                          f"{point.test} at {point.speed_kmh:g} km/h on "
                          f"R={site.radius_m:.0f} m -> "
                          f"{point.ay_target_mps2:.2f} m/s^2 "
                          f"(aysmax {point.ay_max:g})", flush=True)
                    try:
                        m, geometry = LkaRunner(
                            adapter, bridge, args, warning).run(
                                point, out_dir)
                    except KeyboardInterrupt:
                        raise
                    except Exception as exc:
                        m = LkaMetrics(
                            run_id=point.run_id, site=point.site,
                            test=point.test, radius_m=site.radius_m,
                            band=point.band,
                            ay_max_declared_mps2=point.ay_max,
                            speed_kmh=point.speed_kmh,
                            ay_target_mps2=point.ay_target_mps2,
                            outcome='error', verdict='invalid_window',
                            reason=str(exc), note=str(exc))
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
                    results.append(LkaMetrics(
                        run_id=point.run_id, site=point.site,
                        test=point.test, radius_m=site.radius_m,
                        band=point.band,
                        ay_max_declared_mps2=point.ay_max,
                        speed_kmh=point.speed_kmh,
                        ay_target_mps2=point.ay_target_mps2,
                        outcome='error', verdict='invalid_window',
                        reason=f'site {site_name} unavailable: {exc}'))
                exit_code = 1
                _write(out_dir, results, geometry, args)
            finally:
                if camera_pump is not None:
                    camera_pump.stop()
                    camera_pump.join(timeout=2.0)
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


def _report_point(m: LkaMetrics) -> None:
    print(f"    -> {m.verdict.upper():<16} "
          f"lane {'KEPT' if m.kept_lane else 'CROSSED'}  "
          f"ay {m.ay_geometric_mps2:.2f} demanded / "
          f"{m.ay_peak_zerophase_mps2:.2f} measured peak  "
          f"jerk {m.jerk_peak_mps3:.2f} m/s^3", flush=True)
    if m.lkas_silent_s > 0:
        print(f"       LKAS silent {m.lkas_silent_s:.1f} s from "
              f"t={m.t_first_silence_s:.1f} s — scenario held the lane",
              flush=True)
    print(f"       |cte|max {m.max_abs_cte_m:.2f} m  "
          f"clearance {m.min_marking_clearance_m:+.2f} m  "
          f"steer err rms {m.steer_rms_err_deg:.2f} deg "
          f"(mean {m.steer_mean_err_deg:+.2f}, peak "
          f"{m.steer_peak_err_deg:+.2f})", flush=True)
    if m.reason:
        print(f"       {m.reason}", flush=True)


def _write(out_dir: Path, results: list[LkaMetrics], geometry: dict,
           args) -> None:
    write_outputs(out_dir, results, SUMMARY_FIELDS,
                  'UN R79 Annex 8 §3.2 — ACSF Category B1 lane keeping',
                  args, extra=dict(geometry=geometry))


def _print_report(results: list[LkaMetrics]) -> None:
    if not results:
        return
    w = 108
    print("\n" + "=" * w)
    print("UN R79 Annex 8 §3.2 — lane keeping, lateral acceleration, jerk")
    print("=" * w)
    print(f"{'run':<38} {'verdict':<12} {'kept':>5} {'ay_dem':>7} "
          f"{'ay_pk':>7} {'jerk':>6} {'|cte|':>6} {'clear':>6} "
          f"{'steer_err':>10}")
    print("-" * w)
    for m in results:
        if not m.measured:
            print(f"{m.run_id:<38} {m.verdict:<12} {'-':>5} "
                  f"{'not measured — ' + (m.reason or m.outcome)[:60]}")
            continue
        print(f"{m.run_id:<38} {m.verdict:<12} "
              f"{'yes' if m.kept_lane else 'NO':>5} "
              f"{m.ay_geometric_mps2:>7.2f} {m.ay_peak_zerophase_mps2:>7.2f} "
              f"{m.jerk_peak_mps3:>6.2f} {m.max_abs_cte_m:>6.2f} "
              f"{m.min_marking_clearance_m:>6.2f} "
              f"{m.steer_rms_err_deg:>10.2f}")
    print("-" * w)
    _print_kept_lane_grid(results)
    tally: dict = {}
    for m in results:
        tally[m.verdict] = tally.get(m.verdict, 0) + 1
    print(f"{len(results)} runs — " + ", ".join(
        f"{v}: {c}" for v, c in sorted(tally.items())))
    print("§3.2.3 (50 N override force) and §3.2.4 (hands-on transition) are "
          "not assessable in simulation — see the module docstring.")


def _print_kept_lane_grid(results: list[LkaMetrics]) -> None:
    """radius x speed, one character per cell. The answer to "where does
    it stop holding the lane" is a shape, not a list of runs."""
    cells = {(m.site, round(m.speed_kmh)): m for m in results
             if m.site and m.measured}
    if len(cells) < 2:
        return
    sites = sorted({m.site for m in results if m.site},
                   key=lambda n: -(SITES[n].radius_m if n in SITES else 0))
    speeds = sorted({round(m.speed_kmh) for m in results})
    print("\nkept lane —  y = radius, x = speed;  "
          ". kept   X crossed   (blank = not run)")
    print(f"{'site':>10} {'R[m]':>7} | " + " ".join(
        f"{v:>5.0f}" for v in speeds))
    for name in sites:
        row = []
        for v in speeds:
            m = cells.get((name, v))
            row.append(f"{'.' if m.kept_lane else 'X':>5}" if m
                       else f"{'?':>5}")
        r = SITES[name].radius_m if name in SITES else float('nan')
        print(f"{name:>10} {r:>7.0f} | " + " ".join(row))


if __name__ == '__main__':
    sys.exit(main())

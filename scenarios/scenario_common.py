#!/usr/bin/env python3
"""Harness infrastructure shared by every scenario script.

Extracted verbatim from r171_stationary_target.py when the curved-road
(R171 §4.2.5.2.2) and lane-keeping (R79 Annex 8) scripts were added — three
copies of the ROS bridge would have been three places to fix the next
latency bug.

What lives here is scenario-independent: the bridge that stands in for
carlaAccSimTown.py, the camera pump, the approach-speed and lane-hold
controllers, and the process-level guards. What does NOT live here is
anything a regulation defines — pass/fail limits, metric definitions and
verdicts stay in the script that owns the test, so a change to one test's
thresholds cannot silently move another's.

The one exception is `peak_decel_from_trace` and its gating constants:
both R171 scripts measure the same physical quantity the same way, and
duplicating that reasoning would let the two drift apart.
"""

from __future__ import annotations

import csv
import os
import json
import math
import signal
import subprocess
import threading
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Float64, String


LOG_HZ = 20.0

# Deceleration is differentiated from speed, so the collision impulse
# itself shows up as a several-hundred-m/s^2 spike. Ignore samples inside
# this gap, and clamp what remains to something a tyre can actually do.
DECEL_VALID_GAP_M = 0.3
DECEL_PLAUSIBLE_MAX = 15.0
# Speed below which a deceleration sample is the standstill snap, not
# braking. CARLA zeroes a vehicle's velocity over one tick at the end of a
# stop: measured, 5.6 km/h -> 0 in a single 0.26 s sample with the brake at
# 0.10, which is ~0.6 m/s^2 of commanded deceleration reported as 8.06.
# That snap WAS the headline peak of run 20260812_115623. The result is
# insensitive to where the floor sits: 2.0, 3.0, 4.0 and 5.0 m/s all give
# the same 6.67 m/s^2 figure; only 1.0 m/s admits the snap.
DECEL_VALID_SPEED_MPS = 2.0
# Beyond roughly 0.9 g on dry asphalt no tyre can deliver the demand, so
# once a_req crosses this the collision is already locked in and the exact
# figure stops carrying meaning (braking 1 m from the target at 130 km/h
# "requires" 700 m/s^2).
DECEL_PHYSICAL_MAX = 9.0


def peak_decel_from_trace(samples: list, half_window_s: float = 0.12,
                          phase: str = 'measure'):
    """Peak deceleration [m/s^2] from a run trace, measured offline.

    Fits v(t) by least squares over a window centred on each sample and
    takes the steepest negative slope. Centred means no group delay, and
    fitting over a window rather than differencing adjacent samples means
    single-sample speed noise cannot masquerade as 20 m/s^2.

    Excludes the standstill snap and the collision impulse, since neither
    is braking performance, and is restricted to the measurement phase:
    the approach is flown by the scenario itself and opens with a
    kinematic snap to the test speed that differentiates to ~12 m/s^2.
    Including it reported the harness's own setup transient as the system
    under test's peak deceleration.
    """
    pts = [(s['t'], s['v_kmh'] / 3.6, s.get('gap_gt_m', math.inf))
           for s in samples if s.get('phase') == phase]
    if len(pts) < 3:
        return None
    peak = None
    for i, (ti, _, _) in enumerate(pts):
        win = [(t, v) for (t, v, _) in pts if abs(t - ti) <= half_window_s]
        if len(win) < 3:
            continue
        n = len(win)
        mt = sum(t for t, _ in win) / n
        mv = sum(v for _, v in win) / n
        den = sum((t - mt) ** 2 for t, _ in win)
        if den <= 0:
            continue
        slope = sum((t - mt) * (v - mv) for t, v in win) / den
        _, v_i, gap_i = pts[i]
        if (v_i > DECEL_VALID_SPEED_MPS and gap_i > DECEL_VALID_GAP_M
                and -slope <= DECEL_PLAUSIBLE_MAX):
            peak = -slope if peak is None else max(peak, -slope)
    return peak


def required_decel(speed_mps: float, gap_m: float) -> float:
    """Constant deceleration needed to stop in the remaining gap [m/s^2].

    a_req = v^2 / (2 * gap). This is the criticality measure the R171
    longitudinal tests hang on: at the handover point gap = ttc * v, so
    a_req reduces to v / (2 * ttc) — the demand the scenario *hands* the
    system. Every metre the system spends not braking after that drives
    a_req up.
    """
    if gap_m <= 0.0:
        return float('inf')
    if speed_mps <= 0.0:
        return 0.0
    return (speed_mps ** 2) / (2.0 * gap_m)


# ---------------------------------------------------------------------------
# ROS bridge — simulator-agnostic
# ---------------------------------------------------------------------------
class ScenarioBridge(Node):
    """Publishes what the ADAS stack consumes, and captures what it emits.

    Deliberately does NOT apply control to the simulator — the scenario
    director decides each tick whether the stack or the scenario owns each
    channel, then calls the adapter itself. That gate is the whole point
    of the harness.
    """

    def __init__(self, name: str = 'scenario_bridge'):
        super().__init__(name, namespace='Car_1')

        self.camera_pub = self.create_publisher(
            CompressedImage, '/Car_1/camera/front/compressed', 10)
        self.speed_pub = self.create_publisher(
            Float64, '/Car_1/vehicle/speed', 10)
        # The perception nodes default to False, but publish it anyway so a
        # stale True latched by a previous carlaAccSimTown.py session can't
        # leave lane detection paused.
        self.junction_pub = self.create_publisher(Bool, '/Car_1/in_junction', 1)
        # ACC's cruise law caps throttle at its set speed and brakes above
        # it. Left at the 25 km/h default it would fight every approach
        # above 25 km/h, so the scenario tells it the test speed.
        self.target_speed_pub = self.create_publisher(
            Float32, '/ACC/target_speed', 10)

        self.create_subscription(Twist, '/Car_1/cmd_vel', self._on_cmd_vel, 20)
        self.create_subscription(Float32, '/Car_1/cmd_steer',
                                 self._on_cmd_steer, 20)
        self.create_subscription(Float32, '/ACC/lead_vehicle_distance',
                                 self._on_lead_distance, 20)
        # Which branch of controller_node's decision hierarchy is driving.
        # Taken from the node itself rather than inferred from the distance
        # topic: the controller low-passes that signal before thresholding,
        # so an outside reconstruction disagrees near the boundaries.
        self.create_subscription(String, '/ACC/control_mode',
                                 self._on_control_mode, 10)
        self.create_subscription(Float32, '/ACC/tracked_gap',
                                 self._on_tracked_gap, 10)
        self.create_subscription(Float32, '/ACC/speed_reference',
                                 self._on_speed_ref, 10)
        self.create_subscription(Float32, '/ACC/lead_distance_pinhole',
                                 self._on_pinhole, 10)

        ready_qos = QoSProfile(depth=1,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.yolo_ready = False
        self.ufld_ready = False
        self.create_subscription(Bool, '/ACC/perception/model_ready',
                                 self._on_yolo_ready, ready_qos)
        self.create_subscription(Bool, '/LKAS/perception/model_ready',
                                 self._on_ufld_ready, ready_qos)

        self.stack_throttle = 0.0
        self.stack_brake = 0.0
        self.stack_steer = 0.0
        self.stack_steer_stamp = 0.0
        # Event-driven steer application. The original bridge
        # (custom_ROS_pub_sub.CarlaAVT._cmd_steer_cb) called apply_control
        # the instant a steer message landed. Polling it once per director
        # iteration (~19 Hz) instead silently added up to 52 ms of latency
        # to the LATERAL loop — 1.0 m of travel at 70 km/h, and pure phase
        # lag to a controller that has to close around it. Stanley held
        # 70 km/h lanes before and drifted out of them after, so the
        # latency is restored to zero here.
        self._control_sink = None
        self._longitudinal = (0.0, 0.0)
        self._sink_lock = threading.Lock()
        self.stack_cmd_vel_seen = False
        self.lead_distance: float | None = None
        self.lead_distance_stamp = 0.0
        self.acc_mode: str = ''
        self.tracked_gap: float | None = None
        self.speed_ref: float | None = None
        self.pinhole_gap: float | None = None

    # -- inbound from the stack --------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self.stack_throttle = float(min(max(msg.linear.x, 0.0), 1.0))
        self.stack_brake = float(min(max(msg.linear.y, 0.0), 1.0))
        self.stack_cmd_vel_seen = True

    def _on_cmd_steer(self, msg: Float32) -> None:
        self.stack_steer = float(min(max(msg.data, -1.0), 1.0))
        self.stack_steer_stamp = time.monotonic()
        sink = self._control_sink
        if sink is not None:
            # Apply immediately with the most recent longitudinal command.
            # The lock serialises this against the director's own
            # apply_control so the two never interleave a simulator RPC.
            with self._sink_lock:
                thr, brk = self._longitudinal
                sink(thr, brk, self.stack_steer)

    def set_control_sink(self, fn) -> None:
        self._control_sink = fn

    def set_longitudinal(self, throttle: float, brake: float) -> None:
        self._longitudinal = (throttle, brake)

    def _on_lead_distance(self, msg: Float32) -> None:
        d = float(msg.data)
        if d == float('inf') or d <= 0.0:
            self.lead_distance = None
        else:
            self.lead_distance = d
            self.lead_distance_stamp = time.monotonic()

    def _on_control_mode(self, msg: String) -> None:
        self.acc_mode = msg.data

    def _on_tracked_gap(self, msg: Float32) -> None:
        self.tracked_gap = float(msg.data)

    def _on_speed_ref(self, msg: Float32) -> None:
        self.speed_ref = float(msg.data)

    def _on_pinhole(self, msg: Float32) -> None:
        d = float(msg.data)
        self.pinhole_gap = None if d == float('inf') or d <= 0.0 else d

    def _on_yolo_ready(self, msg: Bool) -> None:
        self.yolo_ready = self.yolo_ready or bool(msg.data)

    def _on_ufld_ready(self, msg: Bool) -> None:
        self.ufld_ready = self.ufld_ready or bool(msg.data)

    # -- outbound to the stack ---------------------------------------------
    def publish_frame(self, jpeg: bytes) -> None:
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'Car_1/camera/front'
        msg.format = 'jpeg'
        msg.data = jpeg
        self.camera_pub.publish(msg)

    def publish_speed(self, speed_mps: float) -> None:
        self.speed_pub.publish(Float64(data=float(speed_mps)))

    def publish_target_speed(self, kmh: float) -> None:
        self.target_speed_pub.publish(Float32(data=float(kmh)))

    def publish_not_in_junction(self) -> None:
        self.junction_pub.publish(Bool(data=False))


class CameraPump(threading.Thread):
    """Encodes and publishes camera frames off the control loop's thread.

    Doing this inline cost ~35 ms per iteration (JPEG encode of a 1280x720
    frame) and dragged the control loop from 20 Hz to 11.6 Hz. That
    starved the perception nodes of frames at exactly the moment they
    matter — a 130 km/h approach covers 3.1 m per iteration at 11.6 Hz.
    The adapter's frame queue is a queue.Queue and encoding touches no
    simulator RPC, so this is safe to run concurrently with the director.
    """

    def __init__(self, adapter, bridge: ScenarioBridge, hz: float = 25.0):
        super().__init__(daemon=True)
        self.adapter = adapter
        self.bridge = bridge
        self.period = 1.0 / hz
        # NOT `self._stop` — threading.Thread uses that name internally for
        # a method that join() calls, and shadowing it makes join() raise
        # "'Event' object is not callable" and abort the interpreter.
        self._stop_evt = threading.Event()
        self.frames_published = 0

    def run(self) -> None:
        while not self._stop_evt.is_set():
            try:
                frame = self.adapter.poll_camera()
                if frame is not None:
                    self.bridge.publish_frame(frame)
                    self.frames_published += 1
            except Exception:
                pass
            time.sleep(self.period)

    def stop(self) -> None:
        self._stop_evt.set()


# ---------------------------------------------------------------------------
# Scenario-owned controllers
# ---------------------------------------------------------------------------
class SpeedHold:
    """PI throttle/brake hold, used to fly the VUT at the test speed."""

    def __init__(self, kp=0.8, ki=0.5, i_limit=3.0):
        self.kp, self.ki, self.i_limit = kp, ki, i_limit
        self.integral = 0.0

    def reset(self, seed_throttle: float = 0.0) -> None:
        """`seed_throttle` pre-loads the integral with an estimate of the
        throttle needed to hold the test speed. Starting from cold, the
        integral took longer than the whole settle window to wind up and
        the VUT arrived at the trigger point ~4 % slow — which biases every
        downstream a_req figure. Seeding removes the droop."""
        self.integral = (seed_throttle / self.ki) if self.ki else 0.0
        self.integral = max(min(self.integral, self.i_limit), -self.i_limit)

    def step(self, target_mps: float, actual_mps: float,
             dt: float) -> tuple[float, float]:
        err = target_mps - actual_mps
        u_p = self.kp * err
        # Anti-windup: only integrate while the unsaturated command is in
        # range, otherwise a long approach at 130 km/h winds up a huge
        # integral that overshoots the moment drag drops.
        if abs(u_p + self.ki * self.integral) < 1.0:
            self.integral = max(min(self.integral + err * dt,
                                    self.i_limit), -self.i_limit)
        u = u_p + self.ki * self.integral
        return (max(u, 0.0), max(-u, 0.0))

    @staticmethod
    def drag_throttle_estimate(speed_mps: float) -> float:
        """Rough steady-state throttle for a charger_2020 in CARLA.
        Aerodynamic drag dominates, so it grows with v^2; the constant was
        fitted to hold 30-130 km/h within about a km/h. Only ever a seed —
        the PI trims whatever it gets wrong."""
        return min(0.12 + 0.00022 * speed_mps ** 2, 0.9)


class LaneHold:
    """Stanley-form centreline hold for scenario-owned lateral control.

    Keeps the VUT on the reference path so a longitudinal measurement is
    not polluted by lateral wander. Steer is rate-limited because at
    130 km/h an unfiltered correction is enough to unsettle the car.

    Works on curves as well as straights, but only because the adapter
    supplying `cte`/`heading_err` measures them against a continuous
    reference path. Fed from a per-tick waypoint lookup it would inherit
    that lookup's lane-snapping (see CarlaAdapter.lane_error).
    """

    def __init__(self, k_cte=0.6, k_heading=1.0, rate_limit=0.05):
        self.k_cte, self.k_heading = k_cte, k_heading
        self.rate_limit = rate_limit
        self.prev = 0.0

    def reset(self) -> None:
        self.prev = 0.0

    def step(self, cte: float, heading_err: float, speed: float) -> float:
        # +cte = ego right of centre -> steer left (negative).
        cte_term = -math.atan2(self.k_cte * cte, max(speed, 1.0))
        raw = self.k_heading * heading_err + cte_term
        steer = max(min(raw, 1.0), -1.0)
        delta = max(min(steer - self.prev, self.rate_limit), -self.rate_limit)
        self.prev += delta
        return self.prev


# ---------------------------------------------------------------------------
# Process-level guards and plumbing
# ---------------------------------------------------------------------------
def check_no_bridge_conflict() -> None:
    """carlaAccSimTown.py also owns /Car_1/cmd_vel and the ego's
    apply_control. Two writers per physics tick means the last one wins at
    random — exactly the race documented in DEBUG.md for the duplicate
    MORAI adapters. Refuse rather than produce quietly corrupt runs."""
    try:
        out = subprocess.run(['pgrep', '-af', 'carlaAccSimTown'],
                             capture_output=True, text=True, timeout=5)
    except Exception:
        return
    # `pgrep -af` matches anywhere in the command line, so anything that
    # merely MENTIONS the bridge — an editor, a grep, a shell heredoc
    # containing the name in a comment — used to block the harness. It
    # happened while writing this file's own documentation. Require the
    # name to be an actual argument (i.e. the script being run), and skip
    # our own process tree.
    mine = {str(os.getpid()), str(os.getppid())}
    hits = []
    for line in out.stdout.splitlines():
        pid, _, cmdline = line.partition(' ')
        if pid in mine or 'pgrep' in cmdline:
            continue
        if any(os.path.basename(arg) == 'carlaAccSimTown.py'
               for arg in cmdline.split()):
            hits.append(line)
    if hits:
        raise SystemExit(
            "carlaAccSimTown.py is running — it would fight this script "
            "over /Car_1/cmd_vel and the ego's control.\n  "
            + "\n  ".join(hits)
            + "\nStop the bridge (UI: Stop Bridge) and re-run.")


def wait_for_stack(bridge: ScenarioBridge, timeout: float) -> bool:
    """The ACC holds throttle at 0 until YOLO and UFLD both report ready
    (controller_node's model-load gate). Handing over before that means
    measuring the gate, not the controller."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if bridge.yolo_ready and bridge.ufld_ready:
            print("[stack] YOLO + UFLD ready", flush=True)
            return True
        waiting = [n for n, ok in (('YOLO', bridge.yolo_ready),
                                   ('UFLD', bridge.ufld_ready)) if not ok]
        print(f"[stack] waiting on: {', '.join(waiting)}", flush=True)
        time.sleep(2.0)
    return False


def install_sigterm_handler() -> None:
    """Turn SIGTERM into KeyboardInterrupt.

    UI.py's Stop button calls os.killpg(SIGTERM). Without this the harness
    dies where it stands: no summary.csv for the points that already
    completed, and the ego/target/camera left in the world for the next
    session to trip over. Routing it into the existing abort path writes
    the partial results and destroys the actors.
    """
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _on_sigterm)


def start_bridge(node_name: str):
    """rclpy init + bridge + spinning executor. Returns (bridge, shutdown)."""
    rclpy.init()
    bridge = ScenarioBridge(node_name)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    def shutdown():
        executor.shutdown()
        spin.join(timeout=2.0)
        if rclpy.ok():
            rclpy.shutdown()

    return bridge, shutdown


def write_trace(out_dir: Path, run_id: str, samples: list) -> None:
    if not samples:
        return
    with open(out_dir / f"{run_id}.csv", 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(samples[0].keys()))
        w.writeheader()
        w.writerows(samples)


def write_outputs(out_dir: Path, results: list, fields: list,
                  scenario: str, args, extra: dict | None = None) -> None:
    """summary.csv + manifest.json, rewritten after every scenario point.

    Rewritten rather than appended so a Ctrl-C or a UI Stop leaves a
    complete, readable summary of everything that finished.
    """
    if not results:
        return
    with open(out_dir / 'summary.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for m in results:
            w.writerow(asdict(m))
    with open(out_dir / 'manifest.json', 'w') as fh:
        json.dump(dict(
            scenario=scenario,
            generated=datetime.now().isoformat(timespec='seconds'),
            args=vars(args),
            **(extra or {}),
            runs=[asdict(m) for m in results],
        ), fh, indent=2, default=str)


def make_out_dir(script_file: str, tag: str, out_dir: str | None) -> Path:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    name = f"{stamp}_{tag}" if tag else stamp
    path = (Path(out_dir) if out_dir
            else Path(script_file).resolve().parent / 'results' / name)
    path.mkdir(parents=True, exist_ok=True)
    return path


def fmt_decel(x: float, width: int = 0, nd: int = 2) -> str:
    """Format a deceleration demand, refusing to quote physically
    meaningless figures: past tyre capability a_req is arithmetically true
    but says nothing (braking 1 m from a target at 130 km/h "requires"
    710 m/s^2)."""
    if x is None or not math.isfinite(x):
        s = '-'
    elif x > DECEL_PHYSICAL_MAX:
        s = f">{DECEL_PHYSICAL_MAX:g}"
    else:
        s = f"{x:.{nd}f}"
    return f"{s:>{width}}" if width else s

#!/usr/bin/env python3
"""
ADAS UI — drive the CARLA + custom-bridge + ROS ADAS stack from one window.

Buttons:
  * Start / Stop CARLA           — CARLA 0.9.16 server on the chosen RPC port
  * Start / Stop Bridge          — carlaaccsim/carlaAccSimTown.py
                                   (publishes /Car_1/camera/front/compressed,
                                    /Car_1/vehicle/speed, subscribes /Car_1/cmd_vel)
  * Run start_adas.sh             — launches all four ADAS nodes via the script
  * Stop ADAS Stack              — kills start_adas.sh + any orphan ADAS nodes
  * ACC: ON/OFF                  — toggles perception_node + controller_node
  * LKAS: ON/OFF                 — toggles lane_detection_node + stanley_node
  * Scenario boxes               — UN R171 stationary / UN R171 curved /
                                   UN R79 lane keeping. Each one IS the
                                   CARLA<->ROS bridge while it runs, so
                                   they share a process slot with the
                                   bridge and with each other.

The right side of the window renders the live camera feed by subscribing
to /Car_1/camera/front/compressed (rclpy, runs in a background thread).

Run with system Python 3.10 (ROS-sourceable, has rclpy + PIL + cv2 + numpy).
"""
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, scrolledtext

try:
    import cv2
    import numpy as np
    from PIL import Image, ImageTk
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32
    CAMERA_AVAILABLE = True
    _camera_err = None
except ImportError as e:
    CAMERA_AVAILABLE = False
    _camera_err = str(e)
    # Placeholder base class so `class TelemetryView(Node)` below doesn't
    # NameError at import time. Safe: TelemetryView is only ever
    # instantiated from _start_camera_view, which already checks
    # CAMERA_AVAILABLE first and returns before reaching that line.
    Node = object


# --------------------------------------------------------------------------
# Paths
# --------------------------------------------------------------------------
ADAS_WK       = Path(__file__).resolve().parent
ADAS_INSTALL  = ADAS_WK / 'install' / 'setup.bash'
CARLA_DIR     = Path('/home/sirius/CARLA_0.9.16')

# Lane-detection model dropdown options: (display_name, model_ref).
# `model_ref` is passed to lane_detection_node via -p model_filename:=…
# A bare filename resolves against share/perception/models/; an absolute
# path is used as-is. Add new trained checkpoints below as they become
# available — no other UI code needs to change.
LANE_MODELS: list[tuple[str, str]] = [
    ('Current best (UFLD_F1=0.87.pth)', 'UFLD_F1=0.87.pth'),
    ('Retrained (UFLD_F1=0.67.pth)', 'UFLD_F1=0.67.pth'),
]

# Same pattern as LANE_MODELS but for the YOLO object detector consumed
# by perception_node. Add newly-trained YOLO checkpoints below.
OBJECT_MODELS: list[tuple[str, str]] = [
    ('Current best YOLO (best.pt)', 'best.pt'),
    ('MORAI fine-tuned YOLO (best_MORAI.pt)', 'best_MORAI.pt'),
    # Example after a retrain:
    # ('Retrained YOLO 20260710',
    #  '/home/sirius/workspace/Trained_YOLO/runs/detect/train2/'
    #  'weights/best.pt'),
]
CARLA_SERVER  = './CarlaUE4.sh'
CARLA_INI     = Path('/home/sirius/CARLA_0.9.16/CarlaUE4/Config/DefaultEngine.ini')
CARLA_PYTHON  = Path('/home/sirius/CARLA_0.9.16/carla-env/bin/python3')
BRIDGE_DIR    = Path('/home/sirius/workspace/carlaaccsim')
BRIDGE_SCRIPT = BRIDGE_DIR / 'carlaAccSimTown.py'
# Scenario harness. Note this is an ALTERNATIVE to BRIDGE_SCRIPT, not an
# addition: it is itself the CARLA<->ROS bridge for the duration of a run,
# because it has to gate who owns the longitudinal channel (the scenario
# before the DCAS trigger point, the stack's ACC after). Running both at
# once means two writers on /Car_1/cmd_vel and the ego's apply_control.
SCENARIO_DIR    = ADAS_WK / 'scenarios'
SCENARIO_SCRIPT = SCENARIO_DIR / 'r171_stationary_target.py'
# R171 Annex 4 §4.2.5.2.2 and R79 Annex 8 §3.2. Same mutual exclusion as
# above: all three are the bridge while they run, and they share
# self.scenario_proc so only one can be up at a time.
SCENARIO_CURVE_SCRIPT = SCENARIO_DIR / 'r171_curved_target.py'
SCENARIO_LKA_SCRIPT   = SCENARIO_DIR / 'r79_lka_validation.py'

# The curve site table and R79's speed derivation are imported rather than
# duplicated: the site radii and the 80-90 %/+0.4/+0.25 windows are
# regulation-derived numbers, and a second copy here would be a second
# thing to get wrong. Import failure is not fatal — the panels fall back
# to launching with whatever is typed, and only the previews go away.
try:
    if str(SCENARIO_DIR) not in sys.path:
        sys.path.insert(0, str(SCENARIO_DIR))
    from curve_adapter import SITES as CURVE_SITES, DEFAULT_SITE as CURVE_SITE_DEFAULT
    from r79_lka_validation import (TESTS as LKA_TESTS,
                                    speed_for as lka_speed_for,
                                    parse_declaration as lka_parse_declaration)
    SCENARIO_META_OK = True
    _scenario_meta_err = None
except Exception as _e:            # noqa: BLE001 — UI must still start
    CURVE_SITES = {}
    CURVE_SITE_DEFAULT = 't04_r199'
    LKA_TESTS = ('lane_keeping', 'max_lateral_accel', 'lane_crossing_warning')
    lka_speed_for = lka_parse_declaration = None
    SCENARIO_META_OK = False
    _scenario_meta_err = str(_e)

# R171 §5.3.7.1.2 caps the lateral acceleration an M1 DCAS may induce.
# The curved panel refuses a speed/site pair above it rather than letting
# the run start and fail per-point — same check as
# r171_curved_target.AY_CEILING_MPS2, applied early enough to be useful.
CURVE_AY_CEILING = 3.0
# R79 §3.2's default declaration; see r79_lka_validation --declared-ay.
LKA_DECLARED_AY_DEFAULT = '60:1.5,100:3.0,130:3.0'
START_ADAS_SH  = ADAS_WK / 'start_adas.sh'
ROS_SETUP     = '/opt/ros/humble/setup.bash'

CAMERA_TOPIC       = '/Car_1/camera/front/compressed'
ACC_DEBUG_TOPIC    = '/ACC/perception/debug_image'
LKAS_DEBUG_TOPIC   = '/LKAS/perception/debug_image'
FUSED_DEBUG_TOPIC  = '/ADAS/perception/debug_image'
# KF-only variant: raw + YOLO boxes + KF-smoothed cyan/centerline only
# (no raw UFLD row dots, no conf text). Published by
# debug_image_fusion_node when lane_detection_node is running with
# Kalman ON. See lane_detection_node.annotate_kf_only().
KF_DEBUG_TOPIC     = '/ADAS/perception/debug_image_kf'
IPM_DEBUG_TOPIC    = '/ADAS/ipm/debug_image'
CMD_VEL_TOPIC      = '/Car_1/cmd_vel'
CMD_STEER_TOPIC    = '/Car_1/cmd_steer'
SPEED_TOPIC        = '/Car_1/vehicle/speed'

# Display-name → topic for the camera-source selector. The debug topics
# carry YOLO bounding boxes (ACC), UFLD ego-lane polylines (LKAS), and
# the combined view (ADAS), drawn server-side by the perception nodes.
# IPM_DEBUG_TOPIC has its own permanent BEV panel to the right of the
# camera (see _build_ui), so it's intentionally NOT in this dict.
CAMERA_SOURCES = {
    'Raw':              CAMERA_TOPIC,
    'ACC (YOLO)':       ACC_DEBUG_TOPIC,
    'LKAS (UFLD)':      LKAS_DEBUG_TOPIC,
    'ADAS (YOLO+UFLD)': FUSED_DEBUG_TOPIC,
    'ADAS (YOLO+KF)':   KF_DEBUG_TOPIC  
}

# BEV display widget — native IPM image is 320×480 (see ipm_view_node).
# We render at 1.125× upscale to make it more legible without being
# huge. Aspect ratio (2:3) is preserved in the BEV-only render path.
BEV_W = 320
BEV_H = 480

# A node is "alive" if its heartbeat topic has published within this window.
NODE_ALIVE_WINDOW_S = 1.5

# Tkinter doesn't enjoy a flood of PhotoImage swaps. Bridge publishes at
# ~20 Hz; downsample to ~12 Hz for the widget.
CAMERA_UI_HZ  = 12
# Target render size for the camera widget (preserves the 16:9 bridge feed).
CAMERA_W      = 960
CAMERA_H      = 540

TOWNS = [
    'Town01', 'Town02', 'Town03', 'Town04', 'Town05', 'Town06', 'Town07',
    'Town10HD_Opt', 'Town01_Opt', 'Town02_Opt', 'Town03_Opt',
    'Town04_Opt', 'Town05_Opt', 'Town06_Opt', 'Town07_Opt', 'Town10HD',
]
# Town06 carries the longest dead-straight road in the stock map set —
# spawn 80 (road 48, centre lane of 5) gives 718 m before the lane
# deviates, vs ~116 m for the best Town03 spawn. That runway is what
# makes the UN R171 Annex 4 §4.2.5.2.1 matrix runnable at 130 km/h with
# a 10 s TTC margin (needs 469 m). See scenarios/r171_stationary_target.py.

WEATHER_PRESETS = [
    'ClearNoon', 'CloudyNoon', 'WetNoon', 'WetCloudyNoon',
    'MidRainyNoon', 'HardRainNoon', 'SoftRainNoon',
    'ClearSunset', 'CloudySunset', 'WetSunset', 'HardRainSunset',
    'ClearNight', 'CloudyNight',
    'FogNoon',   # custom, see FOG_PRESETS
]

TRAFFIC_OPTIONS = ['0', '5', '10', '20', '30', '50']

# Junction policy — display name → bridge `--junction-policy` value. The
# bridge's CARLA-map junction monitor revokes LKAS steer authority inside
# any junction zone; the policy decides who owns steer instead. Mirrors
# the `--policy` set in 00_Lane_Assistant/02_UFLD_V2/UI.py.
JUNCTION_POLICIES = {
    'Pure pursuit': 'pp-takeover',
    'Hold straight': 'hold-straight',
}


# --------------------------------------------------------------------------
# Helpers — CARLA-side state via the carla-env Python
# --------------------------------------------------------------------------
def set_boot_map(ini_path: Path, town: str):
    """Rewrite the three *.Map entries in CARLA's DefaultEngine.ini so the
    next server boot lands in the requested town. (In-band load_world()
    segfaults on this install — the boot-map ini is the only reliable way.)
    Returns (n_lines_changed, new_value, observed_after_write)."""
    text = ini_path.read_text()
    new_value = f'/Game/Carla/Maps/{town}.{town}'
    pat = re.compile(r'(EditorStartupMap|GameDefaultMap|ServerDefaultMap)=.+')
    new_text, n = pat.subn(rf'\1={new_value}', text)
    ini_path.write_text(new_text)
    os.sync()
    after = ini_path.read_text()
    m = re.search(r'ServerDefaultMap=(.+)', after)
    observed = m.group(1).strip() if m else '<missing>'
    return n, new_value, observed


# Each of these snippets is fed to the carla-env Python via `-c`. They
# connect to the running CARLA, mutate state, and exit — no long-lived
# subprocess. Failures bubble up to the caller for logging.
# FogNoon is not a CARLA preset. The declared ODD (README) covers "clear,
# cloudy, foggy and rainy", but WeatherParameters ships no foggy preset, so
# fog was untestable and the declaration claimed coverage the rig could not
# produce. Built here from CloudyNoon with the operator's values —
# fog_density 50 %, fog_distance 30 m — so the ODD's fog cell is real.
# fog_falloff 0.2 keeps the fog low and roughly uniform in the camera's
# field rather than stacking it above the road.
FOG_PRESETS = {
    'FogNoon': dict(base='CloudyNoon', fog_density=50.0,
                    fog_distance=30.0, fog_falloff=0.2),
}

_WEATHER_SNIPPET = """
import carla, sys, json
preset = sys.argv[1]
port = int(sys.argv[2])
fog = json.loads(sys.argv[3]) if len(sys.argv) > 3 else None
client = carla.Client('localhost', port); client.set_timeout(10.0)
world = client.get_world()
if fog:
    w = getattr(carla.WeatherParameters, fog['base'])
    w.fog_density  = fog['fog_density']
    w.fog_distance = fog['fog_distance']
    w.fog_falloff  = fog['fog_falloff']
    world.set_weather(w)
    print(f"[weather] applied {preset} "
          f"(fog {fog['fog_density']:.0f}% at {fog['fog_distance']:.0f} m)")
else:
    world.set_weather(getattr(carla.WeatherParameters, preset))
    print(f'[weather] applied {preset}')
"""

_TRAFFIC_SPAWN_SNIPPET = """
import carla, random, sys, time
n = int(sys.argv[1])
port = int(sys.argv[2])
client = carla.Client('localhost', port); client.set_timeout(10.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()
random.shuffle(spawn_points)
vehicle_bps = [bp for bp in bp_lib.filter('vehicle.*')
               if bp.has_attribute('number_of_wheels')
               and int(bp.get_attribute('number_of_wheels')) == 4]

# Use the bridge's default TM port (8000). The previous `port + 6000`
# math landed on 8000 only when CARLA was on port 2000 and silently
# created a *second* TM at 8002 / 8003 / etc. on any other CARLA port.
# Always-default keeps UI and bridge on the same shared TM.
tm = client.get_trafficmanager()
tm.set_global_distance_to_leading_vehicle(2.5)
tm.global_percentage_speed_difference(30.0)
# Critical: explicitly force TM async to match world. TM state persists
# across processes on the CARLA server, so a prior session that left
# this TM in sync mode would freeze every NPC we spawn here.
tm.set_synchronous_mode(False)
# Hybrid physics mode ALSO persists server-side across sessions. With
# it on, NPCs far from any hero are dormant (no physics, no motion) —
# looks identical to "autopilot disengaged". Force it off so every
# spawned NPC gets full physics regardless of distance to the ego.
tm.set_hybrid_physics_mode(False)
# Belt-and-braces: if any NPC ends up dormant despite the line above
# (CARLA can mark distant actors dormant under memory pressure), TM
# will revive it instead of leaving it stuck.
try:
    tm.set_respawn_dormant_vehicles(True)
except Exception:
    pass  # older CARLA builds may not have this method

# Atomic spawn + autopilot via batch commands. Without the .then()
# chaining, there was a window between try_spawn_actor returning and
# set_autopilot landing in which the vehicle existed but had no
# controller — it drifted on default zero-throttle / centred-steer
# and crashed into curbs / other actors. Exactly the bug we fixed
# in carlaaccsim/carlaAccSimTown.py:spawn_traffic; same fix needed
# here because UI.py spawns NPCs independently of the bridge.
SpawnActor   = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor  = carla.command.FutureActor

batch = []
for sp in spawn_points[:n]:
    bp = random.choice(vehicle_bps)
    if bp.has_attribute('color'):
        colors = bp.get_attribute('color').recommended_values
        if colors:
            bp.set_attribute('color', random.choice(colors))
    if bp.has_attribute('role_name'):
        bp.set_attribute('role_name', 'npc')   # cleared by clear-snippet
    batch.append(
        SpawnActor(bp, sp)
        .then(SetAutopilot(FutureActor, True, tm.get_port()))
    )

# due_tick_cue=True so the server processes the SpawnActor + the
# chained SetAutopilot together within this call and we know the
# autopilot attachment has *landed* before the subprocess exits.
# With False the batch was queued and returned immediately — by the
# time the next async tick processed it, the snippet had already
# exited and its TM client connection had died, leaving freshly-
# spawned NPCs orphaned (registered with TM but never controlled).
# Matches the bridge's spawn_traffic which uses True and works.
spawned_ids = []
for resp in client.apply_batch_sync(batch, True):
    if not resp.error:
        spawned_ids.append(resp.actor_id)
print(f'[traffic] spawned {len(spawned_ids)}/{n} NPCs (atomic batch, async TM)',
      flush=True)

# Long-lived heartbeat loop. The previous 2 s sleep wasn't enough —
# NPCs were registered with TM via the batch's SetAutopilot, but the
# moment this subprocess exited and its TM client connection died,
# the NPCs either lost autopilot or were dropped by TM (CARLA 0.9.16
# tracks the registering client per-vehicle, and orphaned vehicles
# freeze even if other clients — like the bridge — are still
# connected to the same TM). Keeping the client alive forever fixes
# that, and re-asserting set_autopilot every few seconds also
# rescues any vehicle TM might have forgotten for any reason.
# The clear-snippet still finds these NPCs by role_name='npc' and
# destroys them; killing this subprocess (UI shutdown / restart)
# then releases TM ownership cleanly.
import signal
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
print('[traffic] heartbeat loop started — keep this process alive to '
      'maintain NPC autopilot. Will re-assert every 5 s.', flush=True)
while True:
    time.sleep(5.0)
    npcs = [a for a in world.get_actors().filter('vehicle.*')
            if a.attributes.get('role_name') == 'npc']
    for a in npcs:
        try:
            a.set_autopilot(True, tm.get_port())
        except Exception:
            pass
    print(f'[traffic] heartbeat: {len(npcs)} NPCs in autopilot',
          flush=True)
"""

_TRAFFIC_CLEAR_SNIPPET = """
import carla, sys
port = int(sys.argv[1])
client = carla.Client('localhost', port); client.set_timeout(10.0)
world = client.get_world()
killed = 0
for a in world.get_actors().filter('vehicle.*'):
    if a.attributes.get('role_name') == 'npc':
        try:
            a.destroy(); killed += 1
        except Exception:
            pass
print(f'[traffic] cleared {killed} NPCs')
"""

# Print every spawn point in the currently-loaded map. Format mirrors
# the bridge's own --list-spawns flag so log output is consistent
# whether you ask via UI or CLI.
_LIST_SPAWNS_SNIPPET = """
import carla, sys
port = int(sys.argv[1])
client = carla.Client('localhost', port); client.set_timeout(10.0)
spawns = client.get_world().get_map().get_spawn_points()
for i, sp in enumerate(spawns):
    loc, rot = sp.location, sp.rotation
    print(f'  {i:3d}  x={loc.x:8.2f}  y={loc.y:8.2f}  '
          f'z={loc.z:6.2f}  yaw={rot.yaw:7.2f}')
print(f'[spawns] {len(spawns)} spawn points')
"""


# --------------------------------------------------------------------------
# ROS subscriber — camera sources + node heartbeats
# --------------------------------------------------------------------------
class TelemetryView(Node):
    """Subscribes to:
      * the three camera-source topics (raw + ACC debug + LKAS debug) so the
        UI can switch what it renders without tearing down subscriptions;
      * the controller / Stanley command topics so the UI can show which
        nodes are actually publishing (perception_node and lane_detection_node
        are covered by the debug-image subs).
    Camera frames are stashed as raw JPEG bytes; decode happens in the Tk
    render tick so we only pay the cost for the source being shown. Every
    received message also updates a last-seen timestamp so the UI can render
    ACC / LKAS alive indicators.
    """

    def __init__(self):
        super().__init__('adas_ui_telemetry')
        self.latest_jpegs = {t: None for t in CAMERA_SOURCES.values()}
        # IPM lives outside CAMERA_SOURCES (separate widget, not source-
        # selectable). Tracked independently so the BEV panel's render
        # path doesn't go through _active_camera_topic / latest_jpegs.
        self.latest_bev_jpeg: bytes | None = None
        self.last_seen: dict[str, float] = {}
        # Ego speed in m/s. None until the bridge publishes a first sample.
        self.speed_mps: float | None = None

        # SensorDataQoS (best-effort, keep-last-1) for image streams.
        # The default reliable + keep-last-10 profile lets up to 10 old
        # JPEGs queue per topic when the render tick can't drain fast
        # enough — switching the Source dropdown then exposes those
        # stale frames as visible per-source lag. With depth 1, older
        # frames are dropped in the middleware before they reach our
        # callback, so `latest_jpegs[topic]` always holds the newest
        # received frame.
        for topic in CAMERA_SOURCES.values():
            self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self._on_image(t, msg),
                qos_profile_sensor_data)
        self.create_subscription(
            CompressedImage, IPM_DEBUG_TOPIC, self._on_bev,
            qos_profile_sensor_data)
        self.create_subscription(
            Twist, CMD_VEL_TOPIC,
            lambda _msg: self._touch(CMD_VEL_TOPIC), 10)
        self.create_subscription(
            Float32, CMD_STEER_TOPIC,
            lambda _msg: self._touch(CMD_STEER_TOPIC), 10)
        # Speed feed: subscribe to stanley_node's UI-facing rebroadcast
        # on /ADAS/telemetry/speed_mps (std_msgs/Float32) rather than
        # the raw /Car_1/vehicle/speed topic. The raw topic is
        # std_msgs/Float64 on CARLA and example_interfaces/Float64 on
        # MORAI, and DDS won't deliver across that type mismatch — so
        # the display used to freeze at "— km/h" whenever the UI
        # dropdown and the running simulator disagreed. The rebroadcast
        # is a single fixed type, so this sub always works.
        self.create_subscription(
            Float32, '/ADAS/telemetry/speed_mps',
            self._on_speed, 10)

    def _on_speed(self, msg):
        self.speed_mps = float(msg.data)
        self._touch(SPEED_TOPIC)

    def _on_image(self, topic, msg):
        # Stash raw bytes; let the render tick decide whether to decode.
        self.latest_jpegs[topic] = bytes(msg.data)
        self._touch(topic)

    def _on_bev(self, msg):
        # Same idea as _on_image but for the always-on BEV panel.
        self.latest_bev_jpeg = bytes(msg.data)
        self._touch(IPM_DEBUG_TOPIC)

    def _touch(self, topic: str):
        self.last_seen[topic] = time.monotonic()

    def is_alive(self, topic: str) -> bool:
        ts = self.last_seen.get(topic)
        return ts is not None and (time.monotonic() - ts) < NODE_ALIVE_WINDOW_S


# --------------------------------------------------------------------------
# UI
# --------------------------------------------------------------------------
class ADASUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title('ADAS UI — CARLA + ACC + LKAS')

        # Long-lived child processes.
        self.carla_proc: subprocess.Popen | None = None
        self.bridge_proc: subprocess.Popen | None = None
        # Scenario harness. Mutually exclusive with bridge_proc — it is
        # itself the CARLA<->ROS bridge while a scenario runs.
        self.scenario_proc: subprocess.Popen | None = None
        self.stack_proc: subprocess.Popen | None = None         # start_adas.sh
        self.foxglove_proc: subprocess.Popen | None = None      # foxglove_bridge
        self.acc_procs: list[subprocess.Popen] = []             # when toggled independently
        self.lkas_procs: list[subprocess.Popen] = []
        self.morai_bridge_procs: list[subprocess.Popen] = []    # state + control adapters
        # NPC spawner runs the long-lived TRAFFIC_SPAWN_SNIPPET. The
        # snippet stays alive in a heartbeat loop to keep its TM client
        # connected — without that, CARLA 0.9.16 drops per-vehicle
        # autopilot ownership and the NPCs freeze. Tracked so Spawn
        # can replace the previous spawner, Clear can kill it before
        # destroying the actors (otherwise the heartbeat would just
        # re-attach autopilot on cars Clear is about to destroy), and
        # UI shutdown can clean it up so it doesn't survive past the UI.
        self.npc_spawn_proc: subprocess.Popen | None = None

        self.acc_on = False
        self.lkas_on = False

        # ROS subscriber for the camera widget + node-alive heartbeats.
        self.ros_node: TelemetryView | None = None
        self.ros_thread: threading.Thread | None = None

        # Frame throttling + active source.
        self._camera_source_var: tk.StringVar | None = None  # set in _build_ui
        self._render_after = None
        self._last_rendered_jpeg: bytes | None = None  # avoid re-decoding the same frame
        self._last_bgr: np.ndarray | None = None        # last decoded BGR frame (for video writer)
        # BEV (always-on /ADAS/ipm/debug_image) — same idempotency trick
        # so we don't re-decode a jpeg we already painted.
        self._last_rendered_bev_jpeg: bytes | None = None

        # Video recorder. None when idle; cv2.VideoWriter while recording.
        # Records whatever the camera widget is currently showing — switching
        # source mid-recording therefore changes what gets written, which is
        # the natural UX (you record what you see).
        self._video_writer: 'cv2.VideoWriter | None' = None
        self._video_path: Path | None = None
        self._video_size: tuple[int, int] | None = None  # (w, h) frozen at record start

        self._build_ui()
        self._start_camera_view()

    # --------------------------------------------------------------------
    # UI construction
    # --------------------------------------------------------------------
    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=8)
        outer.grid(row=0, column=0, sticky='nsew')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        # Column 0 = controls (fixed width), col 1 = camera + log (grows
        # with window width), col 2 = permanent BEV panel (fixed width).
        # weight=1 only on col 1 so the BEV preserves its native aspect
        # without stretching when the window is resized.
        outer.columnconfigure(1, weight=1)
        outer.rowconfigure(0, weight=1)

        # Left column: controls.
        left = ttk.Frame(outer)
        left.grid(row=0, column=0, sticky='nw', padx=(0, 8))

        ttk.Label(left, text='RPC port').grid(row=0, column=0, sticky='w')
        self.port_var = tk.StringVar(value='2000')
        ttk.Entry(left, textvariable=self.port_var, width=8).grid(
            row=0, column=1, sticky='w', padx=4, pady=2)

        ttk.Label(left, text='Quality').grid(row=1, column=0, sticky='w')
        self.quality_var = tk.StringVar(value='Epic')
        ttk.Combobox(left, textvariable=self.quality_var, values=['Epic', 'Low'],
                     state='readonly', width=6).grid(
            row=1, column=1, sticky='w', padx=4, pady=2)

        ttk.Label(left, text='Town').grid(row=2, column=0, sticky='w')
        self.town_var = tk.StringVar(value='Town03')
        self.town_combo = ttk.Combobox(left, textvariable=self.town_var,
                                        values=TOWNS, state='readonly', width=16)
        self.town_combo.grid(row=2, column=1, sticky='w', padx=4, pady=2)
        self.town_combo.bind('<<ComboboxSelected>>', self._on_town_change)

        ttk.Label(left, text='Weather').grid(row=3, column=0, sticky='w')
        self.weather_var = tk.StringVar(value='ClearNoon')
        ttk.Combobox(left, textvariable=self.weather_var, values=WEATHER_PRESETS,
                     state='readonly', width=16).grid(
            row=3, column=1, sticky='w', padx=4, pady=2)
        ttk.Button(left, text='Apply Weather', command=self.apply_weather).grid(
            row=4, column=0, columnspan=2, sticky='ew', padx=4, pady=(0, 2))

        ttk.Label(left, text='Traffic (NPCs)').grid(row=5, column=0, sticky='w')
        self.traffic_var = tk.StringVar(value='0')
        ttk.Combobox(left, textvariable=self.traffic_var, values=TRAFFIC_OPTIONS,
                     state='readonly', width=8).grid(
            row=5, column=1, sticky='w', padx=4, pady=2)
        traffic_btns = ttk.Frame(left)
        traffic_btns.grid(row=6, column=0, columnspan=2, sticky='ew', padx=4, pady=(0, 2))
        traffic_btns.columnconfigure(0, weight=1)
        traffic_btns.columnconfigure(1, weight=1)
        ttk.Button(traffic_btns, text='Spawn Traffic', command=self.spawn_traffic).grid(
            row=0, column=0, sticky='ew')
        ttk.Button(traffic_btns, text='Clear Traffic', command=self.clear_traffic).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))

        # Process group.
        procs = ttk.LabelFrame(left, text='Processes', padding=6)
        procs.grid(row=7, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        procs.columnconfigure(0, weight=1)
        procs.columnconfigure(1, weight=1)
        # Simulator selector — gates every node-launch button below
        # (Run start_adas.sh, ACC toggle, LKAS toggle): picks the
        # `simulator:=carla|morai` ros param each node uses for its
        # camera calibration and speed-message type.
        ttk.Label(procs, text='Simulator:').grid(
            row=0, column=0, sticky='w', pady=(0, 4))
        # Defaults to morai: this dropdown landing on the wrong value
        # (silently keeping CARLA's full-strength gains/speed-message-type)
        # has been the root cause of "throttle pinned at 1" and "v=0" more
        # than once. Flip back to 'carla' explicitly when testing CARLA.
        self.simulator_var = tk.StringVar(value='morai')
        ttk.Combobox(procs, textvariable=self.simulator_var,
                     values=['carla', 'morai'],
                     state='readonly', width=8).grid(
            row=0, column=1, sticky='w', pady=(0, 4))
        ttk.Button(procs, text='Start CARLA', command=self.start_carla).grid(
            row=1, column=0, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop CARLA', command=self.stop_carla).grid(
            row=1, column=1, sticky='ew', pady=2, padx=(4, 0))
        ttk.Button(procs, text='Restart CARLA', command=self.restart_carla).grid(
            row=2, column=0, columnspan=2, sticky='ew', pady=2)
        ttk.Button(procs, text='Start Bridge', command=self.start_bridge).grid(
            row=3, column=0, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop Bridge', command=self.stop_bridge).grid(
            row=3, column=1, sticky='ew', pady=2, padx=(4, 0))
        # Spawn index — passed as --spawn-index to the bridge at Start
        # Bridge. Index into the current town's spawn_points list; the
        # ego appears there and the lead `--lead-gap-m` ahead. Takes
        # effect on the next Start Bridge. The "List" button dumps every
        # spawn point (index, x, y, z, yaw) into the log so the user
        # can pick one — CARLA must be running.
        ttk.Label(procs, text='Spawn index:').grid(
            row=4, column=0, sticky='w', pady=(0, 4))
        spawn_frame = ttk.Frame(procs)
        spawn_frame.grid(row=4, column=1, sticky='w', pady=(0, 4))
        self.spawn_index_var = tk.StringVar(value='0')
        ttk.Spinbox(spawn_frame, from_=0, to=999, width=6,
                    textvariable=self.spawn_index_var).pack(side='left')
        ttk.Button(spawn_frame, text='List',
                   command=self.list_spawns, width=6).pack(
            side='left', padx=(4, 0))
        # Junction policy — passed as --junction-policy to the bridge at
        # Start Bridge. Switching mid-run has no effect; restart the bridge.
        # See DEBUG.md §13.
        ttk.Label(procs, text='Junction policy:').grid(
            row=5, column=0, sticky='w', pady=(0, 4))
        self.junction_policy_var = tk.StringVar(value='Pure pursuit')
        ttk.Combobox(procs, textvariable=self.junction_policy_var,
                     values=list(JUNCTION_POLICIES.keys()),
                     state='readonly', width=14).grid(
            row=5, column=1, sticky='w', pady=(0, 4))
        # Rosbag recording toggle — passed as --record to the bridge at
        # Start Bridge. Off by default (bridge used to leave a GB/min
        # rosbag on disk every run whether the operator wanted it or
        # not). Ticking it means the next Start Bridge will record.
        self.rosbag_record_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(procs, text='Record rosbag',
                        variable=self.rosbag_record_var).grid(
            row=6, column=0, columnspan=2, sticky='w', pady=(0, 4))
        # Dry run: ACC/LKAS keep computing and publishing cmd_vel/
        # cmd_steer normally (so their output is still fully visible/
        # validatable), but control_adapter_node never forwards it to
        # MORAI -- for driving by hand and watching ACC/LKAS output
        # without them actually touching the vehicle. MORAI only; has
        # no effect on CARLA (carlaaccsim bridge doesn't read this).
        self.dry_run_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(procs, text='Dry run (no vehicle commands)',
                        variable=self.dry_run_var).grid(
            row=7, column=0, columnspan=2, sticky='w', pady=(0, 4))
        ttk.Button(procs, text='Run start_adas.sh', command=self.run_start_adas).grid(
            row=8, column=0, columnspan=2, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop ADAS Stack', command=self.stop_stack).grid(
            row=9, column=0, columnspan=2, sticky='ew', pady=2)
        # Foxglove bridge — independent visualisation tool. Opens
        # ws://localhost:8765 for Foxglove Studio (desktop or web).
        # Lives outside the ADAS/CARLA lifecycle so layouts can also be
        # used for rosbag playback after the stack is stopped.
        ttk.Button(procs, text='Start Foxglove',
                   command=self.start_foxglove).grid(
            row=10, column=0, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop Foxglove',
                   command=self.stop_foxglove).grid(
            row=10, column=1, sticky='ew', pady=2, padx=(4, 0))
        # MORAI adapter nodes (state_adapter_node + control_adapter_node,
        # package morai_bridge) — translate MORAI's ROS2 Interface topics
        # to/from this stack's /Car_1/* topics. No CARLA/bridge process
        # involved; just plain `ros2 run` against this workspace, so
        # MORAI's own simulator + ROS2 Interfaces must already be running.
        ttk.Button(procs, text='Start MORAI Bridge',
                   command=self.start_morai_bridge).grid(
            row=11, column=0, sticky='ew', pady=2)
        ttk.Button(procs, text='Stop MORAI Bridge',
                   command=self.stop_morai_bridge).grid(
            row=11, column=1, sticky='ew', pady=2, padx=(4, 0))

        # Feature toggles. Each row has a button (user intent — ON/OFF) and a
        # small status dot reflecting whether the backing nodes are actually
        # publishing on their heartbeat topics. Green = publishing, grey = silent.
        feats = ttk.LabelFrame(left, text='Features', padding=6)
        feats.grid(row=8, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        feats.columnconfigure(0, weight=1)

        # ── Object-detection (YOLO) model selector, above ACC ────────
        # Populated from OBJECT_MODELS at the top of this file. Passed
        # to perception_node as -p model_filename:=<ref> at ACC: ON.
        self.object_model_var = tk.StringVar(value=OBJECT_MODELS[0][0])
        obj_row = ttk.Frame(feats)
        obj_row.grid(row=0, column=0, columnspan=2, sticky='ew',
                     pady=(0, 2))
        obj_row.columnconfigure(1, weight=1)
        ttk.Label(obj_row, text='Object model:').grid(
            row=0, column=0, sticky='w')
        ttk.Combobox(obj_row, textvariable=self.object_model_var,
                     values=[d for d, _ in OBJECT_MODELS],
                     state='readonly', width=32).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))

        self.acc_btn = ttk.Button(feats, text='ACC: OFF', command=self.toggle_acc)
        self.acc_btn.grid(row=1, column=0, sticky='ew', pady=2)
        self.acc_status_var = tk.StringVar(value='○ idle')
        self.acc_status_lbl = ttk.Label(feats, textvariable=self.acc_status_var,
                                         foreground='#888888', width=12)
        self.acc_status_lbl.grid(row=1, column=1, sticky='w', padx=(6, 0))


        # ── Kalman lane smoothing toggle, above Lane model ───────────
        # Passed to lane_detection_node as -p enable_kalman:=<bool>.
        # Toggling mid-run requires LKAS: OFF → change → LKAS: ON.
        self.kalman_enable_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(feats, text='Kalman lane smoothing',
                        variable=self.kalman_enable_var).grid(
            row=2, column=0, columnspan=2, sticky='w', pady=(4, 0))

        # ── KF process-noise PSDs (tuning knobs) ─────────────────────
        # q_a curvature, q_b heading, q_c lateral offset. Passed to
        # lane_detection_node as -p kf_q_a:=... at LKAS: ON. Default
        # values match the node's own defaults; empty box = "use node
        # default" (skipped when building the launch args).
        kf_row = ttk.Frame(feats)
        kf_row.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(2, 0))
        ttk.Label(kf_row, text='KF q_a').grid(row=0, column=0, sticky='w')
        # 0.5 confirmed on Town10 right turns 2026-07-10 — earlier
        # 5e-2 still lagged into curves; 10× that made the KF track
        # curvature in real time without noticeable overshoot.
        self.kf_qa_var = tk.StringVar(value='0.5')
        ttk.Entry(kf_row, textvariable=self.kf_qa_var, width=7).grid(
            row=0, column=1, sticky='w', padx=(2, 8))
        ttk.Label(kf_row, text='q_b').grid(row=0, column=2, sticky='w')
        # 5.0 confirmed on Town10 right turns 2026-07-10; heading state
        # tracks lane-slope changes without visible lag on the BEV.
        self.kf_qb_var = tk.StringVar(value='5.0')
        ttk.Entry(kf_row, textvariable=self.kf_qb_var, width=7).grid(
            row=0, column=3, sticky='w', padx=(2, 8))
        ttk.Label(kf_row, text='q_c').grid(row=0, column=4, sticky='w')
        # Bumped 5× (was 1.0) for the lateral-offset state.
        self.kf_qc_var = tk.StringVar(value='5.0')
        ttk.Entry(kf_row, textvariable=self.kf_qc_var, width=7).grid(
            row=0, column=5, sticky='w', padx=(2, 8))
        # Live-push q_a/q_b/q_c to a running lane_detection_node via
        # `ros2 param set`. Needs the perception node to have registered
        # a SetParametersCallback that rebuilds the KF Q matrix.
        ttk.Button(kf_row, text='Apply KF',
                   command=self._apply_kf_params).grid(
            row=0, column=6, sticky='w')

        # ── Lane-detection (UFLD) model selector, above LKAS ─────────
        # See LANE_MODELS at the top. Passed to lane_detection_node as
        # -p model_filename:=<ref> at LKAS: ON. No filesystem scan at
        # build time (would crash if _log ran before the log widget
        # existed).
        self.lane_model_var = tk.StringVar(value=LANE_MODELS[0][0])
        model_row = ttk.Frame(feats)
        model_row.grid(row=4, column=0, columnspan=2, sticky='ew',
                       pady=(4, 2))
        model_row.columnconfigure(1, weight=1)
        ttk.Label(model_row, text='Lane model:').grid(
            row=0, column=0, sticky='w')
        ttk.Combobox(model_row, textvariable=self.lane_model_var,
                     values=[d for d, _ in LANE_MODELS],
                     state='readonly', width=32).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))
        self.lkas_btn = ttk.Button(feats, text='LKAS: OFF', command=self.toggle_lkas)
        self.lkas_btn.grid(row=5, column=0, sticky='ew', pady=2)
        self.lkas_status_var = tk.StringVar(value='○ idle')
        self.lkas_status_lbl = ttk.Label(feats, textvariable=self.lkas_status_var,
                                          foreground='#888888', width=12)
        self.lkas_status_lbl.grid(row=5, column=1, sticky='w', padx=(6, 0))

        # Recorder. Writes whatever the camera widget is currently showing
        # (active source, decoded once per render tick) to a timestamped
        # .mp4 in <workspace>/recordings/.
        rec_frame = ttk.LabelFrame(left, text='Recording', padding=6)
        rec_frame.grid(row=9, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        rec_frame.columnconfigure(0, weight=1)
        self.record_btn = ttk.Button(rec_frame, text='● Record',
                                     command=self._toggle_recording)
        self.record_btn.grid(row=0, column=0, sticky='ew', pady=2)
        self.record_status_var = tk.StringVar(value='idle')
        self.record_status_lbl = ttk.Label(rec_frame,
                                            textvariable=self.record_status_var,
                                            foreground='#888888')
        self.record_status_lbl.grid(row=1, column=0, sticky='w')

        # Status + clear-log.
        self.status_var = tk.StringVar(value='Ready')
        ttk.Label(left, textvariable=self.status_var, foreground='#0044aa',
                  wraplength=240).grid(
            row=10, column=0, columnspan=2, sticky='w', pady=(10, 2))
        ttk.Button(left, text='Clear log', command=self._clear_log).grid(
            row=11, column=0, columnspan=2, sticky='ew', pady=2)

        # Right column: camera (top) + log (bottom).
        right = ttk.Frame(outer)
        right.grid(row=0, column=1, sticky='nsew')
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=2)

        self._cam_frame = ttk.LabelFrame(right, text='Live camera', padding=4)
        self._cam_frame.grid(row=0, column=0, sticky='nsew')
        self._cam_frame.columnconfigure(0, weight=1)
        self._cam_frame.rowconfigure(1, weight=1)

        # Source selector — switches between raw bridge feed and the YOLO /
        # UFLD / combined annotated debug images published by the perception
        # nodes. The right side of the bar shows live ego speed.
        src_bar = ttk.Frame(self._cam_frame)
        src_bar.grid(row=0, column=0, sticky='ew', pady=(0, 4))
        src_bar.columnconfigure(2, weight=1)
        ttk.Label(src_bar, text='Source:').grid(row=0, column=0, sticky='w')
        self._camera_source_var = tk.StringVar(value='Raw')
        ttk.Combobox(src_bar, textvariable=self._camera_source_var,
                     values=list(CAMERA_SOURCES.keys()),
                     state='readonly', width=18).grid(
            row=0, column=1, sticky='w', padx=(4, 0))
        self._camera_source_var.trace_add('write', lambda *_: self._refresh_cam_title())

        self.speed_var = tk.StringVar(value='Speed: —')
        ttk.Label(src_bar, textvariable=self.speed_var,
                  font=('Monospace', 11, 'bold'),
                  foreground='#0044aa').grid(
            row=0, column=2, sticky='e', padx=(8, 4))

        placeholder = ('waiting for first camera frame…'
                       if CAMERA_AVAILABLE
                       else f'camera disabled: {_camera_err}')
        # tk.Label width/height switch units depending on whether an image is
        # set. Hold the slot open with a pre-sized blank PhotoImage so the
        # widget has CAMERA_W × CAMERA_H pixels even before the first frame.
        self._placeholder_photo = tk.PhotoImage(width=CAMERA_W, height=CAMERA_H)
        self.camera_label = tk.Label(self._cam_frame, background='#222222', anchor='center',
                                      text=placeholder, foreground='#aaaaaa',
                                      image=self._placeholder_photo, compound='center')
        self.camera_label.image = self._placeholder_photo
        self.camera_label.grid(row=1, column=0, sticky='nsew')
        self._refresh_cam_title()

        log_frame = ttk.LabelFrame(right, text='Log', padding=4)
        log_frame.grid(row=1, column=0, sticky='nsew', pady=(6, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = scrolledtext.ScrolledText(log_frame, height=10, width=80,
                                              font=('Monospace', 9), wrap='word')
        self.log.grid(row=0, column=0, sticky='nsew')

        # ── Permanent BEV panel (column 2 of outer) ─────────────────────
        # Always subscribed to /ADAS/ipm/debug_image — distinct from the
        # source-switchable camera widget on its left, so the operator
        # can see the BEV simultaneously with whichever camera source is
        # selected. Native IPM is 320×480, rendered here at BEV_W × BEV_H
        # with aspect preserved (see _render_bev_frame).
        # Column 2 holds the BEV on top and the scenario controls beneath
        # it. The BEV gets the row weight so it keeps absorbing spare
        # vertical space; the scenario panel takes its natural height.
        rightcol = ttk.Frame(outer)
        rightcol.grid(row=0, column=2, sticky='ns', padx=(8, 0))
        rightcol.rowconfigure(0, weight=1)
        rightcol.columnconfigure(0, weight=1)

        self._bev_frame = ttk.LabelFrame(rightcol,
                                         text='BEV — /ADAS/ipm/debug_image',
                                         padding=4)
        self._bev_frame.grid(row=0, column=0, sticky='ns')
        self._bev_placeholder_photo = tk.PhotoImage(width=BEV_W, height=BEV_H)
        bev_placeholder = ('waiting for IPM frame…'
                           if CAMERA_AVAILABLE
                           else f'BEV disabled: {_camera_err}')
        self.bev_label = tk.Label(self._bev_frame, background='#222222',
                                  anchor='center', text=bev_placeholder,
                                  foreground='#aaaaaa',
                                  image=self._bev_placeholder_photo,
                                  compound='center')
        self.bev_label.image = self._bev_placeholder_photo
        self.bev_label.grid(row=0, column=0, sticky='ns')

        # ── Scenario harness (UN R171 Annex 4 §4.2.5.2.1) ────────────────
        # Drives the stack against a stationary target on a straight road.
        # Starts scenarios/r171_stationary_target.py, which acts as the
        # CARLA<->ROS bridge for the run — so Start Bridge has to be
        # stopped first (_start_scenario does that automatically).
        # Lives under the BEV rather than in the left column: that column
        # is already full to the window's height, and a ninth LabelFrame
        # there collided with Features.
        scen = ttk.LabelFrame(rightcol, text='Scenario — UN R171 stationary '
                                             'target', padding=6)
        scen.grid(row=1, column=0, sticky='new', pady=(8, 0))
        scen.columnconfigure(1, weight=1)

        ttk.Label(scen, text='Approach speed:').grid(
            row=0, column=0, sticky='w', pady=2)
        self.scen_speed_var = tk.StringVar(value='50')
        ttk.Combobox(scen, textvariable=self.scen_speed_var,
                     values=['30', '50', '70', '90', '110', '130'],
                     width=7).grid(row=0, column=1, sticky='ew', pady=2)
        ttk.Label(scen, text='km/h').grid(row=0, column=2, sticky='w',
                                          padx=(4, 0))

        ttk.Label(scen, text='Lateral offset:').grid(
            row=1, column=0, sticky='w', pady=2)
        self.scen_offset_var = tk.StringVar(value='0.0')
        ttk.Combobox(scen, textvariable=self.scen_offset_var,
                     values=['0.0', '0.5', '1.0'],
                     width=7).grid(row=1, column=1, sticky='ew', pady=2)
        ttk.Label(scen, text='m').grid(row=1, column=2, sticky='w',
                                       padx=(4, 0))

        # TTC margin sets the handover distance: the scenario holds the
        # approach speed open-loop until gap == ttc * v, then the stack's
        # ACC owns throttle/brake. 4.5 s at 50 km/h = 62.5 m.
        ttk.Label(scen, text='TTC margin:').grid(
            row=2, column=0, sticky='w', pady=2)
        self.scen_ttc_var = tk.StringVar(value='6.0')
        ttk.Combobox(scen, textvariable=self.scen_ttc_var,
                     values=['4.5', '6.0', '10.0'],
                     width=7).grid(row=2, column=1, sticky='ew', pady=2)
        ttk.Label(scen, text='s').grid(row=2, column=2, sticky='w',
                                       padx=(4, 0))

        # locked = the scenario holds the lane centreline, isolating the
        # longitudinal result. lkas = Stanley owns steer via /Car_1/cmd_steer.
        ttk.Label(scen, text='Lateral mode:').grid(
            row=3, column=0, sticky='w', pady=2)
        self.scen_lateral_var = tk.StringVar(value='locked')
        ttk.Combobox(scen, textvariable=self.scen_lateral_var,
                     values=['locked', 'lkas'], state='readonly',
                     width=7).grid(row=3, column=1, sticky='ew', pady=2)

        self.scen_spawn_note = ttk.Label(
            scen, text='Town06 spawn 80 — 718 m straight',
            foreground='#666666')
        self.scen_spawn_note.grid(row=4, column=0, columnspan=3,
                                  sticky='w', pady=(4, 4))

        ttk.Button(scen, text='Run single point',
                   command=self.start_scenario_single).grid(
            row=5, column=0, columnspan=3, sticky='ew', pady=2)
        matrix_frame = ttk.Frame(scen)
        matrix_frame.grid(row=6, column=0, columnspan=3, sticky='ew', pady=2)
        for col in (0, 1, 2):
            matrix_frame.columnconfigure(col, weight=1)
        ttk.Button(matrix_frame, text='Matrix 30',
                   command=lambda: self.start_scenario_matrix(None)).grid(
            row=0, column=0, sticky='ew')
        ttk.Button(matrix_frame, text='Block A',
                   command=lambda: self.start_scenario_matrix('A')).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))
        ttk.Button(matrix_frame, text='Block B',
                   command=lambda: self.start_scenario_matrix('B')).grid(
            row=0, column=2, sticky='ew', padx=(4, 0))
        ttk.Button(scen, text='Stop scenario',
                   command=self.stop_scenario).grid(
            row=7, column=0, columnspan=3, sticky='ew', pady=2)

        # ── Scenario harness (UN R171 Annex 4 §4.2.5.2.2) ────────────────
        # Same target and same KPI as the panel above, round a bend. The
        # site — a surveyed constant-radius arc, see scenarios/README.md —
        # is what makes the run different, so it is the first control.
        # Lateral mode is deliberately NOT repeated: the selector in the
        # stationary panel governs both R171 scripts, because the two
        # results are only comparable if that choice matches.
        curve = ttk.LabelFrame(rightcol, text='Scenario — UN R171 curved '
                                              'target', padding=6)
        curve.grid(row=2, column=0, sticky='new', pady=(8, 0))
        curve.columnconfigure(1, weight=1)

        site_names = self._curve_site_names()
        ttk.Label(curve, text='Curve site:').grid(
            row=0, column=0, sticky='w', pady=2)
        self.curve_site_var = tk.StringVar(
            value=CURVE_SITE_DEFAULT if CURVE_SITE_DEFAULT in site_names
            else (site_names[0] if site_names else ''))
        site_box = ttk.Combobox(curve, textvariable=self.curve_site_var,
                                values=site_names, state='readonly', width=10)
        site_box.grid(row=0, column=1, columnspan=2, sticky='ew', pady=2)
        site_box.bind('<<ComboboxSelected>>',
                      lambda _e: self._update_curve_note())

        ttk.Label(curve, text='Approach speed:').grid(
            row=1, column=0, sticky='w', pady=2)
        self.curve_speed_var = tk.StringVar(value='50')
        speed_box = ttk.Combobox(curve, textvariable=self.curve_speed_var,
                                 values=['30', '50', '70', '90', '110', '130'],
                                 width=7)
        speed_box.grid(row=1, column=1, sticky='ew', pady=2)
        speed_box.bind('<<ComboboxSelected>>',
                       lambda _e: self._update_curve_note())
        ttk.Label(curve, text='km/h').grid(row=1, column=2, sticky='w',
                                           padx=(4, 0))

        ttk.Label(curve, text='TTC margin:').grid(
            row=2, column=0, sticky='w', pady=2)
        self.curve_ttc_var = tk.StringVar(value='6.0')
        ttk.Combobox(curve, textvariable=self.curve_ttc_var,
                     values=['4.5', '6.0', '10.0'],
                     width=7).grid(row=2, column=1, sticky='ew', pady=2)
        ttk.Label(curve, text='s').grid(row=2, column=2, sticky='w',
                                        padx=(4, 0))

        # §4.2.5.2.2.1.1 fixes the target within 0.5 m of the lane centre,
        # so the straight matrix's 1.0 m option does not exist here.
        ttk.Label(curve, text='Lateral offset:').grid(
            row=3, column=0, sticky='w', pady=2)
        self.curve_offset_var = tk.StringVar(value='0.0')
        ttk.Combobox(curve, textvariable=self.curve_offset_var,
                     values=['0.0', '0.5'],
                     width=7).grid(row=3, column=1, sticky='ew', pady=2)
        ttk.Label(curve, text='m').grid(row=3, column=2, sticky='w',
                                        padx=(4, 0))

        # wraplength keeps a long geometry line inside the panel. Without
        # it the label sets the column's width, which both widens the
        # window and makes this box wider than the stationary one above
        # it; with it, all three scenario boxes take the column width the
        # BEV already defines.
        self.curve_note = ttk.Label(curve, text='', foreground='#666666',
                                    justify='left', wraplength=BEV_W - 60)
        self.curve_note.grid(row=4, column=0, columnspan=3, sticky='w',
                             pady=(4, 4))

        ttk.Button(curve, text='Run single point',
                   command=self.start_curve_single).grid(
            row=5, column=0, columnspan=3, sticky='ew', pady=2)
        curve_btns = ttk.Frame(curve)
        curve_btns.grid(row=6, column=0, columnspan=3, sticky='ew', pady=2)
        curve_btns.columnconfigure(0, weight=1)
        curve_btns.columnconfigure(1, weight=1)
        ttk.Button(curve_btns, text='Matrix (this site)',
                   command=self.start_curve_matrix).grid(
            row=0, column=0, sticky='ew')
        ttk.Button(curve_btns, text='Stop',
                   command=self.stop_scenario).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))

        # ── Scenario harness (UN R79 Annex 8 §3.2) ───────────────────────
        # Lane keeping with no target. There is no speed control: R79
        # derives the test speed from the site radius and the declared
        # aysmax so the demand lands in the window the paragraph specifies
        # (see scenarios/README.md). The note below previews what that
        # works out to before anything is launched.
        lka = ttk.LabelFrame(rightcol, text='Scenario — UN R79 lane keeping',
                             padding=6)
        lka.grid(row=3, column=0, sticky='new', pady=(8, 0))
        lka.columnconfigure(1, weight=1)

        ttk.Label(lka, text='Curve site:').grid(
            row=0, column=0, sticky='w', pady=2)
        self.lka_site_var = tk.StringVar(
            value=CURVE_SITE_DEFAULT if CURVE_SITE_DEFAULT in site_names
            else (site_names[0] if site_names else ''))
        lka_site_box = ttk.Combobox(lka, textvariable=self.lka_site_var,
                                    values=site_names, state='readonly',
                                    width=10)
        lka_site_box.grid(row=0, column=1, sticky='ew', pady=2)
        lka_site_box.bind('<<ComboboxSelected>>',
                          lambda _e: self._update_lka_note())

        ttk.Label(lka, text='Test:').grid(row=1, column=0, sticky='w', pady=2)
        self.lka_test_var = tk.StringVar(value=LKA_TESTS[0])
        lka_test_box = ttk.Combobox(lka, textvariable=self.lka_test_var,
                                    values=list(LKA_TESTS), state='readonly',
                                    width=10)
        lka_test_box.grid(row=1, column=1, sticky='ew', pady=2)
        lka_test_box.bind('<<ComboboxSelected>>',
                          lambda _e: self._update_lka_note())

        ttk.Label(lka, text='Declared aysmax:').grid(
            row=2, column=0, sticky='w', pady=2)
        self.lka_ay_var = tk.StringVar(value=LKA_DECLARED_AY_DEFAULT)
        ay_entry = ttk.Entry(lka, textvariable=self.lka_ay_var, width=10)
        ay_entry.grid(row=2, column=1, sticky='ew', pady=2)
        ay_entry.bind('<FocusOut>', lambda _e: self._update_lka_note())
        ay_entry.bind('<Return>', lambda _e: self._update_lka_note())

        self.lka_note = ttk.Label(lka, text='', foreground='#666666',
                                  justify='left', wraplength=BEV_W - 60)
        self.lka_note.grid(row=3, column=0, columnspan=2, sticky='w',
                           pady=(4, 4))

        ttk.Button(lka, text='Run single test',
                   command=self.start_lka_single).grid(
            row=4, column=0, columnspan=2, sticky='ew', pady=2)
        lka_btns = ttk.Frame(lka)
        lka_btns.grid(row=5, column=0, columnspan=2, sticky='ew', pady=2)
        for col in (0, 1, 2):
            lka_btns.columnconfigure(col, weight=1)
        ttk.Button(lka_btns, text='Matrix',
                   command=self.start_lka_matrix).grid(
            row=0, column=0, sticky='ew')
        # The sweep is the radius x speed grid, judged on kept_lane —
        # the engineering view next to the compliance one.
        ttk.Button(lka_btns, text='Sweep R×v',
                   command=self.start_lka_sweep).grid(
            row=0, column=1, sticky='ew', padx=(4, 0))
        ttk.Button(lka_btns, text='Stop',
                   command=self.stop_scenario).grid(
            row=0, column=2, sticky='ew', padx=(4, 0))

        self._update_curve_note()
        self._update_lka_note()
        if not SCENARIO_META_OK:
            self._log(f'[ui] curve site table unavailable '
                      f'({_scenario_meta_err}) — the curved and R79 panels '
                      f'will still launch, but without geometry previews')

    # --------------------------------------------------------------------
    # Logging
    # --------------------------------------------------------------------
    def _log(self, msg: str):
        self.log.insert('end', msg + '\n')
        self.log.see('end')

    def _clear_log(self):
        self.log.delete('1.0', 'end')

    # --------------------------------------------------------------------
    # Live camera subscriber
    # --------------------------------------------------------------------
    def _start_camera_view(self):
        if not CAMERA_AVAILABLE:
            self._log(f'[ui] camera widget disabled: {_camera_err}')
            return
        try:
            rclpy.init()
        except RuntimeError:
            # Already initialised somewhere in this process.
            pass
        self.ros_node = TelemetryView()
        self.ros_thread = threading.Thread(
            target=lambda: rclpy.spin(self.ros_node), daemon=True)
        self.ros_thread.start()
        # Schedule first render tick.
        self.root.after(int(1000 / CAMERA_UI_HZ), self._render_tick)
        self._log(
            '[ui] subscribed to '
            + ', '.join(CAMERA_SOURCES.values())
            + f', {CMD_VEL_TOPIC}, {CMD_STEER_TOPIC}, {SPEED_TOPIC}')

    def _active_camera_topic(self) -> str:
        name = self._camera_source_var.get() if self._camera_source_var else 'Raw'
        return CAMERA_SOURCES.get(name, CAMERA_TOPIC)

    def _refresh_cam_title(self):
        # Show the active topic in the camera frame title so the user always
        # knows which feed they're looking at.
        topic = self._active_camera_topic()
        if hasattr(self, '_cam_frame'):
            self._cam_frame.configure(text=f'Live camera — {topic}')

    def _render_tick(self):
        if self.ros_node is not None:
            jpeg = self.ros_node.latest_jpegs.get(self._active_camera_topic())
            # cv2.imdecode raises a hard C++ assertion (not a graceful None)
            # on an empty buffer, unlike a merely-malformed-but-nonempty one.
            # A publisher sending a zero-byte CompressedImage (e.g. a debug
            # topic with nothing to draw yet) used to crash this callback
            # permanently -- `_render_tick` only reschedules itself via
            # `after()` at the very end, so one bad frame froze the camera
            # view for the rest of the session. Guard both decode sites.
            if jpeg and jpeg is not self._last_rendered_jpeg:
                arr = np.frombuffer(jpeg, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is not None:
                    self._last_bgr = bgr
                    self._render_frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                self._last_rendered_jpeg = jpeg
            if self._video_writer is not None and self._last_bgr is not None:
                self._record_frame(self._last_bgr)
            # BEV panel — independent of the source-selector camera. Same
            # decode-only-if-new pattern.
            bev_jpeg = self.ros_node.latest_bev_jpeg
            if bev_jpeg and bev_jpeg is not self._last_rendered_bev_jpeg:
                arr = np.frombuffer(bev_jpeg, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is not None:
                    self._render_bev_frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                self._last_rendered_bev_jpeg = bev_jpeg
            self._refresh_status_dots()
            self._refresh_speed_label()
        self.root.after(int(1000 / CAMERA_UI_HZ), self._render_tick)

    def _refresh_speed_label(self):
        v = self.ros_node.speed_mps if self.ros_node is not None else None
        if v is None or not self.ros_node.is_alive(SPEED_TOPIC):
            self.speed_var.set('Speed:  —  km/h')
        else:
            self.speed_var.set(f'Speed: {v * 3.6:5.1f} km/h')

    # --------------------------------------------------------------------
    # Video recorder
    # --------------------------------------------------------------------
    def _toggle_recording(self):
        if self._video_writer is None:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        if self._last_bgr is None:
            self._log('[recorder] no camera frame yet — wait for the live '
                      'feed and try again')
            return
        rec_dir = ADAS_WK / 'recordings'
        rec_dir.mkdir(exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        source_tag = (self._camera_source_var.get()
                      .lower().replace(' ', '_').replace('(', '').replace(')', '').replace('+', '_'))
        self._video_path = rec_dir / f'adas_{ts}_{source_tag}.mp4'

        h, w = self._last_bgr.shape[:2]
        # mp4v is bundled with OpenCV's default build — no ffmpeg/H.264
        # licensing dance. Acceptable quality for a UI preview recording.
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(
            str(self._video_path), fourcc, float(CAMERA_UI_HZ), (w, h))
        if not writer.isOpened():
            self._log(f'[recorder] failed to open writer for '
                      f'{self._video_path} ({w}×{h}@{CAMERA_UI_HZ})')
            self._video_path = None
            return
        self._video_writer = writer
        self._video_size = (w, h)
        self.record_btn.configure(text='■ Stop')
        self.record_status_var.set(f'● REC  →  {self._video_path.name}')
        self.record_status_lbl.configure(foreground='#cc0000')
        self._log(f'[recorder] recording {w}×{h}@{CAMERA_UI_HZ} fps → '
                  f'{self._video_path}')

    def _record_frame(self, bgr: np.ndarray):
        # If the source switched and frame dims changed, resize to the
        # writer's original size — VideoWriter requires a constant frame size.
        h, w = bgr.shape[:2]
        if (w, h) != self._video_size:
            bgr = cv2.resize(bgr, self._video_size)
        try:
            self._video_writer.write(bgr)
        except Exception as e:
            self._log(f'[recorder] write failed: {e}; stopping recording')
            self._stop_recording()

    def _stop_recording(self):
        if self._video_writer is None:
            return
        try:
            self._video_writer.release()
        except Exception:
            pass
        path = self._video_path
        self._video_writer = None
        self._video_path = None
        self._video_size = None
        self.record_btn.configure(text='● Record')
        self.record_status_var.set(f'saved {path.name}' if path else 'idle')
        self.record_status_lbl.configure(foreground='#1a9b3c' if path else '#888888')
        self._log(f'[recorder] saved {path}')

    def _refresh_status_dots(self):
        # ACC is "active" when perception_node (publishes ACC debug image) AND
        # controller_node (publishes /Car_1/cmd_vel) are both heartbeating.
        # LKAS is "active" when lane_detection_node (publishes LKAS debug
        # image) AND stanley_node (publishes /Car_1/cmd_steer) are both up.
        node = self.ros_node
        if node is None:
            return
        perc_alive = node.is_alive(ACC_DEBUG_TOPIC)
        ctrl_alive = node.is_alive(CMD_VEL_TOPIC)
        lane_alive = node.is_alive(LKAS_DEBUG_TOPIC)
        stan_alive = node.is_alive(CMD_STEER_TOPIC)

        def label(perc_ok: bool, ctrl_ok: bool) -> tuple[str, str]:
            if perc_ok and ctrl_ok:
                return ('● active', '#1a9b3c')
            if perc_ok or ctrl_ok:
                return ('◐ partial', '#cc8800')
            return ('○ idle', '#888888')

        text, colour = label(perc_alive, ctrl_alive)
        self.acc_status_var.set(text)
        self.acc_status_lbl.configure(foreground=colour)
        text, colour = label(lane_alive, stan_alive)
        self.lkas_status_var.set(text)
        self.lkas_status_lbl.configure(foreground=colour)

    def _render_frame(self, rgb):
        # Resize to fit the camera widget (CAMERA_W × CAMERA_H), preserving
        # aspect ratio. If the user has expanded the window, grow up to the
        # actual label size.
        h, w = rgb.shape[:2]
        target_w = max(self.camera_label.winfo_width(), CAMERA_W)
        target_h = max(self.camera_label.winfo_height(), CAMERA_H)
        scale = min(target_w / w, target_h / h)
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        if (new_w, new_h) != (w, h):
            rgb = cv2.resize(rgb, (new_w, new_h))
        img = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(img)
        self.camera_label.configure(image=photo, text='')
        self.camera_label.image = photo  # keep a reference; Tk won't otherwise

    def _render_bev_frame(self, rgb):
        # BEV panel — fixed target size BEV_W × BEV_H, aspect preserved.
        # IMPORTANT: do NOT use self.bev_label.winfo_width() here as the
        # target. The label resizes to fit each PhotoImage we set, which
        # nudges winfo_width() up; next tick that becomes the new target
        # and the panel grows again. Because the BEV column has no grid
        # weight (so the window doesn't clamp it), this feedback loop
        # snowballs frame-by-frame until the BEV consumes the whole UI.
        # Pinning the target to the constants kills the loop.
        h, w = rgb.shape[:2]
        scale = min(BEV_W / w, BEV_H / h)
        new_w = max(1, int(w * scale))
        new_h = max(1, int(h * scale))
        if (new_w, new_h) != (w, h):
            rgb = cv2.resize(rgb, (new_w, new_h),
                             interpolation=cv2.INTER_NEAREST)
        img = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(img)
        self.bev_label.configure(image=photo, text='')
        self.bev_label.image = photo  # keep a reference; Tk won't otherwise

    # --------------------------------------------------------------------
    # Subprocess helpers
    # --------------------------------------------------------------------
    def _stream(self, proc: subprocess.Popen, prefix: str):
        for line in iter(proc.stdout.readline, ''):
            if not line:
                break
            self.root.after(0, self._log, f'[{prefix}] {line.rstrip()}')
        try:
            proc.stdout.close()
        except Exception:
            pass

    def _popen(self, cmd, cwd=None, source_ros=False, source_workspace=False,
               prefix='proc', extra_env=None) -> subprocess.Popen:
        """Launch a subprocess in its own process group. If `source_ros` is
        true, the command is run under `bash -c` after sourcing ROS (and
        optionally the ADAS workspace) so `rclpy` / `ros2 run` work.
        `extra_env` is merged on top of the current environment for the
        child."""
        env = None
        if extra_env:
            env = os.environ.copy()
            env.update(extra_env)
        if source_ros:
            parts = [f'source {shlex.quote(ROS_SETUP)}']
            if source_workspace and ADAS_INSTALL.exists():
                parts.append(f'source {shlex.quote(str(ADAS_INSTALL))}')
            parts.append('exec ' + ' '.join(shlex.quote(s) for s in cmd))
            shell_cmd = ' && '.join(parts)
            proc = subprocess.Popen(
                ['bash', '-c', shell_cmd], cwd=cwd, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True,
            )
        else:
            proc = subprocess.Popen(
                cmd, cwd=cwd, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True,
            )
        threading.Thread(target=self._stream, args=(proc, prefix),
                         daemon=True).start()
        return proc

    def _terminate(self, proc, label: str):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=4)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self._log(f'[ui] {label} terminated')

    @staticmethod
    def _pkill(patterns):
        for p in patterns:
            subprocess.run(['pkill', '-f', p], check=False, capture_output=True)

    # --------------------------------------------------------------------
    # CARLA
    # --------------------------------------------------------------------
    def _pkill_all_carla(self):
        """Force-kill every CARLA server on the host, including one started
        by hand outside the UI. Without this, Start CARLA's new boot can't
        bind port 2000 because the orphan still owns it, AND we can't apply
        a new town because the orphan ignores our boot-map ini edit."""
        r = subprocess.run(['pkill', '-9', '-f', 'CarlaUE4'],
                           capture_output=True, text=True, check=False)
        if r.returncode == 0:
            self._log('[ui] pkill -9 CarlaUE4: killed running CARLA process(es)')
        time.sleep(2)  # let the OS release the RPC port

    def _on_town_change(self, _evt=None):
        # Town only takes effect at next CARLA boot (in-band load_world()
        # segfaults on this install).
        if self.carla_proc and self.carla_proc.poll() is None:
            self._log(f'[ui] town set to {self.town_var.get()} — '
                      'restart CARLA to load it.')

    def _write_boot_map(self):
        if not CARLA_INI.exists():
            self._log(f'[ui] warn: {CARLA_INI} missing; skipping boot-map edit')
            return
        try:
            n, wanted, observed = set_boot_map(CARLA_INI, self.town_var.get())
            self._log(f'[ui] boot map → {wanted} ({n} ini lines updated)')
            if observed != wanted:
                self._log('[ui] WARNING: ini did not retain the requested map.')
        except Exception as e:
            self._log(f'[ui] warn: could not edit {CARLA_INI}: {e}')

    def start_carla(self):
        # Always pkill first — even if `self.carla_proc` looks dead, there
        # may be a hand-launched CARLA bound to port 2000 that would silently
        # block our new boot AND ignore the boot-map ini we're about to write.
        self._pkill_all_carla()
        self._write_boot_map()
        port = int(self.port_var.get())
        cmd = [CARLA_SERVER, '-RenderOffScreen',
               f'-carla-rpc-port={port}',
               f'-quality-level={self.quality_var.get()}']
        self._log(f'$ (cd {CARLA_DIR} && {" ".join(cmd)})')
        self.carla_proc = self._popen(cmd, cwd=str(CARLA_DIR),
                                       source_ros=False, prefix='carla')
        self.status_var.set(
            f'CARLA starting on port {port} — town: {self.town_var.get()}')
        # Verify after CARLA has had time to boot — if the ini hack didn't
        # take (a known issue on this install per project md), the user
        # sees the mismatch in the log.
        self.root.after(15000, self._verify_loaded_map)

    def restart_carla(self):
        """Stop + Start in one click — useful after changing town/quality
        since those only apply on a fresh boot."""
        self.stop_carla()
        time.sleep(1)
        self.start_carla()

    def _verify_loaded_map(self):
        """Connect via the carla-env Python, read world.get_map().name, log
        it. If it doesn't match the requested town, flag it — map-switching
        is known to be non-deterministic on this install."""
        wanted = self.town_var.get()
        snippet = (
            "import carla, sys\n"
            "c = carla.Client('localhost', int(sys.argv[1])); c.set_timeout(5.0)\n"
            "print('[map] loaded:', c.get_world().get_map().name)\n"
        )
        self._run_carla_snippet(snippet, [self.port_var.get()], 'map')
        self._log(f'[ui] expected town: {wanted} (check [map] line above)')

    # --------------------------------------------------------------------
    # Weather / Traffic — applied via the carla-env Python over the RPC
    # connection. CARLA must already be running.
    # --------------------------------------------------------------------
    def _run_carla_snippet(self, snippet: str, args: list[str], prefix: str):
        """Run a one-shot carla-env snippet, fire-and-forget. For
        long-lived snippets the caller wants the proc handle back; use
        the returned value (None when carla-env is missing)."""
        if not CARLA_PYTHON.exists():
            self._log(f'[ui] carla python missing: {CARLA_PYTHON}')
            return None
        cmd = [str(CARLA_PYTHON), '-c', snippet] + args
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, start_new_session=True,
        )
        threading.Thread(target=self._stream, args=(proc, prefix),
                         daemon=True).start()
        return proc

    def apply_weather(self):
        preset = self.weather_var.get()
        port = self.port_var.get()
        self._log(f'[ui] applying weather {preset} on port {port}')
        args = [preset, port]
        if preset in FOG_PRESETS:
            args.append(json.dumps(FOG_PRESETS[preset]))
        self._run_carla_snippet(_WEATHER_SNIPPET, args, 'weather')

    def spawn_traffic(self):
        n = self.traffic_var.get()
        port = self.port_var.get()
        # Kill the previous spawner first — the snippet is a forever
        # heartbeat loop, so without this every Spawn click would leave
        # an extra background process re-asserting autopilot on the
        # same vehicles. Replacing means the user can re-click Spawn
        # safely (e.g. to change N) without pile-up.
        self._terminate(self.npc_spawn_proc, 'NPC spawner')
        self.npc_spawn_proc = None
        self._log(f'[ui] spawning {n} NPC vehicles on port {port}')
        self.npc_spawn_proc = self._run_carla_snippet(
            _TRAFFIC_SPAWN_SNIPPET, [n, port], 'traffic')

    def clear_traffic(self):
        port = self.port_var.get()
        # Kill the heartbeat FIRST. If we destroyed the actors while it
        # was still running, the next 5-second tick would see them
        # already gone and re-spawn nothing, but during the destroy
        # window the heartbeat could race the clear and re-assert
        # autopilot on a half-destroyed actor. Killing first keeps the
        # teardown deterministic.
        self._terminate(self.npc_spawn_proc, 'NPC spawner')
        self.npc_spawn_proc = None
        self._log(f'[ui] clearing NPC traffic on port {port}')
        self._run_carla_snippet(_TRAFFIC_CLEAR_SNIPPET, [port], 'traffic')

    def list_spawns(self):
        port = self.port_var.get()
        self._log(f'[ui] listing spawn points on port {port}')
        self._run_carla_snippet(_LIST_SPAWNS_SNIPPET, [port], 'spawns')

    def stop_carla(self):
        # Always pkill — handles the case where CARLA was started outside the
        # UI and `self.carla_proc` is None.
        self._terminate(self.carla_proc, 'CARLA')
        self.carla_proc = None
        self._pkill_all_carla()
        self.status_var.set('CARLA stopped')

    # --------------------------------------------------------------------
    # Bridge (carlaAccSimTown.py)
    # --------------------------------------------------------------------
    def start_bridge(self):
        if self.bridge_proc and self.bridge_proc.poll() is None:
            self._log('[ui] Bridge already running')
            return
        policy = JUNCTION_POLICIES.get(
            self.junction_policy_var.get(), 'pp-takeover')
        # The Spinbox value comes through as a string — let argparse on the
        # bridge side parse it. If the user typed garbage, the bridge will
        # error with a clear message; no client-side validation needed.
        spawn_index = self.spawn_index_var.get().strip() or '0'
        cmd = [str(CARLA_PYTHON), str(BRIDGE_SCRIPT),
               '--junction-policy', policy,
               '--spawn-index', spawn_index]
        # Append --record only when the checkbox is ticked. Bridge defaults
        # to no recording (opt-in), so silence == no bag on disk.
        if self.rosbag_record_var.get():
            cmd.append('--record')
        self._log(f'$ (source ROS && cd {BRIDGE_DIR} && {" ".join(cmd)})')
        # BRIDGE_SYNC_MODE=1 was tried here to lock CARLA to real-time,
        # but at the 5 Hz UFLD load the bridge process pegs one core in
        # the tick loop, starves its own ROS callbacks, and _cmd_vel_cb
        # stops firing — CARLA then advances physics forever with the
        # last stored (zero-throttle) control, and the car sits still
        # even though ACC is commanding throttle = 1.0. Async mode
        # keeps callbacks responsive; if the sim runs above real-time
        # on this host, throttle back CARLA's own tick rate instead
        # (Start CARLA with `-benchmark -fps=20`) or bound perception
        # load elsewhere.
        self.bridge_proc = self._popen(
            cmd, cwd=str(BRIDGE_DIR),
            source_ros=True, prefix='bridge')
        rec_note = ' + rosbag' if self.rosbag_record_var.get() else ''
        self.status_var.set(
            f'Bridge starting (spawn={spawn_index}, junction: {policy}{rec_note})')

    def stop_bridge(self):
        self._terminate(self.bridge_proc, 'Bridge')
        self.bridge_proc = None

    # --------------------------------------------------------------------
    # Scenario harness (scenarios/r171_stationary_target.py)
    # --------------------------------------------------------------------
    def start_scenario_single(self):
        """One scenario point from the three spinners."""
        try:
            speed = float(self.scen_speed_var.get())
            offset = float(self.scen_offset_var.get())
            ttc = float(self.scen_ttc_var.get())
        except ValueError:
            self._log('[ui] scenario: speed / offset / TTC must be numbers')
            self.status_var.set('Scenario: invalid parameters')
            return
        self._start_scenario(
            SCENARIO_SCRIPT,
            ['--speed-kmh', str(speed),
             '--offset-m', str(offset),
             '--ttc-s', str(ttc),
             '--lateral-mode', self.scen_lateral_var.get()],
            f'{speed:g} km/h, offset {offset:g} m, TTC {ttc:g} s '
            f'(handover at {ttc * speed / 3.6:.0f} m)')

    def start_scenario_matrix(self, block=None):
        """Block A (18), Block B (12), or the full 30-point matrix."""
        extra = ['--matrix', '--lateral-mode', self.scen_lateral_var.get()]
        if block:
            extra += ['--block', block]
        label = f'Block {block}' if block else 'full 30-point matrix'
        self._start_scenario(SCENARIO_SCRIPT, extra, label)

    # -- curved road (R171 Annex 4 §4.2.5.2.2) ---------------------------
    @staticmethod
    def _curve_site_names() -> list[str]:
        """Sites biggest-radius first — that is the order they matter in,
        since radius sets both the severity and the speed ceiling."""
        return [name for name, _ in
                sorted(CURVE_SITES.items(), key=lambda kv: -kv[1].radius_m)]

    def _curve_site(self):
        return CURVE_SITES.get(self.curve_site_var.get())

    def _update_curve_note(self):
        """Show the selected site's geometry and what the chosen speed
        would demand of the lateral controller."""
        site = self._curve_site()
        if site is None:
            self.curve_note.config(
                text='site table unavailable — run curve_survey.py',
                foreground='#993333')
            return
        try:
            v = float(self.curve_speed_var.get()) / 3.6
        except ValueError:
            v = 0.0
        ay = v ** 2 / site.radius_m
        v_max = math.sqrt(CURVE_AY_CEILING * site.radius_m) * 3.6
        over = ay > CURVE_AY_CEILING
        self.curve_note.config(
            text=(f'{site.town}  R={site.radius_m:.0f} m {site.direction}, '
                  f'arc {site.arc_m:.0f} m, lead-in {site.lead_in_m:.0f} m\n'
                  f'demands {ay:.2f} m/s² · ceiling {CURVE_AY_CEILING:g} '
                  f'→ max {v_max:.0f} km/h\n'
                  f'lateral mode: from the panel above'),
            foreground='#993333' if over else '#666666')

    def start_curve_single(self):
        """One curved-road point. Refuses a speed the site cannot carry."""
        site = self._curve_site()
        if site is None:
            self._log('[ui] no curve site selected — is curve_adapter.py '
                      'importable?')
            return
        try:
            speed = float(self.curve_speed_var.get())
            offset = float(self.curve_offset_var.get())
            ttc = float(self.curve_ttc_var.get())
        except ValueError:
            self._log('[ui] curved scenario: speed / offset / TTC must be '
                      'numbers')
            self.status_var.set('Scenario: invalid parameters')
            return
        # Caught here rather than per-point inside the run: the script
        # refuses the same combination, and finding that out after a world
        # load costs a minute for nothing.
        ay = (speed / 3.6) ** 2 / site.radius_m
        if ay > CURVE_AY_CEILING:
            v_max = math.sqrt(CURVE_AY_CEILING * site.radius_m) * 3.6
            self._log(f'[ui] {speed:g} km/h on {site.name} (R='
                      f'{site.radius_m:.0f} m) demands {ay:.2f} m/s² of '
                      f'lateral acceleration, over the '
                      f'{CURVE_AY_CEILING:g} m/s² R171 §5.3.7.1.2 allows. '
                      f'Max for this site is {v_max:.0f} km/h.')
            self.status_var.set('Scenario: over the lateral limit')
            return
        self._start_scenario(
            SCENARIO_CURVE_SCRIPT,
            ['--site', site.name,
             '--speed-kmh', str(speed),
             '--offset-m', str(offset),
             '--ttc-s', str(ttc),
             '--lateral-mode', self.scen_lateral_var.get()],
            f'curve {site.name} (R={site.radius_m:.0f} m), {speed:g} km/h, '
            f'offset {offset:g} m, TTC {ttc:g} s')

    def start_curve_matrix(self):
        """The speed x offset x TTC matrix on the selected site only.

        Deliberately not the script's default two-site matrix: that one
        spans Town12, which costs minutes of world loading, and a UI
        button should do what the panel above it says.
        """
        site = self._curve_site()
        if site is None:
            self._log('[ui] no curve site selected')
            return
        self._start_scenario(
            SCENARIO_CURVE_SCRIPT,
            ['--matrix', '--matrix-sites', site.name,
             '--lateral-mode', self.scen_lateral_var.get()],
            f'curve matrix on {site.name} (R={site.radius_m:.0f} m)')

    # -- lane keeping (R79 Annex 8 §3.2) ---------------------------------
    def _update_lka_note(self):
        """Preview the speed R79 derives for this site and test.

        The speed is not a free parameter: it is whatever puts the lateral
        demand in the paragraph's window. Showing it here means an
        out-of-band combination is visible before a world is loaded, not
        after the run comes back `invalid_window`.
        """
        site = CURVE_SITES.get(self.lka_site_var.get())
        if site is None or lka_speed_for is None:
            self.lka_note.config(
                text='site table unavailable — the run will still start',
                foreground='#993333')
            return
        try:
            bands = lka_parse_declaration(self.lka_ay_var.get(), False)
        except SystemExit as exc:
            self.lka_note.config(text=f'declaration rejected: {exc}',
                                 foreground='#993333')
            return
        except Exception:
            self.lka_note.config(text='declaration must look like '
                                      '"60:1.5,100:3.0,130:3.0"',
                                 foreground='#993333')
            return

        test = self.lka_test_var.get()
        hit = None
        for band in bands:
            v, ay = lka_speed_for(test, band.ay_max, site.radius_m)
            if band.contains(v):
                hit = (band, v, ay)
                break
        if hit is None:
            self.lka_note.config(
                text=(f'R={site.radius_m:.0f} m: no declared band whose own '
                      f'speed range contains the derived speed — this run '
                      f'would be invalid_window'),
                foreground='#993333')
            return
        band, v, ay = hit
        self.lka_note.config(
            text=(f'{site.town}  R={site.radius_m:.0f} m, arc '
                  f'{site.arc_m:.0f} m\n'
                  f'derived {v:.1f} km/h → {ay:.2f} m/s² '
                  f'(band {band.label}, aysmax {band.ay_max:g})'),
            foreground='#666666')

    def start_lka_single(self):
        site = CURVE_SITES.get(self.lka_site_var.get())
        name = site.name if site else self.lka_site_var.get()
        if not name:
            self._log('[ui] no curve site selected')
            return
        self._start_scenario(
            SCENARIO_LKA_SCRIPT,
            ['--site', name,
             '--test', self.lka_test_var.get(),
             '--declared-ay', self.lka_ay_var.get()],
            f'R79 {self.lka_test_var.get()} on {name}')

    def start_lka_sweep(self):
        """radius x speed over every sweep site, judged on kept_lane.

        Speeds and sites come from the script's own defaults rather than
        the panel: the panel picks ONE site, and a sweep that ran only
        that row would not be a sweep.
        """
        self._start_scenario(
            SCENARIO_LKA_SCRIPT,
            ['--sweep', '--declared-ay', self.lka_ay_var.get()],
            'R79 lane-keeping sweep (radius × speed)')

    def start_lka_matrix(self):
        """Every (site, band, test) whose derived speed is legal.

        Spans several maps, so it reloads the world between site groups —
        expect it to take a while, and expect Town12 sites to dominate
        that time.
        """
        self._start_scenario(
            SCENARIO_LKA_SCRIPT,
            ['--matrix', '--declared-ay', self.lka_ay_var.get()],
            'R79 lane-keeping matrix (all sites)')

    def _start_scenario(self, script, extra_args, label):
        if self.scenario_proc and self.scenario_proc.poll() is None:
            self._log('[ui] scenario already running — Stop scenario first')
            return
        # The harness IS the bridge for the duration of a run. Leaving
        # carlaAccSimTown.py up would put two writers on /Car_1/cmd_vel and
        # on the ego's apply_control; the harness refuses to start in that
        # case, so stop it here rather than surfacing a confusing error.
        if self.bridge_proc and self.bridge_proc.poll() is None:
            self._log('[ui] stopping Bridge — the scenario harness replaces '
                      'it for the duration of the run')
            self.stop_bridge()
            time.sleep(1.0)

        # --lateral-mode is added by the callers that have one: the R79
        # script owns steer unconditionally and does not accept the flag.
        cmd = [str(CARLA_PYTHON), str(script)] + list(extra_args)
        self._log(f'$ (source ROS && cd {SCENARIO_DIR} && {" ".join(cmd)})')
        # source_workspace=True: unlike the bridge, the harness imports the
        # workspace's message types for the topics it publishes.
        self.scenario_proc = self._popen(
            cmd, cwd=str(SCENARIO_DIR),
            source_ros=True, source_workspace=True, prefix='scenario')
        self.status_var.set(f'Scenario running — {label}')

    def stop_scenario(self):
        self._terminate(self.scenario_proc, 'Scenario')
        self.scenario_proc = None
        self.status_var.set('Scenario stopped')

    # --------------------------------------------------------------------
    # MORAI bridge (morai_bridge: state_adapter_node + control_adapter_node)
    # --------------------------------------------------------------------
    @staticmethod
    def _morai_bridge_pids() -> list[str]:
        """Real OS-process check for the adapter nodes, independent of
        this UI.py instance's own in-memory tracking. Needed because
        state_adapter_node/control_adapter_node can also be started by
        start_adas.sh (whenever SIMULATOR=morai) as a completely
        separate launch path, and _popen's start_new_session=True means
        they survive this UI.py process closing -- either case leaves
        self.morai_bridge_procs blind to processes that are genuinely
        still running. Matching just the node name (not the full
        'ros2 run morai_bridge <name>' invocation) so this also catches
        the underlying entry-point process, not only the 'ros2 run'
        wrapper. See DEBUG.md's duplicate-adapter-processes entry.
        """
        pids = []
        for pattern in ('state_adapter_node', 'control_adapter_node'):
            result = subprocess.run(['pgrep', '-f', pattern],
                                     capture_output=True, text=True)
            pids += [p for p in result.stdout.split() if p]
        return pids

    def start_morai_bridge(self):
        if any(p.poll() is None for p in self.morai_bridge_procs):
            self._log('[ui] MORAI bridge already running')
            return
        existing = self._morai_bridge_pids()
        if existing:
            self._log(f'[ui] MORAI bridge already running as OS process(es) '
                       f'{", ".join(existing)} (started outside this UI, e.g. '
                       f'by start_adas.sh, or left over from a previous UI '
                       f'session) — not starting a duplicate. Stop it first.')
            self.status_var.set('MORAI bridge already running elsewhere')
            return
        dry_run = self.dry_run_var.get()
        self._log('[ui] starting MORAI bridge (state + control adapters)'
                   + (' [DRY RUN -- no vehicle commands]' if dry_run else ''))
        control_cmd = ['ros2', 'run', 'morai_bridge', 'control_adapter_node']
        if dry_run:
            control_cmd += ['--ros-args', '-p', 'dry_run:=true']
        self.morai_bridge_procs = [
            self._popen(
                ['ros2', 'run', 'morai_bridge', 'state_adapter_node'],
                cwd=str(ADAS_WK), source_ros=True, source_workspace=True,
                prefix='morai-state'),
            self._popen(
                control_cmd,
                cwd=str(ADAS_WK), source_ros=True, source_workspace=True,
                prefix='morai-control'),
        ]
        self.status_var.set('MORAI bridge running (state + control adapters)')

    def stop_morai_bridge(self):
        for p in self.morai_bridge_procs:
            self._terminate(p, 'MORAI bridge')
        self.morai_bridge_procs = []
        # Also sweep for OS processes this instance isn't tracking (see
        # _morai_bridge_pids) so Stop is effective even against
        # duplicates/orphans from another launch path or a previous UI
        # session, not just this instance's own children.
        leftover = self._morai_bridge_pids()
        if leftover:
            self._log(f'[ui] clearing {len(leftover)} untracked MORAI bridge '
                       f'process(es): {", ".join(leftover)}')
            self._pkill(['state_adapter_node', 'control_adapter_node'])
        self.status_var.set('MORAI bridge stopped')
        self.status_var.set('Bridge stopped')

    # --------------------------------------------------------------------
    # Foxglove bridge — ws://localhost:8765 for the Foxglove Studio app
    # --------------------------------------------------------------------
    def start_foxglove(self):
        if self.foxglove_proc and self.foxglove_proc.poll() is None:
            self._log('[ui] Foxglove bridge already running')
            return
        cmd = ['ros2', 'launch', 'foxglove_bridge',
               'foxglove_bridge_launch.xml']
        self._log(f'$ (source ROS && {" ".join(cmd)})')
        self.foxglove_proc = self._popen(cmd, source_ros=True,
                                          prefix='foxglove')
        self.status_var.set('Foxglove bridge starting on ws://localhost:8765')

    def stop_foxglove(self):
        self._terminate(self.foxglove_proc, 'Foxglove bridge')
        self.foxglove_proc = None
        self.status_var.set('Foxglove bridge stopped')

    # --------------------------------------------------------------------
    # start_adas.sh — launches all four ADAS nodes
    # --------------------------------------------------------------------
    def run_start_adas(self):
        if self.stack_proc and self.stack_proc.poll() is None:
            self._log('[ui] start_adas.sh already running')
            return
        cmd = ['./start_adas.sh', self.simulator_var.get()]
        if self.dry_run_var.get():
            cmd.append('dry_run')
        self._log(f'[ui] ### LAUNCHING WITH SIMULATOR = {self.simulator_var.get().upper()} ### '
                   '(check the dropdown if this is wrong)')
        if self.dry_run_var.get():
            self._log('[ui] DRY RUN -- ACC/LKAS will compute normally but '
                       'no commands will be sent to the vehicle')
        self._log(f'$ (cd {ADAS_WK} && {" ".join(cmd)})')
        # start_adas.sh sources ROS itself.
        self.stack_proc = self._popen(cmd, cwd=str(ADAS_WK),
                                       source_ros=False, prefix='adas')
        # start_adas.sh spawns lane_detection_node + stanley_node (and
        # perception_node + controller_node) with ZERO ros params, so
        # lane_detection_node loads its default model filename
        # (UFLD_best.pth, which doesn't exist) and perception_node
        # loads its default YOLO checkpoint (best.pt) -- the UI's Lane
        # model / Object model dropdowns and KF PSDs are all ignored.
        # Kill the shell-spawned pairs after a short delay and respawn
        # via the shared helpers so the UI selections actually take
        # effect.
        self.root.after(1500, self._restart_lkas_with_ui_params)
        self.root.after(1500, self._restart_acc_with_ui_params)
        self.acc_on = True
        self.lkas_on = True
        self._refresh_toggle_labels()
        self.status_var.set('ADAS stack running (start_adas.sh)')

    def _restart_lkas_with_ui_params(self):
        """Called ~1.5 s after start_adas.sh: replace the shell-spawned
        LKAS pair (which used defaults) with UI-parameterised ones."""
        self._pkill(['lane_detection_node', 'stanley_node'])
        # Give the OS a beat to actually reap them so we don't race
        # our own spawn against a duplicate ros2 run.
        self.root.after(400, self._start_lkas_procs)

    def _restart_acc_with_ui_params(self):
        """Called ~1.5 s after start_adas.sh: replace the shell-spawned
        ACC pair (which used defaults) with UI-parameterised ones."""
        self._pkill(['perception_node', 'controller_node'])
        self.acc_procs.clear()
        # Give the OS a beat to actually reap them so we don't race
        # our own spawn against a duplicate ros2 run.
        self.root.after(400, self._start_acc_procs)

    def stop_stack(self):
        self._terminate(self.stack_proc, 'start_adas.sh')
        self.stack_proc = None
        # start_adas.sh's children were spawned via `ros2 run` and don't share
        # our process group; sweep them up explicitly.
        self._pkill(['perception_node', 'controller_node',
                     'lane_detection_node', 'stanley_node'])
        for p in self.acc_procs + self.lkas_procs:
            self._terminate(p, 'subnode')
        self.acc_procs.clear()
        self.lkas_procs.clear()
        self.acc_on = False
        self.lkas_on = False
        self._refresh_toggle_labels()
        self.status_var.set('ADAS stack stopped')

    # --------------------------------------------------------------------
    # ACC / LKAS toggles
    # --------------------------------------------------------------------
    def _refresh_toggle_labels(self):
        self.acc_btn.configure(text=f'ACC: {"ON" if self.acc_on else "OFF"}')
        self.lkas_btn.configure(text=f'LKAS: {"ON" if self.lkas_on else "OFF"}')

    def toggle_acc(self):
        if self.acc_on:
            self._pkill(['perception_node', 'controller_node'])
            for p in self.acc_procs:
                self._terminate(p, 'acc-sub')
            self.acc_procs.clear()
            self.acc_on = False
            self._log('[ui] ACC OFF')
        else:
            self._start_acc_procs()
            self.acc_on = True
            self._log('[ui] ACC ON')
        self._refresh_toggle_labels()

    def _start_acc_procs(self):
        """Spawn perception_node + controller_node with the UI-selected
        YOLO checkpoint. Shared between toggle_acc (button click) and
        run_start_adas (Start ADAS), so the Start button honours the
        Object model dropdown without needing an OFF/ON cycle to apply
        it. Does NOT toggle self.acc_on -- the caller owns that state
        and its label refresh."""
        simulator = self.simulator_var.get()
        self._log(f'[ui] ### LAUNCHING WITH SIMULATOR = {simulator.upper()} ### '
                   '(check the dropdown if this is wrong)')
        # Look up the selected YOLO checkpoint in OBJECT_MODELS and
        # pass it to perception_node as -p model_filename:=<ref>.
        perc_cmd = ['ros2', 'run', 'perception', 'perception_node',
                    '--ros-args', '-p', f'simulator:={simulator}']
        model_ref = next((ref for name, ref in OBJECT_MODELS
                          if name == self.object_model_var.get()), None)
        if model_ref:
            perc_cmd += ['-p', f'model_filename:={model_ref}']
            self._log(f'[ui] ACC model: {self.object_model_var.get()}')
        self.acc_procs.append(self._popen(
            perc_cmd, cwd=str(ADAS_WK),
            source_ros=True, source_workspace=True,
            prefix='acc-perc'))
        self.acc_procs.append(self._popen(
            ['ros2', 'run', 'controller', 'controller_node',
             '--ros-args', '-p', f'simulator:={simulator}'],
            cwd=str(ADAS_WK), source_ros=True, source_workspace=True,
            prefix='acc-ctrl'))

    def toggle_lkas(self):
        if self.lkas_on:
            self._pkill(['lane_detection_node', 'stanley_node'])
            for p in self.lkas_procs:
                self._terminate(p, 'lkas-sub')
            self.lkas_procs.clear()
            self.lkas_on = False
            self._log('[ui] LKAS OFF')
        else:
            self._start_lkas_procs()
            self.lkas_on = True
            self._log('[ui] LKAS ON')
        self._refresh_toggle_labels()

    def _start_lkas_procs(self):
        """Spawn lane_detection_node + stanley_node with the UI-selected
        model, Kalman toggle, and KF process-noise PSDs. Shared between
        toggle_lkas (button click) and run_start_adas (Start ADAS), so
        the Start button honours UI selections without needing an
        OFF/ON cycle to apply them. Does NOT toggle self.lkas_on —
        the caller owns that state and its label refresh."""
        simulator = self.simulator_var.get()
        self._log(f'[ui] ### LAUNCHING WITH SIMULATOR = {simulator.upper()} ### '
                   '(check the dropdown if this is wrong)')
        perc_cmd = ['ros2', 'run', 'perception', 'lane_detection_node']
        # Build ONE list of ros params, then attach a single
        # `--ros-args` block. `ros2 run` silently drops params when
        # `--ros-args` appears more than once without a `--` fence,
        # which is why splitting them per-group caused the node to
        # fall back to its default model filename and crash.
        ros_params: list[str] = ['-p', f'simulator:={simulator}']
        model_ref = next((ref for name, ref in LANE_MODELS
                          if name == self.lane_model_var.get()), None)
        if model_ref:
            ros_params += ['-p', f'model_filename:={model_ref}']
            self._log(f'[ui] LKAS model: {self.lane_model_var.get()}')
        if not self.kalman_enable_var.get():
            ros_params += ['-p', 'enable_kalman:=false']
            self._log('[ui] LKAS Kalman: OFF')
        else:
            self._log('[ui] LKAS Kalman: ON')
            for pname, var in (('kf_q_a', self.kf_qa_var),
                               ('kf_q_b', self.kf_qb_var),
                               ('kf_q_c', self.kf_qc_var)):
                raw = var.get().strip()
                if not raw:
                    continue
                try:
                    val = float(raw)
                except ValueError:
                    self._log(f'[ui] LKAS bad {pname}={raw!r}, using node default')
                    continue
                ros_params += ['-p', f'{pname}:={val}']
            self._log(f'[ui] LKAS q=({self.kf_qa_var.get()}, '
                      f'{self.kf_qb_var.get()}, {self.kf_qc_var.get()})')
        if ros_params:
            perc_cmd += ['--ros-args'] + ros_params
        self.lkas_procs.append(self._popen(
            perc_cmd, cwd=str(ADAS_WK),
            source_ros=True, source_workspace=True,
            prefix='lkas-perc'))
        self.lkas_procs.append(self._popen(
            ['ros2', 'run', 'controller', 'stanley_node',
             '--ros-args', '-p', f'simulator:={simulator}'],
            cwd=str(ADAS_WK), source_ros=True, source_workspace=True,
            prefix='lkas-ctrl'))

    def _apply_kf_params(self):
        """Push the current KF entry-box values to a running
        lane_detection_node via `ros2 param set`. Perception must
        have a SetParametersCallback that rebuilds Q on change."""
        pairs = (('kf_q_a', self.kf_qa_var),
                 ('kf_q_b', self.kf_qb_var),
                 ('kf_q_c', self.kf_qc_var))
        for pname, var in pairs:
            raw = var.get().strip()
            if not raw:
                continue
            try:
                val = float(raw)
            except ValueError:
                self._log(f'[ui] bad {pname}={raw!r}, skipped')
                continue
            self._popen(
                ['ros2', 'param', 'set',
                 '/lane_detection_node', pname, str(val)],
                cwd=str(ADAS_WK),
                source_ros=True, source_workspace=True,
                prefix='kf-set')
        self._log(f'[ui] KF applied q=({self.kf_qa_var.get()}, '
                  f'{self.kf_qb_var.get()}, {self.kf_qc_var.get()})')


def main():
    root = tk.Tk()
    # Leave headroom around the 960×540 camera for the left controls + log.
    # Widened from 1320 → 1720 to fit the permanent BEV panel (BEV_W=360
    # + LabelFrame padding) to the right of the camera. Camera column
    # keeps its previous size — the new width is added, not redistributed.
    root.geometry('1720x820')
    app = ADASUI(root)
    # The right column now stacks three scenario panels under the BEV, so
    # a fixed height clips the bottom one. Ask Tk what the layout needs
    # and grant it, bounded by the screen — 820 stays the floor so a small
    # display still gets the previous window.
    root.update_idletasks()
    _sw, _sh = root.winfo_screenwidth(), root.winfo_screenheight()
    root.geometry(f'{min(max(1720, root.winfo_reqwidth()), _sw - 60)}'
                  f'x{min(max(820, root.winfo_reqheight()), _sh - 100)}')

    def on_close():
        # Tear down what we own, in reverse-start order. Flush any in-flight
        # video recording first so the .mp4 has a valid trailer.
        app._stop_recording()
        # Kill the NPC heartbeat spawner before anything else — it's
        # the only one we created with start_new_session=True that has
        # no other lifecycle hook, so if we forget it it'll keep
        # running headless after the UI closes.
        app._terminate(app.npc_spawn_proc, 'NPC spawner')
        app.npc_spawn_proc = None
        app.stop_stack()
        # Before the bridge: a running scenario owns the ego's control and
        # holds actors it destroys on its own shutdown path.
        app.stop_scenario()
        app.stop_bridge()
        app.stop_carla()
        app.stop_foxglove()
        if app.ros_node is not None:
            try:
                app.ros_node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


if __name__ == '__main__':
    main()

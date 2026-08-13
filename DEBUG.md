# DEBUG / Known Issues

Working notes on open bugs, regressions, behavioural questions, and
design decisions for the ADAS stack. Clean usage docs live in
[README.md](README.md); this file is the engineering record — what
broke, why, and what we changed.

Items marked **[FIXED]** have an applied patch; the description is
kept so we remember what was wrong and why we changed it. **[DONE]**
is a deliberate change (feature add, refactor). **[DECIDED]** is a
non-code architectural call. **[KNOWN]** is an open limitation with
mitigation. **[PLANNED]** is scoped but not started.

> **Before every run after pulling code changes, rebuild:**
> ```
> cd ROS_ADAS_Stack
> colcon build --packages-select perception controller
> source install/setup.bash
> ```
> The most common silent failure mode is forgetting this and getting
> `No executable found` for `debug_image_fusion_node`, or, more
> insidiously, running stale stanley / controller binaries whose log
> output doesn't match the code in `src/` (e.g. HOLD printing
> `steer=+0.000` instead of `steer=+nan`).

---

## Reading guide

Two views over the same content:

1. **Chronological dev log** — sections §1 onward, in the order
   issues were encountered and resolved. Captures the *iteration
   history*: a fix in §11 is later refined by §12, the junction
   handling in §5 is superseded by §9 and §15, etc. Use this view
   to understand "how we got here".
2. **Thematic index** — below. Groups entries by subsystem.
   Useful for thesis writing or anyone reading the document for
   the first time.

Every entry follows an evolving template that maps cleanly onto a
thesis Objective / Methods / Results structure:

| Section in entry            | Thesis correspondence            |
|-----------------------------|----------------------------------|
| Symptom / Background        | **Objective** — the problem      |
| Root cause                  | **Analysis** — what we found     |
| Applied fix / changes       | **Methods** — what we did        |
| Caveats / follow-ups        | **Results / Discussion**         |

---

## Thematic index

### Chapter 1 — System infrastructure & bridge
- [§31](#31-duplicate-morai-bridge-adapter-processes--real-os-process-guard-added-fixed) Duplicate MORAI bridge adapter processes — real OS-process guard added
- [§27](#27-second-new-machine-move--cuda-13-default-wheel--colconsetuptoolspackaging-conflict-fixed) Second new-machine move — CUDA-13 default wheel + colcon/setuptools/packaging conflict
- [§10](#10-bridge-hard-coded-for-one-scenario--now-fully-argparse-driven-fixed) Bridge — argparse-driven scenario configuration
- [§4](#4-carla-graphics-flicker--ego-bonnet--npc-lods-unstable-partial--sync-mode-regressed-motion-now-opt-in) CARLA graphics flicker — sync vs. async investigation
- [§12](#12-ego-still-stalls-at-full-acc-throttle--bridge-jpeg-encoder-starves-the-ros-executor-at-1920×1080-fixed-with-caveat) JPEG encoder starves the ROS executor at 1920×1080
- [§17](#17-synchronous-mode-ui-control-removed-done) Synchronous-mode UI control removed
- [§14](#14-bonnet-flicker-is-worse-in-town10hd-than-town03-known-mitigation-only) Bonnet flicker worse in Town10HD than Town03
- [§33](#33-carla-side-hardening-session-2026-07-21-done) CARLA-side hardening session — inference-rate sweep, UI camera QoS, sync-mode regression, cruise setpoint, model-ready race, speed-topic mismatch

### Chapter 2 — Perception
- [§2](#2-no-on-screen-indication-that-acc--lkas-are-running-fixed) On-screen indication that ACC / LKAS are running
- [§6](#6-uncontrolled-acceleration--acc-ignores-closing-leads-ego-rear-ends-them-fixed) ACC distance-filter and gap tuning
- [§7](#7-combined-yolo--ufld-topic-does-not-run-fixed--build-dependency--load-tuning) Combined YOLO + UFLD topic — build + load tuning
- [§8](#8-combined-yolo--ufld-view-looks-like-two-overlapping-video-sequences-fixed) Combined view fusion — timestamp-matched overlays
- [§16](#16-acc-lane-roi-via-ufld-vehicle-frame-ipm-done) ACC lane ROI via UFLD vehicle-frame IPM
- [§20](#20-junction-lane-mapping--approaches-and-trade-offs-planned) Junction-lane mapping — approaches and trade-offs [PLANNED]
- [§21](#21-adas-stack-near-cpu-capacity--ufld-diagnosis--rate-limit-fixed-with-planned-follow-up) ADAS stack near CPU capacity — UFLD diagnosis & rate limit
- [§22](#22-lead-distance-pinhole--ipm-and-semantics--bumper-to-bumper-gap-fixed) Lead distance: pinhole → IPM and semantics → bumper-to-bumper gap
- [§23](#23-anchor-based-loop-route-for-lead--pp-fallback-done) Anchor-based loop route for lead + PP fallback
- [§34](#34-morai-specific-yolo-fine-tune-dataset-pipeline--labelimg-pyqt5-crashes-done) MORAI-specific YOLO fine-tune: dataset pipeline + labelImg PyQt5 crashes
- [§35](#35-best_moraipt-showed-no-bounding-box-in-the-live-ui--three-unrelated-causes-not-the-model-fixed--mixed) `best_MORAI.pt` showed no bounding box — three unrelated causes, not the model
- [§40](#40-lane-kf-stuck-in-rejection-lockup--χ²-gate-had-no-escape-path-fixed) Lane KF stuck-in-rejection lockup — χ² gate had no escape path

### Chapter 3 — Control
- [§30](#30-morai-brake-leaves-the-vehicle-stuck--cruise-mode-coasts-instead-of-braking-on-morai-fixed) MORAI brake leaves the vehicle stuck — cruise mode coasts instead of braking
- [§28](#28-startup-safety-gate--throttle-held-at-0-until-yolo--ufld-are-loaded-done) Startup safety gate — throttle held at 0 until YOLO + UFLD are loaded
- [§1](#1-npc-traffic-does-not-follow-the-road--drives-straight-and-crashes-fixed) NPC traffic via TrafficManager autopilot
- [§3](#3-fallback-behaviour-when-acc-or-lkas-is-off-decided) Fallback behaviour when ACC or LKAS is off
- [§11](#11-ego-stalls-at-full-acc-throttle--pp--bridge-race-on-apply_control-fixed) PP / bridge `apply_control` race
- [§15](#15-ufld-inference-paused-in-junction--stanleypp-cmd_steer-race-fixed) Stanley / PP cmd_steer race + UFLD pause

### Chapter 4 — Junction policy (evolution over time)
- [§5](#5-junction-policy--ufld-lane-drops-out-stanley-says-hold-car-still-steered-fixed-with-caveat) Stanley HOLD heuristic (v1)
- [§5b](#5b-junction-policy-did-not-visibly-engage-in-testing--diagnostic-logging-added) Diagnostic logging
- [§9](#9-junction-policy--map-based-supersedes-5--5b-fixed) Map-based junction policy (v2 — current)
- [§13](#13-junction-policy-is-now-a-ui-choice-pure-pursuit--hold-straight-done) UI control: Pure-pursuit vs. Hold-straight
- §15 (cross-listed under Chapter 3 — completes the handoff)

### Chapter 5 — Tooling & visualisation
- [§18](#18-foxglove-studio-integration-done) Foxglove Studio integration
- [§19](#19-ipm-birds-eye-view--ipm_view_node-done) IPM bird's-eye view (`ipm_view_node`)

### Chapter 6 — MORAI simulator integration
- [§26](#26-morai-simulator-integration-ongoing) Full port to a second simulator: environment setup, MORAI's ROS2 Interface, the `morai_bridge` adapter package, camera/control calibration, and open issues
- [§27](#27-second-new-machine-move--cuda-13-default-wheel--colconsetuptoolspackaging-conflict-fixed) Second new-machine move (dependency setup gotchas)
- [§29](#29-morais-imu-never-populates-linear-velocity--switched-speed-source-to-groundtruth-vehicleinfo-fixed) MORAI's IMU never populates linear velocity — switched to GroundTruth VehicleInfo
- [§32](#32-morais-groundtruth-vehicleinfo-local_velocity-is-frozen-not-live--29s-caveat-confirmed-known-unresolved) MORAI's GroundTruth VehicleInfo local_velocity is frozen, not live
- [§36](#36-new-machine-move-pc-acm-02--uipy-crash-on-missing-rclpy-real-degraded-mode-bug-found-fixed) New-machine move (`PC-ACM-02`) — `UI.py` crash on missing `rclpy`
- [§37](#37-morai-groundtruth-vehicleinfo-topic-renamed-to-egovehicleinfo--reconfirms-32s-frozen-value-bug-independently-fixed--confirmed) MORAI GroundTruth VehicleInfo topic renamed to `/Ego/vehicleinfo` — reconfirms §32
- [§38](#38-ipm_view_nodepy-had-no-morai-camera-extrinsics-awareness-fixed) `ipm_view_node.py` had no MORAI camera-extrinsics awareness
- [§39](#39-dry-run-switch--validate-acclkas-while-driving-manually-since-morais-gt-sensor-is-known-broken-done) Dry-run switch — validate ACC/LKAS while driving manually
- [§41](#41-morai-camera-publish-rate-investigation--paused-unresolved-known-unresolved) MORAI camera publish-rate investigation — paused, unresolved

---

## 1. NPC traffic does not follow the road — drives straight and crashes [FIXED]

**Symptom.** NPC vehicles spawned by the CARLA bridge
([carlaaccsim/carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py)) drive
straight from their spawn point, ignore lane geometry, and crash into the
first wall or curb. No routing, no lane keeping, no intersection handling.

**Root cause (suspected).** The bridge spawns the lead vehicle with
`world.try_spawn_actor(...)` but does **not** hand it to CARLA's
TrafficManager. Movement is driven by `run_pure_escape(lead_vehicle,
lead_route, ...)`, which walks a precomputed `lead_route` of waypoints at a
2 m step (carlaAccSimTown.py:79–86). If that thread isn't started, isn't
ticking, or runs out of route, the actor receives no control and physics
carries it straight until impact.

**Reference (working pattern).** `lkas_validate_0.9.16.py:346–393` in
[00_Lane_Assistant/02_UFLD_V2](../../00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py)
uses CARLA's TrafficManager:

```python
tm = client.get_trafficmanager()
tm.set_synchronous_mode(True)
tm_port = tm.get_port()
...
actor = world.try_spawn_actor(bp, sp)
actor.set_autopilot(True, tm_port)
```

TM owns routing, lane-following, traffic-light response, and collision
avoidance for every actor registered to it. NPCs spawned this way "just
work" without a hand-rolled route.

**Applied fix.**
- `carlaAccSimTown.py` now hands the lead vehicle to TrafficManager:
  `tm = client.get_trafficmanager(); lead.set_autopilot(True, tm.get_port())`.
- `lead_route` and the `run_pure_escape` thread were removed — TM owns
  routing now. The scripted route stays available in
  `pure_pursuit_controller.run_pure_escape` for the AEBS scenarios.
- Lead is capped at 60 % of the lane speed limit
  (`tm.vehicle_percentage_speed_difference(lead, 40.0)`) so the ego ACC can
  catch up and engage.
- The UI's "Spawn Traffic" button already used TM autopilot; no change
  needed there.

---

## 2. No on-screen indication that ACC / LKAS are running [FIXED]

**Symptom.** The UI camera feed shows the raw bridge image only. There is no
YOLO bounding box on the lead vehicle, no UFLD lane overlay, and no
indicator showing whether ACC or LKAS is currently controlling the car.

**Current state of the code.**
- `perception_node.py:131` already draws YOLO bounding boxes and publishes
  the annotated image on `/ACC/perception/debug_image`.
- `lane_detection_node.py:233` (`annotate(...)`) already draws the UFLD
  ego-left / ego-right polylines and publishes on
  `/LKAS/perception/debug_image`.
- [UI.py](UI.py) only subscribes to `/Car_1/camera/front/compressed`
  (the raw bridge feed). The debug topics are never displayed.

So the visualisations exist — the UI just isn't wired to show them.

**Applied fix.**
- UI.py replaced the single-topic `CameraView` with `TelemetryView`, which
  subscribes to the three image topics + `/Car_1/cmd_vel` +
  `/Car_1/cmd_steer`. JPEGs are stashed raw; only the active source gets
  decoded each render tick.
- The camera widget now has a "Source" combobox: **Raw / ACC (YOLO) /
  LKAS (UFLD)**. Switching is instant — the same camera subscription set
  is always live, the renderer just picks which JPEG to decode.
- Two status labels next to the ACC and LKAS feature buttons read from the
  heartbeat timestamps (1.5 s window):
  - ACC: `● active` when both `/ACC/perception/debug_image` and
    `/Car_1/cmd_vel` are publishing; `◐ partial` when only one is;
    `○ idle` when neither.
  - LKAS: same logic against `/LKAS/perception/debug_image` and
    `/Car_1/cmd_steer`.

---

## 3. Fallback behaviour when ACC or LKAS is off [DECIDED]

### 3a. ACC off → car coasts (current behaviour, kept) [DECIDED]

Killing `controller_node` (what the UI's "ACC: OFF" does today) stops
`/Car_1/cmd_vel` publication. The bridge holds its last-seen throttle/brake
(0/0 on startup), so the car coasts — drag and friction bleed off speed.
No constant-speed cruise.

The only speed target in the system is
[controller_node.py:65](src/controller/controller/controller_node.py#L65)
(`self.target_speed = 20 / 3.6  # m/s`, i.e. 20 km/h), and it only applies
while `controller_node` is running.

**Decision.** Keep the coast behaviour. It's honest about what "ACC off"
means, requires no extra code, and matches the user's mental model. If we
ever want a real-time ACC enable/disable without process restart, the
follow-up work is to add a parameter or topic on `controller_node` and
gate output on it — out of scope for now.

### 3b. LKAS off → ego steers via pure-pursuit fallback [FIXED]

The bridge previously held the last `/Car_1/cmd_steer` value (0 on
startup), so ACC-only mode could not stay in lane on any curve.

**Applied fix.** Re-enabled the pure-pursuit controller that already
existed in [carlaaccsim/pure_pursuit_controller.py](../../carlaaccsim/pure_pursuit_controller.py)
as the steer fallback. The bridge now:
- Precomputes an `ego_route` (~800 m of forward waypoints in the ego's
  starting lane) in [carlaaccsim/carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py).
- Runs `run_pure_pursuit(hero, ego_route, world, should_apply=...)` in a
  background thread, always on.
- The `should_apply=lambda: not avt_node.is_steer_fresh()` gate is the key:
  `CarlaAVT.is_steer_fresh()` returns True for `STEER_FRESH_WINDOW_S` (0.5 s)
  after the last `/Car_1/cmd_steer` message. While LKAS is publishing the
  fallback yields each tick; the moment LKAS goes silent the fallback owns
  steer and the ego stays in lane.

Longitudinal control is unchanged — pure pursuit's `_ego_speed_policy`
reads back the current `VehicleControl.throttle/brake` (set by ACC's
`/Car_1/cmd_vel`) and replays it, so the ACC controller still owns speed.

**Caveats / follow-ups.**
- `ego_route` walks `wp.next(2.0)[0]` (first successor only). It will not
  initiate lane changes; if the ego drifts to a parallel lane the
  precomputed route still aims it back to the original lane.
- The route is finite (~800 m). When the ego runs past the end the
  fallback aims at the last waypoint, which is fine for short demos but
  not for long autonomous runs. A streaming route refresher is the obvious
  next step.

---

## 4. CARLA graphics flicker — ego bonnet + NPC LODs unstable [PARTIAL — sync mode regressed motion, now opt-in]

**Symptom.** When running the full ROS stack (CARLA + bridge +
perception/controller nodes + UI), the ego car's bonnet flickers between
two states each frame, and NPC vehicles' high-LOD geometry pops in and
out. The non-ROS validator
[00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py](../../00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py)
does not have this problem against the same CARLA install.

**Likely cause (working hypothesis).** The two setups differ in how they
drive CARLA's actor / render pipeline:

- `lkas_validate_0.9.16.py` runs as a single Python process, holds the
  spectator on the ego, and ticks the world directly (sync or async mode
  decided once at startup). One client, one tick source, stable LOD
  selection per frame.
- The ROS stack has at least three concurrent CARLA clients:
  `carlaAccSimTown.py` (bridge), the TrafficManager (lead vehicle
  autopilot — same TM port but separate client session for any extra UI
  NPCs), and incidental clients started by the UI snippets
  (weather/traffic). The bridge currently calls `world.wait_for_tick()` —
  no `world.apply_settings(...)` — so the world is in **async mode**, and
  CARLA's LOD picker sees a non-deterministic frame cadence from each
  client. The bonnet "two-state flicker" is the classic symptom of
  competing client commits between Unreal frames.

**Applied fix.** Mirrored `lkas_validate_0.9.16.py`'s synchronous setup
in [carlaaccsim/carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py):

1. Snapshot `original_settings = world.get_settings()` at startup.
2. Do all spawns (ego, camera, lead) and TM autopilot binding in
   async mode — same order as the validator, since spawn-then-sync is
   the configuration that's known to work.
3. Just before the main loop, flip to sync 20 Hz:
   `settings.synchronous_mode = True;
   settings.fixed_delta_seconds = 0.05; world.apply_settings(settings)`.
4. Set `tm.set_synchronous_mode(True)` so TrafficManager ticks in step
   with the world; otherwise NPC autopilots freeze when we tick.
5. Replace `world.wait_for_tick()` with `world.tick()` in the loop so
   the bridge owns the cadence.
6. Restore `original_settings` (and TM async) in `finally:` so a later
   `lkas_validate` run isn't stuck with our sync configuration.

This also fixes the secondary worry from issue #3 in this list — frame
content was unstable across ticks, which can only have hurt YOLO + UFLD.

**Caveats.**
- All CARLA Python clients touching this world (the bridge, UI snippets,
  TrafficManager) now share one tick source. The UI snippets are
  short-lived and that's still fine, but if anything else opens a
  long-lived client it must avoid `world.tick()` (only one client may
  drive ticks in sync mode).

**Regression observed in field test.** With sync mode forced on, the ego
stalled at 0 m/s — Stanley still entered STANLEY mode with a real lane
error (`e_lat=-0.95 m`), but `vehicle.get_velocity()` stayed at 0 and no
forward motion happened. The exact failure mode (sensor settling? sub-tick
apply_control queueing? interaction with the PP-thread `time.sleep`?) is
not yet diagnosed.

**Mitigation (applied).** Sync mode is now gated behind the
`BRIDGE_SYNC_MODE` env var. Default OFF restores the original
`world.wait_for_tick()` async behaviour; the bridge prints
`sync_mode = False` on startup so it's obvious which path is live. Set
`BRIDGE_SYNC_MODE=1` to opt back into sync mode for flicker debugging.
Stays open until we have a sync setup that keeps the ego moving.

Also added `flush=True` to the bridge's startup `print()` calls — the
prior run produced zero `[bridge]` lines in the UI log because Python
buffers stdout when stdout isn't a TTY (the UI's `subprocess.PIPE`
qualifies). Without those prints flushing, the bridge looked dead even
when it was running.

---

## 5. Junction policy — UFLD lane drops out, Stanley says HOLD, car still steered [FIXED, with caveat]

**Symptom.** Inside a junction the log line shows `[   HOLD]` (Stanley
gave up on UFLD lanes), but the car still steers left or right rather
than holding straight. We had no clean turn behaviour through
intersections.

**What HOLD actually meant in the old code.** In
[stanley_node.py](src/controller/controller/stanley_node.py):

```python
if lookahead is None:
    steer = 0.0
    mode = 'HOLD'
…
self.steer_pub.publish(out)   # always published, even in HOLD
```

So Stanley *did* publish `steer=0.0` every HOLD tick, the bridge applied
it, and the car *should* have driven straight. The "it still steered"
symptom was the bridge's prior `_cmd_steer` value carrying over for a
few frames while Stanley's publication rate caught up, *or* an
intermittent bridge-side stale-state retention — not a Stanley bug per
se, but Stanley's "0.0 is my answer" reply was actively suppressing the
new pure-pursuit fallback (which always sees `is_steer_fresh()` =
True while Stanley is publishing zeros).

**Applied fix.** Stanley no longer publishes during HOLD. The bridge's
`is_steer_fresh()` then goes False after `STEER_FRESH_WINDOW_S`
(0.5 s) and the pure-pursuit fallback (see [3b](#3b-lkas-off--ego-steers-via-pure-pursuit-fallback-fixed))
owns steer through the junction. Stanley resumes the moment UFLD
recovers a lane centre on the far side and the fallback yields back.

This matches the user's proposal: *use pure pursuit at junctions, then
hand back to UFLD on the exit.*

**Caveats / follow-ups.**
- Pure pursuit follows the precomputed `ego_route` from the ego's
  starting lane. At a junction it takes the *first* successor
  (`wp.next(2.0)[0]`), so the chosen turn direction is fixed at
  bridge startup. To pick a turn dynamically per junction we'd need a
  route refresher / decision policy.
- The `[HOLD]` log line now prints `steer=+nan` to make it obvious in
  the log that Stanley deliberately yielded rather than published zero.
- During HOLD Stanley still emits an INFO log every second so the
  operator sees that the fallback is engaged, not that Stanley crashed.

---

## 5b. Junction policy did not visibly engage in testing — diagnostic logging added

The earlier fix in §5 (Stanley stops publishing during HOLD →
pure-pursuit fallback drives the junction) was reported as not working in
the field. Two changes to make the failure mode actually diagnosable on
the next run:

**1. Shorter handoff window.** `STEER_FRESH_WINDOW_S` lowered from 0.5 s
to 0.2 s in [carlaaccsim/custom_ROS_pub_sub.py](../../carlaaccsim/custom_ROS_pub_sub.py).
At 20 Hz Stanley, the bridge now lets PP take over after ~4 missed
publishes instead of ~10. Previously it was plausible the ego had
already traversed enough of a small junction in 0.5 s for the late
handoff to do nothing visible.

**2. Edge-triggered logs on both ends.**
- Stanley now WARN-logs `HOLD — no lane centre at lookahead=… m` on
  every HOLD entry (with `left_pts`/`right_pts` counts so we can tell
  whether UFLD is dropping the polylines entirely or just losing the
  lookahead row), and INFO-logs `STANLEY re-engaged` on exit.
- `pure_pursuit_controller._run_controller` prints
  `[pure_pursuit] ENGAGED (LKAS cmd_steer stale → owning steer)` or
  `[pure_pursuit] YIELDING (LKAS cmd_steer fresh → letting LKAS drive)`
  on every transition.

If after the next run the bridge stdout shows no `[pure_pursuit]`
transitions and Stanley shows no `HOLD` warning at a junction, then
UFLD is not actually dropping the lanes inside junctions — the real
problem is upstream and we need to look at lane_detection_node /
UFLD's confidence floor instead of the Stanley → PP handoff.

---

## 6. Uncontrolled acceleration — ACC ignores closing leads, ego rear-ends them [FIXED]

**Symptom.** With ACC engaged and a lead vehicle clearly visible in the
YOLO debug view (bounding box + distance label drawn each frame), the
ego accelerates as if the road were clear and bumps into the lead at
cruise speed. No EMERGENCY-brake intervention either.

**Root cause.** [controller_node.py:79](src/controller/controller/controller_node.py#L79)
ran the distance filter at `ALPHA = 0.01`:

```python
self.d_lead_filtered = self.ALPHA * d + (1 - self.ALPHA) * self.d_lead_filtered
```

That's a ~100-sample memory. At 5–10 Hz (perception_node's YOLO
inference is slower than the camera publish rate), the filter takes
double-digit seconds to track a real change in distance. Result: when
a lead enters the scene at 30 m and the ego starts closing, the filter
shows the original 30 m for many seconds; the controller computes
`distance_error = d_filtered − d_desired = +large`, requests full
positive acceleration, and the throttle rate limit (1 s ramp to full)
doesn't save us either. EMERGENCY mode only fires below 3 m of *filtered*
distance — by which time we're already through the actual 3 m gap.

**Applied fix.** Raised `ALPHA` to `0.4`. That's a ~2.5-sample memory
(~250–500 ms at typical YOLO Hz), still smoothing single-frame
bounding-box jitter but actually tracking when the lead closes.

**Follow-ups (not done).**
- The pinhole distance estimate in
  [perception_node.py:124](src/perception/perception/perception_node.py#L124)
  uses fixed `OBJECT_HEIGHTS` per class and the YOLO bounding-box
  height. Both numbers are wrong for partially-occluded boxes
  (e.g. when the lead's roof is clipped by the top of the frame at
  close range), and the result is a *systematic over-estimate* of
  distance at close range. A more robust estimate would use the
  bounding-box bottom plus a ground-plane projection (same trick UFLD
  uses for lanes). Today's fix narrows the worst case but doesn't
  eliminate it.

**Tuning follow-up (applied).** Field test showed ACC braking the
moment YOLO acquired a lead. With `d0 = 5 m` and `T_gap = 1.5 s`, the
desired gap formula `d_desired = d0 + T_gap * v_ego` evaluates to
13.4 m at 20 km/h cruise — so any detection inside ~13 m fed a
negative `distance_error` into the PD loop and ACC braked. The
formula is correct (matches the conventional time-headway model);
the values were just too cautious for the demo. Dropped `T_gap` to
**0.5 s**, which gives `d_desired ≈ 7.8 m` at cruise and settles to
`d0 = 5 m` at standstill — matching the user-expected "follow at
roughly 5 m" behaviour. `d0` itself was correct and stays at 5 m.
- `MIN_CONFIDENCE = 0.1` is permissive; spurious low-confidence detections
  on roadside objects could still feed garbage distances into the filter.
  Raise once we know what classes the model produces at confidence
  > 0.3 in our maps.

---

## 7. Combined YOLO + UFLD topic does not run [FIXED — build dependency + load tuning]

**Symptom.** Selecting the new `ADAS (YOLO+UFLD)` source in the UI
shows the placeholder ("waiting for first camera frame…") — the
`/ADAS/perception/debug_image` topic appears to have no publisher.

**Root cause.** Two plausible factors, neither fatal once addressed:

1. **`colcon build` not re-run.** `debug_image_fusion_node` is a *new*
   entry point in `src/perception/setup.py`. Without a fresh build the
   installed `perception` package doesn't know about it, and
   `ros2 run perception debug_image_fusion_node` (invoked from
   `start_acc.sh`) fails immediately with `executable not found`. The
   `start_acc.sh` background runner doesn't surface that failure
   prominently, so the symptom is "topic silently absent".
2. **CPU competition.** Even when launched, the node decodes 2 JPEGs
   and encodes 1 each tick. Running alongside YOLO + UFLD on a single
   GPU/CPU box, the original 12 Hz / quality 85 settings made it the
   most expensive non-inference node in the stack.

**Applied fix.**
- Node lowered to `PUB_HZ = 8` and `JPEG_QUALITY = 75`. Plenty for a UI
  preview; cheap enough not to compete with perception.
- Added a startup INFO log ("Debug-image fusion node started …") so it's
  visible in the start_acc.sh stdout that the entry point loaded.
- Added a 5-second WARN heartbeat that names the source topic(s) that
  haven't produced a frame yet (`/ACC/perception/debug_image`,
  `/LKAS/perception/debug_image`). Tells the operator immediately when
  the fusion node is alive but starved of inputs — i.e. one of the
  upstream perception nodes didn't launch — vs. when fusion itself is
  the missing piece.
- Added a one-shot INFO log on the first successful publish so success
  is visible too.

**Operational note.** After pulling these changes, run
`colcon build --packages-select perception && source install/setup.bash`
before relaunching `start_acc.sh`. The other entry points
(`perception_node`, `lane_detection_node`) survive without a rebuild;
only the new one needs it.

---

## 8. Combined YOLO + UFLD view looks like two overlapping video sequences [FIXED]

**Symptom.** With both ACC and LKAS running, the new combined source in
the UI (`/ADAS/perception/debug_image`) showed a clearly double-exposed
background — as though two video tracks were laid on top of each other.
Felt "unsmoothing and laggy" in real time.

**Root cause.** The first implementation just did
`cv2.max(acc_debug, lkas_debug)`. That's fine *for the bright overlay
pixels* (YOLO greens, UFLD circles), which dominate the max. But the
two debug images came from *different camera frames* — YOLO inference
runs at ~5–10 Hz and UFLD at ~10–15 Hz, so the two perception nodes
publish from camera frames that differ by 50–200 ms. The road,
horizon, and buildings in the BACKGROUND of the two debug images
disagree by exactly that camera-motion offset, and per-pixel max picks
the brighter of the two non-overlay scenes at every pixel — producing
the double-exposed look.

**Applied fix.** Rewrote
[debug_image_fusion_node.py](src/perception/perception/debug_image_fusion_node.py)
to do timestamp-matched mask extraction:

1. Subscribe to the raw camera (`/Car_1/camera/front/compressed`) as a
   third input. Keep a 30-frame ring buffer keyed by `header.stamp`.
2. When an ACC or LKAS debug image arrives, look up the raw frame it
   was computed from (matched by `header.stamp` — both perception
   nodes preserve the original camera header). Compute the overlay
   pixels as
   `mask = max(|debug_bgr − raw_bgr|, axis=-1) > OVERLAY_THRESHOLD`.
3. On every publish tick, paint the latest cached masks onto the
   LATEST raw frame.

So the background is always the most-recent raw frame (smooth, no
ghosting), and the overlays sit at the pixel positions the perception
nodes drew them at. Some lag is unavoidable for the overlays — at
higher speeds the YOLO box may trail the actual lead by a frame or two
— but that's a much milder artifact than the dual-frame ghosting.

`OVERLAY_THRESHOLD = 25` was tuned to reject the JPEG re-encoding noise
between the bridge's q=95 raw and the perception nodes' q=85 debug
outputs while still catching the overlay colours. `MATCH_TOLERANCE_NS =
150 ms` rejects timestamp-matches when the raw buffer hasn't caught up
yet (the perception debug is then skipped that cycle rather than
fused against a stale background).

**Caveats / follow-ups.**
- A "real" fix would have the perception nodes publish their raw
  detection data on separate topics (bounding boxes for ACC, pixel
  polylines for LKAS) and the fusion node would draw fresh onto the
  latest raw. That eliminates the overlay lag entirely. The current
  fix avoids the API changes by recovering the overlay from a pixel
  diff — cheap, but at the cost of small overlay lag.

---

## 9. Junction policy — map-based, supersedes §5 / §5b [FIXED]

§5 and §5b above are the previous "Stanley yields on HOLD → fallback
engages after 0.2 s" approach. It works when UFLD actually loses the
lane inside a junction, but in practice UFLD often keeps producing
*something* (curb lines, crosswalk markings) and Stanley stays in
STANLEY mode through the intersection, steering against whatever
garbage it sees.

**Applied fix.** Lifted the CARLA-map-based junction detection from
`00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py:junction_steer`
and wired it into the bridge:

1. `carlaAccSimTown.py` runs a `junction_monitor` thread at 10 Hz that
   queries `world.get_map().get_waypoint(ego.location)` and decides
   whether the ego is in (or approaching) a junction zone, using the
   same `JUNCTION_ENTRY_LOOKAHEAD_M = 2.0` /
   `JUNCTION_EXIT_LOOKAHEAD_M = 6.0` thresholds as lkas_validate.
2. When in-junction, the monitor calls `avt_node.set_in_junction(True)`.
3. `custom_ROS_pub_sub.CarlaAVT.is_steer_fresh()` now returns False
   *unconditionally* while `_in_junction` is True — Stanley keeps
   publishing, but its authority is revoked.
4. The existing pure-pursuit fallback (via `should_apply=lambda: not
   avt_node.is_steer_fresh()`) takes steer through the intersection
   along `ego_route`. UFLD and Stanley continue to run; their cmd_steer
   is just discarded for the junction window.
5. `build_ego_route()` now picks the lane successor whose yaw is
   closest to the current heading at each fork — i.e. "drive straight
   through" by default — instead of the old `wp.next(2.0)[0]`
   arbitrary-first-successor walker.

This is the user's literal ask: *"use junction detection, switch off
UFLD, use pure_pursuit.py as long as in junction"*. UFLD isn't
literally killed — but its output is ignored — which is the same
operational effect and avoids the complexity of stopping/starting a
heavyweight inference node mid-run.

The bridge prints `[junction] ENTER` / `EXIT` events to its stdout, so
the operator can see the handoffs in the UI log.

Switchable: `carlaAccSimTown.py --junction-policy none` reverts to the
§5/5b heuristic (Stanley yields only when it actually enters HOLD).

**Follow-ups (not done).**
- Heading-aligned route picks "straight through" — to take a specific
  turn at a specific junction we'd need a route planner on top of
  `build_ego_route`, or a higher-level routing client.
- The 2 m entry / 6 m exit constants are taken straight from
  lkas_validate; if junctions in Town01 feel premature/late we can
  expose those as flags too.

---

## 10. Bridge hard-coded for one scenario — now fully argparse-driven [FIXED]

**Symptom.** Town, weather, vehicle blueprint, traffic count, camera
resolution, spawn index were all module constants in
`carlaaccsim/carlaAccSimTown.py`. The non-ROS validator
`lkas_validate_0.9.16.py` had been argparse-driven from the start and
ran cross-town / cross-weather smoothly; the bridge couldn't.

**Applied fix.** Mirrored lkas_validate's argparse interface in the
bridge. New flags (see `carlaaccsim/README.md` for the full table):

- `--port`, `--town`, `--vehicle`, `--weather`, `--traffic`
- `--lead-speed-pct`, `--lead-gap-m`
- `--spawn-index`, `--list-spawns`
- `--cam-width`, `--cam-height`, `--cam-fov`, `--cam-tick`
- `--ego-route-len`
- `--junction-policy {pp-takeover,none}`

Defaults:
- Camera bumped from **1280×720 → 1920×1080** to match the validator.
  This roughly doubles the pixel count of a far-car bounding box and
  improves YOLO distance accuracy at range, which contributed to the
  Town03 early-brake behaviour we were seeing (§6 follow-up).
- All other defaults preserve the previous hard-coded behaviour, so
  running the bridge with no flags is the closest behavioural match to
  the old script.

**Related downstream fix.**
`src/perception/perception/perception_node.py` was hard-coded for
1280-wide imagery (`FOCAL_LENGTH = 640.0`). The distance formula now
computes the focal length per frame from the actual image width and a
ROS parameter `camera_fov_deg` (default 90), so distances stay
correct at 1080p or any other resolution the bridge ships.

**Follow-up (not done).**
- The UI's "Start Bridge" button launches the bridge with no flags.
  Threading the new flags through the UI (town selector → `--town`,
  weather → `--weather`, traffic count → `--traffic`, junction
  toggle → `--junction-policy`) is the next step.

---

## 11. Ego stalls at full ACC throttle — PP / bridge race on `apply_control` [FIXED]

**Symptom.** With everything launched from the UI, `ros2 topic` showed
ACC commanding `/Car_1/cmd_vel.linear.x = 1.0` (full throttle) at a
steady 20 Hz, `/Car_1/vehicle/speed` publishing fine, no spin-thread
exception in the bridge — but the ego sat at 0 m/s. New behaviour
appeared after §9's map-based junction policy went live, but the root
cause was an older latent bug exposed by it.

**Root cause.** The pure-pursuit fallback's speed policy in
[carlaaccsim/pure_pursuit_controller.py](../../carlaaccsim/pure_pursuit_controller.py)
was reading throttle/brake back from CARLA, then re-applying them
alongside its computed steer:

```python
def _ego_speed_policy(vehicle, speed, min_index):
    ctrl = vehicle.get_control()    # ← effective control from PREVIOUS tick
    return ctrl.throttle, ctrl.brake
```

`carla.Actor.get_control()` returns the effective control from the
last physics step, not the latest queued `apply_control` call. Between
ticks the bridge would receive ACC's `cmd_vel(throttle=1.0)` and
queue a fresh `apply_control(throttle=1.0)`; PP would then tick a
millisecond later, read `get_control().throttle = 0` (the still-effective
prior-tick value), and queue `apply_control(throttle=0)`. The later
queue entry wins per actor → CARLA applied throttle=0 for the next
physics step. Car never moved.

This race had always existed, but PP only engages when
`is_steer_fresh()` is False — previously that meant just "LKAS isn't
publishing", which was rare during normal demos. §9's junction monitor
now flips `_in_junction=True` whenever the ego is in a junction zone,
so PP started engaging at startup (Town03's `spawn_points[0]` happens
to sit in a junction). PP then ran constantly and the throttle never
got through.

**Applied fix.** PP now reads throttle/brake from the *bridge's
authoritative state* (the values most recently received on
`/Car_1/cmd_vel`) instead of from `vehicle.get_control()`. Both threads
agree on the same value, so even though PP queues its own
`apply_control` after the bridge's, the throttle written is identical
to what ACC commanded — the bridge's value survives. PP only owns
steer in practice.

The plumbing:
- `custom_ROS_pub_sub.CarlaAVT.current_throttle_brake() → (throttle, brake)`
  returns the most recent `cmd_vel` values.
- `pure_pursuit_controller.run_pure_pursuit(..., throttle_brake_provider=callable)`
  accepts an optional provider; when supplied, the speed policy calls
  it instead of `get_control()`. `_ego_speed_policy` was refactored
  into `_make_ego_speed_policy(throttle_brake_provider)`.
- `carlaAccSimTown.py` passes
  `throttle_brake_provider=avt_node.current_throttle_brake` when
  starting the PP thread.

`run_pure_escape` is unaffected — the lead vehicle has its own escape
speed policy, no ROS handoff involved.

**Caveat / follow-up.** This fix assumes the bridge's ROS callbacks
(spin thread) are alive. If the spin thread dies for any reason,
`current_throttle_brake()` returns the last value it saw before the
crash — possibly stale 0. The earlier spin-exception hunt (still
intermittent in heavy-traffic Town03 runs) remains an open thread
above this in the stack.

---

## 12. Ego still stalls at full ACC throttle — bridge JPEG encoder starves the ROS executor at 1920×1080 [FIXED, with caveat]

**Symptom.** After §11's fix went in we hit the same end-user symptom
again: ACC publishing `/Car_1/cmd_vel.linear.x = 1.0` at a steady 20 Hz,
Stanley publishing fresh `cmd_steer`, but the ego sat at 0 m/s. Visual
camera feed showed no motion. Two diagnostic signatures gave the cause
away:

- `ros2 topic hz /Car_1/vehicle/speed` and `…/ACC/lead_vehicle_distance`
  **timed out** even though `ros2 topic info` showed the bridge as the
  publisher.
- The bridge process was sitting at **117 % CPU** with no ADAS load.

`/Car_1/cmd_vel` was alive and steady (ACC was healthy), but none of the
bridge's *own* topics — speed, distance, the camera — were actually
making it out to the wire. The bridge's spin thread was running but
its callbacks weren't getting CPU.

**Root cause.** `custom_ROS_pub_sub.CarlaAVT` uses a single-threaded
ROS executor (`rclpy.executors.SingleThreadedExecutor`). Every
subscription callback and every timer callback runs on the same
thread. With the §10 camera bump to **1920 × 1080 at JPEG quality 95**,
`_publish_camera` became expensive enough (~30–50 ms per frame; the
inner `while not self.image_queue.empty()` drains any backlog in one
call) that it monopolised the executor. `_cmd_vel_cb` then fired
late or not at all, so the bridge's `self._throttle` stayed at its
initial `0.0`. The pure-pursuit fallback's speed policy read that
stale 0 via `current_throttle_brake()` (§11) and dutifully applied
`apply_control(throttle=0, …)` to CARLA at 20 Hz. Car never moved.

The reason §11's fix held under the validator but broke under the ROS
stack is exactly the same load asymmetry: the validator runs as one
process at low quality presets, the ROS stack runs the bridge alongside
two GPU-heavy perception nodes that steal cycles and make the encoder
fall further behind.

**Applied fix.** Lowered the bridge's default camera resolution from
1920×1080 to **1280×720** in
[carlaaccsim/carlaAccSimTown.py:113-118](../../carlaaccsim/carlaAccSimTown.py#L113-L118).
Encoding cost drops ~2.25× and the single-threaded executor regains
enough headroom that `_cmd_vel_cb` fires on every message. The ego
now moves the moment ACC commands throttle, confirmed end-to-end.

**Caveats / follow-ups.**
- The real fix is a multi-threaded executor in the bridge (or moving
  JPEG encoding off the executor thread), so subscription callbacks
  can run in parallel with image publishing. The resolution drop just
  raises the load ceiling — at full traffic + perception load 1280p
  may still creep up on it.
- 1280p loses pixel area for YOLO at range. §6's follow-up about
  pinhole distance accuracy at long range gets slightly worse.
  Acceptable for now; the alternative was a non-moving car.
- The bridge can still be run at 1920×1080 explicitly with
  `--cam-width 1920 --cam-height 1080` — useful for offline dataset
  capture where ACC throttle isn't in the loop.

---

## 13. Junction policy is now a UI choice (Pure pursuit / Hold straight) [DONE]

**Background.** §9 added CARLA-map junction detection with one
behaviour: inside a junction zone, the bridge sets `_in_junction =
True`, `is_steer_fresh()` returns False, and the pure-pursuit fallback
follows the precomputed `ego_route` through the intersection. That's
fine when the ego is following a well-formed route, but for an
X-junction where "drive straight through" is the right answer, holding
`steer = 0` is simpler, needs no route, and avoids PP's wheel-overshoot
on tight corners. The non-ROS validator at
`00_Lane_Assistant/02_UFLD_V2/lkas_validate_0.9.16.py:320` already
supported both via `--policy {hold-straight, map-follow, pure-pursuit,
none}`, exposed in the validator's UI by a Combobox.

**Applied changes.**

- **Bridge.** `--junction-policy` choices extended from
  `{none, pp-takeover}` to `{none, pp-takeover, hold-straight}` in
  [carlaaccsim/carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py).
  The junction monitor now passes the policy through to
  `CarlaAVT.set_in_junction(in_junc, policy=…)`.
- **CarlaAVT.** New `_junction_policy` field in
  [carlaaccsim/custom_ROS_pub_sub.py](../../carlaaccsim/custom_ROS_pub_sub.py).
  `is_steer_fresh()` returns True inside a junction under
  `hold-straight` (so the PP fallback yields), False under
  `pp-takeover` (so PP engages). `_apply_control()` clamps `steer = 0`
  when in-junction under `hold-straight` — that overrides whatever
  Stanley most recently published.
- **UI.** New "Junction policy" combobox in
  [UI.py](UI.py) Processes section with the two labels
  *Pure pursuit* (maps to `pp-takeover`) and
  *Hold straight* (maps to `hold-straight`). The chosen policy is
  passed to the bridge as `--junction-policy <value>` at Start Bridge.
  Switching mid-run has no effect — restart the bridge to apply.

**Why no UI option for `none`.** The user's only ask was the two
operational policies. `none` (LKAS keeps steer authority through
junctions) is available on the command line for debugging but isn't
useful in normal driving — it's the exact behaviour §5/§9 fixed by
*not* letting LKAS steer against curbs and crosswalk markings.

**Caveats / follow-ups.**
- Like the sync-mode checkbox, the policy choice is baked into the
  bridge subprocess at launch. A future improvement is a runtime ROS
  parameter on `CarlaAVT` so the operator can hot-swap policies; the
  scaffolding (`set_in_junction(policy=…)`) is in place for it.
- `hold-straight` is genuinely "go straight" — at a T-junction where
  the road bends, the car will drive into the kerb. Pick `pp-takeover`
  in maps with frequent T-junctions; `hold-straight` shines on X-grid
  towns (Town01/Town03 cores).

---

## 14. Bonnet flicker is worse in Town10HD than Town03 [KNOWN, mitigation only]

Operator-confirmed during demo: the §4 bonnet flicker visibly worsens
when switching from Town03 to Town10HD. Same code, same bridge config,
same camera resolution — only the map changes.

**Why.** Town10HD is the high-density urban map and has roughly 2–3×
the environment-object count of Town03. The flicker is Unreal's LOD
picker resolving inconsistent state across multi-client commits
between frames (see §4); the more meshes near the camera, the more
likely the bad picks land on geometry that's visually prominent (and
the bonnet is *right* in front of the camera). Town10HD also takes
longer per frame on the same GPU, which widens the window during
which the bridge / TrafficManager / UI snippets can race on
`apply_*` calls — more race window means more inconsistent frames.

**Mitigations (no fix, sync mode would have addressed it but
regressed motion per §4).**
- For demos that care about visual quality, default to **Town03**.
- For Town10HD specifically: set **Quality: Low** in the UI dropdown
  before Start CARLA. Low quality removes a lot of LOD tiers so
  there's less to flicker between, and shortens frame time.
- Combine Low quality with the 1280×720 bridge default (§12) for the
  most stable look.
- Avoid concurrent UI helper actions (Apply Weather, List spawns,
  Spawn Traffic) while recording or screenshotting — each adds a
  client that races on commits.

---

## 15. UFLD inference paused in junction + Stanley/PP cmd_steer race [FIXED]

**Symptom.** With the §9/§13 junction policy active and
`pp-takeover` selected, the operator could see the pure-pursuit
fallback "kick" steer about once a second instead of the smooth 20 Hz
control it produces outside junctions. UFLD continued running on
junction frames (curbs, crosswalk markings) and Stanley kept
publishing `/Car_1/cmd_steer` against that garbage, which felt wrong
even though §9's `is_steer_fresh()` was supposed to override LKAS.

**Two root causes**, both in the bridge's
[carlaaccsim/custom_ROS_pub_sub.py](../../carlaaccsim/custom_ROS_pub_sub.py):

1. **Bridge / PP `apply_control` race.** `_cmd_steer_cb` calls
   `_apply_control()` on every Stanley publish (~20 Hz), writing
   `(throttle, brake, Stanley_steer)` to CARLA. The pure-pursuit
   thread runs its own 20 Hz loop and writes
   `(throttle, brake, PP_steer)`. Whichever call lands closest to
   the next physics tick wins. Roughly half the ticks Stanley's
   stale steer beat PP's fresh value — that's the "1 Hz" feel.
2. **Lane data was still flowing from junction frames.** UFLD ran on
   every camera frame, including inside the junction box. Stanley
   stayed in `STANLEY` mode (not HOLD) and kept publishing
   `cmd_steer` against curbs/crosswalk lines — feeding the race
   above with bogus values.

**Applied fix.**

- **Bridge.** `_apply_control()` early-returns when
  `_in_junction and _junction_policy == 'pp-takeover'`. PP owns
  apply_control exclusively in that window — it already writes
  throttle/brake (via `current_throttle_brake()` per §11) and steer.
  The Stanley write is dropped on the floor for the duration.
- **Bridge.** New publisher `/Car_1/in_junction`
  (`std_msgs/Bool`, depth 1) — published on every `set_in_junction`
  call so downstream subscribers see ENTER/EXIT immediately.
- **lane_detection_node.** Subscribes to `/Car_1/in_junction`. While
  True, skips UFLD inference entirely; emits empty Paths on
  `/LKAS/ego_lane_left` and `/LKAS/ego_lane_right` (Stanley reads
  empty Paths as HOLD and stops publishing `cmd_steer`); publishes
  the raw camera frame with a `JUNCTION (UFLD paused)` overlay on
  `/LKAS/perception/debug_image` so the operator sees the pause.

Net result: in a junction, PP is the only writer to
`vehicle.apply_control`, Stanley is silent (HOLD), UFLD doesn't burn
GPU cycles on frames it can't reason about, and the user-visible
steer trace is a clean 20 Hz curve instead of a 1 Hz step.

**Caveats / follow-ups.**
- The Stanley HOLD message stops firing in the log while UFLD is
  paused — Stanley simply sees no Paths. If you want a positive
  "Stanley paused for junction" log signal, it has to come from
  Stanley reacting to the same `/Car_1/in_junction` topic.

---

## 16. ACC lane ROI via UFLD vehicle-frame IPM [DONE]

**Background.** Pre-existing ACC perception filtered YOLO detections
to "within 20 % of image-centre horizontal", which is a crude
substitute for "is this car in my lane". Cars in the adjacent lane
near the image centre passed; cars in the ego lane at the periphery
of the image (curving road) failed. With UFLD already producing the
ego lane polylines, we can do better.

**First attempt (replaced).** lane_detection published image-space
polylines as `Float32MultiArray` on
`/LKAS/ego_lane_{left,right}_px`; perception built a closed polygon
and called `cv2.pointPolygonTest` on each detection's
bottom-centre. Worked, but introduced a second coordinate frame
(image space) for the same lane data Stanley was already consuming
in vehicle frame.

**Applied design.** Use the IPM that lane_detection already uses for
Stanley. perception subscribes to `/LKAS/ego_lane_left` and
`/LKAS/ego_lane_right` (`nav_msgs/Path`, vehicle frame, REP 103) and
runs the same `ipm_pixel_to_vehicle` to ground-project each
detection's bottom-centre into the road plane. A detection is
in-lane iff its `Y_left` lies between the interpolated left and
right lane Y at its `X_forward`. Same projection model, same
coordinate frame, two consumers — no duplicated topics.

- Files:
  [src/perception/perception/perception_node.py](src/perception/perception/perception_node.py),
  [src/perception/perception/lane_detection_node.py](src/perception/perception/lane_detection_node.py).
- Camera extrinsics added as ROS parameters
  `cam_height_m` (1.35) and `cam_x_offset` (0.6), matching the
  bridge rig and lane_detection_node's existing defaults.
- The image-space `_px` topics were removed — clean redesign.
- Fallback: if either lane Path is empty (UFLD warm-up, junction
  pause, or detection X beyond the polyline range), perception
  falls back to the legacy centre-strip filter so ACC isn't blind.
- The lane polygon was previously drawn on the ACC debug image; the
  user asked for it to be removed so the ACC view stays focused on
  YOLO boxes — the lane is already shown on the LKAS source.

**Caveats / follow-ups.**
- Both nodes hard-code the 1.35 m / 0.6 m camera rig. If the bridge
  ever ships a different mount, both nodes' ROS parameters need to
  be updated together. A bridge-published `/camera_info`-style
  topic would centralise this.
- IPM assumes a flat ground plane. On steep grade or speed bumps,
  the projected (X, Y) gets noisy; we're not seeing this in CARLA
  but real-world deployment would want a tilt-corrected variant.

---

## 17. Synchronous-mode UI control removed [DONE]

§4's sync-mode opt-in was exposed in the UI as a checkbox. After
§12's resolution fix landed and the ego stopped stalling in async
mode, the operator never enabled sync mode in practice — flicker is
tolerated in exchange for guaranteed motion. To reduce surface area
and confusion, the UI control was retired:

- Removed the `Bridge: synchronous mode` Checkbutton and its
  `bridge_sync_var` BooleanVar from [UI.py](UI.py).
- Removed the `BRIDGE_SYNC_MODE=1` env-var injection in
  `start_bridge`.

The bridge still honours `BRIDGE_SYNC_MODE=1` if set on the command
line — kept as an escape hatch for flicker-only investigations. The
UI just doesn't surface it any more. §4's post-mortem stays as-is;
the feature isn't deleted, just hidden behind a CLI knob.

---

## 18. Foxglove Studio integration [DONE]

The team uses Foxglove Studio for live telemetry visualisation.
`foxglove_bridge` is a separate ROS 2 node that opens
`ws://localhost:8765` for the Studio app to connect to — it's not
auto-started by anything in the stack, so the operator had to
remember to launch it manually each session and the saved layout
would silently fail to connect.

**Applied changes.**

- Added **Start Foxglove** / **Stop Foxglove** buttons to the UI's
  Processes group ([UI.py](UI.py)). Behind the scenes:
  `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`, streamed
  into the UI log with the `[foxglove]` prefix.
- The Foxglove process is independent of CARLA / Bridge / ADAS — it
  can be started before any of them and stays up for rosbag playback
  after the stack is torn down. Window-close also tears it down.

**Recommended starter layout.**
- 3-series Plot panel for cmd_vel.linear.x (throttle),
  cmd_vel.linear.y (brake), cmd_steer.data (steer). Put throttle on
  a separate Y-axis from velocity or it'll be crushed at the bottom
  of a shared 0–5 m/s scale.
- Indicator panel on `/Car_1/in_junction` — big colored block that
  lights up on PP / hold-straight handoff.
- Log panel filtered on `/rosout` — replaces grepping the UI log
  for HOLD warnings, junction ENTER/EXIT prints, etc.
- Image panels on `/ACC/perception/debug_image` and
  `/LKAS/perception/debug_image` (and `/ADAS/perception/debug_image`
  for the fused view if launched).

**Caveats / follow-ups.**
- Stanley logs `e_lat`, `e_head`, and STANLEY/HOLD mode as text
  only — none of those are publishable today, so they can't be
  plotted in Foxglove. Adding `/LKAS/stanley/e_lat`,
  `/LKAS/stanley/e_head` (Float32) and `/LKAS/stanley/mode` (String
  or enum) is ~5 lines in `stanley_node.py` and would give full
  closed-loop lateral diagnostics from Foxglove.

---

## 19. IPM bird's-eye view — `ipm_view_node` [DONE]

**Background.** The LKAS perception-debug image shows the lanes in
*pixel space* of the forward camera: convenient for visual cross-
check against the road, but hard to read distances off ("is that
lane 30 m or 10 m away?"). We needed a top-down, metric view of the
same lane geometry — both as a sanity check on the IPM the
controllers consume, and as a foundation for future work that needs
ground-plane reasoning (junction-lane mapping, lead detection in BEV,
etc).

**v1 — blank canvas with lane dots.** First pass at
[src/perception/perception/ipm_view_node.py](src/perception/perception/ipm_view_node.py)
just drew the `/LKAS/ego_lane_left` / `_right` polylines on a black
canvas using `veh_to_bev` (the inverse of UFLD's IPM). Useful for
showing *what UFLD believes the lanes are*, but couldn't show whether
those beliefs matched the actual road — "straight UFLD line on
curving road" and "straight UFLD line on straight road" looked the
same.

**v2 — warped camera + lane overlay [current].** Same node now
warps the live `/Car_1/camera/front/compressed` frame to the BEV
canvas using a fixed homography, then draws the UFLD polylines on
top of the warped asphalt.

- The homography is computed once per camera resolution from four
  ground control points — a trapezoid at 5 m / 25 m forward × ±3 m
  laterally — projected forward into the image with CARLA's pinhole
  + the canonical extrinsics (camera height 1.35 m, x-offset 0.6 m,
  FOV 90°). Same numbers as `lane_detection_node`'s IPM so the warp
  and the polylines share a coordinate frame.
- Re-computed automatically if the camera resolution changes (the
  bridge can run at 720p or 1080p; §12).
- Published on `/ADAS/ipm/debug_image` at 10 Hz so a Foxglove Image
  panel can show it alongside the LKAS / ACC views.
- The warp dims the road texture multiplicatively so the
  blue/green UFLD overlays read clearly against the asphalt.

**Why this is useful.**

1. **Camera-calibration sanity check.** On straight road the warped
   lane paint runs vertically (parallel to the image columns). If it
   diverges with distance, the camera height / x-offset / FOV
   constants are wrong — and the same constants drive ACC's lane-ROI
   filter and Stanley's lateral error. Easier to spot it here than
   to back it out of controller behaviour.
2. **UFLD honesty check.** If UFLD's blue/green dots don't track the
   *real* lane paint on the warped image, UFLD is hallucinating.
3. **Foundation for junction-lane mapping.** Same homography can
   project any vehicle-frame polyline — including `carla.Map`
   junction waypoints — onto the same canvas, making it cheap to
   visualise *every* drivable lane through a junction, not just the
   one UFLD currently follows.

**Caveats / follow-ups.**
- IPM beyond ~25 m is unreliable. The ground-plane assumption breaks
  on grades and the image-pixel quantisation amplifies (1 px maps
  to many metres of ground at the horizon). The far-row control
  point sits at 25 m for that reason; pushing it further would make
  the near texture look correct but the far texture nonsensically
  stretched.
- The node is now wired into `setup.py` (`ipm_view_node =
  perception.ipm_view_node:main`) and `start_acc.sh` (launched
  alongside the other perception nodes), so it comes up
  automatically with the regular ADAS launch and shows up on
  `/ADAS/ipm/debug_image` without any manual `python3` invocation.
- `cv2.warpPerspective` at 10 Hz is cheap on the CPU side
  (~ms-scale at the published 1280×720) and decouples cleanly from
  the GPU-bound perception nodes. No load issue.

### Methods note — interpolation kernel

`cv2.warpPerspective` resamples the camera image at non-integer
pixel positions dictated by the homography $H$. We use
`cv2.INTER_LINEAR`; `cv2.INTER_CUBIC` was tested and gave no visible
benefit on this view (the IPM is data-starved at the horizon, not
interpolation-starved). Both kernels share the machinery of FEM
shape functions on a uniform grid — useful framing for the thesis
methods chapter.

**1D linear interpolation.** Between samples $f_0,\ f_1$, with
$t = (x - x_0)/(x_1 - x_0)\in[0,1]$:

$$
f(x) = (1-t)\, f_0 + t\, f_1
$$

The weights $(1-t)$ and $t$ are the **1D P1 Lagrange shape
functions** $N_0,\ N_1$ — Kronecker-delta at the nodes
($N_i(\xi_j) = \delta_{ij}$) and partition-of-unity
($\sum N_i \equiv 1$). Same form as a 1D linear bar element.

**2D bilinear (`INTER_LINEAR` for images).** Tensor product of the
1D linear basis on a $2\times2$ source neighbourhood with corners
$p_{00}, p_{10}, p_{01}, p_{11}$ at unit-square corners and
$(u, v) \in [0,1]^2$:

$$
f(u, v) = p_{00}(1-u)(1-v) + p_{10}\,u(1-v) + p_{01}(1-u)v + p_{11}\,uv
$$

The four weights $\{(1-u)(1-v),\ u(1-v),\ (1-u)v,\ uv\}$ are
**exactly** the **Q1 (bilinear quadrilateral) FEM shape functions**
— this image-interpolation case is just the same element with the
"mesh" being the regular pixel grid.

**1D cubic (Keys / Catmull-Rom kernel).** Convolution against a
4-tap kernel, sampling neighbours $f_{i-1}, f_i, f_{i+1}, f_{i+2}$
around the floor $i = \lfloor x \rfloor$:

$$
f(x) = \sum_{k=-1}^{2} f_{i+k}\, W\bigl(x - (i+k)\bigr)
$$

with the Keys cubic kernel ($a = -\tfrac{1}{2}$ — OpenCV's default):

$$
W(s) = \begin{cases}
(a+2)|s|^3 - (a+3)|s|^2 + 1, & |s| \le 1\\[2pt]
a|s|^3 - 5a|s|^2 + 8a|s| - 4a, & 1 < |s| \le 2\\[2pt]
0, & \text{otherwise}
\end{cases}
$$

Equivalently, on the segment $t \in [0,1]$ between $f_0$ and $f_1$,
the cubic Hermite form is:

$$
f(t) = f_0 H_{00}(t) + f'_0 H_{10}(t) + f_1 H_{01}(t) + f'_1 H_{11}(t)
$$

with the cubic Hermite shape functions

$$
H_{00}(t) = 2t^3 - 3t^2 + 1,\quad H_{10}(t) = t^3 - 2t^2 + t
$$

$$
H_{01}(t) = -2t^3 + 3t^2,\quad H_{11}(t) = t^3 - t^2
$$

and slope estimates from central differences of neighbours

$$
f'_i \approx \frac{f_{i+1} - f_{i-1}}{2}.
$$

This is **identical to a 1D cubic Hermite FEM element**, with the
difference that the FEM element gets its nodal derivatives from the
analytical DOF list while image interpolation has to guess them
from the pixel grid. The Keys kernel above and the Hermite +
central-difference form are algebraically equivalent.

**2D bicubic (`INTER_CUBIC` for images).** Tensor product on a
$4\times4$ source neighbourhood:

$$
f(u, v) = \sum_{i=-1}^{2}\sum_{j=-1}^{2} f_{i,j}\, W(u-i)\, W(v-j)
$$

Sixteen weighted samples per output pixel, vs. four for bilinear —
about 3-4× the compute. Sharper near edges in the source; very
mild ringing artefacts ($C^1$ but the derivative isn't smooth, so
the kernel has a slight negative lobe).

**Continuity / FEM analogue summary.**

| Image kernel        | FEM analog                                      | Continuity              |
|---------------------|-------------------------------------------------|-------------------------|
| `INTER_NEAREST`     | P0 piecewise-constant                           | $C^{-1}$ (jumps)        |
| `INTER_LINEAR`      | P1 / Q1 Lagrange (linear / bilinear)            | $C^0$                   |
| `INTER_CUBIC`       | Cubic Hermite + central-difference derivatives  | $C^1$                   |
| `INTER_LANCZOS4`    | Truncated $\text{sinc}$ (spectral, non-local)   | $C^\infty$ in the limit |

**Decision for this IPM.** Kept `INTER_LINEAR`. Bicubic helped only
when the source had detail to preserve. The IPM's softness at the
top of the image is **data-starvation** at the horizon (a tiny
number of source pixels covering many BEV pixels), not an
interpolation choice — `INTER_CUBIC` doesn't conjure information
the camera didn't capture. Inside the 5-25 m IPM-trust zone the
visual difference between LINEAR and CUBIC at this output
resolution was negligible.

---

## 20. Junction-lane mapping — approaches and trade-offs [PLANNED]

**Background.** The current junction stack ([§5](#5-junction-policy--ufld-lane-drops-out-stanley-says-hold-car-still-steered-fixed-with-caveat) →
[§9](#9-junction-policy--map-based-supersedes-5--5b-fixed) →
[§13](#13-junction-policy-is-now-a-ui-choice-pure-pursuit--hold-straight-done) →
[§15](#15-ufld-inference-paused-in-junction--stanleypp-cmd_steer-race-fixed))
suppresses UFLD inside junction zones and either holds steer = 0 or
runs pure-pursuit along a *single* precomputed `ego_route`. This
works *operationally* — the ego crosses an X-junction without
straying — but it doesn't actually *map* the junction topology: we
can't see every possible exit, can't pick a turn dynamically from a
route plan, and can't verify post-junction that the ego ended up in
a legal exit lane.

To upgrade beyond a single hard-coded route we need a representation
of **every drivable lane through the junction** in the ego's vehicle
frame, refreshed online. Three families of approaches exist, in
roughly increasing order of effort and decreasing reliance on prior
information:

### Approach A — CARLA Map API (sim-only, ground truth)

CARLA exposes the full lane topology of the loaded town through its
Python API: `world.get_map().get_topology()` returns every connected
`(start_wp, end_wp)` lane pair in the map. For a junction
specifically, `wp.get_junction()` retrieves the junction object and
`junction.get_waypoints(carla.LaneType.Driving)` returns *every
entry-exit waypoint pair through that junction* — left turn, right
turn, straight, and any extra connectors.

- **Method.** Detect the upcoming junction (the existing
  `junction_monitor` already does this). At ENTER, query
  `get_waypoints` for every entry-exit pair, walk each pair at 2 m
  resolution to obtain polylines in *world* coordinates, transform
  to *vehicle* frame using `ego.get_transform()`, hand the
  polylines to the IPM node ([§19](#19-ipm-birds-eye-view--ipm_view_node-done))
  for rendering — each path in a different colour.
- **Effort.** ~50 lines, mostly in the bridge.
- **Result.** Perfect lane topology in sim. Doesn't generalise to
  real-world (no equivalent API).
- **Right next step here** — it builds directly on what we already
  have and shows immediately whether *visualising* every exit is
  useful in the first place.

### Approach B — Online camera-based BEV lane networks

Train (or fine-tune) a neural network that takes the forward camera
(or a multi-camera surround view) and outputs lane geometry directly
in BEV. Modern state of the art:

- **StreamMapNet** (Yuan et al., 2024) — transformer-based
  encoder, temporal stream of BEV features, outputs **vector lanes**
  with type labels (divider, boundary, centreline) in real time. The
  user's intended thesis target.
- **MapTR / MapTRv2** (Liao et al., 2022/2024) — earlier vector
  lane networks; MapTRv2 added multi-class instances and is the
  reference baseline.
- **HDMapNet** (Li et al., 2022) — rasterized BEV semantic maps
  + post-hoc vectorisation. Simpler but less direct.
- **Lift-Splat-Shoot** (Philion & Fidler, 2020) — the lifting
  backbone many BEV networks build on; gives a top-down feature map
  from N cameras via per-pixel depth estimation.

- **Method.** Replace `lane_detection_node`'s single-lane
  UFLD output with a multi-lane vector head. In sim: train on
  nuScenes / Argoverse-2 / Waymo Open Map for transfer, or
  synthesise CARLA ground truth from Approach A's
  `get_waypoints` calls (CARLA-native dataset, no domain gap, but
  no real-world generalisation either).
- **Effort.** Substantial — model architecture, training pipeline,
  evaluation against ground truth, integration into the ROS stack.
  Thesis-scope work.
- **Result.** Generalises beyond CARLA (depending on training data),
  no reliance on a pre-built map. State of the art for *online* HD-
  map prediction.

### Approach C — Pre-built HD maps (production reality)

The map is recorded *offline* with a dedicated survey vehicle
(LiDAR + GNSS-INS), aligned to centimetre scale, and shipped in the
car. Online perception then mostly **localises in the map** and
**confirms it is still valid** (construction, snow, repainted
markings).

- **Method.** Pre-record lane geometry for every junction the car
  is allowed to operate in. At runtime, localise with high accuracy
  (RTK-GNSS + IMU + LiDAR/vision feature matching) and look up the
  junction's lane topology from the on-board HD map.
- **Effort.** Lowest *online* compute, highest *offline* logistics:
  survey vehicles, map storage, change-detection pipeline,
  geographic restriction of the operational domain (ODD).
- **Result.** Highest reliability, lowest ODD breadth. Not suitable
  for a research project in CARLA, but it's what makes
  Level-3-certified consumer systems (see below) possible today.

### What current OEM ADAS systems use

- **Tesla (FSD / AP, vision-only).** Single neural backbone
  ("HydraNet") with multiple heads — among them vector lane
  prediction, object detection, traffic-light state, drivable
  space, and the more recent occupancy network for arbitrary 3D
  obstacles. Eight cameras → BEV transformer → vector lanes. No
  LiDAR, no radar, no HD map. Public direction is increasingly
  end-to-end neural planning (cameras → control). Mobileye and
  Wayve are pursuing similar.
- **Mercedes Drive Pilot (Level 3, S-Class / EQS).** The opposite
  philosophy: **HD map + LiDAR + radar + cameras + ultrasonic +
  high-precision GNSS/IMU.** Operates *only* on pre-mapped highway
  segments (Germany, Nevada, California) at up to 95 km/h. The HD
  map provides lane topology; onboard perception confirms presence
  of lane lines and vehicles and localises within centimetres.
  Conservative ODD is the trade-off for Mercedes taking legal
  liability while engaged.
- **Waymo / Cruise / Mobileye Chauffeur.** Closer to Mercedes' end
  — LiDAR + HD map + multi-modal perception, with neural BEV
  networks layered on top for redundancy. Mobileye additionally
  crowd-builds a thin HD map ("Road Experience Management") from
  production-vehicle camera feeds while the car also runs
  vision-only perception.

The field is bifurcating into "vision-only, big-data, big-model" on
one side (Tesla, Wayve, Mobileye SuperVision) and "HD-map + sensor-
fusion, narrow ODD, certified" on the other (Mercedes, Waymo).
Junction-lane mapping is where the two diverge most visibly:
vision-only systems must *predict* it online; HD-map systems can
*look it up*.

### Recommended sequencing for this stack

1. **Approach A first** (immediate, ~50 lines). Renders every
   junction lane on the existing IPM BEV canvas. Validates the
   visualisation + IPM math against ground truth before any
   neural component is involved.
2. **Approach B (StreamMapNet) for thesis novelty.** Synthesise
   CARLA training data using Approach A as the labeller, train
   StreamMapNet, replace UFLD's single-lane output with multi-lane
   vector predictions. Compare against Approach A ground truth
   inside CARLA; evaluate generalisation by additionally running
   on a real-world dataset (nuScenes mini, OpenLane).
3. **Approach C is out of scope** for a CARLA-only research
   project, but worth a half-page in the thesis discussion as the
   industrial reference point.

**Open questions / decisions.**

- *Training-data realism.* CARLA's camera and lane geometry have a
  domain gap to real-world driving (lighting, weather variety,
  marking deterioration). A CARLA-only-trained model may not
  transfer. The mitigation is mixing CARLA + a real-world dataset,
  or pre-training on real and fine-tuning on CARLA.
- *Evaluation metric.* MapTR-family papers use Chamfer distance and
  AP-by-class against vectorised ground truth. CARLA's
  `get_waypoints` gives us perfect ground truth for free — no
  manual labelling.
- *Latency target.* If the StreamMapNet output feeds the same
  Stanley / pure-pursuit hand-off as today's UFLD does, it needs to
  meet ≥10 Hz with bounded latency. Real-time inference budget on
  the dev GPU is the gating constraint.

---

## 21. ADAS stack near CPU capacity — UFLD diagnosis & rate limit [FIXED, with planned follow-up]

**Symptom.** During normal operation the 28-core box ran with
~50-80 % utilisation spread across roughly half the cores, load
average climbed to ~19, and the UI / camera feed felt sluggish.
Adding `ipm_view_node` to the regular launch ([§19](#19-ipm-birds-eye-view--ipm_view_node-done))
pushed things further. The "fully exhausted" feel was a wide spread
of cores at moderate utilisation rather than a few cores pinned at
100 % — i.e. it was the *number of busy cores* that was the
problem, not any single hot core.

**Process audit (steady-state, before fixes).**

| Process | CPU % (sustained) | Threads |
|---|---|---|
| CarlaUE4 | ~300 % | — |
| **lane_detection_node** | **~660-980 %** | **114** |
| perception_node | ~80 % | 68 |
| debug_image_fusion | ~85 % | 65 |
| foxglove_bridge (relay) | ~80 % | — |
| ipm_view_node | ~25 % | 92 |

GPU was at 45 % utilisation with 3 GB used by UFLD — i.e. the model
was on GPU as intended; the CPU cost was *plumbing around the GPU*,
not inference itself. `lane_detection_node` was the obvious top
target; three independent contributors compounded.

### 21a. UFLD model-load CPU spike — `map_location=device` [FIXED]

**Root cause.** `torch.load('UFLD_best.pth', map_location='cpu')`
deserialises the 1.7 GB state dict into CPU RAM, then `.to(device)`
copies the same data to GPU. Two costs: (i) a multi-threaded
deserialise + allocate burst (measured 770 % CPU on a 23-second-old
process — ~177 CPU-seconds of work in 23 wall-seconds), and (ii) a
transient 1.7 GB RAM footprint that pushed the box into swap
(~1.9 GB swap in use during cold start).

**Applied fix.**
[lane_detection_node.py:90](src/perception/perception/lane_detection_node.py#L90).
`map_location='cpu'` → `map_location=device`. Weights land directly
on GPU, no CPU-resident copy, no redundant CPU→GPU transfer. The
subsequent `net.eval().to(device)` becomes a no-op for the device
move but is kept (`.eval()` is still needed to put BN/dropout into
eval mode). Inference output is bit-for-bit identical.

Measured drop: 770 % → 218 % at age-23 s after restart. Cold-start
swap pressure halved (1.9 GB → 1.0 GB). The `torch.load(..., mmap=True,
weights_only=True)` flags were considered as additional sharpenings
but not applied — `map_location=device` alone solved the symptom.

### 21b. Thread-pool sprawl — cap all parallel libraries [FIXED]

**Root cause.** Even after §21a, `lane_detection_node` still ran 114
threads with 16-20 in R-state during inference bursts, burning ~7
cores at steady state. Every parallel library in the stack —
PyTorch intra-op, PyTorch inter-op, OpenCV, OpenMP, MKL, OpenBLAS,
NumExpr — defaults its thread pool to the number of physical cores.
On a 28-core box each library happily spawns its own ~28 workers.
None of those defaults are visible to each other, so capping
`torch.set_num_threads()` only addresses ~1/N of the actual
parallelism.

**Applied fix.** Cap every relevant pool at 2 threads at module
import time, *before* any third-party import. Applied in both
[lane_detection_node.py](src/perception/perception/lane_detection_node.py)
and [perception_node.py](src/perception/perception/perception_node.py):

```python
import os
os.environ.setdefault('OMP_NUM_THREADS',      '2')
os.environ.setdefault('MKL_NUM_THREADS',      '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '2')
os.environ.setdefault('NUMEXPR_NUM_THREADS',  '2')

import cv2
cv2.setNumThreads(2)

import torch
torch.set_num_threads(2)
torch.set_num_interop_threads(2)
```

Ordering matters: the env vars must precede the first `import
numpy / cv2 / torch / ultralytics`, because those libraries read the
env vars at import time to size their thread pools.
`perception_node.py` originally had the env vars *after* `from
ultralytics import YOLO`, which silently negated them — fixed in
the same pass by reordering imports so all env setup happens first.

Measured effect: system idle jumped from 29 % → 74 % at one
snapshot, the per-core spread tightened (no more 21 simultaneous
R-state threads), and bursts dropped from "always-on" to
intermittent. Wall-time CPU consumption did *not* drop in
proportion to thread count — each inference still does the same
total work — but the cap reduced the **spread** of concurrent
activity across cores, which is what made the box feel responsive
again.

### 21c. Drop UFLD inference rate — `inference_skip_n` (Option A) [FIXED]

**Root cause.** With §21a + §21b applied, the residual sustained
load was dominated by ~140 small tensor operations per frame inside
[UFLDInference.__call__](src/perception/perception/lane_detection_node.py#L114-L149)
— a Python loop over `num_cls_row` row anchors, each iteration
creating tiny tensors and calling `softmax`, elementwise multiply,
`sum`, `.item()`. At 20 Hz camera input that's ~2,800 torch ops/sec,
each with per-op thread-pool startup overhead. The CPU load was
*overhead-bound*, not math-bound.

**Applied fix.** Process only every Nth camera frame.
`lane_detection_node` got a new ROS parameter `inference_skip_n`
(default `4`), and `camera_callback` early-returns on
`(frame_count - 1) % skip_n != 0` *before* `cv2.imdecode`, so JPEG
decode, preprocess, UFLD forward, post-process and JPEG encode are
all skipped on the dropped 3 of 4 frames. Frame 1 always processes
so the first-frame log fires immediately.

UFLD effective rate: 20 Hz → 5 Hz. The polyline topics
`/LKAS/ego_lane_left/right` drop to 5 Hz, but Stanley reads the
latest cached `Path` on its own 20 Hz tick — Stanley's output rate
is unchanged.

**Performance impact on lateral control.** Lane data is at most
`200 ms` stale at 5 Hz UFLD vs `50 ms` at 20 Hz. At the controller's
20 km/h cruise target (`5.5 m/s`), that's `1.1 m` of vehicle travel
between updates — well within lane width and the actuator response
window. On a sweeping curve the steer trace shows small step-shaped
chatter at lane-update boundaries; on straight road it is
imperceptible. At higher speeds (e.g. 60 km/h ≈ 16.7 m/s, 3.3 m
between updates) the trade-off is no longer free — set
`inference_skip_n:=2` (10 Hz) for the middle ground, or `:=1`
(20 Hz, no skip) on launch.

Runtime override (no rebuild):
```
ros2 run perception lane_detection_node --ros-args -p inference_skip_n:=2
```

Final measured state (with §21a + §21b + §21c): per-core CPU stays
under ~80 %, no cores pinned at 100 %, load average dropped from
18+ into single digits, system feels responsive.

### Caveats / follow-ups

- **Option B — vectorise the polyline loop [PLANNED].** The
  fundamental fix is to replace the per-row-anchor Python loop in
  [UFLDInference.__call__](src/perception/perception/lane_detection_node.py#L114-L149)
  with a single vectorised pass: compute `softmax` along the entire
  `loc_row` grid axis once, compute the weighted sum over a centred
  3-cell window for every `(row, lane)` pair in a batched tensor op,
  then mask by `valid` and convert to Python only at the very end.
  Expected ~50 % drop in steady-state CPU on top of Option A, and a
  path back to running at full 20 Hz UFLD without saturating cores.
  Estimated ~20 lines in `__call__`. Not done in this iteration
  because Option A already brought the box back into headroom.
  Worth doing before raising the operating speed target above
  ~30 km/h, where 5 Hz lane updates start to look stale.
- **`/proc/$PID/environ` does not reflect runtime `os.environ`
  edits.** Sanity-checking the §21b caps via `tr '\0' '\n' <
  /proc/$PID/environ` shows nothing — that's expected (`/proc/environ`
  is fixed at fork time) and is *not* proof the caps failed. To
  verify the caps inside the process, log them at startup:
  `self.get_logger().info(f"threads: torch={torch.get_num_threads()}
  cv2={cv2.getNumThreads()}")`.
- **Foxglove bridge (~75-130 %) is the remaining big non-CARLA CPU
  consumer.** Closing the Foxglove Studio panel collapses that to
  ~5 %. Not part of this fix but the easiest further win when
  iterating — and reinforces the §18 note that Foxglove should be
  started/stopped on demand, not left running.
- **`adas-rebuild` shell alias.** Added to `~/.bashrc` during this
  iteration: `(cd ROS_ADAS_Stack && source /opt/ros/humble/setup.bash
  && colcon build --packages-select perception controller)`. Uses a
  subshell so the user's working directory is not affected. The
  install is a hard copy, not a symlink — `--symlink-install` is
  *not* compatible with the current setuptools and breaks the
  package's build (see git history of this DEBUG entry for the
  failure mode), so every `.py` edit in `src/perception/` or
  `src/controller/` still needs a rebuild.

---

## 22. Lead distance: pinhole → IPM and semantics → bumper-to-bumper gap [FIXED]

**Symptom / motivation.** ACC distance estimation lived in
[perception_node.py](src/perception/perception/perception_node.py) as a
pinhole similar-triangles calculation, `distance = focal_px * H_class
/ bb_height_px`, with a fixed `OBJECT_HEIGHTS` table giving each YOLO
class a real-world height (`car: 1.5`, `truck: 3.0`, …). Two problems
with that:

1. **The reported number was a camera-to-lead distance**, not a
   bumper-to-bumper gap. The ACC PD law was tuned against the *wrong*
   semantics — `d0 = 5 m` meant "camera sees 5 m to the lead's
   centre-ish" which, on a Tesla Model 3 (extent.x = 2.4 m, cam at
   x = 0.6 m), is only a ~3.2 m physical bumper gap. On a Dodge
   Charger (extent.x = 2.5 m), ~3.1 m.
2. **The pinhole estimator was 9-22 % off** across the operating
   range. We measured this end-to-end with a static CARLA sweep
   ([carlaaccsim/ipm_validate.py](../../carlaaccsim/ipm_validate.py)):

   | d (bumper gap, m) | IPM err % | Pinhole err % |
   |---|---|---|
   | 3  | +2.0 | -19.7 |
   | 5  | +2.7 | -16.5 |
   | 7  | +1.3 | -11.8 |
   | 10 | +1.3 | -11.5 |
   | 15 | +2.6 | -5.1  |

   Pinhole's residual was structural — the per-class `H` table is a
   single point estimate of an inherently variable quantity (Model 3
   ≠ Cadillac ≠ pickup), and YOLO's bb-top edge is loose / clipped /
   noisy in ways that propagate linearly into the distance. IPM by
   contrast uses only the bb-bottom edge, projected through the same
   `cam_height_m = 1.35` / `cam_x_offset = 0.6` / `cam_fov = 90°`
   triple already in use by the lane-ROI filter and `ipm_view_node`.
   That triple was validated to sub-1 % over the 7-15 m band in the
   same sweep, so the IPM-derived distance inherits that accuracy.

### 22a. perception_node — IPM distance + bumper-gap semantics [FIXED]

**Applied changes** in
[perception_node.py](src/perception/perception/perception_node.py):

- `OBJECT_HEIGHTS` table removed. Pinhole used it for distance;
  IPM has no analogue (no per-class assumption — see §22 intro).
- New module-level `VEHICLE_CLASSES = {'car', 'truck', 'bus',
  'motorcycle'}` keeps the *class filter* that `OBJECT_HEIGHTS` was
  implicitly providing. Same set of YOLO outputs ignored, no other
  behaviour change.
- New ROS parameter `ego_extent_x` (default `2.504`, the CARLA Dodge
  Charger value). Used to convert the IPM's vehicle-frame `X` (from
  the ego pivot) into a bumper-to-bumper gap.
- `estimateLeadDist`: the bottom-centre of each surviving bb is
  IPM-projected via the existing `_pixel_to_vehicle` — *the same call
  that was already happening for the lane-ROI filter*. We just read
  `ground[0]` out of it twice now: once for the lane check, once for
  the distance:
  ```python
  distance_m = max(MIN_PUBLISHED_GAP_M, ground[0] - self.ego_extent_x)
  ```
- `MIN_PUBLISHED_GAP_M = 0.1` clamps the published number above zero.
  At gap ≲ 2 m the bb-bottom clips at the camera frame's lower edge,
  the IPM under-reads (we saw 0.50 m for a 2 m gt), and a raw
  subtraction can land at `≤ 0`. The controller treats `d ≤ 0` as
  "no detection" (resets the low-pass filter and falls through to
  cruise mode), which would be *unsafe* exactly when a lead is right
  in front of the bumper. Clamping at 0.1 m keeps the number positive
  but well below `emergency_distance = 3 m`, so the controller's
  EMERGENCY brake fires instead of cruise re-engaging.
- Per-frame `focal_px` computation in `estimateLeadDist` removed —
  the IPM has its own focal computation inside `_pixel_to_vehicle`.

### 22b. controller_node — semantics + comments updated [FIXED]

The PD math is unchanged; only the *meaning* of the numbers shifted.
[controller_node.py](src/controller/controller/controller_node.py):

- Docstring updated to call out `/ACC/lead_vehicle_distance` as a
  bumper-to-bumper gap, pointing back to this section.
- `d0 = 5.0` and `emergency_distance = 3.0` kept at the same numeric
  values — they now mean a literal 5 m gap (standstill) and 3 m gap
  (emergency brake), which is *slightly more conservative* than the
  pre-IPM behaviour was (5 m camera-to-lead ≈ 3.2 m gap; 3 m
  camera-to-lead ≈ 1.2 m gap). Decision rationale: keeping the
  mental model "follow at ~5 m, hit the brakes at 3 m" wins over
  matching the previous numerical behaviour, especially because the
  previous behaviour was tuned around a pinhole estimate that was
  systematically 10-20 % short — so "what felt like 5 m" was really
  ~4 m. The 5 m gap target after this change is closer to what the
  operator was probably *intending* all along.
- Comments around `T_gap`, `d0`, and `emergency_distance` rewritten
  for the new semantics. The §6 d_desired math is updated in-place:
  at 20 km/h cruise, `d_desired = 5 + 0.3 * 5.5 ≈ 6.65 m` gap,
  settling to 5 m at rest.

### 22c. Validation harness — `ipm_validate.py` [DONE]

Standalone CARLA Python script at
[carlaaccsim/ipm_validate.py](../../carlaaccsim/ipm_validate.py).

- Spawns ego (default Dodge Charger), forces sync mode, attaches the
  same camera rig as the bridge (1280×720 @ 90° FOV, mounted at
  `Location(x=0.6, z=1.35)`).
- For each bumper gap `d ∈ {2, 3, 5, 7, 10, 15}` m, places the lead's
  rear bumper exactly `d` metres ahead of ego's front bumper:
  ```python
  offset = ego_extent_x + d + lead_extent_x   # along ego forward
  lead.set_transform(Transform(ego_loc + offset * fwd, ego_yaw))
  ```
- Captures one frame, runs YOLO via the same `best.pt` perception
  uses, takes the largest `car` bb, IPM-projects its bottom centre,
  subtracts `ego_extent_x` to get the bumper-gap.
- Outputs CSV + annotated PNG per distance.

Two methodological notes for whoever runs it next:

1. **Always query `ego.bounding_box.extent.x` at runtime**, never
   hard-code it. The Charger's 2.504 m is materially different from
   the Model 3's 2.396 m, and a hard-coded value would silently bias
   the gap output by ~10 cm per blueprint swap. The script logs
   `ego_extent_x` and the derived camera→front-bumper offset on
   startup so the geometry is visible.
2. **Don't trust IPM below gap ≈ 3 m.** The script will still
   report numbers there; they're just dominated by YOLO bb-bottom
   misbehaviour (frame clip, shadow inclusion, bumper-edge snap),
   not by IPM precision. See "Why is IPM worse close than far" in
   the live session — it's a feature interpretation issue, not a
   model failure.

### Caveats / follow-ups

- **`ego_extent_x` is a static parameter.** If the bridge spawns a
  different ego blueprint, the user has to remember to override the
  parameter or the gap drifts by `(extent_x − 2.504)` m. The fix is
  for the bridge to query `hero.bounding_box.extent.x` once at
  startup and publish on a latched `/Car_1/ego_extent_x` topic;
  `perception_node` subscribes and uses whatever it gets. ~10 lines
  in the bridge + a `create_subscription` here. Not done in this
  iteration because all current scenarios use the Charger.
- **Slope sensitivity.** IPM's flat-ground assumption breaks on
  grades. On a 5 % uphill, a lead at 20 m physical distance reads
  ~22 m via IPM (1 m of vertical drop interpreted as forward
  distance). For Town03/Town01 demos this is below 1 % at the
  speeds we run, but if §20's StreamMapNet work moves us to highway
  speeds, an IMU-tilt-corrected IPM is the right next step.
- **Close-range saturation behaviour is now a *feature*.** At gap
  ≤ 2 m, the IPM gap drops well below the actual gap (we measured
  0.5 m for a 2 m gt). That value is below `emergency_distance = 3`
  m so the controller's EMERGENCY branch trips — exactly the right
  response. No special "saturation-detected" branch is needed in
  either node, by construction. Documented here so the apparent
  -75 % error at d=2 in the validation table is not interpreted as
  a bug.
- **§6 superseded for the distance estimator.** The §6 ACC distance-
  filter / d_desired tuning rationale is still correct, but the
  pinhole-estimator-specific caveats inside §6 (the rant about
  `OBJECT_HEIGHTS` being a systematic over-estimate at close range)
  no longer apply — that's all IPM now.
- **Validation only covered Town03, ClearNoon, single straight
  spawn.** Cross-town / weather / curved-road validation is open.
  IPM is camera-geometry only — it should generalise — but the YOLO
  side is what changes with scene content, and we haven't measured
  e.g. WetNoon or HardRainNight bb behaviour.

---

## 23. Anchor-based loop route for lead + PP fallback [DONE]

**Background.** Until now the lead vehicle ran TrafficManager's default
autopilot — `lead_vehicle.set_autopilot(True, tm_port)` plus a speed
scaler. TM picked its own direction at every junction, so successive
runs of the same scenario diverged: the ACC test conditions were
non-reproducible. The ego's pure-pursuit fallback ([§3b](#3b-lkas-off--ego-steers-via-pure-pursuit-fallback-fixed))
also walked a *different* route (heading-aligned forward walker
from [build_ego_route](../../carlaaccsim/carlaAccSimTown.py)), so
during junction PP-takeover the ego could end up on a different road
than the lead by the time it re-engaged LKAS.

For repeatable ACC tuning we want both vehicles to traverse the *same*
closed loop, indefinitely.

**Applied changes** in [carlaAccSimTown.py](../../carlaaccsim/carlaAccSimTown.py)
and [pure_pursuit_controller.py](../../carlaaccsim/pure_pursuit_controller.py):

### 23a. CLI: `--loop-spawns "13,38,92,131,192"` (Town03)

Comma-separated spawn-point indices. When set:

- Ego spawn is *forced* to anchor 0 (overrides `--spawn-index`) so the
  loop starts where the ego sits — otherwise the ego would spawn
  off-route and the PP fallback would do something weird at startup.
- `build_anchor_loop_route(carla_map, anchor_indices)` connects each
  consecutive pair (cyclic) with `agents.navigation.global_route_planner.
  GlobalRoutePlanner.trace_route(start, end)`. The result is a list of
  `carla.Location`s that follow actual driveable roads from anchor `i`
  to anchor `i+1`, closing the last segment from `anchor[N-1]` back to
  `anchor[0]`. Duplicate join-point waypoints between segments are
  trimmed.
- The same route is used as:
  1. The lead's TM path via `tm.set_path(lead_vehicle, ego_route)`.
     TM still owns the actor (collisions, signals, speed scaling) — it
     just follows our waypoints instead of picking its own turns.
  2. The ego's `ego_route` consumed by `run_pure_pursuit(...)`. So
     during junction PP-takeover the ego pursues the same waypoints
     the lead has been chewing through.

Anchor coordinates for the Town03 default loop (recorded here for
the operator's reference — printed by `--list-spawns`):

| index | x       | y        | z    | yaw     |
|------:|--------:|---------:|-----:|--------:|
| 13    | -74.39  |  42.00   | 0.95 |  -90.16 |
| 38    |  -9.42  | 113.00   | 0.28 |   89.64 |
| 92    | 125.36  | -135.59  | 8.31 | -178.77 |
| 131   |   0.70  | -189.73  | 0.28 |   91.41 |
| 192   | 207.08  |  -5.19   | 0.28 | -179.14 |

### 23b. Pre-queued laps + monitor-only watcher

**Original design (replaced).** The initial implementation called
`tm.set_path(lead_vehicle, ego_route)` once at startup, then had a
background thread that re-called `set_path` each time the lead came
back near anchor 0 — giving an infinite loop of laps.

**What we observed.** With the default 5-anchor Town03 loop (~1624
waypoints), the lead drove the *first* lap fine. The lap-watcher's
*second* `tm.set_path()` call killed the actor within milliseconds:
the bridge log showed `[lap-watcher] queued another lap` followed
immediately by `rclpy` callbacks crashing with
`RuntimeError: trying to operate on a destroyed actor`. CARLA 0.9.16's
TrafficManager appears to have a bug or limitation around repeated
`set_path` calls on the same actor.

**Replacement design (current).** Pre-queue *N* laps in a single
`tm.set_path` call at startup, then make the watcher monitor-only:

- New CLI flag `--loop-laps N` (default 3). The bridge concatenates
  `N` copies of the route and passes them all to `tm.set_path` once.
  Each lap's Locations are fresh `carla.Location(x, y, z)` instances
  rather than shared references, in case TM dedupes by object identity.
- `_lap_watcher` no longer calls `set_path`. It still polls the lead's
  position at 1 Hz, latches `has_moved_away` when the lead is > 100 m
  from anchor 0, and prints `[lap-watcher] lap N complete (d_to_start=...)`
  on each pass of anchor 0 with a 30 s cooldown to avoid double-counts.
- Exits cleanly if the lead actor becomes unreachable (collision,
  server-side despawn, etc.), printing the lap count it observed
  before exit. No more "destroyed actor" error spam.
- Cleanup: `lap_stop_event.set()` in the bridge's `finally:` clause
  alongside the existing `junction_stop.set()`.

**Trade-off accepted.** The test horizon is now bounded: at `--loop-laps 3`
the lead can drive ~3 laps (≈5 km on the default Town03 anchors) before
TM's queue empties and it falls back to free-roam. For an actual
indefinite-lap mode, the bug in `tm.set_path` would need to be either
fixed upstream or worked around (e.g., destroy & respawn the lead at
anchor 0 each lap — more invasive, not done).

### 23c. ROS spin-thread robustness against destroyed actors

`custom_ROS_pub_sub.CarlaAVT._publish_distance` previously did

```python
dist = self.ego_vehicle.get_location().distance(self.lead_vehicle.get_location())
```

with no `is_alive` guard. When the lead was destroyed mid-run (the
original `tm.set_path` bug, but also: collisions, server cleanup, or
user-driven actor.destroy()), the next 50 ms timer fired this
callback, hit a destroyed actor, raised `RuntimeError`, and crashed
the entire `rclpy` spin thread — silently stopping all bridge ROS
publishes including `/Car_1/vehicle/speed`, `/Car_1/camera/front/compressed`,
and `/ACC/lead_vehicle_distance`. From the operator's view, the bridge
"just stopped working".

Added a defensive guard: check `is_alive` on both actors before
calling `get_location`, and wrap the call itself in `try/except RuntimeError`
for the race between the check and the use. On a missed liveness check,
`_publish_distance` returns silently — the topic just stops updating
until the actors come back (they don't, normally — this is just a
graceful-degradation guard).

### 23d. Pure-pursuit wrap-around (`loop=True`)

`get_target_wp_index` in [pure_pursuit_controller.py](../../carlaaccsim/pure_pursuit_controller.py)
previously clamped the target index at `len(route) - 1`. For an
open-ended (non-loop) route that's correct — once you reach the end,
keep aiming at the last waypoint. For a loop route, it would stall
the ego right at the join point. Added `loop` kwarg to
`_run_controller` / `run_pure_pursuit`:

```python
raw_idx = int(np.argmin(dist)) + 4
if loop:
    idx = raw_idx % len(waypoint_list)
else:
    idx = min(raw_idx, len(waypoint_list) - 1)
```

The bridge passes `loop=True` exactly when `--loop-spawns` is set, so
the legacy ego_route (open-ended forward walker) is unaffected.

### Caveats / follow-ups

- **`anchor[N-1] → anchor[0]` may need to be a *driveable* segment.**
  `GlobalRoutePlanner.trace_route` raises if the closing segment hits
  a one-way road in the wrong direction or crosses a tram-only lane.
  Mitigation: pick anchors that lie on bidirectional driving roads,
  or add intermediate anchors so the planner has more flexibility.
- **`anchor_locations` is held as a Python list of `carla.Location`
  objects in the lap-watcher** — these are server-allocated, and CARLA
  has been known to invalidate Location handles in long-running
  scripts after a re-tick. If the watcher ever stops firing, hard-copy
  to `(x, y, z)` tuples instead.
- **Lead spawn position is unchanged.** It still spawns
  `--lead-gap-m` ahead of the ego along the ego's lane forward
  vector. Since anchor 0 places the ego on the loop, the lead spawns
  on the path's first segment — natural starting state.
- **PP `loop=True` plus a route that doesn't actually close** would
  cause the ego to teleport-aim back to the loop's start when it
  reaches the end. With `GlobalRoutePlanner.trace_route` closing the
  last segment by construction, this can't happen — but if a future
  caller passes `loop=True` with a manually-constructed non-closing
  route, the ego will visibly snap. Documented here in case.
- **Sync-mode interaction (§4).** The lap-watcher runs in real time
  (`time.sleep(1.0)`) regardless of the simulation tick rate. In
  async mode (current default), that's fine. In sync mode, the
  watcher still fires at wall-clock 1 Hz, which is fine because it
  only does a distance check — not a tick-side action.
- **Switching ego or lead blueprint while loop mode is active.**
  Should be transparent — the loop is built from map geometry, not
  vehicle bbox. Verified for Charger; should hold for any
  driving-class blueprint.


## 24. Retrained UFLD checkpoint available in the UI [DONE]

**What changed.** A second UFLD V2 checkpoint has been added to the
stack, trained on a 5× larger dataset (75 000 frames vs the original
15 000). Both models are now selectable from the "Lane model" dropdown
in the Features frame of `UI.py`, so the operator can flip between
them without editing config or rebuilding.

- **`UFLD_F1=0.87.pth`** — original §3.1 fine-tune, F1 = 0.87 on the
  15 K test split (Town03/04/10 clear-weather only).
- **`UFLD_F1=0.67.pth`** — new §3.3 retrain, F1 = 0.67 on the
  75 K test split (Town01/02/03/04/05/10HD × clear + rain).

Both files live in `src/perception/models/`. The UI passes the
selected `.pth` filename to `lane_detection_node` via
`--ros-args -p model_filename:=<ref>` at LKAS: ON. See `LANE_MODELS`
at the top of [UI.py](UI.py).

**Why the F1 dropped from 0.87 to 0.67 — it didn't, the yardstick
changed.** The old number was measured on the old test split (3 easy
towns, all clear). The new number is measured on 6 towns including
Town01 curb-only roads + rain in every town. Direct apples-to-apples
requires evaluating the old checkpoint on the *new* test split
first — planned as follow-up. See full details in
[`00_Lane_Assistant/02_UFLD_V2/DEBUG.md §3.3`](../../00_Lane_Assistant/02_UFLD_V2/DEBUG.md#33-retraining-on-75-k-frames-6-towns--2-weather).

**Effective training samples per epoch was 61 550, not 75 000.**

- `list/train_gt.txt` has 84 000 lines (raw), which is what the trainer
  iterates each epoch.
- After deduplication only **61 550 unique image references** remain.
- Each unique frame was therefore shown ~1.37 times per epoch — the
  extra ~22 K exposures were repeats, not new information.

Root cause is on the *collector* side, not the trainer: `collect_dataset.py`
opens the list files in append mode (`open(list_dir / fname, 'a')`) and
writes the 70 / 15 / 15 split at the end of each per-town run. During the
overnight batch several runs hit the CARLA teardown race — process exit
code `rc=134` from `terminate called after throwing an instance of
'std::runtime_error': trying to operate on a destroyed actor` in the
`finally:` NPC-destroy loop — *after* the loop had already written the
split lines. `collect_all.sh` then retried the failed ranges, and each
retry appended a second set of split lines pointing at the same on-disk
PNGs. Cache builder collapses duplicates by image path (dict key), but
the trainer iterates the raw list file.

**Same mechanism caused a partial train↔eval leak:** sum of per-split
unique frames (61 550 + 16 850 + 16 950 = 95 350) exceeds disk-unique
(75 000) by ~20 K, meaning ~20 K frames appear in more than one split.
Val / test F1 numbers are therefore optimistic by an unknown fraction.
Fix without retraining: `awk '!seen[$0]++' list/<file>.txt` on each
split, then rerun the eval script on the cleaned splits.

Full training config, LR-schedule trajectory, and F1 / P / R progression
in [`00_Lane_Assistant/02_UFLD_V2/DEBUG.md §3.3`](../../00_Lane_Assistant/02_UFLD_V2/DEBUG.md#33-retraining-on-75-k-frames-6-towns--2-weather).

**Closed-loop A/B — pending.** Recommended workflow (record both runs
via the new "Record rosbag" checkbox, play them back through the same
route in Foxglove):

1. Same spawn index + same weather + same route.
2. LKAS: ON with `UFLD_F1=0.87.pth` → bag #1.
3. LKAS: OFF, dropdown switch, LKAS: ON with `UFLD_F1=0.67.pth` → bag #2.
4. Focus on Town01 (curbs) + rain — the scenes the new model was
   trained *for* and the old one wasn't.


## 25. Per-lane confidence output for UFLD (YOLO-analogue) [DONE]

**What was added.** `lane_detection_node` now emits a scalar confidence
`∈ [0, 1]` per ego lane, published on two new topics and rendered as a
colour-coded text overlay in the top-left of `/LKAS/perception/debug_image`.
Analogous to YOLO's per-detection `objectness × class_prob` — but
distilled from UFLD's own row-anchor logits so no extra head or
retraining is required.

New topics (both `std_msgs/Float32`):

| Topic | Meaning |
|-------|---------|
| `/LKAS/ego_lane_left_conf`  | Confidence in the ego-left polyline for this frame |
| `/LKAS/ego_lane_right_conf` | Confidence in the ego-right polyline for this frame |

Published at the LKAS inference rate (~5 Hz, same as the polyline topics).
During junctions the node publishes `0.0` on both — matches the
already-existing `to_path([], header)` empty-Path behaviour, so
downstream consumers can gate uniformly on either signal.

**How the number is computed.** UFLD V2's decode does
`loc_row.argmax(1)` and `exist_row.argmax(1)` for hard row-anchor
predictions, throwing away the underlying soft distributions. The
confidence output recovers them:

```python
# lane_detection_node.py — inside UFLDInference.__call__, after
# pred = self.net(tensor)
exist_soft = torch.softmax(exist_row, dim=1)[:, 1]   # (1, num_row, num_lanes) — P(lane present)
loc_soft   = torch.softmax(loc_row,   dim=1)         # (1, 201, num_row, num_lanes)
pos_peak   = loc_soft.max(dim=1).values              # (1, num_row, num_lanes) — sharpest cell probability

def lane_conf(lane_idx):
    mask = exist_soft[0, :, lane_idx] > 0.5          # anchors where lane is visible
    if mask.sum() == 0: return 0.0
    return float((exist_soft[0, mask, lane_idx]
                  * pos_peak[0, mask, lane_idx]).mean())
```

- `exist_soft` is the "yes-there's-a-lane-at-this-anchor" side of the
  binary softmax over the existence-branch logits (`exist_row` has
  shape `(B, 2, num_row, num_lanes)`; the class=1 slice is
  `P(lane exists)`).
- `pos_peak` is the height of the tallest bin in the 201-bin softmax
  over the position-branch logits (`loc_row` has shape
  `(B, num_cell_row + 1, num_row, num_lanes)`; the argmax of dim=1
  picks the winning cell, `.max` returns its probability).
- The product is the per-anchor confidence: "there's a lane here **and**
  we know precisely which cell it's in."
- Averaging over anchors that pass the `exist > 0.5` mask gives one
  scalar per lane. Anchors where the lane isn't even binary-visible
  are excluded so the mean isn't diluted by masked positions.

**Why multiply exist × pos_peak instead of using either alone.** A
high `exist_prob` on its own means "I know a lane point is at this
row" but says nothing about *where* — a flat position softmax spread
across 5 neighbouring cells (`pos_peak ≈ 0.3`) still argmaxes cleanly
but is spatially untrustworthy. A high `pos_peak` on its own says
"*if* there were a lane it'd be in this cell" but the model may be
99 % sure no lane exists there at all. The product suppresses both
failure modes with one number.

**Visualisation.** `lane_detection_node.annotate()` draws two lines of
text in the top-left corner of the LKAS debug image:

```
L: 0.72        <- ego-left  (colour by tier)
R: 0.85        <- ego-right (colour by tier)
```

Colour tiers:

| Range | Colour | Interpretation |
|-------|--------|----------------|
| `≥ 0.70` | green  | model is confident — polyline safe to use as-is |
| `0.30–0.70` | yellow | borderline — polyline present but position ambiguous |
| `< 0.30` | red    | untrusted — polyline may still be non-empty but not to be believed |

`0.00` during junctions.

**Expected confidence bands with the current models.** Confidence is a
function of both the *scene* and how well the *model* was trained for
that scene. Rough guide:

| Scene | `UFLD_F1=0.87.pth` (old, 15 K) | `UFLD_F1=0.67.pth` (new, 75 K) |
|-------|--------------------------------|--------------------------------|
| Town04 clear straight | 0.45 – 0.55 | expected 0.70 – 0.85 |
| Town03 gentle bend | 0.30 – 0.45 | 0.55 – 0.75 |
| Town01 clear (curbs) | ~0.15 – 0.30 (never trained on) | 0.45 – 0.65 |
| Town01 rain | ~0.10 – 0.20 (never trained on) | 0.35 – 0.55 |
| Inside a junction | 0.00 (explicit) | 0.00 (explicit) |

So-so on absolute values; the useful signal is **relative behaviour** —
the number should *fall* as UFLD gets less certain (edge cases, weather
transitions, tight bends) and *rise* on clean straights. That gradient
is real regardless of which model is loaded.

**Downstream integration (not yet wired).** The topics exist and the
overlay renders — but no controller / gate currently consumes them.
Two natural extensions if we want lane confidence to affect behaviour
rather than just informing the operator:

- **`perception_node` centerline gate.** Currently gates on "polyline
  has ≥ 2 points". Could additionally require `conf > THRESHOLD` on
  both lanes for the centerline to be considered trustworthy. Low
  confidence → treat as if the polyline were empty → fall back to
  centre-strip (or drop, depending on junction state — see §16).
- **Stanley controller.** When either confidence drops below threshold,
  hand control to the bridge's pure-pursuit fallback the same way an
  empty-polyline `HOLD` already does. Softer, more graduated handover
  than the current binary "polyline present / absent" check.

Both would be small changes; deferred until the retrained checkpoint's
absolute confidence values are validated in closed-loop and a stable
threshold is picked.

**Files touched.**

- `src/perception/perception/lane_detection_node.py` — soft-logit
  extraction after `pred = self.net(tensor)`, two new `Float32`
  publishers, `annotate()` overlay, junction-branch `0.0` publish.
- No other node needs to change to *see* the topics; anything
  currently subscribed to `/LKAS/ego_lane_*` can add a matching
  `_conf` subscription with the standard `rclpy.create_subscription`
  pattern.

## 26. MORAI simulator integration [ONGOING]

**Objective.** Port this stack — previously CARLA-only — to also drive an
ego vehicle in MORAI SIM: Drive / MORAI World: Automotive (Scenario
Studio, ROS2 Interface panel), on a freshly cloned machine. Everything
below happened in one long session; recorded here as the memory of what
was actually done and why, since a lot of it was empirical/iterative.

### 26a. New-machine environment setup [DONE]

Fresh clone had none of the runtime installed. Fixed, in order:

- ROS 2 Humble (desktop) + colcon + rosdep via the standard `ros2-apt-source`
  install flow. `python3-tk`, `python3-pip` also missing (needed for
  `UI.py` and pip respectively).
- PyTorch: the RTX 5080 is Blackwell (`sm_120`) — needed the `cu128` wheel
  index (`pip install torch torchvision --index-url
  .../whl/cu128`), not a bare `pip install torch`. Verified with a real
  on-GPU matmul, not just `torch.cuda.is_available()`.
- `ultralytics`, `opencv-python`, `numpy<2` per README, plus **undocumented
  transitive deps of the external UFLD-V2 repo** (`addict`, `tqdm`,
  `tensorboard`, `scikit-learn`, `pathspec`, `imagesize`, `ujson` — from
  that repo's own `requirements.txt`, not this one).
- Two more external, hard dependencies not in this git clone at all,
  found by grepping for the old machine's `/home/sirius/...` paths still
  hardcoded in source:
  - `Ultra-Fast-Lane-Detection-V2` — `lane_detection_node.py`'s
    `ufld_repo` param does `sys.path.insert` + `importlib.import_module`
    into it directly. Found a real clone of it already on this machine
    at a different path; updated the default.
  - `carlaaccsim` (the custom CARLA↔ROS bridge, referenced from `UI.py`'s
    `BRIDGE_DIR`) — genuinely absent, out of scope since this session is
    MORAI-only. `UI.py`'s CARLA-only buttons (Start CARLA/Bridge, etc.)
    remain unusable here as a result; not fixed, not needed.
- Two pre-existing bugs surfaced by trying a truly fresh run, unrelated
  to the machine move: `controller/package.xml` never declared
  `example_interfaces` despite the code importing it; `lane_detection_node`'s
  `model_filename` default (`UFLD_best.pth`) never matched either real
  checkpoint on disk (`UFLD_F1=0.67.pth` / `UFLD_F1=0.87.pth`).

### 26b. MORAI's ROS2 Interface — architecture is different from CARLA's, in a good way [DONE]

CARLA has no native ROS2 publishing, hence `carlaaccsim` had to be a full
bridge (spawn actors, grab frames, encode, publish). **MORAI SIM natively
publishes/subscribes ROS2 topics** via a GUI "Network Interface" panel
(Layer/Interface tabs in the Scenario view) using pre-built Message
Templates bound to sensor/vehicle entities. No bridge process needed for
sensors — only a translator for the couple of places MORAI's message
shapes don't match ours.

Two categories of template, easy to conflate:
- **Non-`ROS2`-prefixed templates** (`Camera RGB`, `Vehicle Manual Control`,
  `TransformControl`, ...) are MORAI's native/UDP protocol — not used here.
- **`ROS2 ...`-prefixed templates** map to real ROS2 types. Some are
  fully standard (`ROS2_CompressedImage` → `sensor_msgs/msg/CompressedImage`,
  `ROS2_Odometry` → `nav_msgs/msg/Odometry`, `ROS2_Imu`, `ROS2_NavSatFix`)
  — decodable immediately, no extra package needed. **`ROS2 VehicleInfo`
  and `ROS2 Vehicle Manual Control` are not standard** — both serialize
  as `morai_v2_1_ros2_msgs/msg/...`, a custom package MORAI does not
  publish the source for on this product line (only compiled Windows
  `rosidl` typesupport `.dll`s ship with the simulator — useless for a
  Linux build). Confirmed this isn't the same as the public
  `MORAI-Autonomous/MORAI-ROS2_morai_msgs` GitHub repo either (different
  message set entirely, e.g. no `VehicleInfo`/`VehicleManualControl` in
  that repo's list) — this one seems to be gated behind a MORAI account
  SDK download we didn't chase down.

**Consequence for design:** avoid the custom-message templates wherever
a standard one covers the same data.
- **Vehicle speed**: skipped `ROS2 VehicleInfo` (custom message) entirely.
  Used `ROS2_Odometry` (standard `nav_msgs/Odometry`) instead, bound to
  an **IMU sensor entity** (`IMUEntity`, not the vehicle entity directly
  — confirmed via MORAI's own "Sensor Data Transmission" doc page, which
  is sensors-only and doesn't cover control at all). `twist.twist.linear`
  gives velocity directly.
- **Control**: no standard-message alternative exists for
  throttle/brake/steer — `ROS2 Vehicle Manual Control` (custom message)
  was unavoidable here. Got its field layout from MORAI's own Message
  Template editor UI (not docs): `throttle`, `brake`,
  `steering_wheel_angle`, all `float64`. Hand-authored a matching
  `.msg` in a local `morai_v2_1_ros2_msgs` package (see §26c) — a
  legitimate technique when the source isn't available, since ROS2/DDS
  message compatibility is structural (package + message name + field
  layout), not centrally registered.

### 26c. New packages: `morai_v2_1_ros2_msgs` + `morai_bridge` [DONE]

- **`morai_v2_1_ros2_msgs`** (`ament_cmake`, message-only) —
  `msg/VehicleManualControl.msg`:
  ```
  float64 throttle
  float64 brake
  float64 steering_wheel_angle
  ```
  Reconstructed from MORAI's Interface editor field table, not from any
  published source (see §26b). If MORAI ever publishes the real package,
  replace this with it.
- **`morai_bridge`** (`ament_python`) — two adapter nodes, translating
  between MORAI's topics and this stack's existing simulator-agnostic
  `/Car_1/*` topics:
  - `state_adapter_node`: subscribes `<odom_topic>` (default
    `/Car_1/odometry`, `nav_msgs/Odometry`), publishes
    `speed = sqrt(vx²+vy²+vz²)` as `example_interfaces/msg/Float64` on
    `/Car_1/vehicle/speed` — the exact type `controller_node`/
    `stanley_node` already expected in `simulator:=morai` mode (that
    branch pre-dated this session; only the publisher side was missing).
    For this to be the true vehicle-origin speed (no `ω × r` lever-arm
    skew from yaw/pitch/roll rate), the IMU sensor entity must sit at
    the vehicle's own local origin: **Position (0,0,0), Orientation
    (0,0,0)** — the asset-library default position was *behind the rear
    bumper*, outside the car body entirely; had to be corrected.
  - `control_adapter_node`: subscribes `/Car_1/cmd_vel` (Twist:
    `linear.x`=throttle, `linear.y`=brake) + `/Car_1/cmd_steer`
    (Float32, normalised steer), publishes
    `morai_v2_1_ros2_msgs/VehicleManualControl` on `/Car_1/control`.
    Exposes `throttle_scale`/`brake_scale`/`steer_to_wheel_angle_deg`/
    `steer_sign` as ROS params specifically because none of MORAI's
    exact units/ranges/sign convention for this message were documented
    anywhere — see §26f for the empirical calibration history.
- `start_adas.sh` (renamed from `start_acc.sh`, see §26i) launches both
  automatically when `$SIMULATOR = morai`; `UI.py` also got matching
  "Start/Stop MORAI Bridge" buttons.

### 26d. Camera calibration [DONE]

`perception_node`/`lane_detection_node`'s IPM math hard-assumes a pinhole
model with **zero pitch/roll** (image vertical centre = the horizon,
exactly) and `focal_px = width / (2·tan(FOV/2))`, i.e. **`focal = width/2`
at 90° FOV** for any resolution — this is CARLA's own convention too, so
no code changes were needed for MORAI, only getting the physical/MORAI
side to match:

- Camera resolution is a free choice — the nodes compute focal
  dynamically per incoming frame width, nothing hardcoded. Started at
  1080×720 (focal 540), later changed to 1280×720 (focal 640) — both
  valid, since `640 = 1280/2` and `540 = 1080/2` both satisfy the 90°
  rule. (Worth flagging: 1280×720/focal=640 is *not* actually "the CARLA
  setting" as believed when made — that pairing is CARLA's old,
  superseded default from before `§10`'s bump to 1920×1080/focal≈960;
  it just happens to be internally self-consistent for our formula
  regardless, so the change was harmless.)
- Physical placement: `cam_x_offset` and `cam_height_m` got MORAI-specific
  defaults (`0.75 m` / `0.9 m`, vs. CARLA's `0.6 m` / `1.35 m`) via a
  `simulator` ROS param already threaded through `perception_node` and
  `lane_detection_node` — set by directly measuring where the user
  placed `Camera_1` in MORAI, then validated by capturing a live frame
  and checking the horizon sits at the image's vertical centre (it did,
  within ~1% of frame height).
- MORAI's Camera_1 "Focal Length" field is the actual tunable parameter;
  "Horizontal/Vertical FOV" are read-only, computed from it. Initially
  left at an asset-library default (320 px at 1080 width → 118.7° FOV,
  not 90°) — silently wrong until corrected to 540.

### 26e. Vehicle state feedback — recurring root cause of `v=0.00 m/s` [FIXED, multiple rounds]

`stanley_node`/`controller_node` showed `v=0.00 m/s` persistently across
several test rounds, each with a different cause:
1. **Message-type mismatch.** `controller_node.py` already branched
   `example_interfaces/Float64` vs `std_msgs/Float64` on a `simulator`
   param that pre-dated this session — but **`stanley_node.py` had no
   such branch at all**, and always subscribed `std_msgs/Float64`,
   which can never match our `example_interfaces/Float64` publisher.
   Added the same `simulator` branch to `stanley_node`.
2. **UI dropdown defaulting to `carla`.** Even after fixing (1), the
   UI's Simulator selector defaulted to `'carla'`, so `Run start_adas.sh`
   / ACC / LKAS toggles kept launching nodes in CARLA mode regardless of
   what MORAI was actually doing. Flipped the UI default to `'morai'`
   and added a loud `### LAUNCHING WITH SIMULATOR = X ###` log line at
   all three launch points so this is never silently wrong again.
3. **`start_adas.sh` itself never passed `-p simulator:=$SIMULATOR` to
   `stanley_node`** (see §26i) — every other node in the script got it;
   this one was missed when the flag was originally added. This alone
   meant `stanley_node` ran fully CARLA-tuned (`stanley_k=0.5`,
   `heading_gain=1.0`, `rate=20Hz`, old HOLD behaviour) every single time
   the stack was launched via the script or the "Run start_adas.sh"
   button, regardless of the script's own `morai` argument — likely
   responsible for a good fraction of the steering behaviour that got
   mis-diagnosed as a gain-tuning problem in §26f/§26g before this was
   found.
4. **UI's own internal speed display** (`TelemetryView`, powers the
   "Speed:" label) had *yet another* independent hardcoded
   `std_msgs/Float64` subscription, never touched by any of the above
   fixes since it's a separate node from `controller_node`/`stanley_node`.
   Attempted a "subscribe with both types" fix first — **ROS2 rejects
   two different message types on the same topic name within one node**
   (`rcl` raises "invalid allocator", not a graceful error) — reverted to
   picking one type at construction time, read from the dropdown's value
   at UI startup (a real remaining limitation: if you flip the dropdown
   *after* `UI.py` has already started, this one display won't pick up
   the change without a restart; the actual control nodes aren't
   affected, they read the dropdown fresh at their own launch time).

Given how many independent causes produced the identical `v=0.00`
symptom, treat that specific log line with suspicion in future — check
`ros2 param get <node> simulator` directly rather than inferring from
behaviour.

### 26f. Control calibration — throttle [FIXED] and steering [ONGOING]

**Throttle.** `cruise_control()`'s gain/cap needed MORAI-specific values
distinct from CARLA's, added as `ACC_GAIN_SCALE_MORAI` (10x softer
`k_p`/`k_d`) and a separate `cruise_throttle_cap`. Iteration history:
- First MORAI runs: throttle pinned at 1.0 constantly. Root cause (see
  §26e #2/#3) was CARLA-strength gains + broken speed feedback, not
  MORAI needing gentler tuning per se — but genuinely-softer gains were
  still wanted regardless once feedback was fixed.
- `cruise_gain` (the cruise-mode P-gain) at CARLA's `0.3` scaled down
  10x to `0.03` alongside `k_p`/`k_d` — too weak, `throttle ≈ 0.042` at
  the typical ~1.4 m/s standstill error, not enough to move the car.
  Reverted `cruise_gain` to full CARLA strength (`0.3`) — safe because
  `cruise_throttle_cap` (started `0.2`, later raised to `0.8` on request)
  is the actual ceiling now, letting errors *reach* the cap instead of
  topping out at a fraction of it, while still tapering smoothly as
  `v_ego → target`.
- Confirmed working: steady, bounded throttle instead of saturating.

**Steering — still not converged, several rounds of tuning:**
- `steer_to_wheel_angle_deg` (the adapter's output scale): `450° → 60° →
  6° → 0.6° → 0.1°`, each step still "too sharp" until suddenly, at
  0.1°, the opposite complaint: barely any physical effect, "just
  steers slightly then holds."
- `steer_sign`: confirmed empirically the car turned *left* on a
  positive `steer_norm`, contradicting the "positive = right"
  convention everywhere else in this stack. Flipped to `-1.0`.
- **Stanley's own gain had no MORAI override at all** until this
  session — `STANLEY_K` (cross-track) and a *newly split-out*
  `STANLEY_HEADING_GAIN` (heading error had *always* had a fixed,
  unscaled weight of `1.0` in both formulas — only cross-track ever got
  a tunable gain). Both converted from hardcoded module constants to
  real ROS params (`stanley_k`, `stanley_heading_gain`) with
  simulator-conditional defaults, specifically so future tuning doesn't
  need a rebuild. Current MORAI values: `stanley_k=15.0` (30x CARLA),
  `stanley_heading_gain=3.0` (CARLA stays `1.0`, unscaled/canonical).
- **Open theory, not yet confirmed or refuted**: MORAI's
  `steering_wheel_angle` field may not be an absolute target angle at
  all — evidence (MORAI's own HUD showing `107.76°` against a commanded
  `0.239`, wildly mismatched) suggests it could be a per-tick
  increment/rate that accumulates. Proposed test, not yet run: publish
  a constant nonzero value for several seconds and watch whether the
  HUD angle settles (absolute) or keeps climbing (incremental). This
  would mean everything scale/gain-related above is fighting the wrong
  variable, and a closed-loop rate controller (using MORAI's own
  steering feedback, which would require the `VehicleInfo` custom
  message after all) might be the real fix.
- Given how many knobs are now in play simultaneously (`stanley_k`,
  `stanley_heading_gain`, `steer_to_wheel_angle_deg`, `control_rate_hz`),
  and that §26e #3 means much of the earlier tuning happened against a
  half-broken launch path, recommend re-baselining from the current
  values as a set rather than continuing to adjust one at a time.

### 26g. HOLD mode has no MORAI fallback — "steers slightly, then holds a stale angle" [FIXED]

Root cause turned out to be architectural, not a gain problem, despite
initially looking identical to the steering-sensitivity issue above.
`stanley_node.control_loop()` deliberately **does not publish**
`/Car_1/cmd_steer` while in HOLD (UFLD lost the lane) — by design, for
CARLA, because the separate `carlaaccsim` bridge has its own
pure-pursuit fallback that takes over the instant Stanley goes silent.
**MORAI has no equivalent fallback.** So whenever UFLD's lane lock
flickered (frequent on MORAI's imagery — lots of `LOW ↔ FEW ↔ REJ`
churn in the Kalman-filter log), Stanley would go quiet and
`control_adapter_node` would just keep re-publishing whatever steer
value it last received, frozen, for as long as HOLD lasted — the car
appeared to "steer slightly, then hold," actually driving in circles on
a stale angle rather than continuing to correct.

**Fix.** MORAI now publishes `0.0` (wheel straight) during HOLD instead
of going silent (`if self.simulator == 'morai': publish 0.0`). Not
"correct" if genuinely mid-turn when HOLD triggers, but far safer than
perpetuating an arbitrary frozen nonzero angle indefinitely. CARLA's
original silent behaviour is completely unchanged. Verified live both
ways (continuous `0.0` stream for MORAI with no lane data; genuinely
zero messages for CARLA in the same test).

**Diagnostic takeaway**: "which ROS values to check" for this class of
symptom is `/LKAS/ego_lane_left` / `/LKAS/ego_lane_right` (empty/short
whenever HOLD triggers) and the Kalman filter state-transition log, not
a control gain.

### 26h. `UI.py` changes [DONE]

- **Simulator selector** (`carla`/`morai` dropdown, defaults `morai`)
  added to the Processes panel, threaded through all three launch paths
  (`run_start_adas`, ACC toggle, LKAS toggle) plus a loud log line at
  each. See §26e #2.
- **Start/Stop MORAI Bridge** buttons — launch/stop
  `state_adapter_node` + `control_adapter_node` independently of the
  rest of the stack, for iterating on calibration without a full
  restart.
- **Empty-`CompressedImage` crash** — `cv2.imdecode` raises a hard C++
  assertion (not a graceful `None`) on a zero-length buffer, unlike a
  merely-malformed-but-nonempty one. Some debug topic sent one at least
  once, and since `_render_tick` only reschedules itself via
  `root.after()` at the very end of the function, this **permanently
  froze the entire camera view** for the rest of the session on one bad
  frame. Fixed with a truthiness guard (`if jpeg and ...`) at both
  decode sites (main camera view + BEV panel). Reproduced the exact
  crash and confirmed the fix survives it, then confirmed it still
  renders a real subsequent frame correctly (not just "doesn't crash").
- **Internal speed-display type mismatch** — see §26e #4.

### 26i. `start_adas.sh` — renamed from `start_acc.sh`, and a real bug found while renaming [DONE]

Renamed via `git mv` (all references updated: `UI.py`'s `START_ADAS_SH`
constant, `run_start_adas` method name, button label, log strings;
`README.md`; `DEBUG.md`'s own historical entries left as-is —
they're a record of what was called at the time). While doing the
rename and re-auditing the file end-to-end, found and fixed the
`stanley_node` missing `-p simulator:=$SIMULATOR` bug described in
§26e #3 — a good example of why a "just rename it" task is worth
actually reading the file being renamed.

### 26j. WSLg blank-window rendering bug [ENVIRONMENT, not this codebase]

Recurring, not a one-off: `python3 UI.py` (and even a bare
three-line Tk test window) sometimes renders as a blank window — taskbar
icon present, no pixel content. `/mnt/wslg/stderr.log` shows
`Xwayland glamor: GBM Wayland interfaces not available / Failed to
initialize glamor, falling back to sw` at Xwayland startup each time.
Confirmed NOT a UI.py/Tkinter bug (the minimal test window fails
identically). `wsl --update` and the NVIDIA driver were both already
current when this recurred, ruling those out as the fix. Leading
open theory: **GPU contention at Xwayland's one-time startup
negotiation** — MORAI's `MoraiSimulator-Win64-Shipping.exe` was measured
at 79-99% GPU utilization on the Windows host when this happened, and
Xwayland's glamor init needs a GPU handshake even before falling back
to software. `wsl --shutdown` (full VM restart) reliably clears it,
but the *why it fails on a fresh boot sometimes and not others* isn't
fully confirmed. Proposed, not yet validated: launch `UI.py` **before**
starting MORAI's scenario, so Xwayland's one-time init happens while
the GPU is idle. Does not block ADAS/MORAI work either way — everything
runs fine from the command line, and Foxglove Studio (browser-based,
doesn't go through WSLg's compositor) covers visualization in the
meantime.

### Open issues / next steps

- Steering calibration unconverged (§26f) — re-baseline against the
  now-fixed launch path (§26e #3) before further tuning; the
  rate-vs-absolute-angle test is the single highest-value next
  experiment.
- UFLD lane-lock stability on MORAI's imagery is poor enough to trigger
  HOLD often (§26g) — may need its own Kalman/confidence-threshold pass
  once steering is stable, independent of the MORAI port itself.
- Camera publish rate inconsistent (measured anywhere 3-60 Hz against a
  60 Hz then 20 Hz MORAI-side config) — RTF was confirmed fine (~1.0)
  when checked, so it's not a global sim-speed problem; likely the
  camera sensor's own capture/encode pipeline specifically. "Record
  data" toggle and resolution were flagged as untried cheap experiments.
- `Start MORAI Bridge` can spin up duplicate `state_adapter_node`/
  `control_adapter_node` instances if clicked again without "Stop"
  first (happened 3+ times this session) — `start_morai_bridge()` only
  checks its own in-memory tracking, not actual OS processes. Not yet
  hardened.
- `carlaaccsim` and its `UI.py` integration (Start CARLA/Bridge buttons,
  `CARLA_DIR`/`CARLA_PYTHON`/`BRIDGE_DIR` paths) remain untouched,
  still pointing at the old machine's `/home/sirius/...` paths —
  out of scope while MORAI-only, but will need fixing if CARLA testing
  resumes on this machine.

## 27. Second new-machine move — CUDA-13 default wheel + colcon/setuptools/packaging conflict [FIXED]

**Objective.** Same class of problem as §26a (`sirius@PC-ACM-01`, RTX
4090, driver 560.94 / CUDA 12.6), but on yet another fresh Ubuntu 22.04
box with *nothing* installed — no ROS 2, no pip, no rosdep. Two new
gotchas surfaced that weren't hit last time, both worth remembering
since neither produces an obviously-related error message.

**1. `pip install torch` silently grabs a CUDA build newer than the
driver supports.** Unlike the Blackwell 5080 case in §26a (which
needed a *special* `cu128` index to get a working build), this time
the *default* PyPI wheel was the problem: a bare `pip install torch
torchvision torchaudio` resolved to **torch 2.13.0+cu130** (CUDA 13
runtime bundled). `torch.cuda.is_available()` returned `False`, and
the real error only appears if you actually try a GPU op:
`RuntimeError: The NVIDIA driver on your system is too old (found
version 12060)` — i.e. the driver reports CUDA 12.6 as its ceiling,
and CUDA-13 wheels refuse to run on it. `is_available()` alone hides
this behind a plain `False`; the fix is to always follow §26a's
practice of testing with a real `torch.randn(...).cuda()` matmul, not
just the boolean check. **Fix:** uninstall the `+cu130` torch/
torchvision/torchaudio and every `nvidia-*-cu13`/`cuda-*` package pip
pulled alongside them, then reinstall pinned to the matching index:
`pip install --index-url https://download.pytorch.org/whl/cu126 torch
torchvision torchaudio`. Verified with a real 4096×4096 matmul on
`cuda:0` afterward. **Takeaway for future machines:** check
`nvidia-smi`'s reported "CUDA Version" ceiling *first*, then pick the
matching `/whl/cuXXX` index explicitly — never trust the bare
`pip install torch` default to match the installed driver.

**2. `colcon build` breaks after installing PyTorch, with an error
that looks nothing like a PyTorch problem.** Symptom: every
`ament_python` package failed identically —
`TypeError: canonicalize_version() got an unexpected keyword argument
'strip_trailing_zero'`, thrown from deep inside `setuptools`'
`_core_metadata.py` during `egg_info`. Root cause: `torch`'s own
install pulled `setuptools` up to `83.0.0` as a transitive dependency
(pip warned about this at install time: `colcon-core 0.21.0 requires
setuptools<80,>=30.3.0, but you have setuptools 83.0.0` — easy to miss
among normal install-log noise). Downgrading `setuptools` to satisfy
that constraint (`pip install "setuptools<80,>=30.3.0"`) fixed the
version bound but **not** the actual crash — the real second half of
the problem was that this machine's only importable `packaging` module
was the apt-installed system one (`python3-packaging` 21.3, pulled in
as a `colcon-core`/`catkin-pkg` dependency, living in
`/usr/lib/python3/dist-packages`), and even `setuptools` 79.x's
`_core_metadata.py` calls `packaging.utils.canonicalize_version()` with
a `strip_trailing_zero` kwarg that doesn't exist until `packaging`
≥24.1. **Fix:** `pip install --upgrade packaging` (user site-packages
takes priority over the apt one on `sys.path`, so this alone was
enough — no need to remove the apt package). After both fixes,
`colcon build` succeeded cleanly across all four packages
(`morai_v2_1_ros2_msgs`, `controller`, `perception`, `morai_bridge`).
**Takeaway:** any `pip install` that touches `setuptools` on a machine
that also runs `colcon` is a latent build breaker — worth a quick
`colcon build` smoke test immediately after big Python dependency
installs, not just at the end of setup.

**3. Stale absolute path for the UFLD-V2 repo.**
`lane_detection_node.py`'s `ufld_repo` parameter default still pointed
at the *previous* new-machine's fix from §26a
(`/home/moore/workspace/01_CV_Models/...`) — itself already a
one-machine-old patch. Updated the default to this machine's path,
`/home/sirius/workspace/01_CV_Models/01_CV_Models/01_Ultra_Fast_Lane_Detection_V2/Ultra-Fast-Lane-Detection-V2`,
once the user copied that folder over. **Takeaway, now twice-confirmed:**
this hardcoded default will need updating on every future machine move
until it's made relative/configurable — worth doing before the third
occurrence.

**4. `RLD_best.pth` — a second, unwired model.** The user also copied
in `src/perception/models/RLD_best.pth` alongside a source repo,
`01_CV_Models/02_RLD/Robust-Lane-Detection` — "RLD" = **Robust Lane
Detection from Continuous Driving Scenes** (Zou et al., TVT 2019;
SegNet/UNet-ConvLSTM). Confirmed via grep that **no node in this
codebase currently loads it** — `README.md` lists the filename under
`models/` but nothing does `importlib`/`torch.load` on it. Its own
`README.md` requires **PyTorch 0.4.0, Python 3.6, CUDA 8.0** — a
completely different, incompatible stack from the PyTorch 2.13/cu126
environment set up here for UFLD-V2 and YOLO. Left untouched at the
user's direction ("focus on UFLD and YOLO"); if RLD is ever wired in,
it'll need its own isolated environment (venv/conda), not a shared
install.

**Environment snapshot for reference:** Ubuntu 22.04.5 (WSL2 kernel
`6.18.33.2-microsoft-standard-WSL2`), RTX 4090, driver 560.94 (CUDA
12.6 ceiling), Python 3.10.12, ROS 2 Humble via the `ros2-apt-source`
flow, torch 2.13.0+cu126.

## 28. Startup safety gate — throttle held at 0 until YOLO + UFLD are loaded [DONE]

**Objective.** First live MORAI run on the new machine (see §27) raised
a separate concern independent of the `v=0.00` speed-feedback bug being
chased in parallel: `controller_node` starts commanding cruise throttle
immediately on launch, with no awareness of whether `perception_node`
(YOLO) or `lane_detection_node` (UFLD) have actually finished loading
their models yet — UFLD in particular can take 10-30s to load its 1.7 GB
state dict (see its own startup log line). Requested: the car should not
move at all until both perception models are confirmed ready.

**Design choice.** Considered and rejected a fixed `sleep N` in
`start_adas.sh` before allowing the stack to move — model load time
varies by machine/GPU (see §27's own environment differences across
two machines already), so any fixed delay is either too short
(race) or wastefully long. Implemented instead as a data-driven ROS
handshake:
- `perception_node` and `lane_detection_node` each publish a `Bool` on
  `/ACC/perception/model_ready` / `/LKAS/perception/model_ready` the
  instant their model load call returns — `TRANSIENT_LOCAL` durability
  (QoS depth 1, `RELIABLE`) so a `controller_node` that starts (or is
  restarted) *after* that publish still receives it, instead of
  depending on subscribe-before-publish ordering.
- `controller_node` subscribes to both with matching QoS, tracks
  `yolo_ready`/`ufld_ready`, and added a new MODE 0 at the very top of
  `control_loop()`: while either is `False`, publish `throttle=0,
  brake=0` and return, before any of the existing CRUISE/ACC/EMERGENCY
  logic runs. Logs which model(s) it's still waiting on (throttled to
  once per 2 s) and a one-time "throttle unlocked" line on the
  transition.

**Verified**, standalone (without the GPU-heavy perception nodes
running, using `ros2 topic pub` to simulate them):
1. `controller_node` alone, before any ready flags: `/Car_1/cmd_vel`
   confirmed `{throttle: 0, brake: 0}`.
2. Publishing both ready flags mid-run: throttle immediately unlocked
   to real cruise output (`0.417`, matching the MORAI cruise-gain math
   for the standstill speed error) within the next control tick.
3. **Late-subscriber case** (the actual point of `TRANSIENT_LOCAL`):
   started two latching publishers first, waited 2 s, *then* started
   `controller_node` — it picked up both flags within ~15 ms of
   starting, proving the gate is robust regardless of node launch
   order (script vs. UI toggles vs. manual `ros2 run`, any order).

**Scope note.** Only throttle is gated, matching what was asked for.
Steering (`stanley_node`) doesn't need the same treatment — it already
degrades safely on its own (`HOLD`/no lane data) whenever UFLD hasn't
published a lane yet, gate or no gate.

## 29. MORAI's IMU never populates linear velocity — switched speed source to GroundTruth VehicleInfo [FIXED]

**Objective.** `/Car_1/vehicle/speed` read a constant `0.0` on every
live MORAI run, even with the car visibly moving (§27's `v=0.00`
symptom, revisited here with the actual root cause).

**Root cause, confirmed live via Foxglove.** `/Car_1/odometry`'s
`pose.orientation` was genuinely live and updating every tick (proving
the topic itself wasn't dead), but `pose.position` was bit-for-bit
frozen at `(1.5, 0, 0.5)` and `twist.linear` frozen at exactly
`(0,0,0)` — across multiple separate sim sessions, regardless of real
vehicle motion, while `twist.angular` showed real noise-level values.
MORAI's `IMU_1` entity ("MA IMU" sensor model) computes orientation
(gyro integration) but never computes translational position/velocity
at all — not a ROS binding issue, a sensor-model limitation. Physically
sensible too: a raw IMU only measures angular rate + linear
acceleration; velocity requires integration, which this sensor model
apparently doesn't do.

**Fix.** Replaced `IMU_1` with a `GroundTruth` entity (`MA GT`,
`GroundTruth_1`, `Position (0,0,0)`), which reports the simulator's own
exact internal vehicle state directly rather than reconstructing it
from an incomplete sensor. Bound to MORAI's `ROS2 VehicleInfo`
template — a **second** custom message (like `VehicleManualControl` in
§26c), field layout captured directly from MORAI's Message Template
editor (21 fields: int32/uint32 timestamp, string `id`, then float32
triples for location/rotation/local_velocity/local_acceleration/
angular_velocity, then float32 throttle/brake/steer_angle telemetry).
Added `morai_v2_1_ros2_msgs/msg/VehicleInfo.msg` matching that exact
order/types (same structural-compatibility technique as §26b: package
+ message name + field layout, no central registry). Topic: MORAI's
`GT` interface publishes `/Car_1/vehicleinfo` at 20 Hz.
`state_adapter_node` now subscribes to that instead of `Odometry`.

**A second, independent bug surfaced once real data was flowing**:
live capture showed `local_velocity_y` coming through as garbage
(`~3.7e19`) while `local_velocity_x` (forward) and `_z` decoded as sane
numbers — some corruption specific to that one axis in MORAI's own
serialization, not something traced further. Sidestepped rather than
chased: `state_adapter_node` now publishes `speed = |local_velocity_x|`
(the vehicle-frame forward component only), not the 3-axis vector
norm the original Odometry-based code used. This is also more
*correct*, not just a workaround — a real speedometer reads
longitudinal speed, not full 3D velocity magnitude, so lateral/vertical
components shouldn't have been part of "speed" regardless of the
corruption.

**Verified**: live Foxglove capture after the fix showed
`/Car_1/vehicle/speed.data = 2.944537401199341`, matching
`local_velocity_x` exactly, tracking real vehicle motion for the first
time in the MORAI port.

**Caveat**: the bit-for-bit-frozen-across-88-samples pattern that
tipped us off to the IMU bug was seen once on `VehicleInfo` too, on a
short steady-cruise window — not yet conclusively distinguished from
"genuinely constant speed for a few seconds on a straight empty road."
Worth actively varying speed/steering while watching
`local_velocity_x` before fully trusting it under all conditions.

## 30. MORAI brake leaves the vehicle stuck — cruise mode coasts instead of braking on MORAI [FIXED]

**Objective.** Once §29 made real speed feedback available, ACC's
cruise controller started doing real closed-loop work — and
immediately revealed that whenever it commanded a nonzero brake on
MORAI, the vehicle would not resume moving properly afterward,
unlike CARLA where proportional braking behaves as expected. Testing
`/ACC/target_speed` become unworkable: any speed_error negative enough
to cross the `-0.5 m/s` deadband triggered brake, and the vehicle got
stuck.

**Fix.** `cruise_control()`'s over-target-speed branch now checks
`self.simulator` (newly stored as `self.simulator` in `__init__`,
previously only a local variable): for `morai`, sets
`throttle=0, brake=0` (coast down naturally) instead of the
proportional-brake formula; `carla` keeps the original behaviour
unchanged. Deliberately scoped narrowly — EMERGENCY (collision-
imminent full brake) and ACC's own lead-distance PD brake are
untouched on both simulators, since those are safety-critical
regardless of this MORAI-specific brake quirk. Only the
"maintain/return to target cruise speed" case coasts instead of
braking on MORAI.

**Verified** standalone: publishing a fake `v_ego=3.0 m/s` against the
default MORAI cruise target (`1.39 m/s`, well past the deadband)
produced `throttle=0, brake=0` post-fix, versus the ~`0.47` the old
formula gave for the same inputs.

**Caveat / follow-up**: coasting-only means the ACC-mode "respect
CRUISE_SPEED_KMH as an upper cap" logic (the `max(acc_brake,
cruise_brake)` blend, see its own comment in the code) no longer
actively brakes to enforce that cap on MORAI when following a distant
lead — only ACC's own lead-distance brake still applies. Accepted
tradeoff for now; the root MORAI brake behaviour itself
(why does *any* brake leave the vehicle stuck?) hasn't been
investigated — this works around it rather than fixing the underlying
cause, which may need a MORAI-side (Vehicle Control Attribute /
physics) explanation.

## 31. Duplicate MORAI bridge adapter processes — real OS-process guard added [FIXED]

**Objective.** `ros2 node list` repeatedly showed `Morai_State_Adapter`
and `Morai_Control_Adapter` duplicated (2x, then observed as high as
4x across this session) — multiple independent `state_adapter_node`/
`control_adapter_node` processes simultaneously racing each other over
`/Car_1/vehicle/speed` and `/Car_1/control`. This was a live,
significant contributor to the session's erratic/inconsistent-seeming
control behaviour — at times, two of the four running duplicates were
confirmed to still be executing **stale, pre-fix code** from before
the §29/§30 changes, alongside newer correctly-patched instances, all
publishing to the same topics at once.

**Root cause.** Two independent launch paths both start these same two
nodes with zero awareness of each other or of processes outside their
own tracking:
- `start_adas.sh` launches them unconditionally whenever
  `SIMULATOR=morai` (see §26c).
- `UI.py`'s `start_morai_bridge()` (the "Start MORAI Bridge" button)
  launches them separately, guarded only by
  `self.morai_bridge_procs` — an **in-memory** Python list local to
  that one `UI.py` process.

Doing both (a completely natural sequence — e.g. clicking "Start MORAI
Bridge" for telemetry, then also "Run start_adas.sh" for the rest of
the stack, not realising the script *also* starts the bridge) produces
2 duplicates in a single clean session — already flagged as an open
issue in §26. Worse: `_popen()` uses `start_new_session=True`, so these
child processes are **not** killed when `UI.py` itself is closed
without clicking Stop first — they become orphans, invisible to a
freshly-started `UI.py`'s empty `self.morai_bridge_procs`, explaining
how this reached 4x duplicates (two separate UI sessions' worth of
orphans, plus a fresh start on top).

**Fix.** Replaced in-memory-only tracking with a real OS-process check
in both launch paths:
- `UI.py`: new `_morai_bridge_pids()` static method (`pgrep -f` against
  each node name, catching both the `ros2 run` wrapper and the actual
  entry-point process regardless of which path started it).
  `start_morai_bridge()` checks this *and* refuses to launch if
  anything matches. `stop_morai_bridge()` now also sweeps and kills any
  untracked/orphaned matches via `_pkill`, not just this instance's own
  tracked children — so Stop is effective regardless of how the
  duplicates got there.
- `start_adas.sh`: same `pgrep -f` guard before its own
  `state_adapter_node`/`control_adapter_node` launch block, warns and
  skips instead of double-launching on top of an already-running
  bridge.

**Verified**: functional test launched a real adapter pair, then ran
the exact guard logic `start_adas.sh` now uses on top of it — correctly
printed the warning and did not launch a duplicate. Both files pass a
syntax check (`bash -n`, `ast.parse`).

## 32. MORAI's GroundTruth VehicleInfo local_velocity is frozen, not live — §29's caveat confirmed [KNOWN, unresolved]

**Objective.** §29 flagged an open caveat when switching to
GroundTruth-sourced speed: `local_velocity` had been observed
bit-identical across 88 samples, not yet distinguished from a
genuinely-steady few seconds of real cruising. Now conclusively
confirmed as a real bug, independent of every other issue chased this
session (duplicate processes, target_speed never being received,
`controller_node` not running, etc. — see the immediately preceding
turns of debugging that each looked like they might explain a "stuck
throttle/speed" symptom before this was isolated).

**Confirmed live**, after clearing all duplicate processes (§31) so
there was exactly one publisher in play: `ros2 topic echo
/Car_1/vehicleinfo` over a clean 5 s window returned **89 samples, all
89 bit-for-bit identical** (`local_velocity_x = 2.944537401199341`),
from a single confirmed publisher (`morai_GT`, MORAI's own native
node, not anything in this repo) using plain `VOLATILE` durability —
which rules out a ROS-side retained/latched-message artifact as the
explanation. MORAI's own HUD showed the vehicle stationary (`0 km/h`)
during this same window. The `GT` entity's `VehicleInfo` publisher is
therefore genuinely re-publishing a stale snapshot at its configured
20 Hz, not recomputing it — a MORAI-side bug/limitation in this sensor
template, not anything in `state_adapter_node` or the ROS graph.

**Downstream effect**: `/Car_1/vehicle/speed` and `UI.py`'s own
telemetry display both correctly relay whatever `local_velocity_x`
says — so both show a stuck `2.94 m/s` / `10.6 km/h` even with the car
provably stationary. Nothing to fix on the relay side; the bad data
originates at MORAI's own publisher.

**Leading theory, not yet tested**: `GroundTruth_1`'s own Configuration
panel exposes a `Detect Radius (m)` field, observed set to `0.05` —
suspiciously tiny. If this GT sensor type is fundamentally a
*nearby-object* detector (common in driving-sim ground-truth tooling,
for labelling surrounding objects) rather than a continuous
ego-vehicle telemetry source, a near-zero detect radius could mean it
detects itself/the ego exactly once at scenario start and then never
re-triggers a refresh, explaining a frozen-after-first-value pattern
without needing any ROS-side bug at all.

**Next diagnostic step, not yet done**: compare against MORAI's own
in-editor "Motion State" property panel (visible when selecting
`Car_1`, showing live `Velocity x/y/z (m/s)` numbers directly from the
sim) while the vehicle is actually moving — if that panel's numbers
visibly change in real time while the ROS-published
`local_velocity_x` stays frozen, that conclusively isolates the bug to
the `VehicleInfo` ROS publish path specifically, and points at trying
a much larger `Detect Radius` (or finding a different, dedicated
vehicle-state output) rather than continuing to use `GroundTruth`'s
`VehicleInfo` template as-is.


## 33. CARLA-side hardening session (2026-07-21) [DONE]

Series of small fixes that made the CARLA path robust to the way the
UI now orchestrates start-up. Grouped here so the individual entries
below are easier to cross-reference; each subsection can stand alone.

### 33a. `inference_skip_n` sweep 5 Hz → 20 Hz → 5 Hz [KEPT AT 5 Hz]

The KF-STAN control channel benefits from higher UFLD rates (more
measurements per second → tighter χ² gate innovations, faster
convergence after RST), so I raised `inference_skip_n` from `4`
(5 Hz) to `1` (20 Hz) with `KF_MAX_COAST_TICKS` scaled from `20` to
`80` to preserve the ~4 s wall-clock coast window. This worked
functionally but pinned one CPU core on the host running the whole
stack (CARLA server + bridge + ROS nodes + Tk UI). A second attempt
at `2` (10 Hz) also saturated CPU. Reverted both to the original
values (`inference_skip_n=4`, `KF_MAX_COAST_TICKS=20`) — that's the
regime the KF q defaults and χ² threshold were tuned against, so
behaviour returns to its validated configuration.

**Takeaway.** Higher UFLD rates are strictly better for the control
loop; the constraint is CPU headroom, not algorithm correctness. If
the host CPU ever gets more headroom (dedicated GPU box, off-loading
CARLA server to a second machine), bump `inference_skip_n` back down
and scale `KF_MAX_COAST_TICKS` proportionally.

### 33b. UI camera QoS — `qos_profile_sensor_data` [FIXED]

The UI subscribed to all five camera sources
(`/Car_1/camera/front/compressed`, `/ACC/perception/debug_image`,
`/LKAS/perception/debug_image`, `/ADAS/perception/debug_image`,
`/ADAS/perception/debug_image_kf`) and the BEV `/ADAS/ipm/debug_image`
with the default reliable + keep-last-10 QoS profile. Because the
12 Hz Tk render tick can't drain a callback queue as fast as a 20 Hz
publisher fills it, up to 10 old JPEGs would pile up per topic;
switching the Source dropdown then exposed those stale frames as
per-source lag — a source that looked "delayed" was actually just
serving from the middle of its own backlog.

**Fix.** Import `qos_profile_sensor_data` from `rclpy.qos` and pass
it in place of the depth-`10` int on every image subscription
(camera sources + BEV). Best-effort + keep-last-1 → older frames are
dropped in the middleware before they reach the callback, so
`latest_jpegs[topic]` always holds the newest received frame.

There is a residual per-source latency floor set by each publisher's
own rate (LKAS at 5 Hz means the newest LKAS frame is up to 200 ms
behind the newest Raw), but that's architectural and unrelated.

### 33c. `BRIDGE_SYNC_MODE=1` — introduced, then reverted [REGRESSED, DO NOT REAPPLY]

After the QoS + inference-rate work freed CPU headroom, CARLA
started running above real-time in async mode — Stanley couldn't
correct fast enough because the vehicle covered more ground per
control tick than in the CPU-pinned baseline. I set
`BRIDGE_SYNC_MODE=1` (via `extra_env` in `UI.start_bridge`) so
`carlaAccSimTown.py` would enable CARLA `synchronous_mode` and
`fixed_delta_seconds = 0.05`.

**What broke.** Under sync mode the bridge process spends
essentially all its CPU inside its tick loop. The bridge's own ROS
`_cmd_vel_cb` (in `custom_ROS_pub_sub.py`) then stops firing between
ticks, so CARLA advances physics forever with the last stored (zero)
throttle. Symptom: ACC controller publishing `cmd_vel.linear.x = 1.0`
into a running bridge, `/Car_1/vehicle/speed = 0.0` at 20 Hz forever,
`pgrep` showing the bridge process at 99 % CPU. `in_junction: false`,
no lead vehicle, no obvious code-level reason the throttle should
be blocked — but throttle simply never reaches CARLA.

**Fix.** Reverted: removed `extra_env={'BRIDGE_SYNC_MODE': '1'}`
from `UI.start_bridge`. Bridge is back to async mode. If the sim
runs above real-time on a beefy host, the correct lever is on the
CARLA server side (`./CarlaUE4.sh ... -benchmark -fps=20`), which
paces the server's own physics loop without changing ROS callback
scheduling. Not wired into the UI yet — TODO.

### 33d. ACC steady-state error — `CRUISE_SPEED_KMH = 25` [FIXED]

At `CRUISE_SPEED_KMH = 20.0`, the P-only cruise law
(`_cruise_control`) plus drag + rolling resistance + the 0.5 m/s
deadband landed at an effective ~15.7 km/h (`v ≈ 4.36 m/s` on every
KF-STAN log line). Not a bug; just what a proportional controller
without integral action does at any operating point where
`k · e_ss = drag(v_ss)`.

**Fix.** Bumped `CRUISE_SPEED_KMH` from `20.0` to `25.0`. This is
purely a setpoint offset to compensate for the controller's own
steady-state error; the effective operating envelope stated in the
thesis ODD (0–20 km/h) is unchanged. The variable-slider path via
`/ACC/target_speed` still overrides at runtime.

Longer-term fix (not done): add an integral term to
`_cruise_control` so the steady-state error goes to zero on its own
and this setpoint offset becomes unnecessary.

### 33e. Model-ready 1 Hz heartbeat [FIXED]

`perception_node` and `lane_detection_node` each publish
`Bool(data=True)` on `/ACC/perception/model_ready` and
`/LKAS/perception/model_ready` respectively, using the `TRANSIENT_LOCAL`
QoS so a late-joining `controller_node` still sees the retained
message. The gate in `controller_node` (§28) holds throttle at 0
until both are received.

**Race identified.** Two failure modes were leaving the gate stuck
even after both models loaded:

1. On rapid startup, DDS occasionally drops the first retained
   message before the subscriber has finished wiring — so the
   controller never gets flagged even though the publisher's
   `TRANSIENT_LOCAL` was correctly configured. This is a known
   rmw edge case with `rmw_fastrtps_cpp`, not a bug in this code.
2. The UI kills the shell-launched `lane_detection_node` ~1.5 s
   after `start_adas.sh` boots and respawns it with the UI-selected
   model (`_restart_lkas_with_ui_params`). When the first
   `lane_detection_node` dies, its retained message dies with it —
   DDS only serves retained messages while the publisher process is
   alive. If the first process died before publishing (UFLD load
   takes 3–5 s on GPU), the retained message never existed at all.

**Fix.** In both `perception_node.py` and `lane_detection_node.py`,
right after the initial `ready_pub.publish(Bool(data=True))`, add a
1 Hz republish timer:

```python
self.create_timer(1.0,
                  lambda: self.ready_pub.publish(Bool(data=True)))
```

Cheap (one `Bool` per second), idempotent, and eliminates both race
paths — no matter when the controller subscribes or how many times
the UI restarts the perception node, the gate flips within 1 s.

### 33f. `ufld_repo` doubled path — `01_CV_Models/01_CV_Models/` [FIXED]

The pulled `lane_detection_node.py`'s default `ufld_repo` parameter
was:

```
/home/sirius/workspace/01_CV_Models/01_CV_Models/01_Ultra_Fast_Lane_Detection_V2/Ultra-Fast-Lane-Detection-V2
```

Two `01_CV_Models/` segments. The path doesn't exist, so
`UFLDInference.__init__` fails immediately with
`ModuleNotFoundError: No module named 'utils'` (the class tries
`from utils.config import Config` after inserting `ufld_repo` into
`sys.path`). Symptom at runtime: `lane_detection_node` crashes on
startup before publishing `model_ready`, the controller's gate holds
throttle at 0 forever, the UI's LKAS button reads "partial" because
Stanley is alive but perception is not.

**Fix.** Deduped the path to
`/home/sirius/workspace/01_CV_Models/01_Ultra_Fast_Lane_Detection_V2/Ultra-Fast-Lane-Detection-V2`
(the folder that actually contains `utils/config.py`).

The IDE will keep flagging the `from utils.config import Config`
line as "unresolved import" — that's expected, since the module is
added to `sys.path` at runtime and static analysis can't follow it.

### 33g. Speed-topic dual-type mismatch — Stanley rebroadcast [FIXED]

`/Car_1/vehicle/speed` is published by:

- CARLA bridge (`custom_ROS_pub_sub.py`) as `std_msgs/msg/Float64`.
- MORAI adapter (`state_adapter_node`) as `example_interfaces/msg/Float64`.

`stanley_node` and `controller_node` pick their subscription type
from `-p simulator:=carla|morai` at their own launch time, so they
always match the publisher. The UI, however, decides its speed
message type at *UI-process startup* based on the value of the
Simulator dropdown at that moment. If the UI was launched with
`morai` selected but the running bridge is CARLA (or vice versa),
DDS silently refuses to deliver messages across the type mismatch —
the UI's `Speed:` label shows `— km/h` and never updates.

Attempted first fix (destroy + recreate the subscription on
dropdown-change trace) worked functionally but was hard to reason
about across UI-restart edge cases; reverted.

**Actual fix.** `stanley_node` now publishes an
`std_msgs/msg/Float32` copy of `self.speed` (m/s) on
`/ADAS/telemetry/speed_mps` every time its `speed_callback` fires.
The UI subscribes to that instead of the raw
`/Car_1/vehicle/speed` topic. Since Stanley's subscription type
already matches whichever simulator is running, and the UI's
subscription is now a fixed `Float32` regardless of dropdown state,
the mismatch surface disappears entirely.

Side-benefit: removed the `simulator: str` parameter from
`TelemetryView.__init__` and the `example_interfaces` import from
`UI.py`, so the UI process no longer has to know which simulator is
running just to render a speed number.

## 34. MORAI-specific YOLO fine-tune: dataset pipeline + labelImg PyQt5 crashes [DONE]

**Objective.** The CARLA-trained `best.pt` (4 classes: car/truck/bus/
motorcycle; 100 epochs, `yolov8n.pt` base, imgsz 1280) doesn't reliably
detect vehicles in MORAI due to the rendering/domain shift. Needed:
label MORAI frames as ground truth, fine-tune on them. Nothing for
this existed in the repo — no dataset, training script, or labeling
tool — only the CARLA weights were ever committed.

**Pipeline built** under `tools/yolo_finetune/` (gitignored):
`extract_frames.py` (samples frames from a `Raw`-source UI recording),
`auto_label.py` (runs the CARLA `best.pt` at `conf=0.25` to draft
labels + preview jpgs so correcting is "fix a box" not "label from
nothing" — encouragingly, CARLA-only `best.pt` already detected most
MORAI cars at 0.87-0.95 confidence), `make_split.py` (train/val file
lists), `train_finetune.py` (warm-starts from `best.pt`, not stock
`yolov8n.pt`, `--freeze 10` + low `lr0` to limit catastrophic
forgetting).

**Correction tool: labelImg crashed repeatedly, root-caused and
patched.** `labelImg` (1.8.6, ~2018 code) passes float pixel
coordinates into `QPainter` calls that current system PyQt5 (5.15.6)
now strictly type-checks, raising `TypeError` instead of silently
coercing. Three call sites patched in-place (`int(...)` casts):
`labelImg/labelImg.py:965` (`scroll_request`, mouse-wheel), and two in
`libs/canvas.py`'s `paintEvent` (`526`: live rectangle preview while
drawing a box — this one was fatal, corrupting the `QPainter` state
into a segfault on the next repaint; `530-531`: hover crosshair).
Label Studio was tried as an alternative (browser-based, avoids PyQt5
entirely; venv at `~/.venvs/label-studio`) but hit an unresolved
import limit; reverted to patched labelImg.

**Result.** All 416 frames (extracted at `--stride 2` from a 832-frame
recording) manually corrected. Fine-tune: 50 epochs, `freeze=10`,
`AdamW(lr=0.00125)` auto-selected, ~4 min on an RTX 4090. Val metrics
(single `car` class only): precision 0.99, recall 0.958, mAP50 0.989,
mAP50-95 0.90 — **caveat**: random split of one continuous ~1 min
recording, so adjacent near-duplicate frames likely landed on both
sides; optimistic, not a real generalization measure (confirmed later
in §35). Weights copied to `src/perception/models/best_MORAI.pt` (and
manually into the colcon-installed share dir — that copy is a plain
file, not a symlink, so needs a rebuild to stay in sync with `src/`)
and added to `OBJECT_MODELS` in `UI.py`.

**Known unfixed bug**: `auto_label.py`'s `project=` path bug (Ultralytics
silently nests any relative `project` path under its own `runs/<task>/`
rather than using it as given — confirmed via Ultralytics' own
docstring example) was fixed there (`os.path.abspath(args.out)` +
reading `results[0].save_dir` back rather than assuming the path).
**`train_finetune.py` has the identical bug and was not fixed** — its
output lands at `runs/detect/tools/yolo_finetune/runs/morai_finetune/
weights/best.pt` (repo root), not `tools/yolo_finetune/runs/
morai_finetune` as documented. Low priority; will bite again next run.

## 35. `best_MORAI.pt` showed no bounding box in the live UI — three unrelated causes, not the model [FIXED / mixed]

**Objective.** After swapping in `best_MORAI.pt` (§34), the `ACC
(YOLO)` debug view showed no bounding box around vehicles clearly
visible on-screen, across three different live MORAI test scenarios.
Each had a different root cause; none were actually about the
fine-tuned model's detection quality.

**Cause 1 — real UI bug: `Run start_adas.sh` never applied the Object
model dropdown.** `perception_node`'s `model_filename` parameter is
only set at process-launch time, and `run_start_adas()` (`UI.py`)
launched `perception_node`/`controller_node` via the raw shell script
with **zero** ROS params — the ACC-side twin of a bug the LKAS half of
the UI had already hit and fixed (`_restart_lkas_with_ui_params`),
just never mirrored for ACC. Toggling `ACC: OFF`→`ON` directly *did*
work; only the `Run start_adas.sh` path was broken.
**Fix**: factored `toggle_acc`'s launch logic into a shared
`_start_acc_procs()` (mirroring `_start_lkas_procs`), added
`_restart_acc_with_ui_params()`, wired into `run_start_adas()`
alongside the existing LKAS restart.

**Cause 2 — working as designed.** One test scenario showed a parked
car, clearly visible, outside the ego lane. The `ACC (YOLO)` debug
view only draws boxes that survive the *entire* ACC lead-vehicle gate
(class filter → `conf ≥ 0.8` → side-clip guard → ego-lane ROI /
centerline test), not a raw-detections viewer. A correctly-gated
non-lead vehicle drawing no box is expected.

**Cause 3 — wrong topic selected.** A separate test showed no box on a
car dead-center in the ego lane; the UI's `Source` combobox was set to
`Raw` (`/Car_1/camera/front/compressed`), not `ACC (YOLO)`
(`/ACC/perception/debug_image`) — boxes only ever draw onto the
latter.

**Cause 4 — real gate limitation on MORAI, worked around, later
superseded.** With `Source` corrected and LKAS nominally active, a car
dead-center in the lane still showed no box because `_centerline_at()`
requires **both** UFLD polylines to have valid data at the same
forward distance, and the right lane's Kalman filter was in steady
rejection (rural road, grass shoulder, no visible right-side paint).
`in_lane is None` (undecidable) was unconditionally dropped by design
(a deliberate CARLA-side diagnostic change — see the "FALLBACK
DISABLED" comment in `perception_node.py`), so MORAI was blind to
every lead whenever this happened, even a real one dead ahead.
**Fix, MORAI-only**: `perception_node.py` now stores `self.simulator`
and the centerline gate skips the unconditional drop on `in_lane is
None` when `self.simulator == 'morai'`, falling through to the earlier
keep-zone/lane-ROI check instead; `in_lane is False` still rejected
regardless of simulator. CARLA unchanged. **Note (§40 below)**: the
*actual* cause of the right lane's rejection streak turned out to be a
separate, more fundamental KF bug (rejections never reset, so one bad
patch of road could wedge a lane shut indefinitely) — this gate change
is still correct/kept, but §40's fix addresses the deeper problem.

**Diagnostic aside**: ran `best_MORAI.pt` directly via `ultralytics`
`model.predict()` (no gating) over the source recording at `conf=0.15`
— good detections, but same recording the fine-tuning frames came
from, so this confirms the training loop worked, not generalization to
new scenes (§34's caveat).

## 36. New-machine move (`PC-ACM-02`) — `UI.py` crash on missing `rclpy`, real degraded-mode bug found [FIXED]

**Symptom.** `python3 UI.py` in a fresh terminal on the newly-set-up
machine: `NameError: name 'Node' is not defined` at `class
TelemetryView(Node):`.

**Root cause.** Simple immediate trigger: ROS2 wasn't sourced in that
shell (`source /opt/ros/humble/setup.bash && source install/
setup.bash` fixed it directly — ROS2 Humble and the built workspace
were both already present). But that exposed a real, independent bug:
`UI.py` wraps `import rclpy` etc. in `try/except ImportError` and sets
`CAMERA_AVAILABLE = False` on failure, clearly intending a degraded
no-camera mode (checked before `TelemetryView()` is ever instantiated,
in `_start_camera_view`) — but `class TelemetryView(Node):` at module
scope is *not* guarded by that flag. Python evaluates base classes at
class-definition time, so any environment where the ROS import fails
crashes immediately, before the intended fallback ever gets a chance
to run.

**Fix.** In the `except ImportError` branch, `Node = object` — a
placeholder base class so the class statement doesn't `NameError`.
Safe: `TelemetryView` is only ever instantiated from
`_start_camera_view`, which already checks `CAMERA_AVAILABLE` first
and returns before reaching that line.

## 37. MORAI GroundTruth VehicleInfo topic renamed to `/Ego/vehicleinfo` — reconfirms §32's frozen-value bug independently [FIXED / CONFIRMED]

**Objective.** MORAI Studio's `GroundTruth_1` ROS2 Interface was found
publishing on `/Ego/vehicleinfo` (confirmed live in Foxglove at
11.55-33.82 Hz), not `/Car_1/vehicleinfo`. `state_adapter_node.py`'s
`vehicleinfo_topic` parameter defaulted to `/Car_1/vehicleinfo`, and
neither `start_adas.sh` nor `UI.py`'s "Start MORAI Bridge" ever
overrode it — ROS topic matching is exact-string, so
**`state_adapter_node` was receiving zero VehicleInfo messages**, not
merely stale ones.

**Fix.** Default changed to `/Ego/vehicleinfo`; docstring updated to
flag that this must match the GroundTruth entity's Interface "Topic"
field exactly (an arbitrary, freely-renameable string, unrelated to
the vehicle's internal `id` field inside the message payload, and
unrelated to any other topic's naming convention — `Camera_1` staying
on `/Car_1/...` while GT moved to `/Ego/...` is not a problem, since
each ROS topic name is independent).

**Confirms, doesn't replace, §32.** After the fix, GT data *did* start
flowing live (11.55 Hz) — but `local_velocity_x=2.9433467388153076`,
`local_velocity_y=-1.0842...e-19`, `location_z=-5.181118965148926`,
`rotation_x=36893488147419103000` were all **bit-for-bit identical**
across two Foxglove captures ~13+ minutes apart. So the topic-name fix
was necessary (zero messages → real messages) but insufficient — the
payload itself is still frozen, strongly supporting §32's "Detect
Radius" theory (`GroundTruth_1`'s was set to a suspiciously tiny
`0.05`) as a MORAI-side sensor bug, independent of the ROS plumbing.
Also noticed: the Stanley log's `v=2.94 m/s` throughout this whole
session is this exact frozen value — displayed speed has been stuck on
one stale GT reading regardless of actual vehicle motion the entire
time. Not yet tried: bumping Detect Radius (§32's proposed next step).

## 38. `ipm_view_node.py` had no MORAI camera-extrinsics awareness [FIXED]

**Objective.** `perception_node.py` and `lane_detection_node.py` both
already branch `cam_height_m`/`cam_x_offset` on the `simulator`
parameter (MORAI: 0.9 m / 0.75 m — camera mounted lower and further
forward than CARLA's 1.35 m / 0.6 m). `ipm_view_node.py` (drives the
BEV panel) did not: `CAM_H_M = 1.35` / `CAM_X_OFF = 0.6` were hardcoded
module constants, and `start_adas.sh` launched it with zero ROS params
at all — the BEV's ground-plane warp was silently using CARLA's
geometry on every MORAI run.

**Fix.** Added the same `simulator` parameter + MORAI-aware defaults
(`DEFAULT_CAM_HEIGHT_M_MORAI = 0.9`, `DEFAULT_CAM_X_OFFSET_MORAI =
0.75`) as the other two nodes. `compute_homography()` now takes
`cam_h_m`/`cam_x_off` as arguments instead of reading module globals.
`start_adas.sh`'s `ipm_view_node` launch line now gets `-p
simulator:=$SIMULATOR` like every other node in the stack. FOV stays a
shared constant (unchanged between simulators in the existing pattern
too).

## 39. Dry-run switch — validate ACC/LKAS while driving manually, since MORAI's GT sensor is known-broken [DONE]

**Objective.** With §37 confirming MORAI's GroundTruth speed sensor is
genuinely broken (frozen, not a plumbing issue), closed-loop ACC
validation on MORAI isn't trustworthy right now. Wanted: drive the car
by hand in MORAI while ACC/LKAS keep computing and publishing normally,
to validate YOLO/UFLD output quality directly, without ADAS actually
touching the vehicle.

**Design.** Single choke point: `control_adapter_node._publish()`
(`src/morai_bridge/`) is the *only* place that sends `/Car_1/control`
to MORAI — `controller_node`/`stanley_node` upstream keep computing
and publishing `/Car_1/cmd_vel`/`/Car_1/cmd_steer` regardless, which is
exactly what stays inspectable (Foxglove plots, debug images, Stanley
logs) to judge ACC/LKAS quality. Added a `dry_run` parameter
(default `false`); when true, `_publish()` returns before the
`control_pub.publish()` call, but all subscriptions/state-tracking/
logging run unchanged.

**Wiring**: new `Dry run (no vehicle commands)` checkbox in `UI.py`'s
Processes panel (next to "Record rosbag"), threaded into both
`start_morai_bridge()` (`-p dry_run:=true` on `control_adapter_node`)
and `run_start_adas()` (`./start_adas.sh morai dry_run`, a new second
positional arg). Scoped to MORAI only — doesn't touch the CARLA
actuation path (`carlaaccsim`, external repo, out of scope while
MORAI-focused).

## 40. Lane KF stuck-in-rejection lockup — χ² gate had no escape path [FIXED]

**Objective.** BEV/debug-view lane lines intermittently vanished
("only sometimes" visible) and, in one capture, visibly diverged from
the road (KF's blue/red lines cutting across the lane while raw UFLD's
green detection dots tracked it correctly). Root-caused via the log:
`[KF R] steady REJ n_coast=0 n_rej=1665`.

**Root cause.** `n_coast=0` means UFLD was delivering strong, valid
detections every frame — the "too few points"/"too low confidence"
coast path never fired. `n_rej=1665` means the filter's own χ²
Mahalanobis outlier gate (`lane_kalman.py`, inside `update()`) rejected
roughly that many consecutive measurements as statistically
inconsistent with the filter's *own* current belief — and unlike the
coast path (which has `KF_MAX_COAST_TICKS`-based reset logic in
`_kf_smooth`), a rejection was a dead end: `n_rejected` just kept
incrementing forever with **no reset on success and no forced
re-initialization**, whether cumulative or consecutive. Once the
filter's state drifted from reality (plausibly during an earlier
sustained curve), it entered a self-reinforcing lockout: every new,
genuinely correct detection looked like a statistical outlier relative
to its own increasingly-wrong prediction, got rejected, and it kept
dead-reckoning forward from stale state indefinitely. This same
mechanism explains the BEV flicker too — `ipm_view_node._on_left`/
`_on_right`/`_on_centerline` directly mirror the latest `/LKAS/
ego_lane_*` Path message with no staleness tolerance, and a rejected
frame's `_kf_smooth` returns `[]` (empty Path) for that side that tick.

**Fix.** `lane_kalman.py`: `LaneKalmanFilter.update()` now resets
`self.n_rejected = 0` on any accepted measurement (mirrors `n_coast`'s
"since last reset" semantics — `n_rejected` now means "consecutive
rejects since the last accepted frame," not a lifetime total).
`lane_detection_node.py`: new `KF_MAX_REJECT_TICKS = 20` (same order
of magnitude as `KF_MAX_COAST_TICKS`, ~4 s at 5 Hz); when a side's
`n_rejected` hits that threshold, `_kf_smooth` forces `kf.initialized
= False` and resets the counter, so the next good measurement re-seeds
the filter from scratch (`initialize()`) instead of being rejected
forever. Mirrors the existing coast-timeout pattern exactly, just
keyed off consecutive REJ instead of consecutive weak/missing frames.

## 41. MORAI camera publish-rate investigation — paused, unresolved [KNOWN, unresolved]

**Status at pause.** MORAI development is paused here to switch back
to CARLA-specific scenario work (see §33 above). This section is the
handoff: what's confirmed, what's ruled out, and what's still open.

**What's confirmed ruled out as the cause:**
- **GPU is not saturated.** `nvidia-smi` during a live run: 32 %
  utilization, 12.4 GB / 24 GB memory on an RTX 4090. The earlier
  hypothesis ("YOLO + UFLD + MORAI's own Epic-quality rendering
  fighting over the GPU") does not hold up against measurement.
- **CPU is not saturated.** 28 cores available, system 95.4 % idle,
  load average ~1.6. `debug_image_fusion_node` was unexpectedly the
  single biggest CPU consumer among our own nodes (53 % of one core,
  more than `perception_node`/YOLO at ~13-15 % or `lane_detection_node`
  /UFLD at ~17-21 %) — worth a look eventually since it's pure
  visualization overhead, not part of the control loop, but nowhere
  near enough to explain a system-wide stall on a 28-core box.
- **`inference_skip_n` tuning was very likely the wrong lever.**
  Raised MORAI's default from `1` to `2` (halving UFLD's per-frame
  load) specifically to test the GPU-contention theory; RTF did not
  improve (0.56 → 0.49 → 0.62 across samples, no clean trend). Once
  the actual camera rate was measured directly (below), this stopped
  being a plausible explanation at all.

**The actual live measurement, and why it reframes everything:**
`ros2 topic hz /Car_1/camera/front/compressed` while the stack was
running: **average 1.242 Hz**, `min 0.054 s, max 1.556 s, std dev
0.751 s` — wildly erratic (instantaneous rate swinging between ~18 Hz
and ~0.64 Hz frame to frame), and nowhere near the 20 Hz "Fixed"
timestep mode was switched to specifically to achieve (see the earlier
Variable→Fixed 20 fps change in this session, which subjectively felt
like an improvement but was never measured this precisely). At this
real rate, `inference_skip_n=2` gives UFLD a fresh frame roughly every
~1.6 s — sparser than even the original `skip_n=1` MORAI default was
designed to avoid. Given GPU/CPU are confirmed idle, **the bottleneck
is upstream of every ROS node in this repo** — in whatever produces
`/Car_1/camera/front/compressed` itself.

**Potential causes, not yet tested, roughly in order of suspicion:**

1. **MORAI's own camera sensor capture/encode/publish pipeline**
   (Windows-host side, outside this repo and outside what a WSL shell
   can inspect — `nvidia-smi` inside WSL2 does report true physical
   GPU utilization across the whole machine, so the 32 % figure above
   *should* include MORAI's Windows-side rendering load, but a single
   snapshot can miss transient spikes). This is the leading suspect
   simply by elimination: everything measurable on the Linux/ROS side
   is idle.
2. **Fixed-timestep + dry-run interaction, untested.** MORAI Studio's
   ROS2 Interface panel shows `Mode: Fixed` with an
   `ExternalControlInterface` **subscriber** — plausible that a
   fixed-step sim expects to receive control input each tick before
   advancing. Since dry-run (§39) means `control_adapter_node` never
   publishes `/Car_1/control` at all anymore, it's a concrete,
   testable hypothesis that dry-run itself is stalling MORAI's
   step-advance loop. **Next action**: toggle dry-run off (accepting
   the car will actually drive) and compare the camera-topic Hz
   measurement above, same scene, before/after.
3. **Quality preset (`Epic`) vs. rendering budget.** Never tested
   lowering MORAI's Quality dropdown from `Epic` to see if RTF/camera
   rate improves — cheap experiment, not yet tried this session.
4. **WSLg/Xwayland or WSL2 networking overhead** on whatever channel
   carries the camera sensor's ROS2 publish from the MORAI process to
   the ROS graph — untested; §26j documents a related but distinct
   WSLg rendering bug (blank window at Xwayland startup), not
   necessarily connected to sustained per-frame throughput.
5. **Scene complexity.** The lowest RTF (0.49) and the highest (0.62)
   were measured in visually different scenes (open intersection vs. a
   vegetation-heavy roundabout) — scene GPU cost on the MORAI/Windows
   side was never controlled for across the RTF samples in this
   session, so some of the variance may just be scene-dependent
   rendering cost rather than a single fixed bottleneck.

**Not yet done, for whoever picks this back up:** a controlled A/B
comparison (same scene, same NPC count, same weather) toggling one
variable at a time — dry-run on/off, Quality preset, Fixed vs.
Variable timestep — while logging `ros2 topic hz` on the camera topic
and MORAI's own RTF/System-FPS readout side by side. Nothing in this
session isolated a single cause; it only ruled out GPU/CPU compute and
`inference_skip_n` as the explanation.


---

## 42. Cruise mode was P-only — set speed never reached, looked like a fallback to 25 km/h [FIXED]

**Symptom.** Driving a scenario at 30 km/h, the VUT held 30 km/h through
the scripted approach and then, the moment ACC took over, settled at
**25.5 km/h** and stayed there. The obvious reading was that
`/ACC/target_speed` had not been received and the node had fallen back
to its `CRUISE_SPEED_KMH = 25.0` default.

**That reading was wrong.** The same scenario at other speeds settled at
44.8 km/h (setpoint 50) and 125.5 km/h (setpoint 130) — nowhere near 25.
Every setpoint undershot by a near-constant 4.3–5.0 km/h. 30 km/h just
happens to land on 25.

**Cause.** `cruise_control()` was proportional-only *with a ±0.5 m/s
deadband*:

```python
if speed_error > 0.5:   throttle = min(speed_error * 0.3, cap)
elif speed_error < -0.5: brake    = min(-speed_error * 0.3, 0.6)
else:                    throttle = brake = 0.0
```

Holding a speed requires a steady non-zero throttle to balance drag, and
the deadband commands exactly zero there — so the loop could not hold any
setpoint. It coasted until the error left the deadband, then settled
wherever `0.3 * error` happened to equal drag. That puts steady state a
fixed distance *below* setpoint, and the distance is exactly
`throttle / cruise_gain`:

| setpoint | steady throttle | predicted droop | predicted v | measured v |
|---|---|---|---|---|
| 30  | 0.413 | 4.95 km/h | 25.05 | **24.77** |
| 50  | 0.414 | 4.97 km/h | 45.03 | **44.82** |
| 130 | 0.359 | 4.31 km/h | 125.69 | **125.51** |

Prediction matches measurement within 0.3 km/h at all three speeds.

**Fix.** `cruise_control()` is now PI (`cruise_ki = 0.2`,
`cruise_i_limit = 5.0`, so the integrator's authority tops out at full
throttle). The throttle-side deadband is gone — it was the thing making
steady-state tracking impossible. A small deadband remains on the brake
side only, to stop the brake chattering against the throttle on a few
cm/s of overshoot.

Three things were load-bearing and are easy to get wrong if this is ever
revisited:

1. **`cruise_control(integrate=False)` from the ACC branch.** MODE 4
   calls cruise every tick purely to take
   `min(acc_throttle, cruise_throttle)` / `max(acc_brake, cruise_brake)`
   as a set-speed cap. During an ACC braking event `v_ego` falls far
   below set speed, so an integrator running there would wind up to full
   throttle and slam it on the instant ACC released authority. Only the
   CRUISE branch integrates.
2. **Integral reset in GATE / STANDSTILL / EMERGENCY**, so cruise never
   resumes with stale windup from a period when it was not driving.
3. **`CRUISE_SPEED_KMH: 25.0 → 20.0`.** The 25.0 existed *solely* to
   offset the droop this fix removes — the comment above it said as much
   ("If an integral term is added to `_cruise_control()` later, this can
   be reduced back to 20"). Leaving it at 25.0 would have converted a
   compensating offset into a real +5 km/h and broken the declared 20 km/h
   ODD. Effective on-road behaviour is unchanged.

The old law is kept as `_legacy_cruise_control()` for reference.

**Verified.** Scenario run at 50 km/h now holds 48.8–49.7 km/h through
the whole approach instead of drooping to ~45.

**Related.** The same `min`/`max` combination in MODE 4 is what stops the
PD law accelerating into a stationary target when its distance term
saturates to `a_max` — see §43.

---

## 43. UN R171 Annex 4 §4.2.5.2.1 scenario harness, and what it found [DONE]

`scenarios/r171_stationary_target.py` + `scenarios/sim_adapter.py` drive
the stack against a stationary target on a straight road (see
`scenarios/README.md` for usage). The harness replaces
`carlaaccsim/carlaAccSimTown.py` for the duration of a run — it is itself
the CARLA↔ROS bridge, because it has to gate who owns the longitudinal
channel: the scenario holds the approach speed open-loop until the DCAS
trigger point at `gap = ttc * v`, then the stack's ACC takes over.

**Town03 spawn 5 cannot host this test.** It sits 2 m from a junction —
0 m of straight runway — and no Town03 spawn exceeds ~116 m. The matrix
needs up to 469 m (130 km/h at a 10 s TTC margin). Measured straight
runway, walking each lane forward until it deviates 1 m from the start
heading: **Town06 spawn 80 gives 718 m** (road 48, centre lane of five,
3.5 m wide, neighbour lanes both sides so UFLD has markings on both
edges). Town06/Town07 added to `UI.py`'s `TOWNS`.

**Headline KPI** is `a_req = v²/(2·gap)` — the constant deceleration
needed to stop in the remaining gap — compared against 5 m/s². At the
trigger point this reduces to `v/(2·ttc)`, so the scenario's own
difficulty is set purely by its parameters; every metre the system spends
not braking after that drives it up.

### What it found

**The PD law's brake onset is a fixed distance schedule, not a TTC one.**
With `a = k_p(d − d_desired) + k_d·ḋ` and a stationary target (`ḋ = −v`),
the ACC first commands `a ≤ 0` at

```
d_brake = d0 + (T_gap + k_d/k_p)·v
```

so the deceleration it implies grows linearly with speed. Measured runs
tracked the prediction closely (30 km/h → pass at 2.4–3.1 m/s²;
50 km/h → 5.8–6.6 m/s², over the limit).

**Perception range is the binding constraint, not the gains.** Measured
first-detection gap: ~25 m at 30 km/h, ~19 m at 50, ~12 m at 130. Once
`d_brake` exceeds that range, raising `T_gap`/`k_d` changes nothing — the
controller cannot brake for a target it has not been told about. Modelled
against measured detection, four different gain sets give *identical*
results at 70 km/h and above. The ceiling is `v_max = sqrt(2·a·D)`: at
D ≈ 15–20 m, even full tyre grip (9 m/s²) caps avoidance near 60 km/h.

**`d0: 3.0 → 7.0` stopped the collisions at 50 km/h.** It moves
`d_brake` from 16.4 m to 20.4 m, past the ~20 m detection range, so the
PD law wants to brake as soon as YOLO reports. 50 km/h went from
`fail_collision` to `pass_over_limit` (stopped with 2.12 m to spare).

**A dropped detection makes CRUISE accelerate into the target.**
`lead_distance_callback` sets `d_lead = None` on a single `inf`, and
MODE 2 (CRUISE) is tested *before* MODE 3 (EMERGENCY) and MODE 4 (ACC) —
so during a dropout neither `emergency_distance` nor the PD law is
reachable, and cruise opens the throttle toward set speed. In one 50 km/h
run 103 of 140 measure-phase samples had no detection; mean throttle was
0.364 while blind versus 0.102 while tracking. Candidate fix is a coast
timeout (hold the last filtered value for ~0.5 s before declaring the
lead gone) — **not yet applied**.

**Why detection drops at close range** (code reading, not yet measured):
`estimateLeadDist` rejects rather than misses. Two gates fail specifically
when close — the side-pass guard `clipped_left ^ clipped_right`, which
assumes a genuine lead is never clipped on exactly one side (false as soon
as the bbox is large and the lead is laterally offset), and the
`in_lane is None → continue` branch, whose fallback is explicitly
disabled "for the diagnostic test". A lead at close range occludes the
lane markings it sits on, so UFLD loses the centerline exactly there.

**Do not cap ACC braking authority yet.** Capping at 5 m/s² is the
obvious way to satisfy the limit, but the geometry says it would cause a
crash. To stop at 5 m/s² *and* leave the `d0 = 7 m` standstill gap,
braking must begin at `v²/10 + d0`:

| v | braking must start by | measured detection | headroom |
|---|---|---|---|
| 30 km/h | 13.9 m | 17.7 m | **+3.7 m** |
| 50 km/h | 26.3 m | 20.0 m | **−6.3 m** |

At 50 km/h the last run needed 9.74 m/s² to stop in the room it had and
delivered 9.32. Capping it at 5 would have put it into the target. The
5 m/s² limit is reachable at 30 km/h and **not** reachable at 50 km/h
until detection range improves. Order of work is therefore: extend
perception range first, then start braking at first detection, and only
then bound the authority.

## 44. GT `VehicleInfo` garbage fields — ruled out "uninitialized memory," found a fixed deterministic pattern instead [KNOWN, unresolved — handoff before pausing MORAI]

**Objective.** §29/§32/§37 documented `morai_v2_1_ros2_msgs/VehicleInfo`
publishing occasional garbage (~3.7e19-ish) on random-looking fields,
attributed in `state_adapter_node.py`'s docstring to "an
uninitialized-memory/buffer-reuse bug inside MORAI's own serializer."
Live investigation with the sim running found something more specific.

**The official `MORAI-ROS2_morai_msgs` GitHub repo is not the right
reference at all.** Cloned it, read every one of its 30 `.msg`/6
`.srv` files plus the README. No `VehicleInfo` message exists there —
the README reveals it targets a *different* MORAI product entirely,
**"MORAI SIM: Drive"**, with fixed non-configurable topics
(`/Ego_topic`, `/ctrl_cmd`, `/Object_topic`, ...). What we're actually
using is MORAI **Studio**'s flexible ROS2 Interface panel
(user-configurable Topic name + a "Message Template" dropdown,
versioned to match our local `morai_v2_1_ros2_msgs` package name) —
a different integration mechanism with its own message set that isn't
published in this repo. One structural pattern is still worth noting:
every message in that repo, without exception, uses `std_msgs/Header`
+ `geometry_msgs/Vector3` (float64 x/y/z) for position/velocity/
acceleration — never flat `float32` scalar triples like our
`VehicleInfo.msg` does.

**Live re-diagnosis, with the sim actually running.** `ros2 topic
list` showed **both** `/Car_1/vehicleinfo` (2 publishers) and
`/Ego/vehicleinfo` (0 publishers) simultaneously — reconfirms §37's
finding that the GT topic name is scenario-specific/arbitrary, and in
*this* scenario config it had been switched back to `/Car_1/
vehicleinfo`, meaning `state_adapter_node`'s current default (`/Ego/
vehicleinfo`, set in §37) was receiving nothing again. Echoing
`/Car_1/vehicleinfo` directly (three samples, `stamp_sec` genuinely
incrementing, other fields like `angular_velocity_z`/`brake`/
`steer_angle` visibly changing between samples — so the message is
live, not frozen) showed:

- `rotation_z`, `local_velocity_y`, `local_acceleration_z` were
  **exactly `0x60000000` (= 2^65) in every single sample, with zero
  exceptions**, while surrounding fields at the same moments read
  ordinary, plausible values.

A fixed value at a fixed set of field *names*, every time, while nearby
fields genuinely vary — that rules out "uninitialized memory" (which
would vary sample to sample). Mapped the field order to byte offsets
(all `float32`, 4-byte CDR-aligned, after `stamp_sec`/`stamp_nanosec`/
the `id` string) to test a byte-alignment-shift theory: garbage lands
at field-index 5, 7, 11 — **an irregular stride** (+2, then +4), not
the contiguous run a simple "our `.msg` has one field's width wrong
so everything after it is shifted" theory would produce. So this is
*not* a clean global misalignment either. Two remaining, undistinguished
hypotheses:
1. Our `.msg` mis-defines/mis-orders these three specific fields
   relative to MORAI's true schema (a scattered, not global, mismatch).
2. MORAI's own serializer simply never populates these three specific
   fields for a GroundTruth-bound `VehicleInfo` (always writes the same
   sentinel there), independent of whether our `.msg` is correct.

Distinguishing these needs the raw CDR bytes off the wire — `ros2
topic echo` output is already decoded *using our own possibly-wrong
schema*, which isn't sufficient to test either theory rigorously. Not
done this session.

**`EgoVehicleStatus.msg` as an alternative — investigated, inconclusive.**
Confirmed the official repo's `EgoVehicleStatus.msg` (fields: `header`,
`unique_id` (int32), `acceleration`/`position`/`velocity`/
`angular_velocity` (all `geometry_msgs/Vector3`, velocity documented in
**km/h**, not m/s), `heading`/`accel`/`brake`/`front_steer`/
`rear_steer` (float64)) is a completely different, fully-documented
schema — attractive *if* MORAI Studio's GroundTruth entity actually
offers it as a selectable "Message Template". Could not confirm either
way this session (two screenshots of the Message Template dropdown
both failed to transmit/render). **This is the single highest-value
next check** for whoever resumes MORAI work: open `GroundTruth_1`'s
ROS2 Interface config in Studio and read off every entry in that
dropdown. If `EgoVehicleStatus` (or equivalent) is offered, switching
to it sidesteps all of the above guesswork with an officially-documented
schema — but would require vendoring `EgoVehicleStatus.msg` into our
own msgs package (depends only on `std_msgs/Header` +
`geometry_msgs/Vector3`, both standard) and rewriting
`state_adapter_node.py`'s field access + a km/h→m/s conversion
(`speed = abs(msg.velocity.x) / 3.6` instead of the current
`abs(msg.local_velocity_x)`). If it's *not* offered (plausible —
GroundTruth-type entities may only expose `VehicleInfo`), the only
remaining path is a raw-byte capture to empirically determine
`VehicleInfo`'s true wire layout.

**Update — confirmed we never followed MORAI's own ROS2 setup guide.**
The user supplied MORAI's official "Developer Setup for ROS2" doc.
Step 4 is explicit: `git clone
https://github.com/MORAI-Autonomous/MORAI-ROS2_morai_msgs.git` into
the workspace `src/`, then `colcon build`. Checked our workspace:
`src/morai_v2_1_ros2_msgs/msg/` contains exactly **2** hand-written
files (`VehicleInfo.msg`, `VehicleManualControl.msg`) sharing **zero**
names with any of the official repo's 30 messages. We never cloned or
built against MORAI's real message definitions at all — someone wrote
a custom, never-validated-against-the-real-schema package from
scratch instead. Toolchain otherwise matches the guide fine (WSL2,
ROS2 Humble, `colcon`/`rosdep`/`build-essential`, `rqt`, `rviz2` all
present; only `ros-humble-compressed-image-transport` is missing, but
this stack never uses the `image_transport` plugin API — every topic
is a plain `sensor_msgs/CompressedImage` via manual `cv2.imencode`/
`imdecode` — so that gap is very likely inconsequential in practice).

This substantially upgrades the `EgoVehicleStatus` recommendation
above from "worth trying" to "very likely the actual fix": rather than
reverse-engineering our custom `VehicleInfo`'s true byte layout, the
straightforward correct path is to follow MORAI's own setup guide
properly — clone the real `MORAI-ROS2_morai_msgs` repo into `src/`,
`colcon build`, reconfigure `GroundTruth_1`'s Message Template in
Studio to `EgoVehicleStatus` (still needs confirming that template
name is actually offered in Studio's dropdown — unconfirmed this
session, see above), and rewrite `state_adapter_node.py` against the
official schema (`unique_id` int32 instead of a string `id`, etc.).
**This is the concrete next step** for whoever resumes MORAI work,
ahead of any raw-byte-capture approach.

**Done, this session**: cloned `MORAI-ROS2_morai_msgs` into
`src/` (ROS2 package name `morai_ros2_msgs`, distinct from our own
`morai_v2_1_ros2_msgs`), `rosdep install` + `colcon build --packages-
select morai_ros2_msgs` succeeded, `ros2 interface show
morai_ros2_msgs/msg/EgoVehicleStatus` confirms it's correctly built
and introspectable. **Correction to a unit claim made earlier in this
same investigation**: re-reading the freshly-cloned source file
directly (`grep` on the actual `.msg`, not a possibly-stale `curl`
fetch from earlier) shows `geometry_msgs/Vector3 velocity  # Velocity
vector [m/s]` — **m/s, not km/h** as stated above. No `/3.6`
conversion needed if/when `state_adapter_node.py` is rewritten against
this message. Cause of the discrepancy not confirmed (repo doc
possibly corrected upstream between the two fetches, or a
transcription slip) — trust the locally cloned file, not the earlier
number in this entry.

Building the package alone doesn't change what MORAI Studio actually
publishes on the wire, though — `GroundTruth_1`'s Interface still has
to be switched to whichever Message Template corresponds to
`EgoVehicleStatus` (if offered) before this new package does anything
useful.

**Conclusive answer, closes out this path**: checked Studio's Message
Template dropdown directly — **none of the official
`MORAI-ROS2_morai_msgs` types (`EgoVehicleStatus` included) are
offered there at all.** Studio's ROS2 Interface system is disconnected
from MORAI's own public messages repo; only its own custom template
set (`VehicleInfo`, etc. — the undocumented schema our
`morai_v2_1_ros2_msgs` package guesses at) is selectable in the
product. This is a MORAI-side product gap, not something fixable from
our side — their official message definitions exist on GitHub but
aren't wired into Studio's own Interface UI. **Status: blocked,
waiting on MORAI to respond/fix.** The newly-cloned `morai_ros2_msgs`
package is built but currently unusable for actually talking to
Studio; kept in `src/` in case MORAI fixes this and it becomes
relevant later, or reference material for whenever `VehicleInfo`'s
true layout needs reverse-engineering. Until then, our custom
`morai_v2_1_ros2_msgs/VehicleInfo` remains the only interface
Studio's GroundTruth entity actually supports, garbage fields and all
— the only remaining self-service diagnostic left (raw CDR byte
capture) still stands as the next option if this needs revisiting
before MORAI responds.

**Aside, not a bug**: also traced why `/ADAS/perception/debug_image`
can show a *higher* Hz than the raw `/Car_1/camera/front/compressed`
it's derived from — `debug_image_fusion_node` (and `ipm_view_node`
for the BEV panel) run their own independent fixed ~10 Hz publish
timers (`self.create_timer(1.0 / PUB_HZ, self._publish)`), re-compositing
whatever's cached (latest raw frame + latest ACC/LKAS overlay masks)
on every tick regardless of whether anything new has actually arrived
from the erratic upstream camera. Working as designed, not a
regression — but means "higher Hz" on those two topics specifically
doesn't imply "more real new content," just a faster re-publish clock.

---

## 45. Longitudinal rework — ACC rebuilt as a speed governor, and the measurement errors that hid the real problems [DONE]

Driven by the R171 harness (§43). The headline was simple — "deceleration
exceeds 5 m/s²" — but four separate causes were stacked behind it, two of
them **in the measurement rather than the controller**. Recording the false
trails as well as the fixes, because three of them looked conclusive at the
time and cost a lot of runs.

### 45.1 Measurement errors that pointed the wrong way [FIXED]

**The harness's own acceleration signal was lagged and attenuated.**
`sim_adapter` estimated acceleration with an EMA (α=0.3) and the KPI read
it directly. On one 50 km/h run it reported a **7.29 m/s² peak where the
truth was 9.11**. Everything derived from that signal inherited the error:

* A brake-vs-deceleration cross-correlation put the loop's dead time at
  **208 ms**, and a whole tuning pass was built on "the dead time is the
  binding constraint". Re-measured with a **centred-window slope of v(t)**
  (lag-free, noise-robust) the two align at **~zero lag**. Most of that
  208 ms was the filter being measured through.
* The brake-authority model `decel = brake*(16.7 - 0.81*v)` was fitted to
  the same lagged signal and came out ~2x too small, so the feed-forward
  asked for roughly twice the brake actually needed.

The KPI now recomputes peak deceleration offline with a centred window.
Raw sample-to-sample differencing is not an alternative — it peaks at
23 m/s² on single-sample noise.

**The KPI also included the approach phase.** `_peak_decel_from_trace` ran
over every sample, and the scenario's own kinematic snap to the test speed
differentiates to ~12 m/s² at t≈1 s. Runs were being reported at 11.81 and
12.26 m/s² when the post-handover peaks were 8.63 and 9.11. Now restricted
to the measure phase.

**`cte_m` hid a lane departure as an oscillation.** The harness computed
cross-track from a per-tick `map.get_waypoint()`, which snaps to whichever
lane is nearest. One 70 km/h run reported cte swinging **-1.66 → +1.47 m
in a single 50 ms sample** while world `y` moved a steady +0.26 m per
sample — a clean 2.89 m lane departure, reported as a 3.13 m oscillation.
Now anchored to the start-line ray, which on a dead-straight scenario IS
the intended path and is unbounded by lane width.

### 45.2 Engine braking dominates, and it invalidated the brake-plant fit [CORRECTED]

**This section previously read "the brake plant is not modellable",**
reporting brake authority (achieved decel / brake command) as ranging 8 to
49 and varying 2-3x *within* a single speed band, and concluded the gain
was unpredictable. That conclusion was an artifact of the model used to
derive it, and is retracted.

The fit was `decel = brake * authority`. It omitted engine braking.
Measured directly — every sample across the runs with throttle AND brake
both at zero:

| v [km/h] | n | mean coast decel | max |
|---|---|---|---|
| 0-7   | 19 | 1.23 | 1.74 |
| 7-14  | 29 | 2.18 | 3.19 |
| 14-22 | 12 | 4.08 | 6.93 |
| 22-29 | 13 | **4.94** | **10.42** |
| 29-36 |  8 | **5.49** | 7.55 |
| 36-43 | 12 | 2.62 | 9.21 |
| 43-50 |  8 | 2.36 | 3.01 |

At 22-29 km/h the vehicle sheds ~4.9 m/s² with no command at all.
Attributing that to a 0.2 brake command yields an "authority" of ~25 which
has nothing to do with the brake. That is the entire source of the 8-49
spread and the within-band variation: a large, speed- and gear-dependent
term sitting unattributed in the residual.

**Two consequences follow, and they matter more than the retraction.**

*The deceleration peaks were never brake spikes.* Every attempt to bound
them by capping brake force — brake_scale, the conservative feed-forward,
the deceleration PI — did nothing because the brake was not producing
them. One run peaked at 8.37 m/s² with throttle 0.00 and brake 0.00 held
for the entire event; velocity topic, position-derived speed and
gap-closure rate all agreed the vehicle really was slowing that hard.

*Releasing the throttle is not a gentle action in this vehicle.* Coasting
alone exceeds the 5 m/s² R171 ceiling (5.49 mean at 29-36 km/h, peaking
10.42). A soft deceleration therefore requires holding PARTIAL THROTTLE
against engine braking. The speed loop applying throttle through 202 of
238 frames of a stop was doing exactly that, correctly. It was misread as
a limit cycle and suppressed, which left the vehicle coasting on engine
braking and stopping **38.6 m short of the target with the brake never
applied at all** (`verdict: no_reaction`). The suppression is reverted;
the comment at that site records why.

**So the stack cannot bound deceleration below 5 m/s² by managing the
brake, because doing nothing already breaches it.** The speed loop's
throttle authority is part of the deceleration argument, not incidental
to it — a constraint no amount of brake-side tuning can substitute for.

**Caveat on the numbers.** 10 m/s² of engine braking is not physically
plausible for a real Dodge Charger; a car coasting in gear sheds perhaps
1-2 m/s². This is very likely a CARLA powertrain artifact (gear ratios,
or `manual_gear_shift` / `gear` defaults on `carla.VehicleControl`). Every
R171 deceleration figure from this simulator inherits that caveat, and it
should be checked against MORAI, or against a CARLA coast-down with the
gearbox explicitly configured, before any of it is quoted externally.

*What does survive from the original section:* a feedback loop closing on
deceleration still has to work through a differentiated, filtered signal,
and the speed governor (§45.3) is still the right architecture. The reason
is simply different — deceleration is dominated by a term the controller
does not command, rather than by an unpredictable brake gain.

### 45.3 The rewrite — speed governor [DONE]

`acc_control` + the kinematic term + `brake_for_decel` are replaced by
`speed_reference()` + a shared `speed_control()`. Distance is turned into a
reference SPEED and the speed loop tracks it:

```
v_lead = max(0, v_ego + closing_rate)
d_safe = d0 + T_gap * v_lead
v_ref  = v_lead + sqrt(2 * a_profile * (gap - d_safe))
```

Why this shape:

* Differentiating the profile along the trajectory gives exactly
  `-a_profile`, so **deceleration is set by the plan, not by the brake**.
  The guarantee moves out of the brake plant — which does not command most
  of the deceleration anyway (§45.2) — and into arithmetic.
* It closes on SPEED — measured directly, at 20 Hz, with no meaningful
  lag — instead of on deceleration.
* One loop now serves CRUISE and ACC, retiring the
  `min(acc_throttle, cruise_throttle)` / `max(acc_brake, cruise_brake)`
  combination that existed only because two controllers were fighting one
  actuator.

Four things were load-bearing and are easy to get wrong:

1. **Headway references the LEAD's speed, not the ego's.** With
   `d_safe = d0 + T_gap*v_ego` the profile asks the vehicle to stop at its
   own current-speed headway — 22.8 m at 50 km/h — so `v_ref` hit zero
   with 20 m of gap left and the car halted 20 m short. Referencing the
   lead degenerates correctly: a stationary lead gives `d_safe = d0`.
2. **The reference is rate-limited at DECEL_LIMIT, not at a_profile.**
   Limiting it to the comfort rate looked right (the profile's own time
   derivative is exactly that) but only holds when tracking is perfect.
   Running 3 km/h above the reference the gap closes faster than the
   profile assumed, the true profile falls away beneath the limiter, and
   the reference never catches up — the vehicle was still doing 23.5 km/h
   with 2.00 m of gap left, and hit the target.
3. **Ratchet: the reference may not rise while the gap is closing.** The
   tracked gap still steps ~1 m frame to frame; without the ratchet `v_ref`
   rose 65 times mid-stop (+5.3 km/h cumulative), releasing the brake,
   sagging deceleration from 3.9 to 2.3 — and that lost distance was repaid
   at the end as 6.2 m/s². The harsh late braking WAS a mid-stop release.
4. **Bumpless integral handover.** The integrator holds whatever throttle
   sustained the previous setpoint (~2.15 while cruising at 50 km/h) and
   that bias survives into a braking event. The loop stayed net-positive
   and drove **throttle 0.43 into a stationary target**; unwinding at
   `ki*e = 0.09/s` would have taken ~12 s. It is now dropped the moment the
   required action reverses sign.

**A failed fix worth not repeating:** clamping `v_ref` to `v_ego - 0.6 m/s`
to bound the tracking error. It starves the loop of the very error signal
that unwinds the integrator, and turned "brakes too hard at the end" into
"does not brake at all". The reference must be free to lead.

### 45.4 Estimator — alpha-beta tracker [DONE]

The `ALPHA = 0.4` low-pass on `/ACC/lead_vehicle_distance` was the single
largest source of lag in the sensing path: a first-order filter lags
`(1-a)/a` samples and it ran at the **perception** rate (~12 Hz), so ~123 ms
of the budget. Replaced with an alpha-beta tracker (`beta = a²/(2-a)`,
critically damped) that predicts to now and compensates a measured ~0.25 s
perception transport lag. Scored on replayed real data: bias **+1.53 →
+0.20 m**, RMS **2.20 → 0.97 m**.

Closing rate is now a tracker STATE rather than a difference quotient over
an already-filtered signal — the old derivative flipped sign on far-range
noise, which dropped the braking latch mid-stop.

Short dropouts are bridged (`LEAD_MISS_TOLERANCE = 3` frames). Before that,
a single `inf` nulled `d_lead`, and MODE 2 (CRUISE) is tested *before*
EMERGENCY and ACC — so mid-stop the brake was released and the throttle
opened toward set speed. Measured: 103 of 140 samples in one run had no
detection, mean brake 0.010 while blind versus 0.445 while tracking.

### 45.5 Cruise was P-only [FIXED]

Covered in §42. Included here because it was the first symptom chased and
looked like a scenario bug ("30 km/h falls back to 25") when it was a
4.3-5.0 km/h steady-state droop affecting every setpoint.

### 45.6 Perception range was the real ceiling [FIXED]

`estimateLeadDist` was *rejecting* detections, not missing them. The
`in_lane is None` branch — "no UFLD centerline at this X", which is the
far-range case by definition — dropped them outright, capping effective
detection at 17-23 m. Replaced with a ground-space ego-lane corridor
(`LANE_FALLBACK_HALF_W = 1.5 m`), used only when the measured centerline is
unavailable. Metres, not pixels: at 40 m an adjacent-lane car sits only
~56 px off centre, so no fixed pixel strip separates the cases at both
ranges. Detection went to 90-155 m immediately.

`MAX_IPM_TRUST_M` then became the binding limit and was raised 40 → 80 m.
70 km/h needs ~44 m of sight to stop within 5 m/s²; the cap was holding
first detection at ~34 m, which demands 5.89. **70 km/h passes at 80 m.**

### 45.7 Steer latency regression [FIXED]

The harness polled `/Car_1/cmd_steer` once per director iteration (~19 Hz)
where the original bridge applied it event-driven in the callback. That
added up to **52 ms of latency to the lateral loop — 1.0 m of travel at
70 km/h** — and Stanley, which had been holding 70 km/h lanes, drifted out
of them. Restored to event-driven application with a lock serialising it
against the director's own `apply_control`. Cross-track went from a 2.89 m
departure to **±0.15 m**.

Worth remembering as a class of bug: nothing in `stanley_node.py` or
`lane_detection_node.py` changed. The regression was entirely in how often
its output reached the vehicle.

### 45.8 Where it stands

| | |
|---|---|
| 30 km/h | passes |
| 50 km/h | passes |
| 70 km/h | passes (with `MAX_IPM_TRUST_M = 80`) |
| 90 km/h+ | untested since the rework |

Peak deceleration still runs above 5 m/s² on some stops. The reference
descent is hard-bounded, but the total is reference + the speed loop's
correction + **engine braking, which the controller does not command and
which alone exceeds the limit** (§45.2). Bounding deceleration here is not
a brake-tuning problem; it needs the speed loop to hold partial throttle
against the powertrain, and it needs the CARLA engine-braking figure
verified before any of it is quoted as a compliance result.

**Open, in rough priority order:**

* **Switch the published distance to the pinhole estimate.** Already
  computed and published in parallel on `/ACC/lead_distance_pinhole`.
  Scored against ground truth over the same runs: bias **+0.19 m vs
  +1.62 m**, RMS **0.47 vs 1.86 m**, and flat with range where IPM's grows
  (+0.87 → +3.28 m). Keep IPM below ~10 m where boxes clip the frame edge.
  Note the pitch hypothesis that motivated it was **wrong** — IPM error is
  the same braking (+1.86 ±0.85) as coasting (+1.82 ±0.85), so it is a
  static calibration offset, not dynamic pitch.
* **Re-check `PERCEPTION_LAG_S = 0.25`** afterwards. It was tuned against
  IPM's +1.6 m bias and currently over-compensates: tracked gap reads
  1.5-3 m BELOW truth.
* **The tracker is amplifying, not smoothing** — raw 0.352 m/sample in,
  tracked 0.413 m/sample out. Suspect the 0.25 s prediction multiplying a
  noisy velocity state; lower `AB_ALPHA` now that prediction covers the lag.
* `acc_control`, `brake_for_decel` and `cruise_control` are called 0x.
  Left in place for comparison against the governor; delete once it proves
  out.

---

## 46. Coast-aware split, the clean coast-down, and why dev_jonas was not merged [DONE / KNOWN]

Continues §45. Everything here was measured against the CARLA charger on
Town06 spawn 80 at 50 km/h.

### 46.1 A whole afternoon evaluated against a node that was not running [PROCESS]

Runs 152823, 153353 and 153814 behaved near-identically across changes
that should have altered them materially. The cause was mundane and
expensive: `colcon build --symlink-install` updates the source the install
tree points at, but **a running node keeps the code it started with**. The
controller had been up since before several builds, so the acquisition
latch, the `a_des <= 0` cap and the tracker seeding were all "tested"
without ever executing.

Conclusions drawn in that window about latch arming and throttle chatter
are unreliable and were re-derived afterwards. Restart the node after every
build, and treat `ros2 topic hz /Car_1/cmd_vel` as the first check of any
run — a dead controller and a passive one look identical in a trace.

### 46.2 Clean coast-down, and the double peak [MEASURED]

A run where nothing published `cmd_vel` turned out to be the most useful
measurement of the day: an uncommanded 50 km/h -> standstill coast, 113
frames with throttle and brake at exactly zero.

| v [m/s] | 0.5 | 1.5 | 2.5 | 3.5 | 4.5 | 5.5 | 7.5 | 8.5 | 9.5 | 10.5 | 11.5 | 12.5 | 13.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| decel | 0.81 | 1.36 | 2.18 | 3.29 | 4.40 | **6.57** | 3.72 | 4.60 | **6.41** | 2.79 | 2.94 | 3.16 | 2.73 |

The **double peak** (6.57 at 5.5 m/s, 6.41 at 9.5, dipping to 3.72 between)
is two gearbox downshifts. It settles a question open since §45.2: the
non-monotonic shape is real, not sampling noise, which is why it cannot be
replaced by a smooth drag curve and why deceleration was swinging ~3 m/s²
for no commanded reason. This table now lives in `COAST_DECEL`.

**Peak uncommanded deceleration is 6.57 m/s², above the R171 ceiling.**
Confirmed directly rather than inferred. And 6.57 is not plausible for a
real Charger (1-2 would be), so this is a CARLA powertrain characteristic;
every deceleration figure from this simulator inherits that caveat.

### 46.3 Coast-aware throttle/brake split [DONE]

`speed_control` used to split at `u = 0`, treating "no command" as "no
acceleration". In this vehicle releasing the throttle at 25 km/h yields
~5.2 m/s², so asking for a gentle 2 m/s² still produced a brake command
which landed on top of that and overshot; the vehicle then fell below the
reference and the loop opened the throttle to recover. That was the
0.4-brake-then-throttle-surge pattern — the brake was never too strong in
itself (0.4 is worth only ~1.7 m/s²), it was added to something
unaccounted.

The split is now at `a_des + coast_decel(v)`, making the command one
continuous axis through zero. Below the coast value the loop holds
THROTTLE to keep deceleration down; it brakes only when coasting is
genuinely insufficient.

Brake authority, refitted with coast removed and the ~104 ms response lag
aligned: **4.44 m/s² per unit brake** (n=54). This supersedes §45.2's
retracted "8 to 49, unpredictable" — that spread was this same regression
with engine braking left in the residual. The number is lag-sensitive
(6.60 unaligned) and good to roughly +/-1.

Supporting changes, each of which was a real failure first:

* **Quadratic slew on `a_act`** replaces the two independent linear rate
  limits that sat on throttle and brake separately. Because they were
  separate, every crossing of zero collapsed one channel and started the
  other from scratch — the sawtooth, and why throttle appeared to drop off
  a cliff rather than taper.
* **No coast compensation while above the reference.** Cancelling engine
  braking when the vehicle is already faster than the plan is simply
  wrong; 53 frames of one run did it, worst case 0.65 throttle at 0.5 km/h
  too fast, 80 m out.
* **Minimum brake 0.1 whenever throttle is off.** Coasting is not a
  decisive action when engine braking varies 2-6 m/s² with gear, so speed
  hovered at the reference and the two toggled. A definite floor makes the
  vehicle actually slow and fall clear.
* **`a_des <= 0` while the stop is latched.** The coast table is coarse and
  wrong somewhere; where it over-reads, compensation becomes net
  acceleration. Measured: throttle 0.71 at 4.5 m/s cancelling a modelled
  3.66 m/s², accelerating 15.7 -> 20.0 km/h while the gap closed 6.5 -> 2.5 m.
  This is NOT the earlier blanket `throttle = 0`, which starved the loop
  and stopped the car 38.6 m short.

### 46.4 Tracker damping and seeding [DONE]

The alpha-beta velocity update is `(beta/dt) * residual` = 0.271 * residual
at 12 Hz perception, so one 10 m staircase step revised velocity by
2.7 m/s. That state multiplies the 0.25 s prediction AND is the closing
rate the governor and ratchet run on; the tracker was reducing jitter by
only 6%. Damped by physics rather than a tuning constant — relative
acceleration bounded at `AB_MAX_REL_ACCEL`, and `|closing rate| <= v_ego`.
Replayed on real data: velocity jitter **0.443 -> 0.161 m/s**, sign flips
3 -> 1.

That damping then delayed acquisition, because from zero it needed ~2.3 s
to reach a 13.9 m/s closing rate, and the latch cannot arm until it does.
Velocity is now **seeded from the first interval**, with damping applied
to everything after.

### 46.5 Early braking [DONE]

`ACC_PROFILE_DECEL: 3.0 -> 1.2`. Counterintuitively a *gentler* profile
brakes *earlier*, because onset is at `d0 + v^2/(2a)`: 34 m at 3.0 versus
82 m at 1.2 for 50 km/h. At 3.0 the vehicle held full speed for 35 m after
seeing the target at 69 m. Deliberately does not rely on the latch, which
can only arm once the tracker has a closing rate — the fallback now brakes
early on its own and the latch refines the rate.

Reference descent is bounded by `2 x` the latched plan rather than by
`DECEL_LIMIT`. At the ceiling the reference chased every downward step in
the gap estimate, descending at up to 9.43 m/s² — four times the plan — on
33 of 265 samples, each opening a tracking error the speed loop answered
with maximum deceleration.

### 46.6 dev_jonas: not merged, and it does not build a working node [KNOWN]

`35bf00e` removes the dead acceleration cascade — the same cleanup done
here — and describes itself as "Pure removal, no behavioural change".

**It is not.** It also removes `__init__` constants on the grounds that
only the dead functions read them, but `BRAKE_RATE_DOWN` is read by the
LIVE `speed_control()`. The node therefore dies the first time it takes
the brake path:

```
File "controller_node.py", line 623, in speed_control
    - self.BRAKE_RATE_DOWN)
AttributeError: 'ACCNode' object has no attribute 'BRAKE_RATE_DOWN'
```

Which is every scenario run. Two runs were attributed to "Jonas's
controller performing badly" before this was found; both were the crash:

* one died at the first brake command, leaving its last published
  `cmd_vel` (throttle 1.00, from `v_ego` still 0 at startup) latched in
  the harness — the car held full throttle to impact at 92.4 km/h. Not a
  control decision, a dead node's last message.
* one had nothing publishing at all (`acc_mode` empty for 125 frames).

Neither says anything about his governor. **Not merged.** To revive:
restore `BRAKE_RATE_DOWN` and audit `BRAKE_RATE_UP`,
`CRUISE_BRAKE_DEADBAND`, `cruise_gain`, `cruise_ki`, `cruise_i_limit`,
`cruise_throttle_cap` the same way.

Also worth passing back: the commit cites "brake authority varies 8-49x
unpredictably" as the motivation for the governor rewrite. That figure was
**retracted** the same day (§45.2) — it was an artifact of fitting
`decel = brake * authority` with engine braking in the residual. The
governor is still the right answer; the reason is that engine braking
dominates deceleration and is not commanded at all.

### 46.7 Dead code removed here [DONE]

`acc_control()`, `brake_for_decel()`, `cruise_control()`,
`_legacy_cruise_control()`, the PD gains `k_p`/`k_d` and `gain_scale`, and
17 constants read only by them. **1660 -> 1262 lines.**

Done with a verification the dev_jonas commit lacked: every `self.X` read
in the remaining source must have an assignment. Note the naive checks are
misleading and produced three separate false answers before one was
correct — subscription callbacks are referenced bare rather than called
(`ego_velocity_callback` looks dead and is not), and `self.simulator ==`
looks like an assignment to a careless regex, which would have deleted the
live MORAI branch. The final check plus a smoke test exercising CRUISE,
the governor, ACC and the braking path is what makes the removal safe.

### 46.8 Where 50 km/h stands

Best behaviour so far: **`pass`, min gap 1.19-1.74 m, peak deceleration
6.92 m/s²** (run 145947, before the coast-aware split). Peak is dominated
by engine braking — at the peak, coast alone accounted for 5.82 of 6.92,
leaving the controller responsible for +1.10.

**Not yet re-measured with the full stack live.** §46.1 means the coast
split, brake floor and early profile have never all run together in a
scenario. That is the next run, not a result.

Open: the pinhole range estimate (§45.8) remains published and unused, and
still measures far better than IPM beyond 30 m (bias +0.19 vs +1.62 m,
RMS 0.47 vs 1.86, flat with range).

---

## 47. `dev_jonas` review notes — verbatim, with editorial replies [REFERENCE]

Copied verbatim from `origin/dev_jonas`'s DEBUG.md (commit `35bf00e`,
Jonas Freyer) rather than merged — see §46.6 for why the branch was not
taken. His code change is superseded by §46.7, but the Stage 4 / Layer 2
KPI numbers are external evidence we have no other source for, and his
§47.3 review found a live bug. Editorial replies are marked **[reply]**
and are the only text added.

Landed on `dev_jonas`, branched from `5ee607f` (this section's own §45.3
governor rewrite), not yet merged to `main`.

### 47.1 Why this branch exists

Independent of this rework: the Stage 4 / Layer 2 CARLA KPI evaluation in
the KIT-IPEK MORAI Simulation Handbook repo
(`notebooks/acc_kpi_evaluation.ipynb`, run against the controller as of
commit `2ca2c47`, i.e. one commit before §45's governor rewrite) measured
achieved peak deceleration of **6.31 m/s² (v30 run) and 7.89 m/s² (v50
run)** against the 5.0 m/s² `DECEL_LIMIT` target — both over the R171
ceiling, and the v30 case notably over-braked relative to what was even
kinematically required (1.64 m/s² required vs. 6.31 achieved). That
matches this section's own §45.2 finding (brake authority varying 8-49x)
and was independently about to be patched (rate-limiting the old `a`
command, tightening `a_min`) when `5ee607f` landed and replaced the whole
acceleration-feedback cascade with the speed governor instead. The governor
is the better fix — it bounds deceleration by construction (reference-rate
limit) rather than by feedback on a ~208 ms-lagged, unpredictable-gain
plant — so the planned patch was dropped in favour of it.

### 47.2 What's actually in this branch

Purely mechanical: removed the now-dead `acc_control()`, `brake_for_decel()`,
`cruise_control()`, `_legacy_cruise_control()` and every `__init__` constant
that only they read (`k_p`, `k_d`, `a_min`, `a_max`, `throttle_scale`,
`brake_scale`, `a_ego`/`ACCEL_ALPHA`/`ACCEL_MIN_DT`/`prev_v_ego`/`prev_v_time`,
`brake_kp`/`brake_ki`/`brake_i_limit`/`brake_integral`, `BRAKE_AUTH_A0`/`A1`/
`MAX`/`MIN`/`FF_AUTHORITY`, `d_stop_margin`, `KIN_ENGAGE_MPS2`,
`KIN_RELEASE_MPS2`, `_kin_latched`, `closing_rate_filt`, `CLOSING_ALPHA`,
`prev_d_lead`, `MAX_TRACK_ERROR_MPS`, `d_lead_filtered`, `gain_scale`,
`ACC_GAIN_SCALE_MORAI`) — confirmed dead by grepping for call sites, cross-
checked against this file's own §45.8 "called 0x" note above. `ego_velocity_callback`
simplified to drop the now-pointless acceleration estimate. Module docstring's
stale `Control law: a = k_p*(...)` updated to describe the governor instead.

No behavioural change: `control_loop()`'s MODE 2/4 branches, `speed_reference()`
and `speed_control()` are untouched. 848 lines vs. 1330 before (-482).

### 47.3 Not changed, flagged instead

Reviewed `speed_reference()`/`speed_control()` for the same class of bug
(deceleration overshoot) rather than assuming the rewrite is bug-free.
Two things worth a second pair of eyes, not fixed here since neither is
verifiable without a CARLA run:

* `speed_control()`'s brake output (`min(-u, brake_cap)`) has no direct
  bound tied to `DECEL_LIMIT` — the limit only holds if the vehicle tracks
  `v_ref` closely enough that the PI's `-u` stays small, since §45.2's
  8-49x authority swing still applies to however much brake the PI *does*
  ask for. Re-running the Stage 4/Layer 2 CARLA KPIs against this branch's
  governor (no `ACC_Morai/` data exists yet either, see that notebook's
  section 6) would confirm whether achieved deceleration is actually
  staying under 5.0 m/s² now, rather than assuming the rewrite fixes it.
* `speed_reference()`'s rise/hold ratchet keys off the *sign* of
  `closing_rate` (`if closing_rate < 0.0: v_ref = min(v_ref, v_ref_last)`).
  With a lead at matched speed, `closing_rate` hovers near zero and can
  flicker negative on tracker noise even though the true situation is
  stable — each flicker ratchets `v_ref` down and it cannot rise again
  until `closing_rate` goes positive. Possibly harmless in practice (the
  alpha-beta tracker is smoother than the old derivative), but the
  condition arguably belongs on `gap_err`'s sign, not `closing_rate`'s.
  Worth watching for a "reference lags below set speed while steadily
  following a matched-speed lead" symptom.


### 47.4 Editorial replies

**[reply] on §47.1 — the 8-49x authority figure is retracted.** It appears
twice above (§47.1 and §47.3) as the justification for the governor. It was
mine, and it was wrong: the regression fitted `decel = brake * authority`
with engine braking left in the residual. Measured with coast removed and
the response lag aligned, brake authority is a stable **4.44 m/s² per unit
brake** (§46.3). The governor is still the right architecture — the actual
reason is that engine braking dominates deceleration and is not commanded
at all, reaching **6.57 m/s² uncommanded** (§46.2), which is itself above
the R171 ceiling. Conclusion unchanged, reasoning replaced.

**[reply] on §47.1 — the KPI numbers corroborate §46.2.** 6.31 m/s²
achieved against 1.64 required on the v30 run is exactly the signature of
uncommanded engine braking: at 30 km/h the coast table gives ~2.8-6.4 m/s²
depending on gear. The "over-braking" was substantially not the
controller.

**[reply] on §47.3 first point — correct, and still true.** `speed_control`'s
brake output has no bound tied to `DECEL_LIMIT`, and cannot have a useful
one, because the brake is not what sets deceleration here. The bound now
lives in the reference-rate limit (§46.5) and in the coast-aware split
(§46.3), which caps `a_des` at zero while a stop is latched. Achieved
deceleration under the current build is **not yet re-measured** — see
§46.8.

**[reply] on §47.3 second point — this is a real bug and it is still
present.** The ratchet keys off `closing_rate`'s sign:

```python
if closing_rate < 0.0:
    v_ref = min(v_ref, self.v_ref_last)     # controller_node.py:813
```

Behind a matched-speed lead `closing_rate` sits near zero, so tracker noise
flickers it negative and each flicker ratchets `v_ref` down permanently —
it cannot recover until the sign goes positive. The stationary-target
scenario never exposes this (closing rate is a solid -13.9 m/s), which is
why it has gone unnoticed; a following scenario would show it as "reference
drifts below set speed while steadily following". Jonas's suggested fix —
key the condition on `gap_err`'s sign instead — looks right. **Not fixed
here**, because it needs a moving-lead scenario to verify and the harness
only implements the stationary case (§43).

---

## 48. Throttle-cap fix kept; acquisition/lift rework tried and reverted; brake reserved for the last 20% [FIXED / REVERTED]

Three runs at 50 km/h, all `pass`:

| run | peak | min gap | lift duration | first brake |
|---|---|---|---|---|
| `165626` | 8.41 | 1.55 m | 2.38 s | 37.9 m = **52%** of acq |
| `170323` | **8.38** | **1.67 m** | 2.13 s | 42.4 m = **58%** of acq |
| `171238` | 9.44 | 1.05 m | 1.57 s | 31.0 m = 44% of acq |

### 48.1 Throttle capped at the set-speed hold value [FIXED — kept]

Approach throttle was a steady 0.445, but during the stop it spiked to
**0.83**, with 31 of 250 measure-phase frames above 0.45.

The `a_des <= 0` cap was supposed to prevent this and does bound throttle —
at `coast(v)/THROTTLE_AUTHORITY` for the **current** speed. Engine braking
peaks mid-range (§46.2), so that ceiling *rises as the vehicle slows*:

| v | old ceiling | now |
|---|---|---|
| 50 km/h | 0.46 | 0.46 |
| 34 km/h | **1.00** | 0.46 |
| 20 km/h | **1.00** | 0.46 |

Capped instead at `coast_decel(target_speed)/THROTTLE_AUTHORITY` whenever a
lead is tracked — the throttle needed to *hold* the set speed, 0.455 at
50 km/h. That the measured cruise value was 0.445 is independent
corroboration of the coast table at that speed; other speeds are not
cross-checked that way. Only while tracking a lead: free CRUISE still needs
full authority to reach set speed at all. Confirmed on `170323`.

### 48.2 Acquisition seed and lift shaping — tried, made it worse, reverted [REVERTED]

`165626` spent 24 m at full throttle after a 76.6 m acquisition, then
compressed the whole lift into 0.45 s. Three plausible causes were found
and all three were fixed together:

* **The seed fired on the second callback, not on real travel.**
  Perception republishes an unchanged distance when a frame drops — 76.6 m
  repeated for 0.44 s at acquisition — so the seed divided a zero
  displacement and seeded `v = 0`, the exact case it was written to
  prevent. With `track_v ~ 0` the governor believed it was behind a
  speed-matched car and held `v_ref` at the set speed.
* **One perception interval is not a velocity measurement.** Seed error is
  `sigma_d*sqrt(2)/dt` — at 12 Hz with ~1.5 m of IPM noise, ~25 m/s of
  noise on a 13.9 m/s estimate, which the latch then squares. Replaced with
  an 8 m baseline.
* **The lift used the quadratic ease-out**, which steps fastest at the
  instant a release begins. Replaced with a ramp-in.

In replay against a Gaussian noise model this looked unambiguous: latch
spread tightened 0.70–2.08 -> 1.00–1.60 m/s^2, and descent onset moved from
t+0.10…**t+2.80** s to t+0.30…t+0.90 s across 24 realisations, with no late
outliers.

**On the vehicle it regressed.** `171238`: peak **9.44** m/s^2 (worst of the
three) and min gap **1.05 m** (smallest). The lift came out *faster*
(1.57 s), not slower, which the release ramp alone cannot do — so at least
one of the three interacted with the governor in a way the replay did not
model. All three reverted to the `170323` state.

The lesson is the model, not the changes: the replay used Gaussian noise on
a regular staircase. Real IPM noise is neither, and the seed and latch are
both **nonlinear** in it (one divides by `dt`, the other squares the
result). A noise model that wrong cannot validate a change whose whole
purpose is noise robustness. Any retry needs the recorded gap trace from a
real run as the input, not a generator.

### 48.3 Service brake reserved for the last 20% of the approach [FIXED]

Across all three runs the brake came on at **44-58% of the acquisition
gap** — roughly 40 m out, and every application was adding deceleration on
top of a coast that was already more than sufficient. Coasting alone, from
the measured `COAST_DECEL` table:

| v | coasts to rest in | available from a 73 m acquisition |
|---|---|---|
| 30 km/h | 9.2 m | 63.8 m spare |
| 50 km/h | 27.6 m | 45.4 m spare |
| 70 km/h | 61.5 m | 11.5 m spare |
| 90 km/h | 106.7 m | **short by 33.7 m** |

So up to ~70 km/h the brake is not needed for the deceleration at all, only
for where the vehicle ends up. `_brake_allowed()` now holds it off until
the gap has closed to `BRAKE_INHIBIT_FRAC` (0.20) of the gap at which the
plan latched, making the brake a trim on the stopping point rather than a
participant in the peak. The `MIN_BRAKE_NO_THROTTLE` floor is suppressed in
the same window, or "no brake" would not be true.

**EMERGENCY is not gated by this.** It is a separate branch in
`control_loop` that returns before `speed_control` is reached. The inhibit
is an authority policy, not a safety interlock, and must never stand
between a collision and the brake.

### 48.4 Open

* The 90 km/h row above is the case this policy does not cover: coasting
  from a 73 m acquisition falls 33.7 m short, so the brake gets released at
  14.6 m with real speed still on the car. Untested — 90 km/h has not been
  run since the longitudinal rework. Expect `BRAKE_INHIBIT_FRAC` to need to
  scale with `v^2/(2*gap)` rather than stay a constant fraction.
* Peak deceleration is still dominated by the powertrain, not the
  controller (§46.2). §48.3 removes brake *on top of* coast, which should
  help; it does nothing about the 6.57 m/s^2 uncommanded coast term, which
  remains the binding problem for the R171 ceiling.
* §48.2's diagnosis of the seed bug still looks correct on its own terms —
  the zero-displacement path is real and reachable. It was reverted because
  the fix package regressed, not because the bug was disproved.

---

## 49. First full 30-point matrix — brake inhibit confirmed working, lateral control is the wall above 50 km/h [DONE / KNOWN]

Run `20260812_104249`, all 30 points of §43's matrix. Headline: **2 pass**.
That number is misleading in both directions and the detail matters more.

### 49.1 The brake inhibit (§48.3) does what it was asked to [CONFIRMED]

14 of 30 runs never commanded the service brake at all. Where it came on,
it came on at **3-9% of the detection gap** — comfortably inside the 20%
window, since the inhibit is keyed to the gap at which the *plan latched*,
not to first detection.

| speed | max brake | brake frames | first brake | % of det. gap |
|---|---|---|---|---|
| 30 km/h | 0.10 | ~20 | 2.1-2.3 m | 4-5% |
| 50 km/h | 0.10-0.51 | 19-33 | 1.9-3.5 m | 3-5% |

Two runs show brake 1.00 (`A_v70_off0`, `B_v70_off0_ttc4.5`) — that is
EMERGENCY, which is correctly not gated.

**And the vehicle still stopped.** At 30 and 50 km/h min gap was 1.9-2.3 m
with zero impact speed, on coast alone. That is the §46.2 claim confirmed
end-to-end: the powertrain, not the brake, does the stopping.

It also confirms the cost. Peak deceleration at those speeds was still
**7.3-8.3 m/s²** with the brake essentially absent. Removing brake from
the equation did not move the peak, because the peak was never the brake's.
Everything above 5 m/s² here is engine braking.

### 49.2 `no_reaction` is now a lying verdict [KNOWN — harness bug]

10 runs are labelled `no_reaction`, including 30 and 50 km/h points that
stopped cleanly at 2.2 m. The harness derives reaction from **brake
onset**, and §48.3 deliberately removed the brake. `brake_onset_gap_m`,
`reaction_latency_s` and `brake_onset_ttc_s` are all `nan` for a run that
reacted correctly.

The verdict must key on **deceleration onset** — the first sustained
departure from the approach speed — not on a brake command. Until that is
fixed the summary's verdict column cannot be read at face value on any run
where the inhibit was active. Not fixed here.

### 49.3 The real wall is lateral, and it is sharp [KNOWN]

Peak `|cte|` by speed, across all 30 points:

| speed | runs | max \|cte\| | departed (>1.5 m) |
|---|---|---|---|
| 30 km/h | 5 | 0.02-0.07 m (+1 outlier 8.66) | 1 of 5 |
| 50 km/h | 5 | **0.05-0.14 m** | **0 of 5** |
| 70 km/h | 5 | 0.19-8.20 m | 4 of 5 |
| 90 km/h | 5 | 4.13-8.88 m | **5 of 5** |
| 110 km/h | 5 | 0.24-7.58 m | 4 of 5 |
| 130 km/h | 4 | 4.39-8.49 m | 4 of 4 |

Below 50 km/h lateral tracking is essentially perfect. At and above 70 it
fails in 17 of 24 runs, with errors of 4-9 m — the vehicle has left the
road, not wandered within the lane.

**This invalidates most of the longitudinal results above 50 km/h.** A run
that departs takes its target out of the camera frame, so `min_gap_m`
becomes meaningless: `A_v130_off0_ttc6` reports a 324.83 m minimum gap and
`fail_collision` simultaneously, and `A_v110_off1_ttc6` reports 147.11 m.
Those are not longitudinal failures being measured — they are a departed
vehicle being measured against a target it drove away from. The high-speed
half of this matrix has to be re-run once lateral holds.

### 49.4 UFLD moved from every 4th frame to every frame [CHANGED — untested]

`inference_skip_n` was 4 on CARLA (~5 Hz), chosen when the target speed was
20 km/h. At 20 km/h a 0.2 s inference interval is 1.1 m of travel; at
130 km/h it is **7.2 m**. Every Stanley tick was acting on a lane estimate
up to a fifth of a second old, with the KF extrapolating across the gap —
which is the shape of the failure in §49.3 and matches the observed break
point.

Now `skip_n = 1` on CARLA: ~20 Hz, 0.7 m at 130 km/h. MORAI stays at 2 for
the GPU-contention reason already recorded there.

Changing the rate silently changed two other things. `KF_MAX_COAST_TICKS`
and `KF_MAX_REJECT_TICKS` were both **20 ticks, chosen as "≈4 s at 5 Hz"**.
At 20 Hz they become 1 s — which reinstates exactly the failure that
raising the coast window from 5 to 20 was meant to fix ("we don't RST
during sustained low-conf windows in curves"). The lane projection would
have started blinking out in curves, and it would have looked like a UFLD
regression rather than a units problem.

Both are now specified as durations (`KF_COAST_SECONDS`,
`KF_REJECT_SECONDS`) and converted to ticks in `__init__` from
`NOMINAL_CAMERA_HZ / skip_n`. The conversion reproduces the original 20
ticks at 5 Hz exactly, and gives 80 at 20 Hz / 40 at 10 Hz — 4.0 s in every
case. The KF's own dynamics were already safe: `dt` is recomputed per tick
from the message header, so only the tick-counted windows were exposed.

**Any constant counted in detector ticks has to move when the detector rate
moves.** These two were found by grep; the class was not audited for others
beyond `KF_LOG_EVERY`, which only affects log cadence.

**Untested.** The risk is the same contention that pushed MORAI to N=2 —
UFLD at a sustained 20 Hz competes with YOLO for the GPU, and MORAI's RTF
sagged to ~0.56 when it was tried there. If the control-loop rate or YOLO's
frame drops get worse, N=2 (~10 Hz, 3.6 m at 130 km/h) is the fallback
rather than a return to 4. Check both before reading the next matrix.

### 49.5 Open

* `A_v130_off0_ttc6.csv` contains no measure rows at all. Not diagnosed.
* `B_v30_off0_ttc4.5` departed (8.66 m) where every other 30 km/h run held
  to 0.07 m. Suspect the 4.5 s trigger puts the start line somewhere
  different rather than anything speed-related; not diagnosed.
* §48.4's prediction stands untested: at 90 km/h+ coasting cannot stop from
  a 73 m acquisition, so `BRAKE_INHIBIT_FRAC` likely needs to scale with
  `v²/(2·gap)`. No usable high-speed longitudinal data yet (§49.3).

---

## 50. CARLA's default off-throttle damping was the deceleration problem [FIXED]

Every peak this project has chased — 8.38, 8.41, 9.44 — was the powertrain
model, not the controller. Confirmed by open-loop coast-down
(`scenarios/coastdown.py`, new): sync mode, `fixed_delta = 0.01`, 16
substeps, no ACC, no ROS, no lead, `throttle = 0 brake = 0`.

### 50.1 The metric was not the problem, and it was worth proving [MEASURED]

The natural suspicion is differentiation artifact — jittered `dt`, dropped
or duplicated ticks, `get_acceleration`, sampling either side of a tick.
The harness runs **async with no `fixed_delta`** (`--sync` is off by
default, DEBUG §4), so this was a live hypothesis.

It is wrong. With nothing commanded, at 100 Hz evenly-spaced sync sampling:

| window | stock damping (2.0) |
|---|---|
| ±0.05 s | 10.51 |
| **±0.12 s** (the KPI's window) | **8.38** |
| ±0.25 s | 6.36 |
| ±0.50 s | 4.95 |
| mean over the coast | 2.11 (from 30), 2.50 (from 50) |

**8.38 is exactly what the matrix reported** (8.38, 8.41). The needles
survive clean sampling, so they are the vehicle. Velocity was also
cross-checked against the logged path: 7.551 m of x/y travel against
7.601 m from integrating reported speed over the peak window, 0.7% apart.
Nothing uses `get_acceleration` (`sim_adapter.py` differentiates
`get_velocity` deliberately).

**The needles are gearbox downshifts.** With gear logged:

```
30 km/h coast:  7.65 m/s^2 at gear 2->1, 23.2 km/h
50 km/h coast:  7.96 m/s^2 at gear 2->1, 22.9 km/h
                5.47 m/s^2 at gear 3->2, 32.4 km/h
```

which matches an independent finding from the matrix traces: samples above
6 m/s² cluster in two speed bands, ~21 and ~33 km/h, and the 30 km/h runs
show only the lower band because they never reach 33. Ratio 33/21 = 1.57,
a gear step.

### 50.2 `damping_rate_zero_throttle_clutch_engaged` [FIXED]

`get_physics_control()` on `vehicle.dodge.charger_2020`:

```
mass                                          1920.0   (correct, not a default)
moi                                              1.0
damping_rate_zero_throttle_clutch_engaged        2.0   <- CARLA default
damping_rate_zero_throttle_clutch_disengaged    0.35
damping_rate_full_throttle                      0.05
wheel damping_rate (x4)                         0.25
final_ratio 3.08, 8 forward gears, autobox on, gear_switch_time 0.1
```

`CarlaAdapter._apply_powertrain_fix()` now sets **0.4** on the VUT at
spawn, and verifies the write by reading it back — `apply_physics_control`
is fire-and-forget, and a silently-ignored write would look exactly like a
controller regression. The target keeps stock physics; it is stationary.

Effect, same coast-down:

| | stock 2.0 | fixed 0.4 |
|---|---|---|
| peak ±0.12 s | 8.38 | **4.73** |
| peak ±0.05 s | 10.51 | 6.62 |
| mean over coast | 2.50 | 0.55 |
| **distance to rest from 50 km/h** | **34.4 m** | **154.1 m** |
| distance to rest from 70 km/h | — | 313.9 m |

The 34.4 m figure is the whole explanation for §49.1: the vehicle stopped
from 50 km/h with no brake because it could not do otherwise.

### 50.3 `COAST_DECEL` re-derived — the old table measured a plant that no longer exists [FIXED]

The controller's coast map was a fingerprint of the stock damping. Every
entry is now wrong by 3-8x:

| v | old | new |
|---|---|---|
| 20 km/h | 6.00 | **0.79** |
| 30 km/h | 4.45 | **0.74** |
| 50 km/h | 2.73 | **0.63** |
| 70 km/h | 3.00 | **0.61** |

Re-measured from a 70 km/h coast-down, median per 1 m/s bin. The 5.5 and
8.5 m/s entries are **interpolated, not measured**: those are the 2->1 and
3->2 downshift speeds and the vehicle dwells in those bins while shifting,
so the raw bins read 2.81 and 2.51 against ~0.75 either side. Median does
not reject it — the transient is most of the bin. This table feeds a
feed-forward, so a shift transient in it would over-throttle at every pass
through those speeds.

### 50.4 What this invalidates [OPEN — do not skip]

The plant changed, so every constant tuned against it is now unjustified:

* **`THROTTLE_AUTHORITY = 6.0`** was cross-checked as "~0.43 throttle holds
  50 km/h where coast is ~2.4 m/s²". Both halves came from the stock
  vehicle. Needs re-measuring from a steady cruise trim.
* **The set-speed throttle cap (§48.5)** is `coast(v_set)/THROTTLE_AUTHORITY`
  and moves with the table: **0.455 -> ~0.105** at 50 km/h, 0.742 -> 0.124
  at 30. Whether that is still the right ceiling depends on
  `THROTTLE_AUTHORITY` being right, which it is not yet known to be.
* **`BRAKE_INHIBIT_FRAC = 0.20` (§48.3) is now indefensible.** It was
  justified by coasting overrunning the available distance — true at 34 m,
  false at 154 m. The brake is now genuinely required and must come back.
* **`BRAKE_AUTHORITY = 4.44`** was fitted with coast removed using the old
  coast values, so the residual it was fitted against was wrong.
* Every gain tuned across §45-§48 was tuned against 2-8x too much drag.

**Do not read the next matrix as a controller comparison against the last
one.** The plant is different; only same-plant runs are comparable.

### 50.5 Rollout — two bugs the change introduced [FIXED]

**`KF_REJECT_SECONDS` was never defined.** §49.4's edit converting the KF
tick windows to durations landed for `KF_COAST_SECONDS` and not for its
sibling, because the anchor text began mid-line in the real file and the
pattern assumed a line start. Every other replacement in that batch was
asserted; that one was not, so it failed silently and left `__init__`
reading an attribute that did not exist. `lane_detection_node` then crashed
on construction and the UI showed no lanes at all.

Now verified by AST rather than by eye: every `self.<CONST>` read in the
class has an assignment. **Assert on every scripted replacement — a silent
no-op inside a batch that otherwise succeeds is the worst case, because
the build passes and the failure surfaces somewhere unrelated.**

**`apply_physics_control` races the read-back.** §50.2's verification read
the damping value straight after writing it and aborted the run with
"asked 0.4, read 2.0". The first guess — that the write needs a server tick
— was wrong: an isolated test read 0.4 immediately. Reproducing the
harness's exact back-to-back spawn/apply/read against a live server, **2 of
6 trials** still read the stock 2.0, and one tick sufficed in every failing
case. It is a race, not a fixed latency, so the fix is retry (up to
`PHYSICS_APPLY_MAX_TICKS`), not sleep. The sync coast-down never hit it
because it ticks before reading.

Worth noting the verification earned its place: without the read-back the
run would have completed and produced numbers for the stock vehicle.

### 50.6 Brake inhibit removed — it became the dominant fault [FIXED]

First run on the corrected plant (`20260812_115103`, 50 km/h): `pass`,
final gap 3.41 m, but **peak 9.37 m/s²** — worse than anything before it.

The trace shows why, and it is not subtle:

```
   dt      v     gap   thr   brk   v_ref
+1.93   47.5   45.61  0.00  0.00   44.4
+2.88   44.8   33.41  0.00  0.00   27.2
+3.85   42.1   21.72  0.00  0.00   18.1
+4.18   41.2   17.95  0.00  0.00   16.1
+4.50   39.5   14.25  0.00  0.50   14.0   <- inhibit releases
+4.82   34.4   10.96  0.00  0.99   11.9
```

For 2.5 s and 30 m the vehicle ran up to **25 km/h above its own
reference** with both actuators idle. The governor was asking for the stop
correctly the whole time; `BRAKE_INHIBIT_FRAC = 0.20` forbade the only
actuator that could deliver it, until 14.25 m — exactly 20% of the latch
gap — at which point the loop did the only thing left and slammed 0.99.

The inhibit was sound against the stock powertrain, which stopped the car
in 34 m unaided (§48.3). At 154 m it inverted: what had been a way to keep
surplus deceleration out of the peak became the cause of the peak.

Removed entirely — constant, gate and `_brake_allowed()`. Braking is set by
the shape of the governor's profile again, which is where §45 put it.

Replayed through the recorded run with controller state carried forward,
first real brake application moves **16.1 m -> 45.0 m, 28.9 m earlier.**
That replay is open-loop and pessimistic: it feeds the recorded speed
(which never slowed, because nothing was braking) against the recorded
reference, so it shows the brake saturating at 1.00 for 1.5 s. In closed
loop the vehicle will actually decelerate and the error will close. **The
peak is not predicted by this — only a run will say.**

### 50.7 `BRAKE_AUTHORITY` re-fitted, and the peak was the standstill snap [FIXED]

Run `20260812_115623` — first with the brake actually available. Brake
onset moved to 45.8 m (from 16.1) and the reported peak fell 9.37 -> 8.06.
Two separate problems remained, and only one of them was the controller.

**The brake was 28% too strong.** `BRAKE_AUTHORITY = 4.44` was fitted with
the *stock* coast values in the residual, so it absorbed 2-8x too much
engine braking and came out low. The controller divides demanded
deceleration by it, so a number that small inflates every brake command.
Measured consequence: the loop reached 0.77 brake, overshot to **4.4 km/h
BELOW its own reference**, then sat at 16 km/h for 2 s and 9 m recovering.

Refit over the same run — braking samples only, throttle off, coast removed
with the corrected table, scanning response lag:

| lag | BA | resid RMS |
|---|---|---|
| 0 ms | 5.70 | 0.92 |
| **25 ms** | **5.69** | **0.88** |
| 100 ms | 5.79 | 1.05 |
| 300 ms | 5.54 | 1.41 |

n = 37. **5.69**, and unlike the old fit it is not lag-sensitive — 5.54 to
5.79 across the whole 0-300 ms scan. The old comment conceded 4.44 was
"only good to roughly +/-1"; this one does not need that caveat.

Effect at 40 km/h: a 4 m/s² demand now commands 0.60 brake instead of 0.77.

**The headline peak was CARLA zeroing the velocity.** 8.06 m/s² occurred at
**4.9 km/h with the brake at 0.10** — which is ~0.6 m/s² of commanded
deceleration. The vehicle went 5.6 km/h -> 0 within one 0.26 s sample at
the end of the stop.

`_peak_decel_from_trace` already claims to exclude the standstill snap; its
guard was simply set below where the snap happens. Raised
`DECEL_VALID_SPEED_MPS` 1.0 -> 2.0.

This is *not* the "widen it until it passes" move, and the evidence is that
the result does not depend on the choice:

| speed floor | peak | at | brake there |
|---|---|---|---|
| 1.0 m/s | 8.06 | 4.9 km/h | 0.10 |
| **2.0 m/s** | **6.67** | 29.2 km/h | 0.68 |
| 3.0 m/s | 6.67 | 29.2 km/h | 0.68 |
| 4.0 m/s | 6.67 | 29.2 km/h | 0.68 |
| 5.0 m/s | 6.67 | 29.2 km/h | 0.68 |

Anywhere from 2 to 5 m/s gives the same answer. Only 1.0 admits the snap.
The real braking peak is **6.67 m/s²** — still over the limit, and 29.2 km/h
sits in the 3->2 downshift band, so some of that is likely a shift
transient rather than the brake.

### 50.8 Brake ceiling, and the 4 m stop that was never d0 [FIXED]

**ACC brake capped at 0.4.** `0.4 * 5.69 = 2.28 m/s²` plus coast, so the
loop cannot command much over 2.9 m/s² whatever the profile asks. It costs
nothing nominally — a 99 m acquisition needs 1.0 m/s² and the latched plan
asks ~1.3, both far under it.

When the cap binds, the loop holds the brake **longer**, not harder: the
speed error persists, so the command stays saturated until the profile is
caught. That is what "brake more often" produces here. Explicitly *not*
pulsed braking — cycling the command would add jerk without adding
retardation, since mean force is what stops the car.

EMERGENCY is a separate branch and is not capped.

**The 4 m standstill gap was `STANDSTILL_WINDOW`, not `d0`.** Runs kept
finishing at 3.4-3.5 m with `d0 = 2.0`, which looked like the governor
targeting the wrong gap. It was not: `d_safe = d0 + T_gap*v_lead` = 2.0 for
a stationary lead, correctly, and the tracked gap was accurate to
**-0.24 m mean inside 25 m**, so perception was not biasing it either.

The actual mechanism is two things meeting:

1. CARLA snaps velocity to zero from ~6 km/h (§50.7). Measured, 6.1 km/h at
   3.93 m gap -> 0.0 at 3.54 m in one sample.
2. `control_loop`'s standstill branch accepted anything inside `d0 + 2.0`
   = **4.0 m** and stopped controlling.

So the final gap was set by wherever the sim's snap caught the vehicle, and
the 4 m window ratified it. Now `d0 + STANDSTILL_WINDOW` = 2.5 m; outside
that the governor keeps control and closes in on its own profile
(`v_ref = sqrt(2a(gap - d0))`, ~7 km/h at 3.5 m, zero exactly at d0).

**The plot now blanks the snap too.** `plot_run.py` recomputed
deceleration without the harness's speed guard, so the trace kept showing
an 8 m/s² needle at the end that the summary no longer counted. A plot
that contradicts the number beside it is worse than either alone — 33 of
227 samples in that run, all at or below 7.2 km/h.

### 50.9 Open

* 0.4 sits at the low end of realistic. The resulting drag (0.5-0.85 m/s²
  across the range) behaves more like coasting in neutral than in gear; a
  real car in gear at 50 km/h sheds nearer 1.0-1.5. If in-gear realism
  matters more than headroom, ~0.8-1.0 damping is the number to try, and
  `COAST_DECEL` must be re-derived again if it changes.
* The harness still runs async with no `fixed_delta`. It is not the cause
  of anything here, but sync at 0.01 is the right way to run a compliance
  measurement and the coast-down proves the setup works.
* Downshift needles are reduced but not gone (6.62 at ±0.05 s). Whether a
  60 ms shift transient should count against the R171 ceiling is a
  definitional question that still needs the regulation text.
* `BRAKE_AUTHORITY = 4.44` is still the pre-fix figure (§50.4). With the
  brake now doing real work it matters directly: `a_des` is clamped to
  `-DECEL_LIMIT`, so brake ~0.98 is *meant* to be exactly 5 m/s², and that
  mapping is only as good as the authority number. Re-fit it from the next
  run before reading the peak as a compliance result.
* `THROTTLE_AUTHORITY` and the §48.5 throttle cap remain unverified against
  the corrected plant. The cap is now ~0.105 at a 50 km/h set speed, down
  from 0.455, and the last run held 0.10-0.14 on approach — consistent, but
  that is one data point, not a calibration.

---

## 51. LKAS regression after the UFLD rate change — the KF's R does not know about sampling rate [FIXED — unconfirmed]

§49.4 moved UFLD from every 4th camera frame (5 Hz) to every frame (20 Hz)
to cut lane staleness at speed. Lateral got worse, not better.

### 51.1 The evidence, and how weak it is [MEASURED]

| run | skip_n | \|cte\| max | \|cte\| RMS | steer max |
|---|---|---|---|---|
| matrix `104249`, 5x 50 km/h | 4 | **0.05 - 0.14 m** | — | — |
| `120117` | 1 | 0.22 m | 0.068 | 0.049 |
| `120816` | 1 | **7.50 m** | 2.787 | **0.545** |

Two runs at the new rate, on identical code: one fine, one departed. That
is not a lot to go on, and it is stated plainly because the fix below is a
hypothesis, not a demonstrated cause. What *is* new is a departure at
50 km/h — the 5 Hz matrix never departed at that speed (§49.3), it only
failed from 70 upward.

Ruled out first:

* **Loop rate.** 18.9-19.2 Hz median in every run, worst-case sample gap
  64-75 ms. No sign of the GPU contention §49.4 warned about.
* **Throttle > 1 in the logs** (max 2.98). Present in the 5 Hz matrix too
  (max 4.10) — a pre-existing approach-phase logging artifact, not new and
  not lateral.
* **KF dynamics.** `LaneKalmanFilter._rebuild_matrices` rebuilds both `F`
  and the CWNA `Q` from the actual `dt` each step, and the CWNA
  discretisation is exact, so accumulated process covariance over a given
  wall-clock interval is rate-invariant. Q is *not* mis-scaled.

### 51.2 The mechanism: R is a within-frame number used as a between-frame one [FIXED]

`R` is `np.polyfit(xs, ys, deg=2, cov=True)[1]` — how well a quadratic fits
*that frame's* anchor points. It carries no information about frame-to-frame
independence, and the KF treats every update as an independent draw.

At 5 Hz that is roughly true. At 20 Hz consecutive UFLD detections run on
nearly-identical images, so their errors are strongly correlated: four of
them carry little more information than one. Treated as four independent
draws, `P` shrinks about 4x faster per second than the data warrants. An
overconfident `P` makes `S = HPH' + R` too small, the chi-square gate
(γ = 7.815) begins rejecting genuine lane changes, `n_rejected` climbs, and
the filter coasts on a stale prior while reporting high confidence.

The trace matches that signature exactly. In `120816`, before the
departure:

```
   t     cte    steer
 3.87   -0.38  -0.202
 4.08   -0.09  -0.248
 4.28   -0.29  -0.257
 4.72   -2.47  -0.458
 4.94   -4.46  -0.500
```

Steer grew steadily left while the true cross-track error was ~0, then the
error followed it out. The controller was tracking a lane estimate that had
stopped following the road — not over-reacting to an error, which is what a
gain problem looks like.

It also explains the intermittency: the gate only bites when the lane
genuinely moves, so a run that stays straight is unaffected.

### 51.3 The fix

Scale `R` by `infer_hz / 5`, so the filter extracts the same information
per **second** at any rate — the right invariant when the extra samples are
not independent.

| skip_n | rate | R scale |
|---|---|---|
| 4 | 5 Hz | **1.0** |
| 2 | 10 Hz | 2.0 |
| 1 | 20 Hz | 4.0 |

Exactly 1.0 at skip_n = 4, so the validated 5 Hz tuning (q = 0.5/5/5) is
preserved untouched and only the faster rates change. The node now logs the
scale at startup alongside the tick windows.

### 51.4 Open

* **Unconfirmed.** This needs a run at skip_n = 1 that holds lane at
  50 km/h, and ideally several — one good run proves nothing given 120117
  was already good.
* **If it still departs, fall back to skip_n = 2 before touching `q`.** The
  q values were validated against real data; this scaling was not. Changing
  the validated thing to compensate for an unvalidated one is the wrong
  order.
* The scaling assumes the detector's decorrelation time is ~0.2 s, which is
  where the reference 5 Hz comes from. That number is inherited from the
  original tuning, not measured. Measuring the actual frame-to-frame error
  correlation of UFLD would replace a plausible constant with a real one.
* §49.3's finding stands independently: even at 5 Hz, lateral failed in 17
  of 24 runs at 70 km/h and above. Whatever fixes 50 km/h here does not
  automatically fix that.

---

## 52. Run 20260812_122114 — brake cap and R scaling both hold; the last metre was the throttle cap [FIXED]

First run with §50.8's brake ceiling and §51's R scaling together.

| | before | this run |
|---|---|---|
| peak decel | 8.06 | **5.96** |
| brake max | 0.77 | **0.40** (at the cap) |
| \|cte\| max | 7.50 (departed) | **0.078** |
| \|cte\| RMS | 2.787 | 0.027 |
| verdict | pass | pass |

### 52.1 What is left of the peak is the gearbox [KNOWN]

5.96 m/s² occurs at **28.9 km/h with the brake at exactly 0.40**, its
ceiling. Decomposed:

```
  commanded by brake : 0.40 * 5.69 = 2.28 m/s^2
  remainder          :               3.68 m/s^2
```

28.9 km/h sits in the 3->2 downshift band (~32 km/h, §50.1). The controller
is now commanding less than half the peak it is charged with, and the
excess is the powertrain, as it has been since §46.2. Lowering
`ACC_BRAKE_CAP` further would not move this number much — 0.30 would trade
2.28 for 1.71 and leave 3.68 untouched.

### 52.2 The throttle cap prevented the last 1.4 m [FIXED]

The run finished at **3.38 m against a `d0` of 2.0** — §50.8's
`STANDSTILL_WINDOW` fix worked (the hold did *not* latch at 3.38, which the
old 4.0 m window would have done), and the governor correctly kept asking
for more: `v_ref` sat at 4.7 km/h for four seconds. The vehicle twitched
between 0.00 and 0.27 km/h and never moved.

Cause: §48.5's set-speed throttle cap,
`coast_decel(target_speed)/THROTTLE_AUTHORITY`. On the corrected plant
(§50.3) that is **0.105** at a 50 km/h set speed, and 0.105 will not launch
1920 kg from rest. The cap is a bound on how hard the loop may accelerate
*toward a lead at road speed*; applying it at standstill was never its
intent and it silently became a handbrake.

Lifted below `CREEP_SPEED = 2.0 m/s`. Nothing else bounds the loop there
except the governor itself, which is the right bound:
`v_ref = sqrt(2a(gap - d0))` is ~5 km/h at 3.4 m and reaches zero exactly
at `d0`, so full authority cannot run away. EMERGENCY still backstops at
1.0 m.

Simulated from the gap this run actually ended at: 3.38 -> 1.80 m, hold
latched. **The 1.80 is 0.2 m inside `d0` and comes from a crude plant
model** — worth watching on the next run, but not worth tuning against a
model this rough.

### 52.3 Two things that look like faults and are not

* **Deceleration "cut off" at the end of the plot.** That is §50.8's
  `DECEL_VALID_SPEED_MPS = 2.0` guard blanking samples below 7.2 km/h,
  where CARLA's standstill snap otherwise reports ~8 m/s² the controller
  never commanded. Intentional, and the harness and plot now agree.
* **Throttle sitting at 0.10 during the approach.** That is the set-speed
  cap doing its job — 0.105 is genuinely what holds 50 km/h on the
  corrected plant, down from 0.455 on the old one. It is only wrong at
  standstill (§52.2).

### 52.4 Open

* §51's R scaling is **still one good run**. 120117 was also good before
  it. Two consecutive clean runs at 50 km/h would be weak evidence; the
  useful test is 70 km/h and above, where 5 Hz failed 17 of 24 (§49.3).
* `THROTTLE_AUTHORITY = 6.0` remains unverified on the corrected plant
  (§50.4). The approach held 0.10-0.14 against a predicted 0.105, which is
  consistent, but that is one operating point.
* Whether a 3->2 downshift transient should count against the R171 ceiling
  is still open and still needs the regulation text (§50.9).

---

## 53. The 3.5 m stop was three separate blockers, not one [FIXED]

`STANDSTILL_WINDOW` (§50.8) was the original cause and is fixed — run
`20260812_122638` shows the mode column containing only `ACC` and `CRUISE`,
with **`STANDSTILL` never appearing at all**, because the vehicle sits at
3.46 m and the window is now `d0 + 0.5` = 2.5 m. The old `d0 + 2.0` = 4.0 m
would have latched there. That the mode disappeared is the fix working, not
a new fault.

What it exposed is that the vehicle could not close the last 1.4 m. Three
things blocked it, each hiding the next:

1. **The set-speed throttle cap** (§52.2) held throttle at 0.105, which
   will not launch 1920 kg. Lifted below `CREEP_SPEED`.
2. **`MIN_BRAKE_NO_THROTTLE`** then held 0.10 for 0.5 s *against* a `v_ref`
   of 4.8 km/h. A floor meant to give the no-throttle side of a
   deceleration a definite value was braking a stationary car away from its
   own target. Now suppressed when `v_target > v_ego + deadband`.
3. **`STOP_HOLD_S`** ended the run 1.0 s after the wheels stopped. Even
   with 1 and 2 fixed the creep needs ~2.0 s from 3.5 m, so the harness was
   recording a final gap the controller had not finished producing.

Simulated end-to-end from the gap this run actually reached:

```
   t=0.00  gap 3.46
   t=1.00  gap 2.75   <- old STOP_HOLD_S cutoff
   t=2.05  gap 1.81   STANDSTILL latches
```

### 53.1 The stop test now waits for the manoeuvre, not the wheels

`ego.speed < STOP_SPEED_MPS` held for 1 s is a reasonable definition of "at
rest" and a poor one of "finished": CARLA snaps velocity to zero from
~5 km/h, so a stop ends wherever the snap lands and the controller then
creeps the remainder. The test now also requires that the controller has
stopped asking for speed (`speed_reference` below `STOP_CREEP_VREF_KMH`, or
mode `STANDSTILL`), bounded by `STOP_CREEP_TIMEOUT_S = 6 s` so a controller
that never settles cannot hang a 30-point matrix. A run that times out
still records `stopped` and says so in `note`.

This is a change to the measurement, so worth being explicit: it does not
make any number pass. It lets the vehicle finish the manoeuvre it was
already performing before the final gap is read.

### 53.2 A bug the simulation caught before the vehicle did [FIXED]

The §53 brake-floor edit referenced `v_ref` inside `speed_control`, where
the parameter is named `v_target`. It compiled — the name resolves at
runtime, not import — and would have raised `NameError` on the first tick
the ACC branch ran, killing the controller node mid-approach.

It surfaced because the creep was being simulated against the real class
rather than reasoned about. Now also checked statically: an AST pass over
`speed_control` confirms every loaded name is a parameter or assigned
locally.

Third scripted edit in this strand to break on a name (§50.5 twice, this
once). **Compiling is not evidence that an edit is correct in Python.**

### 53.3 Open

* All three fixes are verified only against a crude plant model, which puts
  the final gap at 1.81 m — 0.19 m inside `d0`. If the real run undershoots
  similarly the answer is a gentler taper at low speed, not a wider window.
* `STOP_CREEP_TIMEOUT_S = 6 s` is a guess. If runs start hitting it, the
  creep is not converging and that is the thing to fix, not the timeout.

---

## 54. Plot: show what the KPI excludes, do not delete it [FIXED]

§50.8 made `plot_run.py` blank deceleration samples below
`DECEL_VALID_SPEED_MPS`, so the plot would agree with the harness's peak.
It agreed by erasing the last ~1.5 s of every trace — including real
low-speed braking — and the panel simply looked broken from 13 s onward.

Matching a plot to a number by hiding data is the wrong direction. Both are
now drawn: the excluded tail in grey, the valid samples in black, the
excluded span shaded and annotated with the reason. The standstill snap is
visible as an off-scale spike at ~13.4 s in run `20260812_122638`, which is
what the reader should see — it is the thing being argued about.

Also fixed while there: `pk = max(meas_d)` ran over a list containing
`nan`. `max([1.0, float('nan')])` is order-dependent in Python and returns
`nan` if the `nan` comes first, so the annotated peak could silently have
become `nan` on any run whose first measure sample was excluded. Now
filtered before the max, and the peak's time comes from the same tuple
rather than a second `.index()` lookup that would have found the wrong
sample on ties.

## 55. Curved-road (R171 §4.2.5.2.2) and lane-keeping (R79 Annex 8) harnesses [DONE]

Two new scenario scripts, a shared harness module, and a map survey tool.

```
scenarios/curve_survey.py        catalogues constant-radius arcs in every map
scenarios/curve_adapter.py       CarlaCurveAdapter + the surveyed site table
scenarios/scenario_common.py     bridge / camera pump / speed hold, shared
scenarios/r171_curved_target.py  R171 Annex 4 §4.2.5.2.2
scenarios/r79_lka_validation.py  R79 Annex 8 §3.2.1, §3.2.2, §3.2.5
```

`r171_stationary_target.py` now imports its bridge, camera pump, speed
hold, lane hold and process guards from `scenario_common` instead of
defining them. The move is code-identical (verified by AST diff — only
docstrings and an added `name` argument differ), and the refactored script
was re-flown at 30 km/h afterwards: `pass`, a_req 0.70, achieved
5.89 m/s², first detection 75 m, which matches §49's figures for that
point.

### 55.1 The survey's radius was wrong by the distance from the walk's start [FIXED]

`radius_of(samples)` computed the arc length as `samples[-1][0]` — the
s-coordinate of the last sample — rather than `samples[-1][0] -
samples[0][0]`. For a slice starting at index 0 the two agree, which is
why the function looked right in isolation; for the windowed radii the
constant-arc search runs on, it scales every radius by (distance from the
walk's start / window length).

The result was a site table where Town06 road 20 was published as a 400 m
curve. It is 40 m. Every radius in the first survey was inflated by
roughly 10×, and the curve sites chosen from it did not exist.

What caught it: `CarlaCurveAdapter.connect()` re-measures the radius from
the loaded map and refuses to run if it disagrees with the table by more
than 25 %. That check was written as belt-and-braces against map version
drift and instead caught the tool that produced the table. Keep it.

### 55.2 Stock CARLA has no R171-grade curve geometry; Town12 does [KNOWN]

R171 §4.2.4.1's reference section is a clothoid S-bend, first turn
R = 787 m, second turn R = 374 m. Measured across Town03/04/05/06/07/10HD:

| map | largest constant arc | usable arc length | lead-in |
|---|---|---|---|
| Town07 | 386 m | 38 m | 14 m |
| Town07 | 364 m | 34 m | 92 m |
| Town04 | **199 m** | **296 m** | **462 m** |
| Town04 | 182 m | 270 m | 126 m |
| Town06 | 144 m | 36 m | 132 m |

So on the small maps the tightest usable radius is 199 m, which reaches
the 3 m/s² M1 lateral limit at 88 km/h — the curved matrix cannot carry
110 or 130 km/h there at all.

Town12 (Large Map, minutes to load) does have highway geometry:

| site | R | arc | lead-in | note |
|---|---|---|---|---|
| t12_r1185 | 1185 m | 1026 m | 500 m | only site that fits 130 km/h |
| t12_r500 | 500 m | 276 m | 500 m | first-turn analogue, more severe |
| t12_r417 | 417 m | 460 m | 500 m | second-turn analogue, +11 % |
| t12_r345 | 345 m | 314 m | 210 m | second-turn analogue, −8 % |

§4.2.4.1 permits a different curvature "provided this does not change the
intention or lower the severity", so sites are chosen at or below the
reference radius wherever possible and every run records the radius the
map actually has. t12_r1185 is larger than the reference and is therefore
labelled a high-speed case rather than a substitute for §4.2.4.1.

All twelve sites were re-measured through the adapter against the live
maps: table vs measured agree to within 1 % (worst: t12_r417, 417.1 vs
412.3).

### 55.3 Arc length, not chord — and the ego's half-length [FIXED]

Two placement/measurement decisions the straight harness does not have to
make:

* **Gap is arc length along the lane.** The straight adapter projects onto
  the ego's forward axis. On a 200 m radius a 100 m separation has a 6.3 m
  sagitta, so the chord under-reads distance-to-go by 6 % and biases
  `a_req` by the same margin. `gap_m()` is arc length;
  `gap_projected_m()` keeps the chord in the trace because that is what a
  monocular range estimate approximates, so `gap_perceived_m` has
  something fair to be compared against.
* **The ego's half-length was missing from the placement.** `arm()` placed
  the target at `s_start + placement`, but both are centre positions and
  only the target's half-length was added back by the nudge. Every target
  landed 2.5 m closer than requested. Caught by the placement assertion
  (`-2.37 m off`) before a single run was recorded — the same assertion
  that caught the stale-pose bug in the straight adapter.

### 55.4 Cross-track on a curve cannot come from `get_waypoint()` [DECIDED]

§2 of the straight adapter's `lane_error` already records why a per-tick
waypoint lookup is unusable as a lateral metric: it snaps to whichever
lane is nearest, so a departure reads as an oscillation. The straight
scenario's answer was to measure against the start-line ray, which only
works on a straight.

The curve adapter builds the lane centreline as a polyline once at
`connect()` (1 m spacing, ~1 km) and projects the ego onto it. The
reference cannot re-snap, so the reported error is unbounded by lane
width — which is what a lane-departure criterion needs — and the same
projection gives the ego's arc position for free.

### 55.5 R79's lateral-acceleration measurement chain [DONE]

Annex 8 §2.4 requires ay at the CoG, sampled at ≥ 100 Hz, filtered with a
fourth-order Butterworth at 0.5 Hz, with jerk as the 500 ms moving average
of its derivative. The director loop runs at ~20 Hz, so:

* a CARLA IMU is attached at 200 Hz (measured 205.8 Hz in the first run)
  and drained per tick;
* samples are resampled onto a uniform grid before filtering — CARLA's
  sensor ticks jitter and a Butterworth assumes constant dt;
* **both** filterings are reported. `ay_peak_mps2` applies the
  regulation's filter causally (the compliance figure);
  `ay_peak_zerophase_mps2` is the zero-phase equivalent, which places the
  peak correctly in time. In a steady-state curve they agree; in the
  §3.2.2 transient they do not, and the difference is the filter's group
  delay, not the vehicle. §45.1 is the reason both are kept.

The IMU sits at the actor origin, roughly 0.6 m below the CoG, so a
roll-rate term rides on ay. CARLA does not expose the CoG height, so this
is recorded rather than corrected.

### 55.6 What R79 cannot be assessed for in simulation [KNOWN]

* **§3.2.3 overriding force** — needs 50 N measured at the steering
  control. CARLA has no steering-torque interface and the stack's steer is
  a normalised position command, so there is no force to measure.
* **§3.2.4 hands-on transition** — needs hands-on detection and the
  escalating warning chain of §5.6.2.2.5. The stack has no driver
  monitoring, so there is nothing to test. This is itself the finding: a
  Category B1 approval is not reachable without it.
* **§3.2.5 lane-crossing warning** — the crossing is measured; the warning
  is not, because nothing in the stack publishes a lane-departure signal.
  Reported as `not_assessable` with the crossing time attached, never as a
  pass. `--warning-topic` wires a Bool topic in the day one exists.

### 55.7 First results [DONE]

Single points, ADAS stack live, to prove the chain rather than to report:

* **Curved, t04_r199 (R = 199 m), 50 km/h, TTC 6 s:** handover at 83.3 m
  of arc (chord 81.4 m), target bearing +2.2°, already outside the
  straight-ahead corridor. First detection 101 m at +1.8°. Brake onset
  16 m → a_req 5.83 m/s², `fail_collision`. The straight matrix's 50 km/h
  point sits at 5.8–6.6 m/s² (§49), so the curve did not by itself move
  the reaction — worth confirming across the matrix before drawing the
  conclusion.
* **R79 lane keeping, t04_r076 (R = 76 m), 35.5 km/h, declared
  aysmax 1.5:** demand 1.27 m/s² (inside the 80–90 % window), measured
  peak 2.14 m/s² zero-phase / 2.30 causal, jerk 2.19 m/s³, |cte| max
  0.61 m, closest approach to the marking +0.26 m → `pass`.

  The 68 % overshoot of the demanded figure at curve entry is the number
  to look at next: it is inside §3.2.1's criteria (which are crossing and
  jerk only) but would breach §5.6.2.1.1's sustained allowance
  (1.5 + 0.3 = 1.8 m/s²) if it lasted, and it lasted 0.67 s.

## 57. The UI's interpreter has no scipy, and an errored run wrote defaults that read like results [FIXED]

The first R79 run launched from the UI drove the whole scenario and then
died in the analysis step with `No module named 'scipy'`. UI.py starts
scenarios with `CARLA_PYTHON`
(`/home/sirius/CARLA_0.9.16/carla-env/bin/python3`), which has numpy 1.24
but no scipy; every CLI run until then had used the system interpreter,
which has scipy 1.15.

Two separate defects, and the second is the dangerous one.

### 57.1 The dependency [FIXED]

`analyse_lateral` imported `scipy.signal.butter/lfilter/filtfilt` for
R79 Annex 8 §2.4's fourth-order Butterworth. The harness has to run under
whichever interpreter launches it, so the filter is now written out:
`butter_lowpass()` designs the cascade by bilinear transform with
prewarping, `_biquad` runs each section seeded in DC steady state (a
zero-state start rings for about a second at 0.5 Hz, and the R79 window
opens mid-corner, so that ring would land straight in the reported peak).

Verified against scipy on a realistic ay trace (step into a curve, 1.7 Hz
ripple, noise, 200 Hz):

| quantity | agreement |
|---|---|
| causal 4th order vs `lfilter` + `lfilter_zi` | 1.1e-8 |
| zero-phase peak vs `filtfilt` | 1.3e-4 m/s² (0.005 %) |

The zero-phase residual is `filtfilt`'s edge padding; the peak is what
gets reported and it agrees to four decimal places.

### 57.2 A failed run must not be readable as a measured one [FIXED]

The errored run still wrote a summary row — and every metric in it was a
dataclass default: `kept_lane=True`, `max_abs_cte_m=0.0`,
`min_marking_clearance_m=nan`, `window_valid=False`. Read straight, that
row says the vehicle held the lane perfectly with zero error. It says
nothing of the sort: nothing was measured.

`LkaMetrics.measured` is now set only when a run reaches the end of its
loop, and:

* the summary table prints `not measured — <reason>` instead of numbers;
* the kept-lane grid shows `?` rather than `.`;
* `plot_lka.py --sweep` drops unmeasured rows instead of plotting a
  default as a passing cell.

### 57.3 A crossing under the fallback is not the LKAS's crossing [FIXED]

While the LKAS is silent (§56.3) the scenario steers. A tyre over a
marking during that stretch was still being recorded as
`tyre_crossed_marking`. The check now requires `lateral_owner == 'lkas'`;
the outage is already reported, far more precisely, as `lkas_silent_s`.

### 57.4 Steering export [DONE]

`plot_lka.py --export-steering` writes `<run_id>_steering.csv`: time,
phase, who was steering, speed, the lane's radius and curvature, and the
four angles (commanded, required, feed-forward, realised) plus the
cross-track they were reacting to. Curve and exit rows only by default —
on the straight lead-in every angle is ~0 and the comparison says
nothing. `--whole-run` keeps everything.

### 57.5 The same point, measured [DONE]

t04_r199, lane_keeping, aysmax 1.5 → derived 57.4 km/h, R = 199 m, run
under the CARLA interpreter that previously failed:

| metric | value |
|---|---|
| verdict | **pass** |
| lane kept | yes, closest approach to the marking 0.67 m |
| max \|cte\| | 0.21 m |
| lateral demand | 1.28 m/s² (window 1.20–1.35) |
| ay peak | 2.75 m/s² causal / measured |
| jerk peak | 2.07 m/s³ (limit 5) |
| steer error vs geometry | rms 0.68°, mean +0.47° |

The mean +0.47° is the interesting figure: Stanley sits slightly outside
the geometric requirement for the whole curve, which is consistent with
the small positive cte bias in the same run and is what a steady-state
Stanley offset looks like.

## 58. The sweep died on Town12, and one failing site took the run with it [FIXED]

The first full 27-cell sweep stopped after nine cells. Timeline: last cell
written 17:49, next group was `t12_r1185`, and the CarlaUE4 process that
is running now started at 17:50 — the server went away while loading
Town12 into a session that had already been through Town03, Town04 and
Town10HD.

Three fixes:

* **A failing site no longer aborts the run.** Its remaining cells are
  recorded as `not measured — <reason>` and the next site is attempted.
  Before, the summary simply ended early with no indication why.
* **Large Maps get a 600 s RPC timeout** (`CarlaCurveAdapter.LARGE_MAP_TIMEOUT_S`)
  instead of the stock 120 s.
* **A memory preflight** refuses a Large Map when less than 8 GB is
  available, naming the fix (restart CARLA, run Town12 in its own
  session) rather than letting the load take the server down.

## 59. Every site after the first measured nothing, because the harness used load_world() [FIXED]

The overnight sweep produced 27 cells and 25 of them were meaningless.
Only the first site group — whichever town CARLA already had loaded —
had the LKAS in the loop; every group after a `client.load_world()` ran
with `/Car_1/cmd_steer` stale for the entire run while the harness's own
fallback held the lane. `kept_lane` came back True on nearly all of them,
which reads like a passing lane-keeping result and is not one.

### 59.1 It was in the repo the whole time

`UI.py:213`, `set_boot_map`:

> "Rewrite the three *.Map entries in CARLA's DefaultEngine.ini so the
> next server boot lands in the requested town. **(In-band load_world()
> segfaults on this install — the boot-map ini is the only reliable
> way.)**"

The harness called `load_world()` once per site anyway. The operator had
independently worked out the same thing from the other end — "I clicked
restart when switching the town" — which is what prompted looking here.

### 59.2 Evidence

| configuration | result |
|---|---|
| `t04_r076` alone, fresh process (one load_world at start) | **pass**, camera 13.7 Hz, coeffs 12.9 Hz, cmd_steer 20 Hz |
| `t03_r060` then `t04_r076`, one process | first passes; second silent 21.8 s, camera still 328 frames @ 13 fps |
| same pair, server rebooted between towns | **both steer**: silent 0.0 s, steer_max_age 0.05 s |

So it is not the town, not the site, and not the ADAS stack — a stack
that has been up for hours is fine. It is the second world inside one
server session.

### 59.3 The fix

`scenarios/carla_server.py`: `ensure_town()` writes the boot map, kills
the server, restarts it and waits until it reports the requested town.
Both scenario scripts call it before each site group; `--no-carla-restart`
falls back to the old `load_world` path for setups where CARLA is managed
elsewhere. A server already in the right town is left alone.

### 59.4 Three things that had to be fixed to make rebooting survivable

* **A control-sink error killed the ROS executor.** When a group ended, a
  late `/Car_1/cmd_steer` reached a sink still pointing at the destroyed
  ego: `AttributeError` inside a subscription callback, which rclpy
  re-raises out of the executor — killing the spin thread and every other
  topic with it. The sink is now detached in the group's `finally`,
  `apply_control`/`poll_camera` are no-ops on a closed adapter, and the
  sink call is wrapped so a failure detaches it instead of taking the
  process down.
* **The client outlived its server and core-dumped the process.**
  `connect()` calls `get_trafficmanager()`, which leaves a background
  connection and thread inside the client. Rebooting CARLA while that
  existed raised `carla::client::TimeoutException` on a non-Python thread
  — `terminate()`, core dump, no traceback that Python can catch.
  `close()` now shuts the traffic manager down, drops the client, world
  and map references and forces a `gc.collect()`.
* **Teardown order**: detach sink → stop pump (5 s join) → close adapter →
  only then reboot.

### 59.5 What the corrected run shows

`t03_r060` then `t04_r076`, 30 km/h, with the reboot:

| run | silent | steer err rms | verdict |
|---|---|---|---|
| t03_r060 | 0.0 s | 3.14° | pass |
| t04_r076 | 0.0 s | 2.33° | **fail — lateral jerk 7.50 > 5 m/s³** |

t04_r076 is the first real controller result from that site: the lane is
kept with 0.37 m of clearance and it fails on ride quality, not on
tracking. Every earlier conclusion drawn from a multi-site sweep should be
re-run before it is quoted.

## 60. Large Maps need a nested boot-map path, and a silent boot failure looks exactly like a slow one [FIXED]

A sweep reached the Town12 group, rebooted CARLA for it, and then sat
there. Nine cells were done; nothing moved for ten minutes.

State at the time: no `CarlaUE4-Linux-Shipping` process, nothing listening
on port 2000, and two `CarlaUE4.sh` entries in `Zs` (defunct) — the engine
had exited immediately and the harness was polling for a server that was
never coming.

### 60.1 The path

Stock towns are a single umap:

```
Content/Carla/Maps/Town04.umap   ->  /Game/Carla/Maps/Town04.Town04
```

Large Maps are a directory of streaming tiles with a master umap inside:

```
Content/Carla/Maps/Town12/Town12.umap  ->  /Game/Carla/Maps/Town12/Town12.Town12
```

`carla_server.set_boot_map` wrote the flat form for every town. The engine
starts, cannot find the map, and exits — no error the harness could see,
because `start()` sent stdout and stderr to `/dev/null`.

`boot_map_value()` now decides from the filesystem (`Maps/<town>/<town>.umap`
exists?) rather than from a hardcoded list, so a newly installed Large Map
works without editing anything. Measured after the fix: **Town12 boots in
20 s.**

The server's own map name reflects the same nesting — Town12 reports as
`Town12/Town12`, stock towns as `Town04` — which `is_town()` already
handled by comparing the last path element.

### 60.2 Three ways this hid itself

* **Output discarded.** `start()` used `DEVNULL`. It now writes
  `/tmp/carla_server_harness.log`, and a boot failure quotes its last
  eight lines.
* **No liveness check.** `wait_until_ready()` polled for the full 420 s
  without noticing the child had exited. It now takes the `Popen` handle
  and fails the moment `poll()` returns, naming the boot map it used.
* **The memory preflight was in the wrong place.** It lived in
  `CarlaCurveAdapter.connect()`, which runs *after* `ensure_town()` — i.e.
  after the old server has already been killed. Moved into `ensure_town`,
  before the kill, so a machine that cannot hold the map keeps the one it
  has.

### 60.3 Recovery, and a note on running processes

Booting Town12 by hand with the corrected path let the stalled sweep
continue: every remaining site was Town12, so its `ensure_town` found the
right town already loaded and skipped the reboot entirely. The first
Town12 cell then ran properly — `t12_r1185` at 30 km/h, lane kept, LKAS
silent 0.0 s, steer error rms **0.41°**, jerk 2.26 — the first genuine
high-radius result from this stack.

Worth remembering: a scenario process holds the version of
`carla_server.py` it imported at start. Fixing the file does not fix a run
already in flight.

## 61. t12_r417 was a broken site, not a broken controller [FIXED]

In the first complete sweep, `t12_r417` failed at **every** speed —
including 30 km/h, where |cte| reached 1.76 m and 50 km/h upwards ended in
a departure. On the same map at the same speeds, `t12_r500` held 0.09 m
and `t12_r1185` 0.11 m. A site that fails at 30 km/h while its neighbours
hold 0.1 m is not telling you about the controller.

The operator spotted it from the camera: there is a lane split shortly
before the bend, the left marking peels away to the left while the road
goes right, and the detector has two plausible ego lanes.

### 61.1 vet_site.py

`curve_survey.py` finds arcs; it never asked what the markings do on the
way in. `scenarios/vet_site.py` walks the approach backwards from the arc
entry and reports what a lane detector will meet:

| site | junction | road/lane changes | lanes either side | verdict |
|---|---|---|---|---|
| t12_r417 | 4 m | 3 | right only | dirty — 924 → 1103 → junction → 927 |
| t12_r345 | 40 m | 4 | right only | dirty |
| t12_r1185 | 64 m | 2 | both | dirty |
| t12_r500 | 0 m | 0 | both | **clean** |
| t12_r412 | 0 m | 0 | both | **clean** |

t12_r500 being the only clean Town12 site is exactly the one that
behaved, which is the correlation worth remembering: vet a site before
believing a result from it.

### 61.2 The replacement

Moving the entry later along road 924 does not help — the whole area is
interchange geometry, and every entry keeps a junction 72 m behind it. A
scan of 34 candidate (road, lane) pairs in Town12 with R 250-700 m, arc
≥ 120 m and lead-in ≥ 150 m found nine clean sites, of which three have
traffic lanes on both sides.

`t12_r417` → **`t12_r412`**: Town12 road 1157 lane -3, s = 632.7,
R = 412 m right, 130 m of arc, 500 m of lead-in, R_spread 0.02, middle
lane of three so UFLD gets a marking on each edge. Still a second-turn
analogue (+10 % on the reference 374 m, against the old site's +11 %).
The trade is arc length: 130 m instead of 460 m.

Measured immediately after the swap, and to be read against the old
site's failures at the same speeds:

| speed | ay | kept | \|cte\|max | clearance | jerk | steer err rms |
|---|---|---|---|---|---|---|
| 30 | 0.17 | yes | 0.13 m | 0.74 m | 0.24 | 0.43° |
| 50 | 0.47 | yes | 0.25 m | 0.63 m | 0.90 | 0.81° |
| 70 | 0.92 | yes | 0.28 m | 0.59 m | 1.96 | 1.18° |

All pass. The old site departed at 50.

### 61.3 Still to do

`t12_r345` and `t12_r1185` both vet dirty and their notes now say so.
t12_r1185's high-speed departures (90/110/130 km/h) were read in §56 as a
controller limit; with 64 m of junction in the approach that reading is
not safe yet. The small-map sites have not been vetted at all — each
needs its town loaded, so do it the next time one is up.

## 62. Stanley went unstable above 70 km/h because its loop gain grows with speed [FIXED]

**Symptom.** On `t12_r500` (R = 500 m, CLEAN approach), 30/50/70 km/h kept
lane and 90/110/130 km/h departed — `|cte|` pinned at 3.1–3.2 m, jerk 12–31
against R79's 5 m/s³ ceiling, runs aborting after 4.5–5.7 s.

### It was not the curve

`kappa_1pm` is `0.00000` for every logged sample of all three failures, and
`v_mean_in_curve_kmh` is `0.0`. They departed on the straight lead-in, before
the measurement window opened. That is also why `steer_rms_err_deg` and
`steer_ff_mean_deg` read `nan` in those summary rows — not a metric bug, the
window never opened.

### Mechanism

Regressing the published command against lane heading error over the traces:

| v [km/h] | lag(hdg→cmd) | K_ψ regressed | envelope growth/cycle | outcome |
|---|---|---|---|---|
| 70 | 0.20 s | 1.10 | 1.06 | rings at 0.8 Hz, passes |
| 90 | 0.28 s | 0.79 | **2.01** | departs t = 5.7 s |
| 110 | 0.28 s | 1.05 | 1.39 | departs t = 4.9 s, cmd peaks −42.8° |
| 130 | 0.28 s | 1.06 | 1.45 | departs t = 4.5 s |

Measured K_ψ ≈ 1.0 is exactly `STANLEY_HEADING_GAIN`, and at these speeds it
is the only term doing anything: at 90 km/h with `cte` = 0.07 m the
cross-track term `atan2(0.5·e, v+0.5)` contributes **0.08°** while the command
tracks heading error one-for-one (−1.72° against −1.75°). So the loop is pure
heading feedback through a dead time — `ψ̇ = v·δ/L`, stable only while

    STANLEY_HEADING_GAIN · v · τ / L  <  π/2

With K = 1.0, L = 3.048 m, τ = 0.20 s that crosses over at 86 km/h: 81 % of
the limit at 70, 104 % at 90. Predicted from two constants, and it lands
where the runs actually fail.

τ is perception, not transport — `steer_age_s` peaked at 0.048 s, so the
0.2 s is UFLD tick + IPM + KF. **Raising the detector rate is the durable
fix.** The gain schedule buys margin against the dead time we have.

### Fix: cap the feedback loop gain above 50 km/h

    g = min(1, V_GAIN_SCHED_MPS / v)
    δ = g·[ψ + arctan(k·e/v)] + arctan(κ·L)

`g` multiplies the **feedback terms only**. The curvature feed-forward is
open-loop, has no bearing on the criterion, and scaling it would just make
the car run wide in curves at exactly the speeds where holding the line
matters. Verified: steady-state δ on R = 500/200/60 is unchanged to
0.000000° at every speed, and every case is unchanged to 0.000000° at and
below 50 km/h.

Since the criterion is linear in K·v, freezing K·v holds the margin constant
(58 %) at all higher speeds instead of letting it erode.

### Why 50 km/h and not 70

70 was the obvious boundary — the fastest speed that passed before the
change, so the most conservative no-op. **Repeating the column showed 70 is
not a pass, it is a coin flip:** `|cte|` max came out 0.37, 0.23, 3.18 and
3.18 m over four runs of the same site, two of them departures. At 70 the cap
is inactive (`g` = 1.0) in all four, so that scatter is the *untouched
original controller* sitting at 81 % of the stability limit. Anchoring a
"safe" boundary to a value already known to be marginal is the opposite of
what the schedule is for. 30 and 50 repeated at 0.09–0.11 m in every run —
that is the region the tune is genuinely validated in.

This is the general trap: **a single passing run is not a pass.** Four runs
of one cell spanned 0.23 m to a departure with the code held constant.

### Result — `t12_r500`, `|cte|` max [m], two repeats per cell

| v [km/h] | 30 | 50 | 70 | 90 | 110 | 130 |
|---|---|---|---|---|---|---|
| no cap | 0.09 | 0.10 | 0.37 / 0.23 / 3.18 / 3.18 | 3.12 | 3.20 | 3.20 |
| cap @ 50 | 0.09 / 0.08 | 0.10 / 0.10 | 0.12 / 0.11 | 0.19 / 0.18 | 0.53 / 0.63 | 0.87 / 0.90 |

11 of 12 pass (`20260813_141200_cap50`), against 3 of 6 before. Low speed is
bit-identical by construction. The repeat spread collapsing — v70 from
{0.23 … 3.18} to {0.12, 0.11} — is the strongest single piece of evidence
that the marginal loop was the cause.

130 km/h remains on the line (clearance 0.00 m then −0.02 m). It no longer
oscillates: jerk 2.63/3.05 against 15.36, and it now runs *wide by 0.9 m*
rather than diverging. That residual is the cross-track term being too weak
to pull it back — gain `k/(v+ε)` is 0.0137 rad/m at 130 against 0.057 at 30 —
and `ay` demanded there is 2.61 against a declared aysmax of 3.0, i.e. near
the envelope anyway. Open, and a different problem from this one.

### Dead-time compensation: three variants, all rejected [DECIDED]

Do not re-derive these.

1. **Spatial preview** — evaluate `y(x)` at `x_p = v·τ`. Provably useless
   here: on a straight lane the slope is identical at every x, so it moves
   the heading term by exactly 0.0000° at every speed — and every failure was
   on the straight. On a curve it double-counts the bend the feed-forward
   already handles (−0.35° of excess steer at R = 500).
2. **Smith predictor from commanded steer** — `ψ̇ = (v/L)·δ − v·κ`. Exact if
   the car yaws at `v·δ/L`. Regressing measured yaw rate against `v·δ/L`
   gives 0.76 at 30 km/h falling to 0.22 at 90 (part real tyre slip, part
   phase corruption from regressing an oscillating signal — unusable either
   way). Bench-simulated at those gains it was the *only* variant worse than
   doing nothing: 6.33 m peak offset at 130 against a 1.58 m baseline,
   because over-predicting the yaw makes it under-command.
3. **Model-free derivative lead** — `ψ̂ = ψ + T·dψ/dt` from successive coeff
   messages, EMA-smoothed, clamped at 5°. Flown live at T = 0.15 s
   (`20260813_135414`) and worse than the cap alone at every speed:

   | steer err rms [deg] | 30 | 50 | 70 | 90 | 110 | 130 |
   |---|---|---|---|---|---|---|
   | cap + lead 0.15 s | 0.58 | 1.70 | 9.92 | 11.44 | 1.53 | 2.03 |
   | cap alone | 0.27 | 0.41 | 0.76 | 0.77 | 0.36 | 1.47 |

   A derivative across a ~10 Hz signal is dominated by perception noise, not
   by the heading swing it was meant to lead. The clamp bounded it without
   fixing it (v50 peak steer error +10.75°, one clamped spike per noisy
   pair). This is a *sample-rate* defect rather than a design defect — the
   same lead on a 20–30 Hz detector has a far better noise floor — but it
   does not pay at the rate perception actually runs, so it is not in the
   tree.

### Also worth knowing

The baseline high-speed runs had `imu_rate_hz` at 32–66 against a nominal
100, and ~10 fps camera; the post-change runs sit at 85–104 Hz. Some of the
baseline's severity was a starved simulator. The effect survives that — the
capped runs are bounded across two repeats where the uncapped departed in all
three — but do not quote the exact crossover speed as a property of the
controller alone.

Live override, no rebuild: `-p v_gain_sched_mps:=1e9` restores the exact
pre-§62 formula. That is how the before/after columns above were measured.

## 63. The sweep's tyre cap is looser than R79's own limit, and the matrix hid it [FIXED]

**Question that surfaced it.** "Why is R = 42 m only tested at 30 km/h — would
higher speeds exceed the 3 m/s² R79 threshold?" Yes, and the more useful
answer is that the sweep's cap and the regulation's limit are two different
numbers, and the looser one was doing the gating.

`build_sweep` drops any cell whose geometric demand `v²/R` exceeds
`SWEEP_AY_CAP = 4.5` m/s² — a **plant** limit ("past roughly 0.45 g this
vehicle understeers out of the lane whatever the controller does"). R79's
limit is a **regulatory** one: aysmax is declared at 3.0, and §5.6.2.1.1
allows a transient of `min(1.4·aysmax, table_max + 0.3)` = 3.30.

Highest speed each radius allows, by which limit is applied:

| site | R [m] | aysmax 3.0 | ceiling 3.30 | tyre cap 4.5 | actually run |
|---|---|---|---|---|---|
| t12_r1185 | 1185 | 215 | 225 | 263 | 30–130 |
| t12_r500 | 500 | 139 | 146 | 171 | 30–130 |
| t12_r412 | 412 | 127 | 133 | 155 | 30–130 |
| t04_r199 | 199 | 88 | 92 | 108 | 30–90 |
| t04_r076 | 76 | 54 | 57 | 67 | 30–50 |
| t03_r060 | 60 | 48 | 51 | 59 | 30–50 |
| t10_r042 | 42 | **40** | 42 | 49 | 30 |

So R = 42 m stops at 30 km/h because 50 km/h demands 4.59 m/s² — over the
4.5 cap, and **1.5× the declared aysmax**. At 70 it would be 9.00, 3× the
declaration. Those are not lane-keeping tests at all: above aysmax the
applicable paragraph is §3.2.2 (maximum lateral acceleration), where the
correct system behaviour is to *refuse* the demand and run wide. A departure
there would be compliance, not failure.

Because 4.5 > 3.30, three cells in the grid already demand more than the
declaration and get run anyway: t12_r412 @ 130 (3.17), t04_r199 @ 90 (3.14),
t03_r060 @ 50 (3.22). Left as-is deliberately — they are useful as controller
characterisation, and §3.2.1.2's verdict (crossing + jerk) is still
meaningful — but they are not R79 lane-keeping points and should not be
quoted as compliance evidence. Run with `--sweep-ay-cap 3.3` for a grid that
only contains cells the regulation would actually ask for.

### The matrix was drawing one channel of two

`plot_matrix` coloured cells by `kept_lane` alone. On the 27-cell sweep that
showed 3 red squares while **5** runs had breached the ay ceiling — the two
missing ones being cells that kept the lane while exceeding it:

| run | peak ay | ceiling | kept | verdict |
|---|---|---|---|---|
| t12_r500 v130 | 3.88 | 3.30 | no | fail |
| t12_r412 v130 | 4.37 | 3.30 | no | fail |
| t04_r076 v30 | 5.78 | 3.30 | **yes** | fail (jerk) |
| t04_r076 v50 | 10.89 | 3.30 | no | fail |
| t03_r060 v50 | 4.84 | 3.30 | **yes** | **pass** |

`t03_r060 v50` passing while pulling 4.84 against a 3.30 ceiling is *by
design*, not a bug: `_verdict` scopes a sweep cell to §3.2.1.2's two
questions (did a tyre cross, was jerk ≤ 5 m/s³) and routes §5.6.2.1.1
breaches to the note. Defensible — but it means the acceleration envelope is
a genuinely independent axis, and a plot that renders only the crossing
channel makes a green square look unambiguous when it is not.

Fixed by giving it its own visual channel rather than folding it into the
fill: **amber border = ay over the §5.6.2.1.1 ceiling**, fill still
crossing/kept. Each cell now carries both numbers — `demand` (v²/R, what the
cell was built from) and `peak` (measured zero-phase ay, against the
ceiling). The legend counts the breaches so the number is on the figure.

The border flag reads the harness's own `ay_within_limits` column rather than
recomputing a threshold in the plotter; `ay_ceiling()` reads `ay_table_max`
back from the manifest for the *displayed* number, so a plot can never
quietly disagree with the verdict it is drawing. Same reason `peak` uses the
zero-phase figure: that is what §5.6.2.1.1 is judged on and what the run note
quotes, and the causal peak lags its own filter.

## 64. Trace plots trim 2 s off each end [DONE]

`plot_lka.plot_run` now drops the first and last `TRIM_EDGE_S` (2.0 s) of
every trace before drawing. Both ends are harness artefacts, not controller
behaviour: the run opens with the teleport settling and the scenario's own
warm-up centreline hold, and it ends either at the departure abort or at the
tail past the arc where the lane demands nothing. Drawing them compresses the
y axis around transients nobody is trying to read — on `t12_r500_v130` the
useful 0.5 deg of steering detail was sharing an axis with the settling
transient.

Three things keep it honest:

* **It is a display window, not a measurement one.** Every statistic
  annotated on the panels (`steer_rms_err_deg`, `steer_mean_err_deg`, the
  crossing time) comes from `summary.csv`, which the harness computes over
  its own window. The trim therefore cannot move a reported number — but the
  two windows are now visibly different, so the time axis states which one is
  drawn. Silently trimming an axis while leaving statistics unqualified is
  how a reader measures one window with another's ruler.
* **Short runs are not destroyed.** A departure can abort in under 4 s;
  `trim_edges` refuses below `TRIM_MIN_KEEP_S` (3 s) or `TRIM_MIN_KEEP_N`
  (20 samples) and draws the whole run with a note saying why. In the 27-cell
  sweep the shortest survivor keeps 5.2 s (`t04_r076_v50`), so this only
  bites on aborts.
* **`--trim-s` overrides it**, `0` disables. `--export-steering` is
  deliberately *not* trimmed: it is the raw comparison the CSV exists to
  provide.

Only `plot_run` is affected. `plot_sweep` and `plot_matrix` read scalars from
`summary.csv` and have no time axis to trim.

## 65. Curved R171: the ACC brake cap turned late detection into collisions [FIXED]

Block `20260813_153123_curve_t04_r199` (R = 199 m, stationary target 35 m
into the arc): **5 of 12 runs ended in a collision**, at 26-58 km/h impact,
including runs whose geometry was comfortably stoppable.

### Detection, and what is NOT causing it

Ten of twelve runs first perceived the target at **24-33 m**. Two saw it at
92 and 101 m. Same site, same target.

`gap_perceived_m` and `gap_pinhole_m` turn non-null on the *identical*
sample in all twelve runs, so whatever gates it sits **upstream of both
range estimators** — it is the ego-lane selection in `perception_node`, not
IPM, and not `MAX_IPM_TRUST_M`. Raising a range ceiling cannot delay a
detection in any case.

The gate is `_bb_intersects_centerline`, and specifically its fallback.
When UFLD's polylines do not reach the target's range the test returns
`None`, and the code falls back to a **straight ±`LANE_FALLBACK_HALF_W`
(1.5 m) corridor at Y = 0**. On a curve that assumption is wrong: a target
35 m into an R = 199 m bend sits 35²/(2·199) ≈ **3.08 m** off the ego's
tangent while genuinely in the ego lane. Measured lateral offset at first
detection is 2.1-3.2 m in every run — twice the corridor. The target is
therefore rejected until the ego has rotated far enough into the bend for
its ground-Y to fall inside ±1.5 m, which is exactly the 24-33 m observed.
The two early acquisitions are the runs where UFLD's centreline did reach,
so the real (curved) test ran instead of the fallback.

**Open.** The fallback needs to bend with the road — the ego's own
curvature estimate is already available from the lane KF (`kappa`), so the
corridor could be swept along an arc instead of a straight line. Not
attempted yet; recorded so the next person does not re-diagnose it as YOLO.

`MAX_IPM_TRUST_M` was nevertheless **restored to 80.0** — it had drifted to
150.0 while every word of its comment still said 80, i.e. the far half of
its range was publishing distances measured at ~38 % error. This costs the
two good acquisitions (92 m and 101 m are now clipped to 80) and is a
deliberate trade: a +38 % gap feeding a stopping profile is worse than none.

### The cap is what made it a collision

`ACC_BRAKE_CAP = 0.4` bounded ACC service brake at 0.4 × 5.69 = 2.28 m/s²
plus coast. Its comment argued the cap "costs nothing … when it does bind,
the loop's answer is to hold the brake LONGER, not harder". That holds only
when there is time left to be long in.

`A_t04_r199_v50_off0_ttc6`, from the trace: target perceived at 30 m at
50 km/h; brake command sits pinned at **exactly 0.40 for 1.3 s** while
achieved deceleration plateaus at ~2.6 m/s² and the required rate `v²/2s`
climbs through 5 and past 8. EMERGENCY (uncapped, brake = 1.0) only arms
below a 1 m gap, with 32.6 km/h still on the clock. Impact at 29.0 km/h.

The stop needed ~4.5 m/s² from 30 m — inside both the tyres and R171, and
simply not commandable. **`ACC_BRAKE_CAP` is now 1.0.**

Consequence, stated plainly: ACC can now command more than 5 m/s², so an
R171 deceleration breach is reachable through this branch instead of being
structurally impossible. That is the accepted trade for now — stopping
beats a compliant collision — and bounding deceleration belongs in the
PROFILE (`a_profile` / `DECEL_LIMIT`), where it is designed, not in a brake
clamp. Operator is aware and will take the 5 m/s² limit separately.

### The standstill hold was stopping the car early

`STANDSTILL_WINDOW` was 0.5 m, so the hold latched at `d_lead < d0 + 0.5`
and *ended control there*. Whatever gap the car happened to be at when the
window admitted it became the final gap. `A_t04_r199_v30_off0_ttc6` latched
on a tracked gap of 2.36 m and finished with **3.55 m of real gap**.

Now 0.0, and the test is `<=`: the hold latches AT d0 = 2.0 m and the
governor keeps control all the way down, where `v_ref = sqrt(2a(gap-d0))`
reaches zero exactly at d0. The hold itself is kept — it exists to stop the
derivative term chasing estimator noise at rest — it just no longer fires
before the car has arrived.

Note the last metre is closed against the TRACKED gap, which reads ~1.2 m
short of truth at close range as the bounding box clips. Resting gap is
still set by estimator bias, now conservatively. Perception problem,
deliberately not papered over in the controller.

### Plotting

`plot_acc.py` already handles curved traces unchanged — same columns plus
`cte_m`. Added:

* **lateral acceleration** on a twin axis of the lateral panel, computed as
  `|v·ψ̇|` from the logged path because this trace has no ay column
  (`accel_mps2` is longitudinal). Same estimator as `plot_lka.py`.
* **lane kept/crossed** in the title and header strip. The curved-R171
  summary has no `kept_lane` column — that scenario scores the ACC side —
  so it is derived from `cte_m` against R79's tyre-on-the-marking rule
  (`LANE_HALF_W - TYRE_HALF_W`), and reports "not measured" rather than
  "kept" when the trace carries no cte.
* removed the grey "excluded from KPI" overlay from the deceleration panel
  (§54). The masking still drives the reported peak and the event markers —
  the sim's standstill snap cannot be quoted as a braking result — only the
  two-tone drawing is gone.

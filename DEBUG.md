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

# Working notes for Claude

Instructions for anyone (including Claude) working in this repo.

## Documentation is part of the change, not a follow-up

**1. Every significant change goes into [DEBUG.md](DEBUG.md).**

DEBUG.md is the engineering record: what broke, why, and what we changed.
Add a numbered section (check the tail of the file for the next free number
— it is easy to collide when several strands run in parallel). Mark it
`[FIXED]`, `[DONE]`, `[DECIDED]`, `[KNOWN]` or `[PLANNED]` per the legend at
the top of that file.

Write down the things that will otherwise be rediscovered the hard way:

* **The measurement you trusted that turned out to be wrong.** §45.1 exists
  because three tuning passes were built on an acceleration signal that was
  lagged by its own filter. That cost more time than any real bug.
* **The fix that looked right and made things worse.** §45.3's clamped
  reference is recorded precisely so nobody re-derives it.
* **Numbers, not adjectives.** "IPM error ~38% beyond 40 m, measured
  +16.7..+29.2 m against ground truth" is useful. "IPM is inaccurate at
  range" is not.
* **What is still open**, with enough context to pick it up cold.

**2. Keep [README.md](README.md) current with the newest version.**

DEBUG.md is the history; README.md is the present tense. When behaviour,
topics, tuned values or the run procedure change, update README in the same
change — especially:

* `### ACC tuned values` / `### LKAS tuned values` / `### KF tuned values`
  — these are quoted in reports and go stale silently.
* `## ROS graph` when topics are added or renamed.
* `### Run` when the launch procedure changes.

If a change makes a README section wrong, fixing it is part of the change.

## Layout

```
src/perception/        YOLO lead detection, UFLD lanes, IPM, debug images
src/controller/        ACC longitudinal (controller_node), Stanley lateral
src/morai_bridge/      MORAI <-> /Car_1/* adapters
scenarios/             UN R171 harness — see scenarios/README.md
scenarios/results/     run output; plot_run.py renders traces
UI.py                  orchestration + telemetry
start_adas.sh          launches the stack (carla | morai)
```

## Running a scenario

CARLA must be up, then:

1. **Start CARLA**
2. **Run start_adas.sh** — check the UI's Simulator dropdown reads `carla`;
   it defaults to `morai`, which silently applies 10x-softer ACC gains and
   the wrong speed-message type.
3. **Start the scenario** (UI: *Run single point* / *Matrix*)

**Do not start the bridge for a scenario run.** The harness *is* the
CARLA<->ROS bridge for the duration — running `carlaAccSimTown.py` as well
puts two writers on `/Car_1/cmd_vel` and the ego's `apply_control`.

Plot afterwards with `python3 scenarios/results/plot_run.py`.

## Things that have bitten us more than once

* **The Simulator dropdown defaulting to `morai`** — root cause of
  "throttle pinned at 1" and "v = 0" several times over.
* **Measuring through a filter.** Any conclusion drawn from a low-passed or
  EMA'd signal needs the filter's group delay accounted for first. Prefer a
  centred window offline; it has no lag and does not amplify noise the way
  raw differencing does.
* **Metrics that hide the failure they exist to reveal.** `cte` was computed
  from a per-tick waypoint lookup, so a lane departure was reported as an
  oscillation. Check what a metric does at its boundaries.
* **Latency introduced by restructuring, not by logic.** Moving steer from
  an event callback to a polled loop added 52 ms and destabilised a lateral
  controller nobody had touched.
* **Brake authority in CARLA is not predictable** — measured 8 to 49, and
  2-3x within one speed band. Do not calibrate a brake-force feed-forward
  against it; bound deceleration in the trajectory instead (DEBUG §45).

## Conventions

* Distances on `/ACC/*` are **bumper-to-bumper gaps**, not camera-to-object.
* Vehicle frame is REP 103: X forward, Y left.
* `steer` is normalised [-1, 1], positive = right.
* Comments should say *why*, and cite the measurement or DEBUG section that
  justifies a constant. A tuned number with no provenance is a number
  nobody can safely change later.

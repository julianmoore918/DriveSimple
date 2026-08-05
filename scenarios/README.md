# Scenario harness — UN R171 (DCAS) Annex 4 §4.2.5.2.1

Stationary vehicle ahead, straight road. Longitudinal-only, single target.
CARLA is implemented; MORAI is a documented seam (`MoraiAdapter`).

```
src/                          the ADAS stack under test (unchanged)
scenarios/
  r171_stationary_target.py   scenario director + ROS bridge + metrics
  sim_adapter.py              SimAdapter ABC | CarlaAdapter | MoraiAdapter
  results/<timestamp>_<tag>/  per-run traces, summary.csv, manifest.json
```

## Running

The ADAS stack must be up first; the harness waits for YOLO and UFLD to
report ready before it arms anything.

```bash
./start_adas.sh carla

# one scenario point
python3 scenarios/r171_stationary_target.py --speed-kmh 50 --offset-m 0 --ttc-s 6

# the full 30-point matrix, or one block
python3 scenarios/r171_stationary_target.py --matrix
python3 scenarios/r171_stationary_target.py --matrix --block A

# print the matrix without touching a simulator
python3 scenarios/r171_stationary_target.py --list
```

Or from `UI.py` → **Scenario — UN R171 stationary target**.

> The harness **is** the CARLA↔ROS bridge while it runs. It replaces
> `carlaAccSimTown.py` and refuses to start if that is running — both
> would write `/Car_1/cmd_vel` and the ego's `apply_control`, and the last
> writer per physics tick would win at random. The UI stops the bridge for
> you.

## The three parameters

| Flag | Values in the matrix | Meaning |
|---|---|---|
| `--speed-kmh` | 30 / 50 / 70 / 90 / 110 / 130 | VUT approach speed |
| `--offset-m` | 0.0 / 0.5 / 1.0 | target offset from lane centre, +ve = right |
| `--ttc-s` | 4.5 / 6 / 10 | TTC margin → handover distance `ttc × v` |

Block A = speed × offset @ TTC 6 s (18 points).
Block B = speed × TTC @ offset 0 m (12 further points; its TTC 6 s column
is already in Block A). 30 total.

## How a run is flown

```
ARM        ego on the start line at rest; stationary target placed
           (settle + ttc) × v ahead, bumper-to-bumper. Placement is
           asserted against the request before anything is recorded.
APPROACH   the scenario owns throttle and holds the test speed. The
           stack's ACC runs and publishes, but its cmd_vel is ignored.
HANDOVER   at ground-truth gap == ttc × v, the stack's ACC takes the
           longitudinal channel. This is the DCAS trigger point.
MEASURE    20 Hz trace until the VUT stops, hits, passes, or times out.
```

Lateral is held by the scenario on the lane centreline (`--lateral-mode
locked`, the default) so lateral wander cannot pollute a longitudinal
measurement. `--lateral-mode lkas` puts Stanley in the loop instead.

## The KPI

`a_req = v² / (2 × gap)` — the constant deceleration needed to stop in the
remaining gap. At the handover point this reduces to `v / (2 × ttc)`, so
the scenario's own difficulty is set purely by its parameters. Every metre
the system spends not braking after that drives `a_req` up.

The headline comparison is **`a_req_at_brake_onset_mps2` vs 5 m/s²**
(`--decel-limit`). Verdicts:

| verdict | meaning |
|---|---|
| `pass` | collision avoided and `a_req` at brake onset stayed under the limit |
| `pass_over_limit` | avoided, but only by demanding more than the limit |
| `fail_collision` | contact |
| `no_reaction` | the system never braked |

`unavoidable_at_onset` flags runs where `a_req` at brake onset already
exceeded ~9 m/s² (dry-asphalt tyre capability) — the impact was locked in
by the time the system acted, and the raw figure (which can reach several
hundred) stops carrying meaning.

`gap_at_first_detection_m` records where `/ACC/lead_vehicle_distance`
first went valid. It is usually the column that explains a late reaction.

## Why Town06 spawn 80

The matrix needs `(settle + ttc) × v` of dead-straight road — 469 m at
130 km/h with a 10 s margin. Measured straight runway, walking each lane
forward until it deviates 1 m from the start heading:

| map | best spawn | straight runway |
|---|---|---|
| Town03 | 264 | ~116 m |
| Town06 | **80** | **718 m** |
| Town06 | 74 | 834 m (merge-heavy from 20 m) |

Town03 spawn 5 — the original plan — has **0 m**: it sits 2 m from a
junction. Spawn 80 is road 48, centre lane of five, 3.5 m wide, with
neighbour lanes both sides so UFLD has clean markings on both edges.

## What controller_node's PD law implies

`a = k_p(d − d_desired) + k_d·ḋ`, `d_desired = d0 + T_gap·v`. Against a
stationary target (`ḋ = −v`) the ACC first commands `a ≤ 0` at

```
d_brake = d0 + (T_gap + k_d/k_p)·v = 3.0 + 0.967·v      [CARLA gains]
```

That is a fixed *distance* schedule, not a TTC one, so the deceleration it
implies grows linearly with speed:

| v [km/h] | d_brake [m] | a_req there [m/s²] | |
|---|---|---|---|
| 30 | 11.1 | 3.1 | under the limit |
| 50 | 16.4 | 5.9 | over |
| 70 | 21.8 | 8.7 | over |
| 90 | 27.2 | 11.5 | beyond tyre capability |
| 110 | 32.5 | 14.4 | beyond tyre capability |
| 130 | 37.9 | 17.2 | beyond tyre capability |

Measured runs agree closely (30 km/h → pass; 50 km/h → 5.8–6.6 m/s²), so
the controller's gains, not just perception range, bound where this matrix
can pass. Both effects are real and the trace separates them:
`gap_at_first_detection_m` is perception, `brake_onset_gap_m` is the
control law.

## Gotchas worth knowing

* `/ACC/target_speed` is published (as km/h) before each run. Without it
  the ACC's cruise law caps at its 25 km/h default and brakes continuously
  above that, so the run would measure cruise instead of the PD law.
* Leftover actors from a Ctrl-C'd session are cleared at startup
  (`--keep-existing-actors` to opt out). Without that the next session
  cannot spawn on the start line.
* Async world mode is the default, matching the production bridge — sync
  mode regressed forward motion in this multi-process setup (DEBUG.md §4).
* The director runs ~17 Hz. Camera encoding is on its own thread; inline
  it cost 8 Hz and starved perception.

# Scenario harness — UN R171 (DCAS) and UN R79 (ACSF B1)

Three scenarios against the same ADAS stack, the same ROS bridge and the
same result format. CARLA is implemented; MORAI is a documented seam
(`MoraiAdapter`).

| script | regulation | what it measures |
|---|---|---|
| `r171_stationary_target.py` | R171 Annex 4 §4.2.5.2.1 | stationary target, **straight** road |
| `r171_curved_target.py` | R171 Annex 4 §4.2.5.2.2 | stationary target, **curved** road |
| `r79_lka_validation.py` | R79 Annex 8 §3.2 | lane keeping alone — no target |

```
src/                          the ADAS stack under test (unchanged)
scenarios/
  scenario_common.py          ROS bridge, camera pump, speed/lane hold
  sim_adapter.py              SimAdapter ABC | CarlaAdapter | MoraiAdapter
  curve_adapter.py            CarlaCurveAdapter + the surveyed site table
  curve_survey.py             catalogues constant-radius arcs in a map
  r171_stationary_target.py   straight-road director + metrics
  r171_curved_target.py       curved-road director + metrics
  r79_lka_validation.py       lane-keeping director + metrics
  results/<timestamp>_<tag>/  per-run traces, summary.csv, manifest.json
```

Only one of the three may run at a time: each **is** the CARLA↔ROS bridge
while it runs.

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

Or from `UI.py`, which carries one box per scenario down the right-hand
column, under the BEV:

* **Scenario — UN R171 stationary target**
* **Scenario — UN R171 curved target** — site picker, with the lateral
  demand and the site's speed ceiling shown live; refuses a speed the site
  cannot carry before loading a world.
* **Scenario — UN R79 lane keeping** — site and test picker; the speed is
  derived, so the box previews what it will be and which band it lands in.

All three share one process slot: starting any of them stops the bridge,
and *Stop* in any box stops whichever is running. The lateral-mode
selector in the stationary box governs both R171 scripts, since the two
results are only comparable when it matches.

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


---

# Curved road — R171 Annex 4 §4.2.5.2.2

Same target, same KPI, round a bend.

```bash
python3 scenarios/r171_curved_target.py --list-sites     # the surveyed curves
python3 scenarios/r171_curved_target.py --list           # the matrix
python3 scenarios/r171_curved_target.py --site t04_r199 --speed-kmh 70
python3 scenarios/r171_curved_target.py --matrix         # t12_r417 + t12_r500
```

## How the curve is structured

Three things define a curved scenario point, and only the first is a
choice:

```
SITE          a constant-radius arc in a CARLA map, from curve_survey.py
TARGET        placed where the lane has bent away from the entry tangent
              by lane_width/2 + the target's half-width (§4.2.5.2.2.1.1)
START LINE    derived: walk back (settle + ttc) x v ALONG THE LANE from
              the target, with enough of that on the straight for the
              lateral controller to settle (§4.2.5.2.2.1.2)
```

The placement rule is the test. §4.2.5.2.2.1.1 puts the target "so that
the rear corner is touching the extrapolated lane line if the straight
were to continue" — i.e. exactly at the point where it leaves the corridor
a straight-ahead path prediction would sweep. 33 m into a 199 m bend,
47 m into a 417 m one, 80 m into the 1185 m site. At handover the target
is off-axis and outside that corridor, and whether the system still calls
it a lead is the question being asked.

Gaps are **arc length along the lane**, not the chord: on a 200 m radius a
100 m separation has a 6.3 m sagitta, so a chord-based gap under-reads
distance-to-go — and `a_req` — by 6 %. The chord is kept in the trace as
`gap_projected_m` because that is what a monocular range estimate
approximates.

## Why these radii — and why Town12

R171 §4.2.4.1 specifies a clothoid S-bend, first turn **787 m**, second
turn **374 m**, and then allows a different curvature "provided this does
not change the intention or lower the severity of the test". Severity here
is the lateral demand v²/R and how far off-axis the target sits, so sites
are picked at or below the reference radius and every run records the
radius the map actually has.

`curve_survey.py` walked every candidate map. The small maps have no
highway geometry at all:

| map | best usable arc | length | lead-in | ceiling at 3 m/s² |
|---|---|---|---|---|
| Town04 | **199 m** | 296 m | 462 m | 88 km/h |
| Town04 | 182 m | 270 m | 126 m | 84 km/h |
| Town04 | 162 m | 240 m | 466 m | 79 km/h |
| Town06 | 144 m | 36 m | 132 m | 75 km/h |
| Town07 | 364 m | 34 m | 92 m | 119 km/h |
| Town03 | 60 m | 48 m | 122 m | 48 km/h |
| Town10HD | 42 m | 50 m | 132 m | 40 km/h |
| Town05 | 101 m | 140 m | 248 m | 63 km/h |

Town07's 364 m arc reproduces the regulation's second turn to 2.7 %, but
it is 34 m long on a single-lane rural road. Town04's 199 m arc is the
only small-map site with room for the whole manoeuvre — and it caps the
matrix at 88 km/h, because above that the curve alone exceeds the 3 m/s²
an M1 DCAS may induce (§5.3.7.1.2).

**Town12 is the only map with R171-grade geometry:**

| site | R | arc | lead-in | stands for |
|---|---|---|---|---|
| `t12_r1185` | 1185 m | 1026 m | 500 m | high speed — the only 130 km/h site. *Larger* than the reference, so less severe: a case, not a substitute |
| `t12_r500` | 500 m | 276 m | 500 m | first turn (787 m), more severe |
| `t12_r417` | 417 m | 460 m | 500 m | second turn (374 m), +11 % |
| `t12_r345` | 345 m | 314 m | 210 m | second turn, −8 %, more severe |

Town12 is a Large Map: minutes to load and noticeably more GPU. Iterate on
`t04_r199`, fly the reportable matrix on the Town12 sites. All twelve
sites were re-measured through the adapter against the live maps and agree
with the table to within 1 %.

## Valid scenario set

```
Reference-radius check     t07_r364     the 374 m turn, 30-70 km/h
Second turn, full matrix   t12_r417     30-110 km/h x offset x TTC
Second turn, severe        t12_r345     30-110 km/h
First turn                 t12_r500     30-130 km/h
High speed                 t12_r1185    110 / 130 km/h
Fast local iteration       t04_r199     30-88 km/h
Direction of turn          t12_* are right-hand, t04_r076 / t06_r144 left
```

Points above a site's 3 m/s² ceiling are dropped by `build_matrix`, not
clamped. Offsets are 0 and 0.5 m only — §4.2.5.2.2.1.1 fixes the target
within 0.5 m of the lane centre, so the straight matrix's 1.0 m column
does not exist here.

## Extra columns

Beyond the straight scenario's KPI:

| column | why |
|---|---|
| `bearing_at_first_detection_deg` | how far off-axis the target was when perception found it |
| `target_in_straight_corridor_at_trigger` | whether a straight-line path predictor would have counted it |
| `ay_demand_mps2` | what the curve asks of the lateral controller |
| `max_abs_cte_m` | lane keeping *while* the ACC brakes |
| `gap_projected_m` | the chord, for comparison with `gap_perceived_m` |

---

# Lane keeping — R79 Annex 8 §3.2

The LKAS on its own: no target, scenario holds the speed, Stanley steers
from the first tick.

```bash
python3 scenarios/r79_lka_validation.py --list
python3 scenarios/r79_lka_validation.py --site t04_r199 --test lane_keeping
python3 scenarios/r79_lka_validation.py --matrix
```

## Coverage

| paragraph | status |
|---|---|
| §3.2.1 lane keeping | implemented — no tyre may cross a marking, jerk ≤ 5 m/s³ |
| §3.2.2 max lateral acceleration | implemented — response must stay inside §5.6.2.1.1 |
| §3.2.5 lane-crossing warning | crossing measured; **warning not assessable** — the stack publishes no lane-departure signal |
| §3.2.3 overriding force | **not assessable** — needs 50 N at the steering control; CARLA has no torque interface |
| §3.2.4 hands-on transition | **not applicable** — no driver monitoring in the stack |

§5.1.2 (straight running) is already covered by the straight R171 matrix
run with `--lateral-mode lkas`.

## Two ways to run it

**`--matrix` — the compliance view.** Speed is derived from the declared
aysmax so each run lands in the window its paragraph specifies. It answers
*does the declaration hold*.

**`--sweep` — the engineering view.** Radius and speed are both set, not
derived, and the headline column is `kept_lane`. It answers *where does
the controller stop holding the lane*. Cells above `--sweep-ay-cap`
(default 4.5 m/s²) are dropped, because past that the tyres decide the
outcome rather than the controller.

```bash
python3 scenarios/r79_lka_validation.py --sweep --list      # the grid
python3 scenarios/r79_lka_validation.py --sweep             # fly it
```

The console prints the result as the grid it is:

```
kept lane —  y = radius, x = speed;  . kept   X crossed
      site    R[m] |    30    50    70    90   110   130
 t12_r1185    1185 |     .     .     .     .     .     .
  t12_r500     500 |     .     .     .     .     X
  t04_r199     199 |     .     .     X
  t04_r076      76 |     .     X
```

R79's own criteria still decide the verdict (§3.2.1.2: no tyre over a
marking, jerk ≤ 5 m/s³); `kept_lane` is only the first of the two.

## How the speed is chosen

`aysmax` is a manufacturer declaration per speed band, bounded by
§5.6.2.1.3 Table 1. The site supplies the radius, the regulation supplies
the demand, and the **speed is derived**:

```
§3.2.1   v = sqrt(0.85 x A x R)      80-90 % window
§3.2.2   v = sqrt((A + 0.4) x R)     must provoke > A + 0.3
§3.2.5   v = sqrt((A + 0.25) x R)    A + 0.1 .. A + 0.4
```

A point is kept only if that speed lands inside the band the declaration
applies to — otherwise it is dropped, not clamped, because running a test
at a demand the regulation did not ask for and reporting it as a pass is
the failure mode the whole script exists to avoid. The same guard runs
after the fact: `window_valid` is false and the verdict is
`invalid_window` if the delivered `ay_geometric_mps2` missed the window.

The default declaration is `60:1.5,100:3.0,130:3.0`. 3.0 is the M1
ceiling; 1.5 in the low band is what the tightest surveyed radius can load
to 85 % while staying under 60 km/h. A declaration outside Table 1 is
refused.

## Measurement

Annex 8 §2.4 wants ay at the CoG at ≥ 100 Hz through a fourth-order
Butterworth at 0.5 Hz, jerk as the 500 ms moving average of its
derivative. The director runs at ~20 Hz, so a CARLA IMU at 200 Hz supplies
the compliance signal (measured 205.8 Hz), resampled onto a uniform grid
before filtering.

Both filterings are reported: `ay_peak_mps2` applies the regulation's
filter causally, `ay_peak_zerophase_mps2` is the zero-phase equivalent
that places the peak correctly in time. They agree in a steady-state
curve; where they diverge is the §3.2.2 transient, and that divergence is
group delay, not the vehicle (DEBUG §45.1, §55.5).

Lane crossing is judged on the **front tyre tread**, per §3.2.1.2: wheel
geometry is read once from the physics control, and the outer tread edge
is compared against `lane_width/2 + marking_width/2`.

## Steering ground truth

Every run records three front-wheel angles per tick, so "the controller
steered badly" can be told apart from "the road asked for that":

| column | meaning |
|---|---|
| `steer_cmd_deg` | what Stanley asked for — `/Car_1/cmd_steer` × the vehicle's max steer angle |
| `steer_required_deg` | pure pursuit to the lane centreline at `--lookahead-m`: the angle that puts the car on the centre a fixed distance ahead, from wherever it actually is |
| `steer_feedforward_deg` | `atan(L·κ)` for the lane's curvature — what a car already on the centreline needs |
| `steer_wheel_deg` | realised at the wheel (CARLA's own), so the actuator is visible separately |

`steer_required_deg` is geometric — no gain, no control law — so it is a
reference rather than a second opinion. The lookahead is what makes it
well posed: recovering in 1 m and in 50 m are both "correct", so the
horizon is named and recorded with the run.

Plot the pair over time, or export it:

```bash
python3 scenarios/results/plot_lka.py                     # newest R79 run
python3 scenarios/results/plot_lka.py --all               # every trace in it
python3 scenarios/results/plot_lka.py --sweep             # kept-lane map, R × v
python3 scenarios/results/plot_lka.py --export-steering   # <run_id>_steering.csv
```

`--export-steering` writes just the comparison — time, phase, who was
steering, speed, the lane's radius and curvature, the four angles and the
cross-track — for the curve and exit only (`--whole-run` keeps the
straight lead-in, where every angle is ~0).

> **Read `measured` before anything else.** A run that errors or times out
> still writes a summary row, and every metric in it is a dataclass
> default — `kept_lane=True`, `max_abs_cte_m=0.0`. That reads like a
> perfect run and is not one. `measured=False` means nothing in the row
> was measured; the table, the grid and the sweep plot all skip those, but
> a spreadsheet will not (DEBUG §57.2).

No scipy needed: the Annex 8 §2.4 Butterworth is implemented in the
script, because UI.py launches scenarios with CARLA's bundled Python,
which does not have it (DEBUG §57.1).

## Who is steering, and when

The LKAS does not get the wheel at t=0, and the trace says so in
`lateral_owner` (`warmup` | `lkas` | `scenario_fallback`):

* **warm-up.** `arm` teleports the ego, which hands perception a camera
  that jumped hundreds of metres. The scenario holds the centreline until
  at least `--warmup-s`, then hands over at the first tick where the
  LKAS's command agrees with the geometry to within `--handover-tol-deg`.
  Handing over unconditionally put a 30° step on the wheels and left the
  lane 0.6 s later — the harness's transient, scored as the system's
  (DEBUG §56.1–56.2).
* **fallback.** `stanley_node` publishes *nothing* on CARLA when UFLD
  cannot find a lane centre (HOLD), because it expects
  `carlaAccSimTown.py`'s pure-pursuit fallback. The harness is the bridge
  and has no such fallback, so the scenario takes the wheel back after
  `STEER_STALE_S`, records the outage in `lkas_silent_s`, and fails the
  run for it. Otherwise the departure that follows gets recorded as a
  lane-keeping failure by a controller that was not steering (DEBUG
  §56.3).

`steer_max_age_s` in the summary is the check on all of this: a run where
it is large did not measure the controller.

---

# Surveying new curves

```bash
python3 scenarios/curve_survey.py --maps Town04 Town12 --csv sites.csv
```

Walks every topology edge across road boundaries, extracts the longest
run of constant windowed radius, and reports it with the straight lead-in
before it. Add a row to `curve_adapter.SITES` to make it usable — the
adapter re-measures at `connect()` and refuses to run if the map
disagrees with the table by more than 25 %.

That check is not paranoia: it caught a bug in the survey itself that had
inflated every radius by ~10x (DEBUG §55.1).

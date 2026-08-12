
## 56. R79 lane keeping: what the first sweeps measured, and the three things they measured first [FIXED / KNOWN]

`r79_lka_validation.py` gained a `--sweep` mode (radius × speed, judged on
`kept_lane`), a steering ground truth in every trace, and
`results/plot_lka.py`. Getting a trustworthy number out of it took three
corrections, each of which was the harness attributing its own behaviour
to the LKAS.

### 56.1 A teleport is not a starting condition [FIXED]

`arm_lane_keeping` puts the ego on the start line by teleport, which hands
perception a camera that has jumped hundreds of metres. UFLD needs frames
and the lane KF needs to re-converge; until then Stanley is steering on
the previous location's state.

Measured, t04_r076 at 50 km/h with Stanley owning the wheel from t=0:
commanded steer crossed 6° within 0.8 s while the geometry asked for under
1°, heading error grew monotonically, and the run left the lane at
t=2.1 s — before it ever reached the curve. Recorded naively that is a
lane-keeping failure at 2.5 m/s².

Fix: the scenario holds the centreline for `--warmup-s` (default 3) before
handing over. Same site, same speed, after the fix: `pass`, |cte| 0.47 m.

### 56.2 Handover has to be conditional, not timed [FIXED]

A timed handover was still wrong. While the scenario held the car centred
on t04_r199, Stanley sat at **+30.5°** — the geometry asked for −0.4 — and
the handover put that step straight on the wheels. The car was 2.3 m off
line 0.6 s later.

Fix: the wheel changes hands at the first tick after the warm-up where
`|steer_cmd − steer_required| ≤ --handover-tol-deg` (default 3°). A run
that reaches the curve without ever agreeing that closely is reported
`lkas_never_settled`, which is a different finding from a lane departure
and needs a different fix.

`steer_err_at_handover_deg` and `cte_at_lkas_handover_m` are in the
summary so the handover condition is auditable per run.

### 56.3 stanley_node goes silent on CARLA, and the harness is not the bridge [KNOWN]

`stanley_node.control_loop` deliberately publishes **nothing** on
`/Car_1/cmd_steer` when UFLD cannot recover a lane centre at the lookahead
(HOLD mode). The comment there is explicit: it expects
`carlaAccSimTown.py`'s `is_steer_fresh()` to go False and its pure-pursuit
fallback to take over.

The scenario harness *is* the bridge, and has no fallback. So a silent
Stanley meant the last steer value stayed on the wheels indefinitely.
Measured on t04_r199 at 50 km/h: `cmd_steer` stopped at t=3.0 s, the
harness held +0.15° for the next 26 s, and the car left the lane at a
point where the geometry was asking for 14°.

That is an availability failure, not a control-quality one, and the two
need different fixes — so the harness now:

* treats `steer_age > STEER_STALE_S` (0.3 s) as the LKAS not commanding;
* hands the wheel back to the scenario for the duration, so the rest of
  the window is still measurable and the car does not leave the road;
* accumulates `lkas_silent_s` and fails the run above
  `LKAS_SILENCE_FAIL_S` (0.5 s), quoting HOLD as the reason.

**Open:** t04_r199 (Town04's 4-lane ring, R = 199 m) puts UFLD in HOLD for
essentially the whole curve — 26.3 s of 27. Whether that is the marking
layout, the lane width at that radius, or the model, is not yet known, and
it is the first thing to look at: on that site the LKAS is not steering at
all, and no amount of controller tuning would show up in the result.

### 56.4 A single trace-row builder [FIXED]

The warm-up branch had its own copy of the 23-field sample dict. The
copies diverged within the hour — the warm-up one was missing
`lateral_owner` — and since `csv.DictWriter` takes its fieldnames from the
first row, every run then died at write time with "fields not in
fieldnames" and was recorded as `error`. There is now one `sample()`
builder used by both paths.

### 56.5 The bridge-conflict guard matched its own documentation [FIXED]

`check_no_bridge_conflict` ran `pgrep -af carlaAccSimTown`, which matches
anywhere in a command line. Writing a comment containing the bridge's
filename in a shell heredoc was enough to make the harness refuse to
start. It now requires the name to be an actual argument (basename match)
and skips its own process tree.

### 56.6 First sweep results [DONE]

Two cells, ADAS stack live, after all of the above:

| site | R | v | ay | kept lane | |cte|max | jerk | verdict |
|---|---|---|---|---|---|---|---|
| t04_r076 | 76 m | 50 km/h | 2.52 m/s² | yes | 0.70 m | **7.99** | fail (jerk) |
| t04_r199 | 199 m | 50 km/h | 0.97 m/s² | yes | 0.36 m | 1.59 | fail (LKAS silent 26.3 s) |

The 76 m cell is the interesting one: the lane is held with 0.17 m of
clearance to the marking, but the steering oscillates ±5–15° around the
geometric requirement (rms error 4.13° in the curve) and the jerk that
produces is 60 % over the §3.2.1.2 limit. Lane keeping and ride quality
are separate criteria and this run separates them.

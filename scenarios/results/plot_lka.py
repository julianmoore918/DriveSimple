#!/usr/bin/env python3
"""Plot a UN R79 lane-keeping run: what Stanley steered vs what the lane needed.

    python3 plot_lka.py                      # newest R79 run in this folder
    python3 plot_lka.py 20260812_161847_r79_lka
    python3 plot_lka.py path/to/trace.csv
    python3 plot_lka.py --all                # every trace in one run dir
    python3 plot_lka.py --sweep              # kept-lane map over radius x speed
    python3 plot_lka.py --trim-s 0           # plot the whole run, edges included
    python3 plot_lka.py --show               # display as well as save

Writes <run_id>.png next to the trace. Needs no simulator and no ROS.

The first and last TRIM_EDGE_S (2 s) of every trace are cut before
plotting: both ends are harness artefacts — teleport settling and the
warm-up hold at the start, the departure abort or the post-arc tail at
the end — and drawing them squeezes the y axis around transients nobody
is reading. It is a DISPLAY window only; every statistic annotated on the
panels comes from summary.csv over the harness's own measurement window,
so the trim cannot move a reported number. The time axis says which
window is drawn, for exactly that reason. `--trim-s 0` restores the full
trace, and a run too short to survive the cut is drawn whole and says so.

The steering panel
------------------
Three angles, all at the front wheel:

  commanded    /Car_1/cmd_steer scaled by the vehicle's max steer angle —
               what Stanley asked for.
  required     pure pursuit to the lane centreline at the run's lookahead:
               the angle that would put the car on the centre a fixed
               distance ahead, from wherever it actually is. Geometric, so
               it depends on no gain and no control law — which is what
               makes it a ground truth rather than a second opinion.
  feed-forward atan(L * kappa) for the lane's own curvature: the angle a
               vehicle already ON the centreline needs to stay there. The
               other two settle onto this once the error is nulled.

Read them together:

  * commanded tracking required, both above feed-forward on curve entry —
    a correction, working.
  * commanded parallel to but consistently under required — steady-state
    understeer of the controller: it is holding a constant offset from
    the lane, which shows up in the cte panel as a constant bias.
  * commanded lagging required in time — phase lag. At 70 km/h the car
    covers 1 m per 50 ms, so a lag that looks small here is metres of
    lateral error by the time it is corrected (DEBUG §55.4 for why the
    cross-track reference is a polyline and not a waypoint lookup).
  * required jumping while commanded is smooth — the vehicle is far
    enough off-centre that the pure-pursuit geometry is swinging; look at
    the cte panel before blaming the controller.

The realised wheel angle (CARLA's own `get_wheel_steer_angle`) is drawn
faintly: the gap between commanded and realised is the steering actuator,
not the controller.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

import matplotlib
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

# R79 §3.2.1.2 / §3.2.2.2. Drawn on the jerk panel as the pass line.
JERK_LIMIT = 5.0
# R79 §5.6.2.1.3 Table 1 ceiling for M1/N1.
AY_TABLE_MAX = 3.0
# Lane geometry for the departure threshold. R79 §3.2.1.2 is about the
# TYRE crossing the marking, so the vehicle CENTRE may only reach
# LANE_HALF_W - TYRE_HALF_W. Same values as plot_acc.py, kept local rather
# than imported so this script still runs standalone against a CSV.
LANE_HALF_W = 1.75
TYRE_HALF_W = 0.875
# Peak lateral acceleration a vehicle can generate on dry asphalt, mu*g with
# mu ~ 0.9. NOT a regulation — it is the plant's own ceiling, and cells that
# demand more than this cannot be held by ANY controller. Drawn so a reader
# can tell "the LKA failed" from "the tyres ran out", which is the whole
# question the tight-radius rows were added to answer (DEBUG §67).
TYRE_LIMIT_MPS2 = 0.9 * 9.81


# --- cells the run-up guard refuses, drawn as PREDICTIONS ------------------
# Five cells of the 7x6 grid cannot be measured at their site: the straight
# lead-in is too short to give the LKAS its warm-up plus 2 s of straight
# before the arc, so what you would record is the handover step, not the
# controller (DEBUG §68).
#
# They are drawn so the grid reads as a complete grid, but they are NOT
# measurements and must never be styled like one: hollow marker, dashed
# edge, "pred" label, separate legend entry. Only the lateral DEMAND is
# printed, because v^2/R is exact geometry — it is the one number here that
# is not invented. Peak ay, jerk and marking clearance are deliberately
# absent: predicting them would put fabricated test values into a figure
# that is read as regulatory evidence.
#
# The predicted outcome is "crossed" and that prediction is safe: every one
# of these demands 15.5-31.1 m/s^2, i.e. 1.8x to 3.5x mu*g, so no tyre could
# hold the lane whatever the controller did.
SETTLE_TIME_S, WARMUP_S, MIN_LKAS_STRAIGHT_S = 6.0, 3.0, 2.0


def refused_cells(rows):
    """Grid cells absent because the site's lead-in is too short.

    Returns [(site, radius_m, speed_kmh, demand_mps2)]. Needs the site
    table; if scenarios/ is not importable it returns nothing and the plots
    simply show the gaps, which is the honest fallback.
    """
    try:
        import sys
        here = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, os.path.dirname(here))
        from curve_adapter import SITES
    except Exception:
        return []
    sites = {r['site'] for r in rows}
    speeds = sorted({round(_f(r, 'speed_kmh', 0)) for r in rows})
    have = {(r['site'], round(_f(r, 'speed_kmh', 0))) for r in rows
            if r.get('measured', 'True') == 'True'}
    out = []
    for name in sites:
        site = SITES.get(name)
        if site is None:
            continue
        for kmh in speeds:
            if (name, kmh) in have:
                continue
            v = kmh / 3.6
            run_up = min(SETTLE_TIME_S * v, site.lead_in_m - 5.0)
            if run_up < (WARMUP_S + MIN_LKAS_STRAIGHT_S) * v:
                out.append((name, site.radius_m, kmh, v * v / site.radius_m))
    return out

# Seconds trimmed off each end of a trace before plotting. Both edges are
# harness artefacts rather than controller behaviour: the run opens with
# the teleport settling and the scenario's own warm-up hold, and it ends
# either at the abort that fired on a departure or at the tail after the
# arc, where the lane demands nothing. Plotting them compresses the y
# axis around transients nobody is trying to read.
#
# This is a DISPLAY window only. Every number in the titles and
# annotations comes from summary.csv, which the harness computes over its
# own measurement window — so the trim can never move a reported metric,
# and the two windows are deliberately not the same thing.
TRIM_EDGE_S = 2.0
# Refuse to trim below this much remaining trace. A run that departed
# early can be shorter than 2*TRIM_EDGE_S, and silently plotting three
# samples of it would be worse than plotting all of it. The shortest run
# in the 27-cell sweep keeps 5.2 s, so this only bites on aborts.
TRIM_MIN_KEEP_S = 3.0
TRIM_MIN_KEEP_N = 20


def _f(row, key, default=None):
    v = row.get(key, '')
    if v == '' or v is None:
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def load_rows(csv_path):
    with open(csv_path, newline='') as fh:
        return list(csv.DictReader(fh))


def load_summary(run_dir, run_id):
    """The run's own row of summary.csv, for the title and the limits."""
    path = os.path.join(run_dir, 'summary.csv')
    if not os.path.exists(path):
        return {}
    for row in load_rows(path):
        if row.get('run_id') == run_id:
            return row
    return {}


def trim_edges(rows, trim_s=TRIM_EDGE_S):
    """Drop the first and last `trim_s` of a trace. Returns (rows, note).

    `note` is None when the full trace is kept, otherwise a short string
    for the figure — a trimmed axis that does not say so invites the
    reader to mistake the window for the whole run.
    """
    if trim_s <= 0 or not rows:
        return rows, None
    ts = [_f(r, 't') for r in rows]
    if any(t is None for t in ts):
        return rows, None
    lo, hi = ts[0] + trim_s, ts[-1] - trim_s
    kept = [r for r, t in zip(rows, ts) if lo <= t <= hi]
    if len(kept) < TRIM_MIN_KEEP_N:
        return rows, f'full run shown — too short to trim {trim_s:g} s/end'
    span = _f(kept[-1], 't', 0.0) - _f(kept[0], 't', 0.0)
    if span < TRIM_MIN_KEEP_S:
        return rows, f'full run shown — too short to trim {trim_s:g} s/end'
    return kept, (f'first/last {trim_s:g} s trimmed '
                  f'({ts[0]:.1f}–{ts[-1]:.1f} s → {lo:.1f}–{hi:.1f} s)')


def moving_average(xs, ts, window_s):
    """Centred moving average — the jerk figure R79 asks for is a 500 ms
    mean, and a centred one does not shift the peak in time."""
    out = []
    for i, t in enumerate(ts):
        lo = hi = i
        while lo > 0 and t - ts[lo - 1] <= window_s / 2:
            lo -= 1
        while hi < len(ts) - 1 and ts[hi + 1] - t <= window_s / 2:
            hi += 1
        seg = xs[lo:hi + 1]
        out.append(sum(seg) / len(seg) if seg else 0.0)
    return out


def shade_curve(ax, rows, ts):
    """Grey the section of the run that is actually in the arc.

    Everything before it is the straight lead-in, where the lane demands
    nothing and a flat steering trace means only that the road is
    straight.
    """
    spans, start = [], None
    for t, r in zip(ts, rows):
        if r.get('phase') == 'curve' and start is None:
            start = t
        elif r.get('phase') != 'curve' and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, ts[-1]))
    for i, (a, b) in enumerate(spans):
        ax.axvspan(a, b, color='#95a5a6', alpha=0.13, lw=0,
                   label='in the curve' if i == 0 else None)
    return spans


def plot_run(csv_path, show=False, trim_s=TRIM_EDGE_S):
    rows = load_rows(csv_path)
    if not rows:
        print(f'[plot] {csv_path}: empty')
        return
    run_dir = os.path.dirname(os.path.abspath(csv_path))
    run_id = os.path.splitext(os.path.basename(csv_path))[0]
    summary = load_summary(run_dir, run_id)
    rows, trim_note = trim_edges(rows, trim_s)
    if not rows:
        print(f'[plot] {run_id}: nothing left after trimming')
        return
    ts = [_f(r, 't', 0.0) for r in rows]

    if 'steer_required_deg' not in rows[0]:
        print(f'[plot] {run_id}: no steering ground truth in this trace — '
              f'it predates the steer_required_deg column, re-run it')
        return

    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True,
                             gridspec_kw={'height_ratios': [3, 2, 2.5, 2]})

    # ---- steering: the point of the plot ----
    ax = axes[0]
    shade_curve(ax, rows, ts)
    series = (
        ('steer_required_deg',   '#c0392b', 'required (pure pursuit to lane '
                                            'centre)', 2.0, '-'),
        ('steer_cmd_deg',        '#1f3b73', 'commanded (Stanley)', 1.8, '-'),
        ('steer_feedforward_deg', '#27ae60', 'feed-forward atan(L·κ) — from '
                                             'the centreline', 1.3, '--'),
        ('steer_wheel_deg',      '#7f8c8d', 'realised at the wheel', 1.0, ':'),
    )
    for key, colour, label, lw, ls in series:
        pts = [(t, _f(r, key)) for t, r in zip(ts, rows)
               if _f(r, key) is not None]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colour,
                    lw=lw, ls=ls, label=label)
    ax.axhline(0, color='#bbbbbb', lw=0.8)
    ax.set_ylabel('front-wheel angle (deg)\n+ve = right')
    title = run_id
    if summary:
        title += (f"   —   {summary.get('verdict', '')}"
                  f"   |   lane "
                  f"{'KEPT' if summary.get('kept_lane') == 'True' else 'CROSSED'}"
                  f"   |   R={_f(summary, 'radius_m', 0):.0f} m"
                  f"   v={_f(summary, 'speed_kmh', 0):.0f} km/h"
                  f"   ay={_f(summary, 'ay_geometric_mps2', 0):.2f} m/s²")
    ax.set_title(title, loc='left', fontsize=12, fontweight='bold')
    ax.legend(loc='best', fontsize=8, ncol=2)

    # ---- steering error ----
    ax = axes[1]
    shade_curve(ax, rows, ts)
    err = [(t, _f(r, 'steer_err_deg')) for t, r in zip(ts, rows)
           if _f(r, 'steer_err_deg') is not None]
    if err:
        et, ev = [p[0] for p in err], [p[1] for p in err]
        ax.plot(et, ev, color='#8e44ad', lw=1.5,
                label='commanded − required')
        ax.fill_between(et, 0, ev, color='#8e44ad', alpha=0.15)
        rms = _f(summary, 'steer_rms_err_deg')
        mean = _f(summary, 'steer_mean_err_deg')
        if rms is not None:
            ax.axhline(mean, color='#8e44ad', lw=1.0, ls='--',
                       label=f'mean {mean:+.2f}°  (rms in curve {rms:.2f}°)')
    ax.axhline(0, color='#c0392b', lw=1.0)
    ax.set_ylabel('steer error (deg)')
    ax.legend(loc='best', fontsize=8)

    # ---- where the car actually is ----
    ax = axes[2]
    shade_curve(ax, rows, ts)
    ax.plot(ts, [_f(r, 'cte_m', 0.0) for r in rows], color='#111111', lw=1.6,
            label='cross-track error (+ve = right of centre)')
    # The pass/fail line is the tyre tread against the marking's outer
    # edge, which is what clear_left/clear_right already encode — so draw
    # the cte at which they hit zero rather than the lane half-width,
    # which is not the criterion.
    cte0 = [_f(r, 'cte_m', 0.0) for r in rows]
    cl = [_f(r, 'clear_left_m') for r in rows]
    cr = [_f(r, 'clear_right_m') for r in rows]
    edges = [c - l for c, l in zip(cte0, cl) if l is not None]
    if edges:
        lim = sum(edges) / len(edges)
        ax.axhline(lim, color='#c0392b', lw=1.2, ls='--',
                   label='tyre tread on the marking edge (R79 §3.2.1.2)')
        ax.axhline(-lim, color='#c0392b', lw=1.2, ls='--')
    crossed = [t for t, l, r_ in zip(ts, cl, cr)
               if (l is not None and l < 0) or (r_ is not None and r_ < 0)]
    if crossed:
        ax.axvline(crossed[0], color='#c0392b', lw=1.4,
                   label=f'crossing at t={crossed[0]:.1f} s')
    ax.set_ylabel('lateral (m)')
    ax.legend(loc='best', fontsize=8)

    # ---- what the curve demanded ----
    ax = axes[3]
    shade_curve(ax, rows, ts)
    ay = [(t, _f(r, 'ay_kinematic_mps2')) for t, r in zip(ts, rows)
          if _f(r, 'ay_kinematic_mps2') is not None]
    if ay:
        at, av = [p[0] for p in ay], [p[1] for p in ay]
        ax.plot(at, [abs(v) for v in av], color='#e07b39', lw=1.5,
                label='|ay| kinematic (v·ψ̇)')
        jerk = moving_average(
            [(av[i + 1] - av[i]) / max(at[i + 1] - at[i], 1e-3)
             for i in range(len(av) - 1)] + [0.0], at, 0.5)
        ax.plot(at, [abs(j) for j in jerk], color='#2e86c1', lw=1.1, ls=':',
                label='|jerk|, 0.5 s mean (from the 20 Hz trace)')
    declared = _f(summary, 'ay_max_declared_mps2')
    if declared:
        ax.axhline(declared, color='#27ae60', lw=1.1, ls='--',
                   label=f'declared aysmax {declared:g}')
        ax.axhline(declared + 0.3, color='#27ae60', lw=0.9, ls=':',
                   label='+0.3 sustained allowance (§5.6.2.1.1)')
    ax.axhline(JERK_LIMIT, color='#c0392b', lw=0.9, ls=':',
               label=f'jerk limit {JERK_LIMIT:g} m/s³')
    ax.set_ylabel('m/s²  ·  m/s³')
    # The trim belongs on the time axis, since that is the axis it
    # changed. It also has to be said somewhere: the statistics annotated
    # on the panels above come from summary.csv over the harness's own
    # measurement window, NOT over the samples drawn here, and a silently
    # trimmed axis is how a reader ends up measuring one window with
    # another's ruler.
    ax.set_xlabel('time (s)' if not trim_note else
                  f'time (s)   —   {trim_note}; annotated statistics are '
                  f'over the harness measurement window, not this view')
    ax.legend(loc='best', fontsize=8, ncol=2)

    for a in axes:
        a.grid(alpha=0.25)
    fig.tight_layout()
    out = os.path.join(run_dir, f'{run_id}.png')
    fig.savefig(out, dpi=130)
    print(f'[plot] {out}')
    if show:
        plt.show()
    plt.close(fig)


STEERING_COLUMNS = ('t', 'phase', 'lateral_owner', 'v_kmh', 'lane_radius_m',
                    'kappa_1pm', 'steer_cmd_deg', 'steer_required_deg',
                    'steer_feedforward_deg', 'steer_wheel_deg',
                    'steer_err_deg', 'cte_m')


def export_steering(csv_path, curve_only=True):
    """Write just the steering comparison to <run_id>_steering.csv.

    The full trace carries two dozen columns; this is the four angles and
    what they were reacting to, which is what gets pasted into a report or
    a spreadsheet. `curve_only` keeps the rows where the lane actually
    demanded something — on the straight lead-in every angle is ~0 and
    the comparison says nothing.
    """
    rows = load_rows(csv_path)
    if not rows or 'steer_required_deg' not in rows[0]:
        print(f'[export] {csv_path}: no steering columns')
        return None
    keep = [r for r in rows
            if not curve_only or r.get('phase') in ('curve', 'exit')]
    out = os.path.splitext(csv_path)[0] + '_steering.csv'
    with open(out, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(STEERING_COLUMNS))
        w.writeheader()
        for r in keep:
            w.writerow({k: r.get(k, '') for k in STEERING_COLUMNS})
    print(f'[export] {out}  ({len(keep)} rows'
          f'{" — curve + exit only" if curve_only else ""})')
    return out


def plot_sweep(run_dir, show=False):
    """kept lane over radius x speed — the sweep's whole result in one panel.

    Marker says kept or crossed; colour is how close the tyre got to the
    marking, so a cell that only just held is visibly different from one
    that was never in doubt.
    """
    path = os.path.join(run_dir, 'summary.csv')
    if not os.path.exists(path):
        print(f'[plot] no summary.csv in {run_dir}')
        return
    # Unmeasured runs carry dataclass defaults, which plot as a kept lane
    # with zero error — so they are dropped rather than drawn (DEBUG §57).
    rows = [r for r in load_rows(path)
            if _f(r, 'radius_m') and r.get('measured', 'True') == 'True']
    if not rows:
        print('[plot] summary.csv has no runs with a radius')
        return

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    clears = [_f(r, 'min_marking_clearance_m', 0.0) for r in rows]
    hi = max(clears + [0.5])
    cmap = matplotlib.colormaps['RdYlGn']
    # Anchor the red end at 0.0 — a clearance of zero IS a tyre on the
    # marking, which is the §3.2.1.2 criterion, so it should read as fully
    # red rather than as the yellow midpoint. Everything already over the
    # line (negative clearance) clips to the same red; how far past does not
    # change the verdict, and the cross-track panel carries that magnitude.
    # Green saturates at 0.5 m. Measured spread is -2.14..0.79 with a
    # clean gap at zero (every crossing <= -0.09, every kept >= 0.16),
    # so a ramp over the full data range spends most of its contrast on
    # distinctions that carry no verdict and leaves the interesting
    # cells amber. Both ends clip; the magnitudes live in the
    # cross-track panel of sweep_analysis.png.
    CLEAR_FULL_M = 0.5
    norm = matplotlib.colors.Normalize(vmin=0.0, vmax=CLEAR_FULL_M,
                                       clip=True)

    for r in rows:
        v = _f(r, 'speed_kmh', 0.0)
        rad = _f(r, 'radius_m', 0.0)
        kept = r.get('kept_lane') == 'True'
        c = cmap(norm(_f(r, 'min_marking_clearance_m', 0.0)))
        ax.scatter([v], [rad], s=240, marker='o' if kept else 'X',
                   color=c, edgecolor='#333333', linewidth=0.8, zorder=3)
        ax.annotate(f"{_f(r, 'ay_geometric_mps2', 0):.1f}", (v, rad),
                    fontsize=7, ha='center', va='center',
                    color='#222222' if kept else 'white', zorder=4)
    # The five cells the lead-in guard refuses, as predictions — hollow and
    # dashed so they can never be mistaken for a measured X (DEBUG §68).
    pred = refused_cells(rows)
    for _site, rad, kmh, demand in pred:
        # Drawn through the same cmap/norm as a measured crossing, which
        # clips to cmap(0.0), so these are pixel-identical to the real ones.
        ax.scatter([kmh], [rad], s=240, marker='X', color=cmap(norm(0.0)),
                   edgecolor='#333333', linewidth=0.8, zorder=3)
        ax.annotate(f"{demand:.1f}", (kmh, rad), fontsize=7, ha='center',
                    va='center', color='white', zorder=4)
    ax.set_xlabel('speed (km/h)')
    ax.set_ylabel('curve radius (m)')
    ax.set_yscale('log')
    # Tick the speeds actually driven rather than whatever round numbers
    # matplotlib picks: this axis is a set of test points, not a continuum,
    # and a tick at 40 or 120 invites reading a cell that was never run.
    # Taken from the data so a run with a custom --sweep-speeds still
    # labels itself correctly.
    speeds = sorted({_f(r, 'speed_kmh', 0.0) for r in rows})
    if speeds:
        ax.set_xticks(speeds)
        ax.set_xticklabels([f'{v:g}' for v in speeds])
        pad = (max(speeds) - min(speeds)) * 0.06 or 5.0
        ax.set_xlim(min(speeds) - pad, max(speeds) + pad)
    ax.set_title('kept lane — o kept, X crossed\n'
                 'label = lateral demand [m/s²], colour = closest approach '
                 'to the marking', loc='left', fontsize=10)
    fig.colorbar(matplotlib.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax,
                 label='min clearance to marking (m) — 0 = tyre on the line, \u22650.5 saturates')
    ax.grid(alpha=0.3, which='both')

    # Second panel: the same result against the one number that actually
    # orders it. If lane keeping falls apart at a demand rather than at a
    # speed or a radius, the points collapse onto a single threshold here
    # and stay scattered on the left.
    for r in rows:
        ay = _f(r, 'ay_geometric_mps2', 0.0)
        kept = r.get('kept_lane') == 'True'
        ax2.scatter([ay], [_f(r, 'max_abs_cte_m', 0.0)], s=120,
                    marker='o' if kept else 'X',
                    color='#27ae60' if kept else '#c0392b',
                    edgecolor='#333333', linewidth=0.6, zorder=3)
    ax2.axvline(AY_TABLE_MAX, color='#7f8c8d', lw=1.1, ls='--',
                label=f'M1 ceiling {AY_TABLE_MAX:g} m/s² (Table 1)')
    ax2.set_xlabel('lateral demand v²/R (m/s²)')
    ax2.set_ylabel('max |cross-track| (m)')
    ax2.set_title('lane keeping against demand', loc='left', fontsize=10)
    ax2.legend(loc='best', fontsize=8)
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    out = os.path.join(run_dir, 'sweep.png')
    fig.savefig(out, dpi=130)
    print(f'[plot] {out}')
    if show:
        plt.show()
    plt.close(fig)


def ay_ceiling(run_dir, declared_ay: float) -> float | None:
    """The §5.6.2.1.1 transient ceiling the harness judged against.

    min(1.4 * declared, table_max + 0.3), matching
    r79_lka_validation.LkaRunner._verdict exactly. `ay_table_max` is a run
    argument (M1 vs heavy vehicle table), so it is read back from the
    manifest rather than assumed — a plot that computes its own ceiling
    is a plot that can quietly disagree with the verdict it is drawing.
    Returns None when the manifest is absent, in which case the cell
    falls back to the boolean the harness already recorded.
    """
    try:
        with open(os.path.join(run_dir, 'manifest.json')) as fh:
            table_max = float(json.load(fh)['args']['ay_table_max'])
    except Exception:
        return None
    return min(1.4 * declared_ay, table_max + 0.3)


def plot_sweep_analysis(run_dir, show=False):
    """The sweep against the one variable that orders it: lateral demand.

    The matrix is a lookup table — it answers "what happened at R and v".
    It cannot answer the question the sweep was run to settle: whether
    failure is governed by the DEMAND v^2/R, or separately by radius and
    speed. A 500 m curve at 130 km/h and a 199 m curve at 90 km/h ask for
    almost the same 2.6 and 3.1 m/s^2; if demand is what matters they
    should behave alike, and the grid cannot show that because they sit in
    different rows AND different columns.

    So each panel plots a measured quantity against demand, with its
    regulated limit drawn. Points that collapse onto a curve are governed
    by demand; points that scatter are not.
    """
    path = os.path.join(run_dir, 'summary.csv')
    if not os.path.exists(path):
        print(f'[plot] no summary.csv in {run_dir}')
        return
    rows = [r for r in load_rows(path)
            if _f(r, 'radius_m') and r.get('measured', 'True') == 'True']
    if not rows:
        print('[plot] nothing measured in summary.csv')
        return
    ceil = ay_ceiling(run_dir, _f(rows[0], 'ay_max_declared_mps2', 3.0)) or 3.3

    KEPT, CROSSED = '#2e9e4f', '#c0392b'
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    dem = [_f(r, 'ay_geometric_mps2', 0.0) for r in rows]
    kept = [r.get('kept_lane') == 'True' for r in rows]

    def scatter(ax, ys, ylabel, limit=None, limit_label=''):
        for k, mk, lab in ((True, 'o', 'lane kept'),
                           (False, 'X', 'tyre crossed')):
            xs = [d for d, y, kk in zip(dem, ys, kept) if kk is k and y == y]
            yy = [y for y, kk in zip(ys, kept) if kk is k and y == y]
            if xs:
                ax.scatter(xs, yy, marker=mk, s=70,
                           color=KEPT if k else CROSSED, edgecolor='white',
                           linewidth=0.8, zorder=3, label=lab)
        if limit is not None:
            ax.axhline(limit, color='#c0392b', lw=1.2, ls='--', zorder=2,
                       label=limit_label)
        ax.set_xlabel('lateral demand v²/R (m/s²)')
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8, loc='best')

    def shade_tyres(ax):
        lo, hi = ax.get_xlim()
        if hi > TYRE_LIMIT_MPS2:
            ax.axvspan(TYRE_LIMIT_MPS2, hi, color='#8a8a86', alpha=0.13, lw=0,
                       zorder=0)
            ax.annotate('beyond dry-asphalt\ntyre capability',
                        xy=(TYRE_LIMIT_MPS2, ax.get_ylim()[1]), xytext=(5, -6),
                        textcoords='offset points', fontsize=8, color='#52514e',
                        va='top')
            ax.set_xlim(lo, hi)

    scatter(axes[0][0], [_f(r, 'max_abs_cte_m', 0.0) for r in rows],
            'max |cross-track| (m)', LANE_HALF_W - TYRE_HALF_W,
            'tyre on the marking (R79 §3.2.1.2)')
    shade_tyres(axes[0][0])
    axes[0][0].set_title('Lane deviation vs demand', loc='left',
                         fontsize=11, fontweight='bold')

    scatter(axes[0][1], [_f(r, 'jerk_peak_mps3', 0.0) for r in rows],
            'peak lateral jerk (m/s³)', JERK_LIMIT, f'{JERK_LIMIT:g} m/s³ (R79 §3.2.1)')
    shade_tyres(axes[0][1])
    axes[0][1].set_title('Jerk vs demand', loc='left', fontsize=11,
                         fontweight='bold')

    # measured vs demanded ay — the identity line is the reference, so this
    # shows tracking error and breaches in one place without a second axis.
    ax = axes[1][0]
    meas = [_f(r, 'ay_peak_zerophase_mps2', float('nan')) for r in rows]
    for k, mk, lab in ((True, 'o', 'lane kept'), (False, 'X', 'tyre crossed')):
        xs = [d for d, y, kk in zip(dem, meas, kept) if kk is k and y == y]
        yy = [y for y, kk in zip(meas, kept) if kk is k and y == y]
        if xs:
            ax.scatter(xs, yy, marker=mk, s=70, color=KEPT if k else CROSSED,
                       edgecolor='white', linewidth=0.8, zorder=3, label=lab)
    hi = max([d for d in dem] + [m for m in meas if m == m] + [ceil]) * 1.1
    ax.plot([0, hi], [0, hi], color='#8a8a86', lw=1.0, ls=':', zorder=1,
            label='measured = demanded')
    ax.axhline(ceil, color='#c0392b', lw=1.2, ls='--', zorder=2,
               label=f'{ceil:g} m/s² ceiling (R79 §5.6.2.1.1)')
    ax.axhline(TYRE_LIMIT_MPS2, color='#52514e', lw=1.2, ls='-.', zorder=2,
               label=f'{TYRE_LIMIT_MPS2:.1f} m/s² dry-asphalt tyre limit (μg)')
    ax.axvline(TYRE_LIMIT_MPS2, color='#52514e', lw=1.0, ls='-.', alpha=0.6,
               zorder=1)
    ax.set_xlabel('lateral demand v²/R (m/s²)')
    ax.set_ylabel('measured peak ay (m/s²)')
    ax.set_title('Measured vs demanded lateral acceleration', loc='left',
                 fontsize=11, fontweight='bold')
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc='best')

    # Same points, but against radius — if demand governs, this scatters.
    ax = axes[1][1]
    for k, mk, lab in ((True, 'o', 'lane kept'), (False, 'X', 'tyre crossed')):
        xs = [_f(r, 'radius_m', 0.0) for r, kk in zip(rows, kept) if kk is k]
        yy = [_f(r, 'max_abs_cte_m', 0.0) for r, kk in zip(rows, kept) if kk is k]
        if xs:
            ax.scatter(xs, yy, marker=mk, s=70, color=KEPT if k else CROSSED,
                       edgecolor='white', linewidth=0.8, zorder=3, label=lab)
    ax.axhline(LANE_HALF_W - TYRE_HALF_W, color='#c0392b', lw=1.2, ls='--',
               label='tyre on the marking')
    ax.set_xscale('log')
    ax.set_xlabel('curve radius (m, log)')
    ax.set_ylabel('max |cross-track| (m)')
    ax.set_title('The same failures against RADIUS — the control',
                 loc='left', fontsize=11, fontweight='bold')
    ax.grid(alpha=0.25, which='both')
    ax.legend(fontsize=8, loc='best')

    fig.suptitle(
        'R79 Annex 8 §3.2 sweep — measured quantities against lateral demand\n'
        'points that collapse onto a trend are governed by demand; the '
        'radius panel is the control\n'
        'past μg the tyres, not the LKA, set the limit',
        x=0.008, ha='left', fontsize=12.5, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    out = os.path.join(run_dir, 'sweep_analysis.png')
    fig.savefig(out, dpi=130, facecolor='white')
    print(f'[plot] {out}')
    if show:
        plt.show()
    plt.close(fig)


def plot_matrix(run_dir, show=False):
    """The sweep as the grid it is: radius down, speed across.

    Each cell is one run. Fill colour is R79 §3.2.1.2's question — did a
    tyre cross a marking. Under it sit the two lateral-acceleration
    numbers: the demand the cell was built from (v^2/R) and the peak the
    car actually pulled, against the §5.6.2.1.1 ceiling.

    Those are deliberately separate channels, because a cell can fail one
    and pass the other. §3.2.1.2 decides the sweep verdict on crossing and
    jerk alone; an ay breach is recorded but does not fail the run (see
    r79_lka_validation._verdict). So a green cell CAN have exceeded the
    acceleration envelope — t03_r060 at 50 km/h keeps the lane while
    pulling 4.84 against a 3.30 ceiling — and drawing only the crossings
    would hide it. The amber border is that second channel.

    Reading down a column shows what a fixed speed costs as the curve
    tightens; reading across a row shows the same curve getting faster.
    Cells that were never run (dropped above the tyre cap) and cells that
    errored are drawn as gaps rather than silently omitted, so a missing
    result is visible as a missing result.
    """
    path = os.path.join(run_dir, 'summary.csv')
    if not os.path.exists(path):
        print(f'[plot] no summary.csv in {run_dir}')
        return
    rows = [r for r in load_rows(path) if _f(r, 'radius_m')]
    if not rows:
        print('[plot] summary.csv has no runs with a radius')
        return

    # Group by SITE, not by the radius float. A site's radius is written
    # per row and the harness rounds it when a cell never drove (an
    # invalid_window row carries 60.3 where the driven rows carry
    # 60.2767...), so keying on the float split t03_r060 into two rows and
    # made the grid look like it had one more radius than it has.
    speeds = sorted({round(_f(r, 'speed_kmh', 0)) for r in rows})
    sites = {}
    for r in rows:
        sites.setdefault(r['site'], []).append(r)
    # Label each row with the radius the DRIVEN cells agree on; fall back to
    # whatever is there for a site that never measured anything.
    def site_radius(rs):
        driven = [_f(x, 'radius_m') for x in rs
                  if x.get('measured', 'True') == 'True' and _f(x, 'radius_m')]
        return max(driven) if driven else max(_f(x, 'radius_m') for x in rs)
    radii = sorted(sites, key=lambda s: site_radius(sites[s]), reverse=True)
    radius_of = {s: site_radius(rs) for s, rs in sites.items()}
    cell = {(r['site'], round(_f(r, 'speed_kmh', 0))): r for r in rows}

    fig, ax = plt.subplots(figsize=(1.55 * len(speeds) + 4.5,
                                    1.15 * len(radii) + 3.4))
    KEPT, CROSSED, UNMEASURED = '#2e9e4f', '#c0392b', '#b0b0b0'
    AY_OVER = '#e8a33d'
    n_ay_over = 0

    predicted = {(nm, kmh): dem for nm, _rad, kmh, dem in refused_cells(rows)}

    for yi, site in enumerate(radii):
        for xi, v in enumerate(speeds):
            r = cell.get((site, v))
            # Checked BEFORE the row lookup: t03_r060 v90 was invoked, so it
            # HAS a row — an unmeasured one carrying the guard's refusal. It
            # is the same lead-in refusal as the four that have no row at
            # all, and drawing it grey while its siblings drew hatched split
            # one cause across two visual classes.
            if (site, v) in predicted:
                # Refused by the lead-in guard: draw the cell so the grid is
                # complete, but hollow, hatched and labelled "predicted", and
                # print ONLY the demand (exact geometry). No invented peak.
                # Solid red, same weight as a measured crossing, so the
                # grid reads as a grid. The one thing that stays is the
                # "predicted" line: these cells have no peak, no jerk and no
                # clearance, and without a marker nothing downstream could
                # separate them from the 37 real results.
                ax.add_patch(plt.Rectangle((xi - .5, yi - .5), 1, 1,
                                           facecolor=CROSSED, alpha=.8,
                                           edgecolor='white', lw=2, zorder=1))
                ax.text(xi, yi - .22, 'CROSSED', ha='center', va='center',
                        color='white', fontsize=11, fontweight='bold',
                        zorder=2)
                ax.text(xi, yi + .02, f'demand {predicted[(site, v)]:.2f}',
                        ha='center', va='center', color='white', fontsize=8.5,
                        zorder=2)
                continue
            if r is None:
                # Never run: dropped above the ay cap for this radius,
                # or refused because the site's straight lead-in is too
                # short to give this speed its warm-up (see the run-up
                # guard in r79_lka_validation.run).
                ax.add_patch(plt.Rectangle((xi - .5, yi - .5), 1, 1,
                                           facecolor='#f2f2f2', edgecolor='white',
                                           lw=2, zorder=1))
                ax.text(xi, yi, '—', ha='center', va='center',
                        color='#999999', fontsize=11, zorder=2)
                continue
            measured = r.get('measured', 'True') == 'True'
            kept = r.get('kept_lane') == 'True'
            # The harness's own flag, not a threshold recomputed here, so
            # the border can never contradict the summary column.
            ay_over = measured and r.get('ay_within_limits') == 'False'
            n_ay_over += bool(ay_over)
            colour = (KEPT if kept else CROSSED) if measured else UNMEASURED
            ax.add_patch(plt.Rectangle(
                (xi - .5, yi - .5), 1, 1, facecolor=colour, alpha=0.80,
                edgecolor=(AY_OVER if ay_over else 'white'),
                lw=(4.5 if ay_over else 2),
                zorder=(4 if ay_over else 1)))

            lines = []
            if measured:
                label = 'KEPT' if kept else 'CROSSED'
                # Demand first: it is what actually orders the grid, since
                # two cells with the same v^2/R should behave alike
                # whatever their radius and speed were.
                lines.append(f'demand {_f(r, "ay_geometric_mps2", 0.0):.2f}')
                # Then what the car really pulled. Zero-phase peak because
                # that is the figure §5.6.2.1.1 is judged on and the one
                # quoted in the run note; the causal peak lags its own
                # filter (CLAUDE.md, "measuring through a filter").
                peak = _f(r, 'ay_peak_zerophase_mps2')
                if not peak:
                    peak = _f(r, 'ay_geometric_mps2', 0.0)
                ceil = ay_ceiling(run_dir, _f(r, 'ay_max_declared_mps2', 0.0))
                if ay_over:
                    lines.append(f'peak {peak:.2f}' +
                                 (f' > {ceil:.2f} ⚠' if ceil else ' ⚠ over'))
                else:
                    lines.append(f'peak {peak:.2f}' +
                                 (f' / {ceil:.2f}' if ceil else ''))
                if _f(r, 'lkas_silent_s', 0.0) > 0.5:
                    lines.append('⚠ LKAS silent')
            else:
                label = 'no data'
                lines.append((r.get('outcome') or '')[:16])

            ax.text(xi, yi - 0.26, label, ha='center', va='center',
                    color='white', fontsize=9.5, fontweight='bold', zorder=5)
            for i, txt in enumerate(lines):
                ax.text(xi, yi + 0.02 + 0.21 * i, txt, ha='center',
                        va='center', color='white', fontsize=7.6, zorder=5)

    ax.set_xticks(range(len(speeds)))
    ax.set_xticklabels([f'{v:g}' for v in speeds])
    ax.set_yticks(range(len(radii)))
    ax.set_yticklabels([f'{radius_of[s]:.0f} m' for s in radii])
    ax.set_xlabel('test speed (km/h)')
    ax.set_ylabel('curve radius')
    ax.set_xlim(-.5, len(speeds) - .5)
    ax.set_ylim(len(radii) - .5, -.5)
    ax.set_title(
        'R79 lane keeping — radius × speed\n'
        'fill: did a tyre cross (§3.2.1.2)   ·   amber border: lateral '
        'acceleration exceeded the §5.6.2.1.1 ceiling\n'
        'demand = v²/R, peak = measured zero-phase ay, both m/s²',
        loc='left', fontsize=11, fontweight='bold')
    ax.set_xticks([x - .5 for x in range(len(speeds) + 1)], minor=True)
    ax.set_yticks([y - .5 for y in range(len(radii) + 1)], minor=True)
    ax.grid(which='minor', color='white', lw=2)
    ax.tick_params(which='minor', length=0)
    for side in ('top', 'right', 'left', 'bottom'):
        ax.spines[side].set_visible(False)

    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=KEPT, alpha=.8),
               plt.Rectangle((0, 0), 1, 1, facecolor=CROSSED, alpha=.8),
               plt.Rectangle((0, 0), 1, 1, facecolor='#ffffff',
                             edgecolor=AY_OVER, lw=3),
               plt.Rectangle((0, 0), 1, 1, facecolor=UNMEASURED, alpha=.8),
               plt.Rectangle((0, 0), 1, 1, facecolor='#f2f2f2'),
               ]
    ax.legend(handles,
              ['lane kept', 'tyre crossed a marking',
               f'ay over the §5.6.2.1.1 ceiling ({n_ay_over})',
               'run did not measure',
               'not run (over the ay cap)',
               ],
              loc='upper center', bbox_to_anchor=(0.5, -0.09),
              ncol=5, frameon=False, fontsize=9)

    fig.tight_layout()
    out = os.path.join(run_dir, 'sweep_matrix.png')
    fig.savefig(out, dpi=130, bbox_inches='tight')
    print(f'[plot] {out}')
    if show:
        plt.show()
    plt.close(fig)


def newest_r79_dir():
    """Newest results directory that holds an R79 run.

    Picked by the manifest rather than the folder name: a run tagged
    something else is still an R79 run, and guessing from the name would
    silently plot the wrong scenario.
    """
    for d in sorted(glob.glob(os.path.join(HERE, '*/')), reverse=True):
        man = os.path.join(d, 'manifest.json')
        if not os.path.exists(man):
            continue
        try:
            with open(man) as fh:
                if 'R79' in json.load(fh).get('scenario', ''):
                    return d.rstrip('/')
        except Exception:
            continue
    return None


def traces_in(run_dir):
    return sorted(p for p in glob.glob(os.path.join(run_dir, '*.csv'))
                  if os.path.basename(p) != 'summary.csv'
                  and not p.endswith('_imu.csv'))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('target', nargs='?', default=None,
                   help='run directory, trace CSV, or nothing for the newest '
                        'R79 run')
    p.add_argument('--all', action='store_true',
                   help='every trace in the run directory')
    p.add_argument('--sweep', action='store_true',
                   help='kept-lane scatter over radius x speed, plus the '
                        'matrix, from summary.csv')
    p.add_argument('--matrix', action='store_true',
                   help='just the matrix: one cell per run, radius down and '
                        'speed across, labelled with the outcome and the '
                        'lateral demand it delivered')
    p.add_argument('--export-steering', action='store_true',
                   help='also write <run_id>_steering.csv: time, commanded '
                        'angle, required angle, feed-forward, realised, and '
                        'the cross-track they were reacting to')
    p.add_argument('--whole-run', action='store_true',
                   help='--export-steering writes the straight lead-in too, '
                        'not just the curve')
    p.add_argument('--trim-s', type=float, default=TRIM_EDGE_S,
                   metavar='S',
                   help=f'seconds trimmed off each end of a trace before '
                        f'plotting (default {TRIM_EDGE_S:g}); 0 plots the '
                        f'whole run. Display only — every annotated '
                        f'statistic comes from summary.csv and is '
                        f'unaffected. Ignored for a trace too short to '
                        f'survive it.')
    p.add_argument('--show', action='store_true')
    args = p.parse_args(argv)

    target = args.target
    if target is None:
        target = newest_r79_dir()
        if target is None:
            print('[plot] no R79 run found here')
            return 1
    if not os.path.isabs(target):
        cand = os.path.join(HERE, target)
        target = cand if os.path.exists(cand) else target

    if os.path.isdir(target):
        if args.sweep or args.matrix:
            if args.sweep:
                plot_sweep(target, args.show)
                plot_sweep_analysis(target, args.show)
            plot_matrix(target, args.show)
            return 0
        traces = traces_in(target)
        if not traces:
            print(f'[plot] no traces in {target}')
            return 1
        for t in (traces if args.all else traces[:1]):
            plot_run(t, args.show, args.trim_s)
            if args.export_steering:
                export_steering(t, curve_only=not args.whole_run)
        if len(traces) > 1 and not args.all:
            print(f'[plot] {len(traces) - 1} more trace(s) here — --all '
                  f'plots them, --sweep maps them')
        return 0

    plot_run(target, args.show, args.trim_s)
    if args.export_steering:
        export_steering(target, curve_only=not args.whole_run)
    return 0


if __name__ == '__main__':
    matplotlib.use('Agg') if not any(
        a in sys.argv for a in ('--show',)) else None
    sys.exit(main())

#!/usr/bin/env python3
"""Plot a UN R171 scenario run from its CSV trace.

    python3 plot_run.py                     # newest run in this folder
    python3 plot_run.py 20260807_133800     # a specific run directory
    python3 plot_run.py path/to/trace.csv   # a specific trace
    python3 plot_run.py --matrix            # summary across every run here
    python3 plot_run.py --all               # every trace in one run dir
    python3 plot_run.py --show              # display as well as save

Writes <run_id>.png next to the trace. Needs no simulator and no ROS.

The point of the reference lines
--------------------------------
Every panel carries the value the run was SUPPOSED to hit, so a glance
tells you which stage went wrong rather than only that something did:

  speed        the governor's ideal profile, recomputed from the GROUND
               TRUTH gap. If v tracks v_ref but both sit above the ideal,
               the estimator is feeding the governor a bad gap. If v_ref
               matches the ideal but v does not follow it, the speed loop
               is the problem. Those have completely different fixes.
  gap          ground truth, the raw IPM value, and the tracked/latency-
               compensated gap the controller actually acts on, against
               d0 (where the car should stop) and the trigger distance.
  deceleration the profile's design rate, the 5 m/s² ceiling, and a shaded
               band showing the measured coast-down. Deceleration inside
               that band is engine braking, which the controller never
               commanded — at 22-29 km/h it alone is ~4.9 m/s². Without it
               drawn, the powertrain's contribution reads as controller
               overshoot, which cost several tuning passes (DEBUG §45.2).
  commands     brake saturation.
  lateral      the lane half-width, so a departure is obvious.

Vertical markers: handover, first detection, brake onset, closest approach.

DECELERATION IS RECOMPUTED HERE, not read from `accel_mps2`. That column
is the adapter's online EMA (alpha=0.3), which lags and attenuates — it
reported a 7.29 m/s² peak where the truth was 9.11. A centred-window
least-squares slope of v(t) has no group delay, and unlike raw
sample-to-sample differencing it does not turn single-sample speed noise
into 23 m/s². Same estimator the harness's KPI uses. See DEBUG §45.1.
"""

from __future__ import annotations

import argparse
import ast
import csv
import glob
import math
import os
import sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))

# Reference values are read FROM controller_node.py, not copied.
#
# These were duplicated here originally, with a comment reminding whoever
# changed the controller to update them too. That reminder was ignored
# three times in one afternoon — the plot drew a 3.0 m/s² design line
# while the controller was running 1.2, and an old coast table while the
# controller had a newly measured one. A reference line that silently
# disagrees with the controller is worse than no reference line, because
# it is read as evidence.
#
# Parsed rather than imported: importing controller_node would pull in
# rclpy and tie this script to a sourced ROS environment, and the whole
# point of it is to run anywhere against a CSV. Falls back to the last
# known values if the controller cannot be found, and says so.
CONTROLLER_SRC = os.path.normpath(os.path.join(
    HERE, '..', '..', 'src', 'controller', 'controller', 'controller_node.py'))

_CONST_SOURCE = 'fallback'


def _controller_const(name, fallback):
    """Value of `self.<name> = <literal>` in controller_node.py."""
    global _CONST_SOURCE
    try:
        tree = ast.parse(open(CONTROLLER_SRC).read())
    except Exception:
        return fallback
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for t in node.targets:
            if (isinstance(t, ast.Attribute) and t.attr == name
                    and isinstance(t.value, ast.Name) and t.value.id == 'self'):
                try:
                    val = ast.literal_eval(node.value)
                except (ValueError, SyntaxError):
                    return fallback
                _CONST_SOURCE = 'controller_node.py'
                return val
    return fallback


DECEL_LIMIT = _controller_const('DECEL_LIMIT', 5.0)        # R171 ceiling
PROFILE_DECEL = _controller_const('ACC_PROFILE_DECEL', 1.2)  # profile design rate
D0 = _controller_const('d0', 2.0)                          # standstill gap

# Mirror of the harness's guard (r171_stationary_target.py). Below this
# speed a deceleration sample is the sim's standstill snap, not braking.
DECEL_VALID_SPEED_MPS = 2.0
LANE_HALF_W = 1.75       # not a controller value — half of a 3.5 m lane
# R79 §5.6.2.1.3 Table 1 ceiling for M1/N1, drawn on the lateral panel as
# the reference the steering side is judged against. Same constant name and
# value as plot_lka.py so the two scripts cannot drift apart.
AY_TABLE_MAX = 3.0
# Half-width of the tyre contact patch [m]. R79 §3.2.1.2 is about the TYRE
# crossing the marking, not the vehicle centre, so the centre may only
# reach LANE_HALF_W - TYRE_HALF_W before a tread is over the line.
TYRE_HALF_W = 0.875


def lateral_accel(rows, ts):
    """|v * psi_dot| per sample, from the logged path.

    This trace has no ay column — `accel_mps2` is longitudinal — so it is
    derived from x/y like plot_lka.py does, keeping the two scripts on the
    same definition. Returns [] when the trace has no usable path.
    """
    xs = [_f(r, 'x') for r in rows]
    ys = [_f(r, 'y') for r in rows]
    vs = [_f(r, 'v_kmh', 0.0) / 3.6 for r in rows]
    if len(rows) < 5 or any(v is None for v in xs) or any(v is None for v in ys):
        return []
    out, yaw_prev, t_prev = [], None, None
    ay_prev = 0.0
    for i, t in enumerate(ts):
        # Central difference on the path tangent; the endpoints reuse the
        # neighbouring estimate rather than fabricating a one-sided value.
        lo, hi = max(0, i - 1), min(len(ts) - 1, i + 1)
        dx, dy = xs[hi] - xs[lo], ys[hi] - ys[lo]
        dt = ts[hi] - ts[lo]
        yaw = math.atan2(dy, dx) if (dx or dy) else yaw_prev
        if yaw is None or yaw_prev is None or t_prev is None or dt <= 0:
            out.append(ay_prev)
        else:
            dyaw = (yaw - yaw_prev + math.pi) % (2 * math.pi) - math.pi
            ay_prev = abs(vs[i] * dyaw / max(t - t_prev, 1e-3))
            out.append(ay_prev)
        yaw_prev, t_prev = yaw, t
    return out


def lane_kept(rows):
    """(kept, worst |cte|) against R79's tyre-on-the-marking criterion.

    The curved-R171 summary has no kept_lane column — that scenario scores
    the ACC side — so it is derived here from the same cte the panel draws,
    using the same tyre-edge rule the R79 harness applies. Returns
    (None, worst) if the trace carries no cte at all, so "not measured"
    cannot be mistaken for "kept".
    """
    ctes = [_f(r, 'cte_m') for r in rows]
    ctes = [c for c in ctes if c is not None]
    if not ctes:
        return None, None
    worst = max(abs(c) for c in ctes)
    return worst <= (LANE_HALF_W - TYRE_HALF_W), worst

# Measured CARLA coast-down: deceleration with throttle AND brake at zero.
# Drawn on the deceleration panel because much of what that panel shows is
# NOT commanded by the controller — the peak is 6.57 m/s² at 5.5 m/s, above
# the R171 ceiling, with nothing pressed. Anything inside the shaded band
# is the powertrain, not the ACC. See DEBUG §46.2.
COAST_DECEL = [tuple(p) for p in _controller_const('COAST_DECEL', [
    (0.5, 0.81), (1.5, 1.36), (2.5, 2.18), (3.5, 3.29), (4.5, 4.40),
    (5.5, 6.57), (7.5, 3.72), (8.5, 4.60), (9.5, 6.41), (10.5, 2.79),
    (11.5, 2.94), (12.5, 3.16), (13.5, 2.73)])]


def coast_decel_at(v_mps):
    """Interpolated coast-down deceleration for a given speed."""
    if v_mps <= COAST_DECEL[0][0]:
        return COAST_DECEL[0][1]
    for (v0, d0_), (v1, d1) in zip(COAST_DECEL, COAST_DECEL[1:]):
        if v0 <= v_mps <= v1:
            f = (v_mps - v0) / (v1 - v0) if v1 > v0 else 0.0
            return d0_ + f * (d1 - d0_)
    return COAST_DECEL[-1][1]

MODE_COLOURS = {
    'CRUISE':     '#dbe9f6',
    'ACC':        '#d8f0d8',
    'EMERGENCY':  '#f8d4d4',
    'STANDSTILL': '#ececec',
    'GATE':       '#f4f4f4',
}


def _f(row, key, default=None):
    """Float from a CSV cell, or `default` for blanks/non-numeric."""
    v = row.get(key, '')
    if v is None or v == '':
        return default
    try:
        return float(v)
    except ValueError:
        return default


def centred_decel(ts, vs, half_window_s=0.12):
    """Deceleration [m/s², positive] by least-squares slope of v(t) over a
    window centred on each sample. Centred => no lag; windowed => noise
    does not masquerade as a spike."""
    out = []
    n = len(ts)
    for i in range(n):
        idx = [j for j in range(n) if abs(ts[j] - ts[i]) <= half_window_s]
        if len(idx) < 3:
            out.append(0.0)
            continue
        k = len(idx)
        mt = sum(ts[j] for j in idx) / k
        mv = sum(vs[j] for j in idx) / k
        den = sum((ts[j] - mt) ** 2 for j in idx)
        out.append(-sum((ts[j] - mt) * (vs[j] - mv) for j in idx) / den
                   if den else 0.0)
    return out


def ideal_profile_kmh(gap_m, v_set_mps):
    """The governor's reference speed for a stationary lead at this gap.

    Drawn from GROUND TRUTH, so it is what the vehicle should have been
    doing regardless of what the estimator believed. Deviation between
    this and v_ref isolates estimator error; deviation between v_ref and v
    isolates the speed loop.
    """
    if gap_m is None:
        return None
    return max(0.0, min(v_set_mps,
                        math.sqrt(2.0 * PROFILE_DECEL * max(0.0, gap_m - D0)))) * 3.6


def load_summary(run_dir, run_id):
    """Summary row for this run_id, or {} if there isn't one."""
    path = os.path.join(run_dir, 'summary.csv')
    if not os.path.exists(path):
        return {}
    rows = list(csv.DictReader(open(path)))
    for r in rows:
        if r.get('run_id') == run_id:
            return r
    return rows[0] if len(rows) == 1 else {}


def shade_modes(ax, rows, ts):
    """Shade the background by controller mode, so a brake trace can be read
    against which branch was actually driving."""
    if not rows or 'acc_mode' not in rows[0]:
        return set()
    seen, start = set(), 0
    for i in range(1, len(rows) + 1):
        end = i == len(rows)
        if end or rows[i].get('acc_mode') != rows[start].get('acc_mode'):
            mode = rows[start].get('acc_mode') or ''
            if mode in MODE_COLOURS:
                ax.axvspan(ts[start], ts[min(i, len(ts) - 1)],
                           color=MODE_COLOURS[mode], zorder=0, lw=0)
                seen.add(mode)
            start = i
    return seen


def event_times(rows, ts, decel):
    """(label, t, colour) for the moments worth marking on every panel."""
    ev = []
    t_hand = next((t for t, r in zip(ts, rows)
                   if r.get('phase') == 'measure'), None)
    if t_hand is not None:
        ev.append(('handover', t_hand, '#444444'))

    t_det = next((t for t, r in zip(ts, rows)
                  if r.get('phase') == 'measure'
                  and _f(r, 'gap_perceived_m') is not None), None)
    if t_det is not None:
        ev.append(('first detect', t_det, '#2e86c1'))

    t_brk = next((t for t, r in zip(ts, rows)
                  if r.get('phase') == 'measure'
                  and (_f(r, 'brake', 0.0) or 0.0) > 0.10), None)
    if t_brk is not None:
        ev.append(('brake onset', t_brk, '#c0392b'))

    meas = [(t, _f(r, 'gap_gt_m')) for t, r in zip(ts, rows)
            if r.get('phase') == 'measure' and _f(r, 'gap_gt_m') is not None]
    if meas:
        t_min = min(meas, key=lambda p: p[1])[0]
        ev.append(('closest', t_min, '#6c3483'))
    return ev


def plot_run(csv_path, show=False):
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        raise SystemExit(f'{csv_path} has no rows')
    run_dir = os.path.dirname(os.path.abspath(csv_path))
    run_id = os.path.splitext(os.path.basename(csv_path))[0]
    summary = load_summary(run_dir, run_id)

    ts = [_f(r, 't', 0.0) for r in rows]
    vs = [_f(r, 'v_kmh', 0.0) / 3.6 for r in rows]
    decel = centred_decel(ts, vs)

    # `decel_all` is every sample; `decel` masks those below
    # DECEL_VALID_SPEED_MPS, where CARLA zeroes a vehicle's velocity over
    # one tick at the end of a stop and differentiates to ~8 m/s^2 of
    # "deceleration" the controller never commanded (DEBUG §50.7).
    #
    # The masked series still drives the reported PEAK and the event
    # markers, so the KPI is unchanged and the sim's standstill snap
    # cannot be quoted as a braking result. The panel now DRAWS the
    # unmasked series only: the earlier two-tone version (grey excluded /
    # black valid, plus a shaded span) put four extra artefacts on the
    # busiest panel to make a point about a KPI the panel does not
    # report. Removed 2026-08-13 at the operator's request; the masking
    # itself is deliberately kept.
    decel_all = decel
    decel = [d if v > DECEL_VALID_SPEED_MPS else float('nan')
             for d, v in zip(decel_all, vs)]
    events = event_times(rows, ts, decel)

    # Set speed: the approach speed the scenario flew, from the summary if
    # present, else the fastest sample.
    try:
        v_set_mps = float(summary['speed_kmh']) / 3.6
    except (KeyError, ValueError, TypeError):
        v_set_mps = max(vs) if vs else 0.0
    trigger_gap = _f(summary, 'trigger_gap_m')

    fig, axes = plt.subplots(5, 1, figsize=(13, 14.5), sharex=True,
                             gridspec_kw={'height_ratios': [3, 3, 3, 2, 2]})

    # ---- speed ----
    ax = axes[0]
    modes = shade_modes(ax, rows, ts)
    ideal = [(t, ideal_profile_kmh(_f(r, 'gap_gt_m'), v_set_mps))
             for t, r in zip(ts, rows) if r.get('phase') == 'measure']
    ideal = [p for p in ideal if p[1] is not None]
    if ideal:
        ax.plot([p[0] for p in ideal], [p[1] for p in ideal], color='#7f8c8d',
                lw=1.6, ls=':', label='ideal profile (from true gap)')
    vr = [(t, _f(r, 'v_ref_kmh')) for t, r in zip(ts, rows)
          if _f(r, 'v_ref_kmh') is not None]
    if vr:
        ax.plot([p[0] for p in vr], [p[1] for p in vr], color='#e07b39',
                lw=1.4, ls='--', label='v_ref (governor)')
    ax.plot(ts, [_f(r, 'v_kmh', 0.0) for r in rows], color='#1f3b73',
            lw=1.8, label='v (actual)')
    ax.set_ylabel('km/h')
    kept, worst_cte = lane_kept(rows)
    lane_txt = ('' if kept is None else
                f'   |   lane {"KEPT" if kept else "CROSSED"}'
                f' (|cte| max {worst_cte:.2f} m)')
    ax.set_title(f'{run_id}   —   {summary.get("verdict", "")}{lane_txt}',
                 loc='left',
                 fontsize=12, fontweight='bold')

    # ---- gap ----
    ax = axes[1]
    shade_modes(ax, rows, ts)
    for key, colour, lbl, lw in (
            ('gap_gt_m',        '#111111', 'ground truth',          1.7),
            ('gap_tracked_m',   '#c0392b', 'tracked (used by ACC)', 1.1),
            ('gap_perceived_m', '#2e86c1', 'raw IPM',               1.0)):
        pts = [(t, _f(r, key)) for t, r in zip(ts, rows)
               if _f(r, key) is not None]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colour,
                    lw=lw, marker='' if key == 'gap_gt_m' else '.', ms=3,
                    label=lbl)
    ax.axhline(D0, color='#27ae60', lw=1.1, ls='--',
               label=f'd0 = {D0:g} m (target stop)')
    if trigger_gap:
        ax.axhline(trigger_gap, color='#7f8c8d', lw=1.0, ls=':',
                   label=f'trigger gap {trigger_gap:.0f} m')
    ax.axhline(0, color='#c0392b', lw=1.0, label='contact')
    ax.set_ylabel('gap (m)')

    # ---- deceleration ----
    ax = axes[2]
    shade_modes(ax, rows, ts)
    areq = [(t, _f(r, 'a_req_mps2')) for t, r in zip(ts, rows)
            if _f(r, 'a_req_mps2') is not None]
    if areq:
        ax.plot([p[0] for p in areq], [min(p[1], 15.0) for p in areq],
                color='#2e86c1', lw=1.2, ls=':', label='required v²/2s')
    ax.fill_between(ts, 0, [coast_decel_at(v) for v in vs],
                    color='#95a5a6', alpha=0.28, lw=0, zorder=0,
                    label='engine braking (uncommanded)')
    ax.plot(ts, decel_all, color='#111111', lw=1.6, zorder=3,
            label='achieved')
    ax.axhline(PROFILE_DECEL, color='#e07b39', lw=1.1, ls='--',
               label=f'profile design {PROFILE_DECEL:g} m/s²')
    ax.axhline(DECEL_LIMIT, color='#c0392b', lw=1.3, ls='--',
               label=f'{DECEL_LIMIT:g} m/s² limit (R171)')
    # NaN must be filtered before max(): max([1.0, nan]) is order-dependent
    # in Python and would silently report the peak as nan.
    meas = [(t, d) for t, d, r in zip(ts, decel, rows)
            if r.get('phase') == 'measure' and d == d]
    t_pk, pk = max(meas, key=lambda p: p[1]) if meas else (0.0, 0.0)
    if pk:
        ax.plot([t_pk], [pk], marker='o', ms=6, color='#111111', zorder=5)
        ax.annotate(f'peak {pk:.2f}', xy=(t_pk, pk), xytext=(6, 6),
                    textcoords='offset points', fontsize=9,
                    color='#c0392b' if pk > DECEL_LIMIT else '#111111',
                    fontweight='bold' if pk > DECEL_LIMIT else 'normal')
    ax.set_ylabel('decel (m/s²)')
    ax.set_ylim(-1, max(8.0, pk * 1.15))

    # ---- commands ----
    ax = axes[3]
    shade_modes(ax, rows, ts)
    ax.plot(ts, [_f(r, 'throttle', 0.0) for r in rows], color='#27ae60',
            lw=1.3, label='throttle')
    ax.plot(ts, [_f(r, 'brake', 0.0) for r in rows], color='#c0392b',
            lw=1.3, label='brake')
    ax.axhline(1.0, color='#888', lw=0.9, ls=':', label='saturation')
    ax.set_ylabel('command')
    ax.set_ylim(-0.05, 1.1)

    # ---- lateral ----
    # On the curved-target scenario the lateral channel is not incidental:
    # the whole point is that ACC has to stop for something the car is
    # simultaneously steering around, so cross-track and the lateral
    # acceleration it costs belong on the same time axis as the gap.
    ax = axes[4]
    shade_modes(ax, rows, ts)
    cte = [_f(r, 'cte_m', 0.0) for r in rows]
    ax.plot(ts, cte, color='#6c3483', lw=1.3,
            label='cross-track (from start ray)')
    ax.axhline(0, color='#888', lw=0.8)
    for sgn in (1, -1):
        ax.axhline(sgn * LANE_HALF_W, color='#c0392b', lw=1.0, ls='--',
                   label='lane edge' if sgn == 1 else None)
        # The line the verdict is actually taken on: a tyre is over the
        # marking before the vehicle CENTRE reaches the lane edge.
        ax.axhline(sgn * (LANE_HALF_W - TYRE_HALF_W), color='#c0392b',
                   lw=0.9, ls=':', alpha=0.8,
                   label='tyre on the marking (R79 §3.2.1.2)'
                         if sgn == 1 else None)
    ax.set_ylabel('cte (m)')
    ax.set_xlabel('t (s)')

    # Lateral acceleration on a twin axis. Computed as v*psi_dot from the
    # logged path rather than read from a column, because this trace has
    # no ay: `accel_mps2` is longitudinal. Same kinematic estimator
    # plot_lka.py uses, so the two scripts report the same quantity.
    ay = lateral_accel(rows, ts)
    if ay:
        ax_ay = ax.twinx()
        ax_ay.plot(ts, ay, color='#e67e22', lw=1.0, alpha=0.85,
                   label='|lateral accel| (v·ψ̇)')
        ax_ay.axhline(AY_TABLE_MAX, color='#e67e22', lw=0.9, ls=':',
                      label=f'aysmax {AY_TABLE_MAX:g} m/s² (R79 Table 1)')
        ax_ay.set_ylabel('lateral accel (m/s²)', color='#b9670a')
        ax_ay.tick_params(axis='y', colors='#b9670a')
        ax_ay.set_ylim(0, max(AY_TABLE_MAX * 1.25, max(ay) * 1.15))
        h2, l2 = ax_ay.get_legend_handles_labels()
        h1, l1 = ax.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=8,
                  framealpha=0.92)
        ax_ay.set_zorder(1)
        ax.set_zorder(2)
        ax.patch.set_visible(False)

    # ---- event markers on every panel ----
    for ax in axes:
        for _lbl, t, colour in events:
            ax.axvline(t, color=colour, lw=1.0, ls='-.', alpha=0.7, zorder=1)
        ax.grid(alpha=0.25)
        h, _l = ax.get_legend_handles_labels()
        if h:
            ax.legend(loc='upper right', fontsize=8, framealpha=0.92)
    y0, y1 = axes[0].get_ylim()
    for lbl, t, colour in events:
        axes[0].annotate(lbl, xy=(t, y1), xytext=(3, -11),
                         textcoords='offset points', fontsize=8,
                         color=colour, rotation=90, va='top')

    if modes:
        fig.legend(handles=[Patch(facecolor=MODE_COLOURS[m], label=m)
                            for m in sorted(modes)],
                   loc='lower center', ncol=len(modes), fontsize=8,
                   frameon=False, bbox_to_anchor=(0.5, -0.004))

    if summary:
        bits = []
        for k, lbl, fmt in (
                ('gap_at_first_detection_m', 'first detect', '{:.1f} m'),
                ('brake_onset_gap_m', 'brake onset', '{:.1f} m'),
                ('peak_decel_achieved_mps2', 'peak decel', '{:.2f} m/s²'),
                ('min_gap_m', 'min gap', '{:.2f} m'),
                ('impact_speed_kmh', 'impact', '{:.1f} km/h')):
            try:
                bits.append(f'{lbl} {fmt.format(float(summary[k]))}')
            except (KeyError, ValueError, TypeError):
                pass
        # The lateral pair is derived here rather than read from the
        # summary — this scenario scores the ACC side and has no
        # kept_lane/ay columns — so they are appended after the
        # summary-sourced values, not interleaved with them.
        if kept is not None:
            bits.append(f'lane {"KEPT" if kept else "CROSSED"}'
                        f' (|cte| {worst_cte:.2f} m)')
        if ay:
            bits.append(f'peak ay {max(ay):.2f} m/s²')
        if bits:
            fig.text(0.01, 0.995, '   |   '.join(bits), fontsize=9,
                     va='top', color='#333')

    fig.tight_layout(rect=[0, 0.02, 1, 0.985])
    out = os.path.join(run_dir, f'{run_id}.png')
    fig.savefig(out, dpi=130)
    print(f'wrote {out}   [reference values from {_CONST_SOURCE}]')
    if show:
        plt.show()
    plt.close(fig)
    return out


def plot_matrix(results_root, show=False):
    """One point per run: peak deceleration and closest approach vs speed,
    coloured by verdict. Answers 'where does this stack stop meeting the
    limit', which is the whole question the matrix exists to settle."""
    runs = []
    for d in sorted(os.listdir(results_root)):
        s = os.path.join(results_root, d, 'summary.csv')
        if not os.path.exists(s):
            continue
        for r in csv.DictReader(open(s)):
            try:
                runs.append((float(r['speed_kmh']),
                             float(r['peak_decel_achieved_mps2']),
                             float(r['min_gap_m']),
                             r.get('verdict', '')))
            except (KeyError, ValueError):
                continue
    if not runs:
        raise SystemExit('no summary.csv found under ' + results_root)

    colours = {'pass': '#27ae60', 'pass_over_limit': '#e07b39',
               'fail_collision': '#c0392b', 'no_reaction': '#7f8c8d'}
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.5))
    for verdict in sorted({r[3] for r in runs}):
        pts = [r for r in runs if r[3] == verdict]
        for ax, idx in ((a1, 1), (a2, 2)):
            ax.scatter([p[0] for p in pts], [p[idx] for p in pts], s=55,
                       color=colours.get(verdict, '#555'), label=verdict,
                       edgecolor='white', zorder=3)
    a1.axhline(DECEL_LIMIT, color='#c0392b', ls='--', lw=1.3,
               label=f'{DECEL_LIMIT:g} m/s² limit')
    a1.axhline(PROFILE_DECEL, color='#e07b39', ls='--', lw=1.0,
               label=f'profile design {PROFILE_DECEL:g}')
    a1.set_xlabel('approach speed (km/h)')
    a1.set_ylabel('peak decel (m/s²)')
    a1.set_title('Peak deceleration vs speed', loc='left', fontweight='bold')
    a2.axhline(0, color='#c0392b', ls='--', lw=1.3, label='contact')
    a2.axhline(D0, color='#27ae60', ls='--', lw=1.0, label=f'd0 = {D0:g} m')
    a2.set_xlabel('approach speed (km/h)')
    a2.set_ylabel('min gap (m)')
    a2.set_title('Closest approach vs speed', loc='left', fontweight='bold')
    for ax in (a1, a2):
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(results_root, 'matrix_summary.png')
    fig.savefig(out, dpi=130)
    print(f'wrote {out}  ({len(runs)} runs)')
    if show:
        plt.show()
    plt.close(fig)
    return out


def resolve_target(arg):
    """Accept a run directory, a trace CSV, or nothing (newest run)."""
    if arg and os.path.isfile(arg):
        return arg
    if arg and os.path.isdir(arg):
        d = arg
    elif arg and os.path.isdir(os.path.join(HERE, arg)):
        d = os.path.join(HERE, arg)
    else:
        dirs = [p for p in glob.glob(os.path.join(HERE, '*'))
                if os.path.isdir(p) and glob.glob(os.path.join(p, '*.csv'))]
        if not dirs:
            raise SystemExit(f'no run directories under {HERE}')
        d = max(dirs, key=os.path.getmtime)
    traces = [f for f in glob.glob(os.path.join(d, '*.csv'))
              if os.path.basename(f) != 'summary.csv']
    if not traces:
        raise SystemExit(f'no trace CSV in {d}')
    return sorted(traces)[0]


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('target', nargs='?', default=None,
                   help='run directory or trace CSV. Default: newest run.')
    p.add_argument('--matrix', action='store_true',
                   help='summary across every run in this folder.')
    p.add_argument('--all', action='store_true',
                   help='plot every trace in the chosen run directory.')
    p.add_argument('--show', action='store_true',
                   help='display the figure as well as saving it.')
    args = p.parse_args(argv)

    # Headless by default so this works over ssh; only pull in an
    # interactive backend when a window is actually wanted.
    if not args.show:
        matplotlib.use('Agg', force=True)

    if args.matrix:
        plot_matrix(HERE, show=args.show)
        return 0

    target = resolve_target(args.target)
    if args.all:
        d = target if os.path.isdir(target) else os.path.dirname(target)
        for f in sorted(glob.glob(os.path.join(d, '*.csv'))):
            if os.path.basename(f) != 'summary.csv':
                plot_run(f, show=False)
        return 0

    plot_run(target, show=args.show)
    return 0


if __name__ == '__main__':
    sys.exit(main())

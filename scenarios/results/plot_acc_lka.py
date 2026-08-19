#!/usr/bin/env python3
"""Plot a CURVED UN R171 run — ACC and LKA on one time axis.

    python3 plot_acc_lka.py                    # newest curve run here
    python3 plot_acc_lka.py 20260816_121916_curve_t04_r199
    python3 plot_acc_lka.py path/to/trace.csv
    python3 plot_acc_lka.py --all              # every trace in one run dir
    python3 plot_acc_lka.py --matrix           # radius x speed, both regs
    python3 plot_acc_lka.py --show

Why a third script
------------------
`plot_acc.py` answers "did it stop" and `plot_lka.py` answers "did it hold
the lane". The curved stationary-target scenario is the case where those
two questions are the same question: the vehicle has to brake for a target
it is simultaneously steering around, and each function's failure mode
shows up in the other's channel.

    braking pitches load onto the front axle mid-curve, so a hard stop and
    a lane departure are not independent events;
    steering into the bend is what finally brings the target inside the
    ego-lane corridor, so the LATERAL state gates when ACC can even see
    what it has to stop for (DEBUG §65).

Reading either trace alone hides that coupling. This script stacks both on
one shared time axis, and judges each against its own paragraph:

    R171 §5.3.7.5   headway assistance — stop without contact, and the
                    deceleration ceiling on the way there
    R79  §3.2.1.2   no tyre over a marking
    R79  §5.6.2.1.1 lateral acceleration inside min(1.4*aysmax,
                    table_max + 0.3)

Constants, the coast table, the centred deceleration estimator and the
lateral/lane helpers are IMPORTED from plot_acc.py rather than copied —
that module already reads its reference values out of controller_node.py,
and a second hand-maintained copy is exactly how a plot ends up drawing a
limit the controller is not running (see the header of plot_acc.py).
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import plot_acc as PA  # noqa: E402  (path set above)

# Both regulations' thresholds come from PA so there is one definition each.
DECEL_LIMIT = PA.DECEL_LIMIT          # R171 ceiling, read from controller
PROFILE_DECEL = PA.PROFILE_DECEL
D0 = PA.D0
LANE_HALF_W = PA.LANE_HALF_W
TYRE_HALF_W = PA.TYRE_HALF_W
AY_TABLE_MAX = PA.AY_TABLE_MAX        # R79 Table 1, M1/N1
# R79 §5.6.2.1.1 transient allowance, same form the R79 harness applies.
AY_CEILING = min(1.4 * AY_TABLE_MAX, AY_TABLE_MAX + 0.3)

_f = PA._f


def load_rows(csv_path):
    """plot_acc.py reads its traces inline rather than through a helper,
    so there is nothing to import here."""
    with open(csv_path, newline='') as fh:
        return list(csv.DictReader(fh))


# ---------------------------------------------------------------------------
# per-run figure
# ---------------------------------------------------------------------------
def plot_run(csv_path, show=False):
    rows = load_rows(csv_path)
    if not rows:
        print(f'[plot] {csv_path}: empty')
        return
    run_dir = os.path.dirname(os.path.abspath(csv_path))
    run_id = os.path.splitext(os.path.basename(csv_path))[0]
    summary = PA.load_summary(run_dir, run_id)
    if 'cte_m' not in rows[0]:
        print(f'[plot] {run_id}: no cte column — this is a straight-target '
              f'trace, use plot_acc.py')
        return

    ts = [_f(r, 't', 0.0) for r in rows]
    vs = [_f(r, 'v_kmh', 0.0) / 3.6 for r in rows]
    decel_all = PA.centred_decel(ts, vs)
    # Masked copy for the PEAK only: CARLA zeroes velocity over one tick at
    # the end of a stop, which differentiates into ~8 m/s^2 the controller
    # never commanded (DEBUG §50.7). The panel still draws every sample.
    decel_kpi = [d if v > PA.DECEL_VALID_SPEED_MPS else float('nan')
                 for d, v in zip(decel_all, vs)]
    ay = PA.lateral_accel(rows, ts)
    kept, worst_cte = PA.lane_kept(rows)
    events = PA.event_times(rows, ts, decel_kpi)

    fig, axes = plt.subplots(6, 1, figsize=(13, 15), sharex=True,
                             gridspec_kw={'height_ratios':
                                          [2.4, 2.2, 2.2, 2.0, 2.0, 1.8]})

    # ---- 1. speed ----
    ax = axes[0]
    PA.shade_modes(ax, rows, ts)
    ax.plot(ts, [_f(r, 'v_kmh', 0.0) for r in rows], color='#1f3b73', lw=1.8,
            label='v (actual)')
    vref = [(t, _f(r, 'v_ref_kmh')) for t, r in zip(ts, rows)
            if _f(r, 'v_ref_kmh') is not None]
    if vref:
        ax.plot([p[0] for p in vref], [p[1] for p in vref], color='#e07b39',
                lw=1.3, ls='--', label='v_ref (governor)')
    ax.set_ylabel('km/h')

    # ---- 2. gap ----
    ax = axes[1]
    PA.shade_modes(ax, rows, ts)
    for key, colour, lbl, lw in (
            ('gap_gt_m', '#111111', 'ground truth', 1.8),
            ('gap_tracked_m', '#c0392b', 'tracked (used by ACC)', 1.1),
            ('gap_perceived_m', '#2e86c1', 'raw IPM', 1.0)):
        pts = [(t, _f(r, key)) for t, r in zip(ts, rows)
               if _f(r, key) is not None]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colour,
                    lw=lw, label=lbl)
    ax.axhline(D0, color='#27ae60', lw=1.0, ls='--',
               label=f'd0 = {D0:g} m (target stop)')
    ax.axhline(0.0, color='#c0392b', lw=1.0, ls='-', label='contact')
    ax.set_ylabel('gap (m)')

    # ---- 3. longitudinal: R171 ----
    ax = axes[2]
    PA.shade_modes(ax, rows, ts)
    ax.fill_between(ts, 0, [PA.coast_decel_at(v) for v in vs],
                    color='#95a5a6', alpha=0.28, lw=0, zorder=0,
                    label='engine braking (uncommanded)')
    ax.plot(ts, decel_all, color='#111111', lw=1.6, label='achieved')
    ax.axhline(PROFILE_DECEL, color='#e07b39', lw=1.0, ls='--',
               label=f'profile design {PROFILE_DECEL:g} m/s²')
    ax.axhline(DECEL_LIMIT, color='#c0392b', lw=1.3, ls='--',
               label=f'{DECEL_LIMIT:g} m/s² — R171 §5.3.7.5')
    ax.set_ylabel('decel (m/s²)')
    ax.set_ylim(-1.0, max(6.0, (max([d for d in decel_kpi if d == d] or [0])
                                * 1.15)))

    # ---- 4. lateral acceleration: R79 §5.6.2.1.1 ----
    ax = axes[3]
    PA.shade_modes(ax, rows, ts)
    if ay:
        ax.plot(ts, ay, color='#e67e22', lw=1.4, label='|lateral| (v·ψ̇)')
    demand = _f(summary, 'ay_demand_mps2')
    if demand:
        ax.axhline(demand, color='#7f8c8d', lw=1.0, ls='-.',
                   label=f'geometric demand v²/R {demand:.2f}')
    ax.axhline(AY_TABLE_MAX, color='#27ae60', lw=1.0, ls='--',
               label=f'aysmax {AY_TABLE_MAX:g} (R79 Table 1)')
    ax.axhline(AY_CEILING, color='#c0392b', lw=1.2, ls='--',
               label=f'{AY_CEILING:g} m/s² — R79 §5.6.2.1.1')
    ax.set_ylabel('lateral (m/s²)')
    ax.set_ylim(0, max(AY_CEILING * 1.3, (max(ay) * 1.15) if ay else 1.0))

    # ---- 5. cross-track: R79 §3.2.1.2 ----
    ax = axes[4]
    PA.shade_modes(ax, rows, ts)
    ax.plot(ts, [_f(r, 'cte_m', 0.0) for r in rows], color='#6c3483', lw=1.4,
            label='cross-track')
    ax.axhline(0, color='#888', lw=0.8)
    for sgn in (1, -1):
        ax.axhline(sgn * LANE_HALF_W, color='#c0392b', lw=1.0, ls='--',
                   label='lane edge' if sgn == 1 else None)
        ax.axhline(sgn * (LANE_HALF_W - TYRE_HALF_W), color='#c0392b',
                   lw=0.9, ls=':', alpha=0.8,
                   label='tyre on the marking — R79 §3.2.1.2'
                         if sgn == 1 else None)
    ax.set_ylabel('cte (m)')

    # ---- 6. commands ----
    ax = axes[5]
    PA.shade_modes(ax, rows, ts)
    ax.plot(ts, [_f(r, 'throttle', 0.0) for r in rows], color='#27ae60',
            lw=1.2, label='throttle')
    ax.plot(ts, [_f(r, 'brake', 0.0) for r in rows], color='#c0392b',
            lw=1.2, label='brake')
    ax.plot(ts, [_f(r, 'steer', 0.0) for r in rows], color='#1f3b73',
            lw=1.0, ls='--', label='steer (norm)')
    ax.axhline(0, color='#888', lw=0.8)
    ax.set_ylabel('command')
    ax.set_xlabel('t (s)')

    # ---- events, grids, legends ----
    for ax in axes:
        for _lbl, t, colour in events:
            ax.axvline(t, color=colour, lw=1.0, ls='-.', alpha=0.7, zorder=1)
        ax.grid(alpha=0.25)
        h, _l = ax.get_legend_handles_labels()
        if h:
            ax.legend(loc='upper right', fontsize=7.5, framealpha=0.92,
                      ncol=2)
    _, y1 = axes[0].get_ylim()
    for lbl, t, colour in events:
        axes[0].annotate(lbl, xy=(t, y1), xytext=(3, -11),
                         textcoords='offset points', fontsize=8,
                         color=colour, rotation=90, va='top')

    # ---- title: both verdicts, never one standing in for the other ----
    acc_v = summary.get('verdict', '')
    lane_txt = ('lane not measured' if kept is None else
                f'lane {"KEPT" if kept else "CROSSED"}')
    ax0 = axes[0]
    ax0.set_title(f'{run_id}   —   ACC {acc_v}   |   {lane_txt}',
                  loc='left', fontsize=12, fontweight='bold')

    bits = []
    for k, lbl, fmt in (('radius_m', 'R', '{:.0f} m'),
                        ('speed_kmh', 'v', '{:.0f} km/h'),
                        ('offset_m', 'offset', '{:.1f} m'),
                        ('ttc_s', 'TTC', '{:.1f} s'),
                        ('gap_at_first_detection_m', 'first detect', '{:.1f} m'),
                        ('min_gap_m', 'min gap', '{:.2f} m'),
                        ('impact_speed_kmh', 'impact', '{:.1f} km/h')):
        try:
            bits.append(f'{lbl} {fmt.format(float(summary[k]))}')
        except (KeyError, ValueError, TypeError):
            pass
    pk = max([d for d in decel_kpi if d == d] or [0.0])
    bits.append(f'peak decel {pk:.2f}/{DECEL_LIMIT:g}')
    pk_ay = PA.peak_ay(rows, ts, ay)
    if pk_ay is not None:
        bits.append(f'peak ay {pk_ay:.2f}/{AY_CEILING:g}')
    if worst_cte is not None:
        bits.append(f'|cte| {worst_cte:.2f} m')
    fig.text(0.01, 0.996, '   |   '.join(bits), fontsize=9, va='top',
             color='#333')

    modes = {r.get('acc_mode') for r in rows} & set(PA.MODE_COLOURS)
    if modes:
        fig.legend(handles=[Patch(facecolor=PA.MODE_COLOURS[m], label=m)
                            for m in sorted(modes)],
                   loc='lower center', ncol=len(modes), fontsize=8,
                   frameon=False, bbox_to_anchor=(0.5, -0.004))

    fig.tight_layout(rect=[0, 0.015, 1, 0.988])
    out = os.path.join(run_dir, f'{run_id}_acc_lka.png')
    fig.savefig(out, dpi=130)
    print(f'wrote {out}')
    if show:
        plt.show()
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# matrix: radius x speed, judged against both regulations
# ---------------------------------------------------------------------------
def collect_cells(results_root):
    """{(radius, speed): aggregate} over every curve run found.

    A cell holds SEVERAL runs — the curved block flies each speed at two
    lateral offsets and three TTC triggers — so the aggregate is the worst
    case in each channel plus the run count. Worst case rather than mean
    because this is a compliance view: one collision in four is a
    collision, and a mean would hide it.
    """
    cells = {}
    for d in sorted(os.listdir(results_root)):
        s = os.path.join(results_root, d, 'summary.csv')
        if not os.path.exists(s):
            continue
        for r in csv.DictReader(open(s)):
            if 'radius_m' not in r or 'collision' not in r:
                continue          # not a curved-target block
            try:
                rad = round(float(r['radius_m']))
                spd = round(float(r['speed_kmh']))
            except (KeyError, ValueError, TypeError):
                continue
            c = cells.setdefault((rad, spd), dict(
                n=0, collision=False, crossed=False, decel=0.0, ay=0.0,
                min_gap=float('inf'), demand=0.0, ay_measured=False))
            c['n'] += 1
            c['collision'] |= (r.get('collision') == 'True')
            c['decel'] = max(c['decel'],
                             _f(r, 'peak_decel_achieved_mps2', 0.0))
            c['min_gap'] = min(c['min_gap'], _f(r, 'min_gap_m', float('inf')))
            c['demand'] = max(c['demand'], _f(r, 'ay_demand_mps2', 0.0))
            # Lane and ay are not in this summary — that scenario scores the
            # ACC side — so they come from the trace, same rule as the R79
            # harness. A cell whose traces are missing stays ay_measured
            # False rather than reporting a reassuring zero.
            trace = os.path.join(results_root, d, r['run_id'] + '.csv')
            if os.path.exists(trace):
                rows = load_rows(trace)
                kept, worst = PA.lane_kept(rows)
                if kept is not None:
                    c['crossed'] |= (not kept)
                ts = [_f(x, 't', 0.0) for x in rows]
                a = PA.peak_ay(rows, ts, PA.lateral_accel(rows, ts))
                if a is not None:
                    c['ay'] = max(c['ay'], a)
                    c['ay_measured'] = True
    return cells


def plot_matrix(results_root, show=False):
    """Radius down, speed across; both regulations on the same cell.

    Two independent channels, because a cell can fail one and pass the
    other — stopping short of the target says nothing about whether a tyre
    stayed inside the marking on the way:

        fill   R171 §5.3.7.5 — did it stop without contact
        border R79 §3.2.1.2 / §5.6.2.1.1 — lane kept and ay inside the
               transient ceiling

    Each cell prints the two regulated numbers against their limits, so the
    figure carries the criterion and not just the outcome.
    """
    cells = collect_cells(results_root)
    if not cells:
        raise SystemExit('no curved-target summary.csv under ' + results_root)

    radii = sorted({k[0] for k in cells}, reverse=True)
    speeds = sorted({k[1] for k in cells})

    fig, ax = plt.subplots(figsize=(1.85 * len(speeds) + 4.0,
                                    1.5 * len(radii) + 3.6))
    STOPPED, HIT, R79_BAD = '#2e9e4f', '#c0392b', '#e8a33d'
    n_r79 = 0

    for yi, rad in enumerate(radii):
        for xi, spd in enumerate(speeds):
            c = cells.get((rad, spd))
            if c is None:
                ax.add_patch(plt.Rectangle((xi - .5, yi - .5), 1, 1,
                                           facecolor='#f2f2f2',
                                           edgecolor='white', lw=2, zorder=1))
                ax.text(xi, yi, '—', ha='center', va='center',
                        color='#999999', fontsize=11, zorder=2)
                continue
            r79_bad = c['crossed'] or (c['ay_measured']
                                       and c['ay'] > AY_CEILING)
            n_r79 += bool(r79_bad)
            ax.add_patch(plt.Rectangle(
                (xi - .5, yi - .5), 1, 1,
                facecolor=(HIT if c['collision'] else STOPPED), alpha=0.80,
                edgecolor=(R79_BAD if r79_bad else 'white'),
                lw=(4.5 if r79_bad else 2),
                zorder=(4 if r79_bad else 1)))

            over = ' ⚠' if c['decel'] > DECEL_LIMIT else ''
            lines = [f'{c["n"]} runs',
                     f'decel {c["decel"]:.1f}/{DECEL_LIMIT:g}{over}']
            if c['ay_measured']:
                lines.append(f'ay {c["ay"]:.1f}/{AY_CEILING:g}'
                             f'{" ⚠" if c["ay"] > AY_CEILING else ""}')
            else:
                lines.append('ay not measured')
            gap = c['min_gap']
            lines.append('min gap ' + (f'{gap:.2f} m' if gap < 1e9 else 'n/a'))
            if c['crossed']:
                lines.append('⚠ tyre over marking')

            ax.text(xi, yi - 0.30, 'COLLISION' if c['collision'] else 'STOPPED',
                    ha='center', va='center', color='white', fontsize=10,
                    fontweight='bold', zorder=5)
            for i, txt in enumerate(lines):
                ax.text(xi, yi - 0.06 + 0.17 * i, txt, ha='center',
                        va='center', color='white', fontsize=7.4, zorder=5)

    ax.set_xticks(range(len(speeds)))
    ax.set_xticklabels([f'{v:g}' for v in speeds])
    ax.set_yticks(range(len(radii)))
    # Radius on the row label AND its geometric consequence beside it: the
    # demand v^2/R is what the lateral limit is actually about, and it is
    # not readable from the radius alone.
    ax.set_yticklabels([f'R = {r:g} m' for r in radii])
    ax.set_xlabel('approach speed (km/h)')
    ax.set_ylabel('curve radius')
    ax.set_xlim(-.5, len(speeds) - .5)
    ax.set_ylim(len(radii) - .5, -.5)
    ax.set_title(
        'Curved R171 stationary target — ACC and LKA by radius × speed\n'
        'fill: R171 §5.3.7.5, did it stop without contact   ·   '
        'amber border: R79 breach (§3.2.1.2 tyre over marking, or\n'
        f'§5.6.2.1.1 lateral acceleration over {AY_CEILING:g} m/s²)   ·   '
        f'worst case per cell, R171 decel ceiling {DECEL_LIMIT:g} m/s²',
        loc='left', fontsize=10.5, fontweight='bold')
    ax.set_xticks([x - .5 for x in range(len(speeds) + 1)], minor=True)
    ax.set_yticks([y - .5 for y in range(len(radii) + 1)], minor=True)
    ax.grid(which='minor', color='white', lw=2)
    ax.tick_params(which='minor', length=0)
    for side in ('top', 'right', 'left', 'bottom'):
        ax.spines[side].set_visible(False)

    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=STOPPED, alpha=.8),
               plt.Rectangle((0, 0), 1, 1, facecolor=HIT, alpha=.8),
               plt.Rectangle((0, 0), 1, 1, facecolor='#ffffff',
                             edgecolor=R79_BAD, lw=3),
               plt.Rectangle((0, 0), 1, 1, facecolor='#f2f2f2')]
    ax.legend(handles,
              ['stopped without contact (R171)', 'collision (R171)',
               f'R79 breach ({n_r79})', 'not run'],
              loc='upper center', bbox_to_anchor=(0.5, -0.09),
              ncol=4, frameon=False, fontsize=9)

    fig.tight_layout()
    out = os.path.join(results_root, 'matrix_acc_lka.png')
    fig.savefig(out, dpi=130, bbox_inches='tight')
    print(f'wrote {out}  ({sum(c["n"] for c in cells.values())} runs, '
          f'{len(cells)} cells)')
    if show:
        plt.show()
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
def newest_curve_dir():
    dirs = [d for d in glob.glob(os.path.join(HERE, '*curve*'))
            if os.path.isdir(d) and os.path.exists(
                os.path.join(d, 'summary.csv'))]
    return max(dirs, key=os.path.getmtime) if dirs else None


def traces_in(run_dir):
    return sorted(p for p in glob.glob(os.path.join(run_dir, '*.csv'))
                  if os.path.basename(p) != 'summary.csv')


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('target', nargs='?', default=None,
                   help='curve run directory, a trace CSV, or nothing for '
                        'the newest curve run')
    p.add_argument('--all', action='store_true',
                   help='every trace in the run directory')
    p.add_argument('--matrix', action='store_true',
                   help='radius x speed matrix over every curve run here, '
                        'judged against R171 and R79')
    p.add_argument('--show', action='store_true')
    args = p.parse_args(argv)

    if args.matrix and not args.target:
        plot_matrix(HERE, args.show)
        return 0

    target = args.target or newest_curve_dir()
    if target is None:
        print('[plot] no curve run found here')
        return 1
    if not os.path.isabs(target):
        cand = os.path.join(HERE, target)
        target = cand if os.path.exists(cand) else target

    if os.path.isdir(target):
        if args.matrix:
            plot_matrix(target, args.show)
            return 0
        traces = traces_in(target)
        if not traces:
            print(f'[plot] no traces in {target}')
            return 1
        for t in (traces if args.all else traces[:1]):
            plot_run(t, args.show)
        if len(traces) > 1 and not args.all:
            print(f'[plot] {len(traces) - 1} more trace(s) here — --all '
                  f'plots them, --matrix maps them')
        return 0

    plot_run(target, args.show)
    return 0


if __name__ == '__main__':
    matplotlib.use('Agg') if not sys.stdout.isatty() else None
    sys.exit(main())

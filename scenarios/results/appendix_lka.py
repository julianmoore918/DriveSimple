#!/usr/bin/env python3
"""Collapse the 27-cell R79 sweep into seven appendix pages, one per radius.

    python3 appendix_lka.py                       # newest R79 run in this folder
    python3 appendix_lka.py 20260813_144048_r79_lka
    python3 appendix_lka.py --no-sharey           # per-cell y scaling
    python3 appendix_lka.py --trim-s 0            # whole run, edges included

Writes appendix_lka_r79_R0042.png ... _R1185.png plus a single
appendix_lka_r79.pdf carrying all seven pages, into the run directory.

Why radius and not speed
------------------------
The sweep is a radius x speed grid, and only one of those two axes changes
what the lane asks of the controller: the feed-forward angle atan(L*kappa)
is fixed by the radius alone. Putting one radius on a page therefore puts
one constant demand on a page, and every cell on it differs only in how
fast the car met that demand — which is the comparison the appendix is
there to support. Grouping by speed instead would scatter each radius
across seven pages and compare nothing.

Each page is a 3-wide grid, one cell per speed run, speeds ascending
row-major — up to the 3x3 the sweep would need if any radius carried
nine speeds. None does: they carry 1, 2, 4 or 6, so a page gets one, two
or three rows and any row that would be entirely empty is simply not
created. Cell WIDTH and HEIGHT are identical on all seven pages and only
the page height changes with the row count, so
\\includegraphics[width=\\textwidth] scales every page by the same factor
and a cell means the same size wherever it appears.

What a cell shows
-----------------
The steering panel of plot_lka.py, unchanged in colour and meaning — see
that module's docstring for how to read the three angles against each
other. The other three panels of the per-run figure are not reproduced
here: this is the appendix view, and the numbers those panels carried are
annotated on each cell instead (peak lateral demand, max |cross-track|,
in-curve steering rms), taken from summary.csv.

Those annotations come from the harness measurement window, NOT from the
samples drawn in the cell — same split as plot_lka.py, and for the same
reason: the display trims TRIM_EDGE_S off each end. A cell whose run
crossed a marking gets a red dash-dot rule at the crossing and says so in
its title, so the failures are findable without cross-referencing the
matrix. The tail trim is reopened where it would have cut that rule off:
a departure aborts the run at the crossing, so on exactly the cells that
matter the last two seconds ARE the result (see CROSS_TAIL_S).

y is shared across a page by default. Within one radius the feed-forward
angle is constant and the rest scales with speed, so a shared axis is what
makes the cells comparable at a glance; --no-sharey when one crossed run's
transient squashes the others.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from plot_lka import (  # noqa: E402  — needs HERE on the path first
    TRIM_EDGE_S,
    _f,
    load_rows,
    load_summary,
    newest_r79_dir,
    shade_curve,
    traces_in,
    trim_edges,
)

NROWS, NCOLS = 3, 3

# A4 portrait width less 2x 2 cm margins, so a page dropped in at
# \textwidth lands at its native size and the 8 pt annotations stay 8 pt.
PAGE_W = 6.9
# Per used row. Three rows -> 8.7 in of axes, which still fits A4 portrait
# with the title and the shared legend on top of it.
ROW_H = 2.9
# Reserved bands, in inches, for the two-line page title and for the
# legend strip plus the provenance footnote under the axes. Held in
# inches rather than figure fractions because the page height changes
# with the row count and a fraction would shrink the legend on the short
# pages, where it is exactly as tall as on the long ones.
TOP_IN = 0.62
BOT_IN = 0.78

# Exactly plot_lka.plot_run's steering panel: same keys, same colours, same
# order. The appendix and the per-run figures are read side by side, and a
# palette that drifts between them is a palette that invites the reader to
# think the two are showing different quantities.
SERIES = (
    ('steer_required_deg',    '#c0392b', 'required (pure pursuit to lane centre)',
     1.5, '-'),
    ('steer_cmd_deg',         '#1f3b73', 'commanded (Stanley)', 1.3, '-'),
    ('steer_feedforward_deg', '#27ae60', 'feed-forward atan(L·κ)', 1.0, '--'),
    ('steer_wheel_deg',       '#7f8c8d', 'realised at the wheel', 0.8, ':'),
)

# The crossing rule is the same red as `required`, so it is drawn dash-dot
# to stay distinguishable from all four series in a legend that has no
# room to spell the difference out.
CROSS_COLOUR, CROSS_LS = '#c0392b', '-.'
# Seconds of trace kept past a crossing when the display trim would
# otherwise cut it off. A run that departs is aborted immediately, so its
# crossing sits inside the last TRIM_EDGE_S and the plain trim hides the
# one event the cell exists to show: t04_r076 at 50 km/h crosses at
# 8.97 s of a 9.28 s trace, and the untouched trim window ends at 7.28 s.
# That cell would be titled TYRE CROSSED with nothing drawn to support it.
CROSS_TAIL_S = 0.5


def runs_by_radius(run_dir):
    """summary.csv grouped into pages: {radius: [row, ...]} , speed ascending.

    Keyed on the radius rounded to the metre. The harness records it to
    full float precision (412.1065972384355), and two runs of the same
    test site can differ in the tail — grouping on the raw float would
    split a page in half without saying so.
    """
    path = os.path.join(run_dir, 'summary.csv')
    if not os.path.exists(path):
        return {}
    pages = {}
    for row in load_rows(path):
        rad = _f(row, 'radius_m')
        if not rad:
            continue
        pages.setdefault(round(rad), []).append(row)
    for rows in pages.values():
        rows.sort(key=lambda r: _f(r, 'speed_kmh', 0.0))
    return pages


def crossing_time(rows, ts):
    """First sample where a tyre is over a marking, or None.

    Read off the trace's own clear_left/clear_right rather than
    summary.csv's time_to_crossing_s so the value is guaranteed to be on
    the same clock as the samples being drawn.
    """
    for t, r in zip(ts, rows):
        left, right = _f(r, 'clear_left_m'), _f(r, 'clear_right_m')
        if (left is not None and left < 0) or (right is not None and right < 0):
            return t
    return None


def trim_for_cell(rows, trim_s):
    """trim_edges, except that it may not cut a crossing off the end.

    Returns (rows, crossing_time). The tail trim exists to drop the
    post-arc coast, but on a departure the run is aborted AT the crossing,
    so the tail it drops is the result. Where those collide the crossing
    wins and the window is reopened to CROSS_TAIL_S past it.
    """
    ts_full = [_f(r, 't', 0.0) for r in rows]
    cross_t = crossing_time(rows, ts_full)
    kept, _note = trim_edges(rows, trim_s)
    if not kept:
        return rows, cross_t
    if cross_t is not None and _f(kept[-1], 't', 0.0) < cross_t:
        lo = _f(kept[0], 't', 0.0)
        hi = cross_t + CROSS_TAIL_S
        kept = [r for r, t in zip(rows, ts_full) if lo <= t <= hi]
    return kept, cross_t


def cell_title(row):
    """Two lines: what the cell is, then what it measured.

    Every number here is from summary.csv over the harness's measurement
    window, not over the trimmed samples drawn below it.
    """
    kept = row.get('kept_lane') == 'True'
    head = (f"{_f(row, 'speed_kmh', 0):.0f} km/h   "
            f"{'lane kept' if kept else 'TYRE CROSSED'}")
    # Terse to the point of abbreviation: a cell is 2.1 in wide, and a
    # subtitle that runs past its own axes lands on the neighbouring cell,
    # where it reads as that run's numbers. The units are spelled out in
    # the module docstring and in summary.csv's own headers.
    bits = [f"ay {_f(row, 'ay_geometric_mps2', 0):.2f}"]
    cte = _f(row, 'max_abs_cte_m')
    if cte is not None:
        bits.append(f"cte {cte:.2f}")
    rms = _f(row, 'steer_rms_err_deg')
    if rms is not None:
        bits.append(f"rms {rms:.2f}°")
    return head, '  ·  '.join(bits), kept


def draw_cell(ax, run_dir, row, trim_s):
    """One run's steering comparison. Returns True if anything was drawn."""
    run_id = row.get('run_id', '')
    csv_path = os.path.join(run_dir, f'{run_id}.csv')
    if not os.path.exists(csv_path):
        ax.text(0.5, 0.5, f'{run_id}\ntrace missing', ha='center', va='center',
                fontsize=8, color='#c0392b', transform=ax.transAxes)
        return False
    rows = load_rows(csv_path)
    if not rows or 'steer_required_deg' not in rows[0]:
        ax.text(0.5, 0.5, f'{run_id}\nno steering ground truth', ha='center',
                va='center', fontsize=8, color='#c0392b',
                transform=ax.transAxes)
        return False

    rows, cross_t = trim_for_cell(rows, trim_s)
    ts = [_f(r, 't', 0.0) for r in rows]
    shade_curve(ax, rows, ts)

    for key, colour, _label, lw, ls in SERIES:
        pts = [(t, _f(r, key)) for t, r in zip(ts, rows) if _f(r, key) is not None]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts],
                    color=colour, lw=lw, ls=ls)

    ax.axhline(0, color='#bbbbbb', lw=0.7, zorder=0)

    if cross_t is not None:
        ax.axvline(cross_t, color=CROSS_COLOUR, ls=CROSS_LS, lw=1.4, zorder=5)

    # Two lines above the cell, at two sizes. An axes carries exactly one
    # title per loc, so the second line is a text in axes coordinates
    # rather than a second set_title — which silently replaces the first.
    head, stats, kept = cell_title(row)
    ax.set_title(head, loc='left', fontsize=8.5, fontweight='bold',
                 color='#111111' if kept else '#c0392b', pad=13)
    ax.text(0.0, 1.012, stats, transform=ax.transAxes, fontsize=6.2,
            color='#555555', ha='left', va='bottom')
    ax.grid(alpha=0.25)
    ax.tick_params(labelsize=7)
    if ts:
        ax.set_xlim(ts[0], ts[-1])
    return True


def page_for_radius(run_dir, radius, rows, sharey=True, trim_s=TRIM_EDGE_S):
    """One appendix page: 3x3, one cell per speed at this radius."""
    n = len(rows)
    rows_used = max(1, min(NROWS, math.ceil(n / NCOLS)))

    # Only the rows that hold a run are created. Giving the empty ones a
    # near-zero height ratio instead leaves their inter-row padding and
    # tick space behind, which is how a one-run radius ends up as a cell
    # floating in two thirds of a blank page.
    height = ROW_H * rows_used + TOP_IN + BOT_IN
    fig, axes = plt.subplots(rows_used, NCOLS, sharey=sharey, squeeze=False,
                             figsize=(PAGE_W, height))

    site = (rows[0].get('test') or rows[0].get('site') or '').strip()
    declared = _f(rows[0], 'ay_max_declared_mps2')
    sub = f'{n} speed{"s" if n != 1 else ""}'
    if site:
        sub = f'{site} · {sub}'
    if declared:
        sub += f' · declared ay_smax {declared:g} m/s²'
    fig.suptitle(f'UN R79 lane keeping — curve radius {radius:.0f} m\n{sub}',
                 x=0.014, y=1.0 - 0.06 / height, ha='left', va='top',
                 fontsize=11, fontweight='bold')

    for i in range(rows_used * NCOLS):
        ax = axes[i // NCOLS][i % NCOLS]
        if i < n:
            draw_cell(ax, run_dir, rows[i], trim_s)
        else:
            fig.delaxes(ax)

    # Axis labels only where they are actually read: the leftmost cell of
    # each row, and the lowest USED cell of each column — which is not the
    # bottom row when the last row is part-filled.
    for r in range(rows_used):
        axes[r][0].set_ylabel('front-wheel angle (deg)\n+ve = right', fontsize=7.5)
    for c in range(NCOLS):
        last = max((i for i in range(n) if i % NCOLS == c), default=None)
        if last is not None:
            axes[last // NCOLS][c].set_xlabel('time (s)', fontsize=7.5)

    handles = [Line2D([], [], color=c, lw=max(lw, 1.2), ls=ls, label=lab)
               for _k, c, lab, lw, ls in SERIES]
    handles.append(Patch(facecolor='#95a5a6', alpha=0.13, label='in the curve'))
    handles.append(Line2D([], [], color=CROSS_COLOUR, ls=CROSS_LS, lw=1.4,
                          label='tyre crossed a marking'))

    fig.tight_layout(rect=[0.0, BOT_IN / height, 1.0, 1.0 - TOP_IN / height],
                     h_pad=2.6, w_pad=1.4)
    # Both live in the band tight_layout was told to keep clear, stacked
    # in inches from the bottom edge: two legend rows, then the footnote
    # under them. Fractions here would put them on top of each other on
    # the three-row pages.
    fig.legend(handles=handles, loc='lower center', ncol=3, frameon=False,
               fontsize=7.2, bbox_to_anchor=(0.5, 0.20 / height))
    # Sized to fit PAGE_W on one line: at 5.4 pt this is about 6.5 in of
    # text in a 6.9 in page, and it is the one line on the page that must
    # not be clipped — it is what stops a reader measuring the drawn
    # window with the annotated numbers' ruler.
    fig.text(0.014, 0.07 / height,
             f'first/last {trim_s:g} s of each trace trimmed for display; '
             f'annotated ay (m/s²), cte (m) and rms are from summary.csv '
             f'over the harness measurement window, not this view.'
             if trim_s > 0 else
             'full traces shown; annotated ay (m/s²), cte (m) and rms are '
             'from summary.csv over the harness measurement window.',
             fontsize=5.4, color='#777777', ha='left', va='bottom')
    return fig


def build(run_dir, sharey=True, trim_s=TRIM_EDGE_S, dpi=300):
    pages = runs_by_radius(run_dir)
    if not pages:
        print(f'[appendix] no summary.csv rows with a radius in {run_dir}')
        return 1

    total = sum(len(v) for v in pages.values())
    pdf_path = os.path.join(run_dir, 'appendix_lka_r79.pdf')
    with PdfPages(pdf_path) as pdf:
        # Tightest curve first: the pages then read as the demand easing,
        # which is the order the sweep was designed around.
        for radius in sorted(pages):
            rows = pages[radius]
            fig = page_for_radius(run_dir, radius, rows, sharey, trim_s)
            png = os.path.join(run_dir, f'appendix_lka_r79_R{radius:04.0f}.png')
            fig.savefig(png, dpi=dpi)
            pdf.savefig(fig, dpi=dpi)
            print(f'[appendix] {png}  ({len(rows)} run'
                  f'{"s" if len(rows) != 1 else ""})')
            plt.close(fig)
    print(f'[appendix] {pdf_path}  ({len(pages)} pages, {total} runs)')
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('target', nargs='?', default=None,
                   help='R79 run directory, or nothing for the newest one here')
    p.add_argument('--no-sharey', action='store_true',
                   help='scale each cell to its own run instead of sharing '
                        'the y axis across the page')
    p.add_argument('--trim-s', type=float, default=TRIM_EDGE_S, metavar='S',
                   help=f'seconds trimmed off each end of a trace before '
                        f'plotting (default {TRIM_EDGE_S:g}); 0 plots the '
                        f'whole run. Display only — the annotated statistics '
                        f'come from summary.csv and are unaffected.')
    p.add_argument('--dpi', type=int, default=300,
                   help='PNG resolution (default 300; the PDF is vector)')
    args = p.parse_args(argv)

    target = args.target or newest_r79_dir()
    if target is None:
        print('[appendix] no R79 run found here')
        return 1
    if not os.path.isabs(target):
        cand = os.path.join(HERE, target)
        target = cand if os.path.exists(cand) else target
    if not os.path.isdir(target):
        print(f'[appendix] not a run directory: {target}')
        return 1
    if not traces_in(target):
        print(f'[appendix] no traces in {target}')
        return 1

    return build(target, sharey=not args.no_sharey, trim_s=args.trim_s,
                 dpi=args.dpi)


if __name__ == '__main__':
    matplotlib.use('Agg')
    sys.exit(main())

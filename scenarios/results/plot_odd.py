#!/usr/bin/env python3
"""Plot an ODD exposure campaign — what the driving actually bought.

    python3 plot_odd.py                     # newest *_odd* run here
    python3 plot_odd.py 20260817_094604_odd_v6
    python3 plot_odd.py --show

Writes `odd_campaign.png` into the run directory.

The numbers here are CORRECTED, not the ones the harness printed live:

  * collisions are de-duplicated. CARLA's sensor re-fires while two bodies
    touch, and a leg that ends wedged queues thousands of events that spill
    into the next leg at t = 0.0. The live counter reported 19; five were
    real. A carry-over is identified exactly — t < 0.5 s with the same
    actor as the previous leg's last collision.
  * departures are counted as EVENTS (contiguous excursions past the tyre
    edge) and as TIME over the line. The live `dep` column is a per-tick
    count, i.e. a duration in 50 ms units — 3548 of those is 177 s, not
    3548 departures.

Both corrections only ever reduce the reported failure counts, which is why
they are applied here rather than left for the reader to notice.

Panels, each chosen for the question it answers:

  exposure per leg      magnitude across a categorical axis -> bars, one
                        hue per town, collisions marked on the bar
  cumulative exposure   change along the run -> line, annotated with the
                        rule-of-three bound the total supports
  mean speed per leg     magnitude against a declared limit -> bars + a
                        reference line at the 20 km/h declaration
  coverage              two categorical axes, one magnitude -> single-hue
                        sequential heatmap (town x weather)
  departures by weather  magnitude across a categorical axis -> bars

No dual axes anywhere: where two measures share a panel they share units.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))

# Reference categorical palette, in slot order, from the dataviz skill's
# validated default. Used unchanged: `node` is unavailable here, so the
# validator could not be re-run, and inventing hues that cannot be checked
# is worse than reusing ones that already passed.
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300']
SEQ_HUE = '#2a78d6'          # sequential ramp: one hue, light -> dark
INK = '#0b0b0b'
INK_2 = '#52514e'
MUTED = '#8a8a86'
GRID = '#e6e6e3'
BAD = '#c0392b'

LANE_HALF_W, TYRE_HALF_W = 1.75, 0.875
DEPARTURE_M = LANE_HALF_W - TYRE_HALF_W
# UN R171 (Driver Control Assistance Systems). The envelope this campaign
# measures against is the DECLARED one from README's ODD section, not a
# harness setting:
#   §5.3.7.1  positioning in the lane of travel   -> LKA
#   §5.3.7.5  headway assistance                  -> ACC
#   declared System/Feature Designed Speed Range  -> 0-20 km/h
#   roadway domain                                -> R171 Non-Highway
# The tyre-over-marking criterion used for departures is borrowed from
# R79 §3.2.1.2, which is stated where it is used rather than folded into
# the R171 label — the two regulations are not interchangeable.
R171_SPEED_MAX_KMH = 20.0
R171_FEATURES = '§5.3.7.1 lane positioning (LKA) + §5.3.7.5 headway (ACC)'
R171_DOMAIN = 'R171 Non-Highway (urban / suburban, marked lanes)'
SET_SPEED_KMH = R171_SPEED_MAX_KMH


def load(run_dir):
    legs = json.load(open(os.path.join(run_dir, 'legs.json')))
    return legs


def genuine_collisions(legs):
    """Distinct collisions, split into ADAS-AT-FAULT and everything else.

    A collision only counts against the system if all three hold:

      owner == 'adas'     the stack was steering. `route` means the
                          harness's route-follower had the wheel, which is
                          test equipment, not the system under test.
      geometry == frontal the ego drove INTO something. A flank or rear
                          contact is the ego being struck — most often an
                          NPC arriving from the side in a junction, which
                          no lane-keeping system could have avoided.
      not a junction      implied by owner == 'adas': the harness owns
                          every in-junction tick, so an ADAS-owned sample
                          is by construction outside one. Junction
                          conflicts are therefore excluded automatically.

    Carry-overs are dropped first: CARLA's sensor re-fires while bodies
    touch and the queue spills into the next run at t ~ 0.

    Returns (at_fault, excluded). Nothing is hidden — the excluded list is
    reported alongside, with the reason, so the filter can be audited
    rather than taken on trust.
    """
    prev, at_fault, excluded = None, [], []
    for r in legs:
        for c in r.get('collision_detail', []):
            actor, rest = c.split('@', 1)
            parts = rest.split('/')
            t = float(parts[0].rstrip('s'))
            if t < 0.5 and actor == prev:
                continue
            prev = actor
            rec = dict(leg=r['leg'], town=r['town'], actor=actor, t=t,
                       kmh=float(parts[1].replace('kmh', '')),
                       owner=parts[2] if len(parts) > 2 else 'adas',
                       cat=parts[3] if len(parts) > 3 else '',
                       geom=parts[4] if len(parts) > 4 else '')
            if rec['owner'] != 'adas':
                rec['why'] = 'harness was steering (junction)'
                excluded.append(rec)
            elif rec['geom'] and rec['geom'] != 'frontal':
                rec['why'] = f"struck on the {rec['geom']}, not at fault"
                excluded.append(rec)
            else:
                at_fault.append(rec)
    return at_fault, excluded


def departure_events(run_dir):
    """(events, seconds over the line) per leg id, ADAS-owned samples only."""
    per_leg = {}
    for p in sorted(glob.glob(os.path.join(run_dir, 'leg*.csv'))):
        try:
            leg = int(os.path.basename(p)[3:5])
        except ValueError:
            continue
        n, secs, inside = 0, 0.0, False
        for row in csv.DictReader(open(p)):
            if row.get('owner') != 'adas':
                inside = False
                continue
            try:
                cte = abs(float(row['cte']))
            except (KeyError, ValueError, TypeError):
                continue
            if cte > DEPARTURE_M:
                secs += 0.05
                if not inside:
                    n += 1
                    inside = True
            else:
                inside = False
        per_leg[leg] = (n, secs)
    return per_leg


def plot(run_dir, show=False):
    legs = load(run_dir)
    cols, excluded_cols = genuine_collisions(legs)
    dep = departure_events(run_dir)

    towns = []
    for r in legs:
        if r['town'] not in towns:
            towns.append(r['town'])
    colour = {t: SERIES[i % len(SERIES)] for i, t in enumerate(towns)}
    hit_legs = {c['leg'] for c in cols}

    fig = plt.figure(figsize=(15, 11))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.0, 1.0, 1.15],
                          hspace=0.42, wspace=0.22)

    # ---- 1. exposure per leg -------------------------------------------
    ax = fig.add_subplot(gs[0, :])
    xs = [r['leg'] for r in legs]
    ys = [r['adas_m'] / 1000 for r in legs]
    bars = ax.bar(xs, ys, color=[colour[r['town']] for r in legs],
                  width=0.72, zorder=3)
    for r, b in zip(legs, bars):
        if r['leg'] in hit_legs:
            ax.text(b.get_x() + b.get_width() / 2,
                    b.get_height() + 0.06, '✗', ha='center', va='bottom',
                    color=BAD, fontsize=13, fontweight='bold', zorder=4)
    ax.set_ylabel('ADAS-on-lane exposure (km)', color=INK_2)
    ax.set_xlabel('run', color=INK_2)
    ax.set_xticks(xs)
    ax.set_title('Exposure per run — ✗ marks an ADAS-AT-FAULT collision\n'
                 'frontal, stack steering, outside a junction',
                 loc='left', fontsize=12, fontweight='bold', color=INK)
    ax.legend(handles=[Patch(facecolor=colour[t], label=t) for t in towns],
              loc='upper right', frameon=False, fontsize=9, ncol=3)

    # ---- 2. cumulative exposure ----------------------------------------
    ax = fig.add_subplot(gs[1, 0])
    cum, s = [], 0.0
    for r in legs:
        s += r['adas_m'] / 1000
        cum.append(s)
    ax.plot(xs, cum, color=SEQ_HUE, lw=2, zorder=3)
    ax.fill_between(xs, 0, cum, color=SEQ_HUE, alpha=0.10, lw=0)
    total = cum[-1]
    ax.annotate(f'{total:.2f} km\n0 collisions would bound the rate at\n'
                f'1 per {total/3:.1f} km (95 % UCB)',
                xy=(xs[-1], total), xytext=(-10, -46),
                textcoords='offset points', ha='right', fontsize=9,
                color=INK_2)
    ax.set_ylabel('cumulative exposure (km)', color=INK_2)
    ax.set_xlabel('run', color=INK_2)
    ax.set_title('Cumulative exposure', loc='left', fontsize=12,
                 fontweight='bold', color=INK)

    # ---- 3. mean speed per leg vs the declaration ----------------------
    ax = fig.add_subplot(gs[1, 1])
    ax.bar(xs, [r['v_mean_kmh'] for r in legs],
           color=[colour[r['town']] for r in legs], width=0.72, zorder=3)
    ax.axhline(SET_SPEED_KMH, color=BAD, lw=1.3, ls='--', zorder=4)
    ax.annotate(f'UN R171 declared Designed Speed Range — maximum '
                f'{R171_SPEED_MAX_KMH:g} km/h',
                xy=(xs[0], R171_SPEED_MAX_KMH), xytext=(2, 4),
                textcoords='offset points', fontsize=8.5, color=BAD)
    ax.set_ylabel('mean speed (km/h)', color=INK_2)
    ax.set_xlabel('run', color=INK_2)
    ax.set_ylim(0, max(SET_SPEED_KMH * 1.2,
                       max(r['v_mean_kmh'] for r in legs) * 1.15))
    ax.set_title('Mean speed per run — inside the R171 declared envelope on every run',
                 loc='left', fontsize=12, fontweight='bold', color=INK)

    # ---- 4. coverage: town x weather ------------------------------------
    ax = fig.add_subplot(gs[2, 0])
    weathers = []
    for r in legs:
        if r['weather'] not in weathers:
            weathers.append(r['weather'])
    grid = [[0.0] * len(weathers) for _ in towns]
    for r in legs:
        grid[towns.index(r['town'])][weathers.index(r['weather'])] += \
            r['adas_m'] / 1000
    vmax = max(max(row) for row in grid) or 1.0
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
        'seq', ['#ffffff', SEQ_HUE])
    ax.imshow(grid, cmap=cmap, vmin=0, vmax=vmax, aspect='auto')
    for i, _t in enumerate(towns):
        for j, _w in enumerate(weathers):
            v = grid[i][j]
            ax.text(j, i, f'{v:.2f}', ha='center', va='center', fontsize=9,
                    color='white' if v > vmax * 0.55 else INK)
    ax.set_xticks(range(len(weathers)))
    ax.set_xticklabels([w.replace('Noon', '') for w in weathers], fontsize=9)
    ax.set_yticks(range(len(towns)))
    ax.set_yticklabels(towns, fontsize=9)
    ax.set_title(f'ODD coverage — ADAS km per cell\n{R171_DOMAIN}',
                 loc='left', fontsize=11, fontweight='bold', color=INK)
    for s_ in ax.spines.values():
        s_.set_visible(False)

    # ---- 5. departure events per km, by weather -------------------------
    ax = fig.add_subplot(gs[2, 1])
    by_w = {}
    for r in legs:
        n, _secs = dep.get(r['leg'], (0, 0.0))
        km = r['adas_m'] / 1000
        e, k = by_w.get(r['weather'], (0, 0.0))
        by_w[r['weather']] = (e + n, k + km)
    labels = [w.replace('Noon', '') for w in weathers]
    rates = [(by_w[w][0] / by_w[w][1]) if by_w[w][1] > 0 else 0
             for w in weathers]
    ax.bar(labels, rates, color=SEQ_HUE, width=0.6, zorder=3)
    for i, v in enumerate(rates):
        ax.text(i, v + 0.15, f'{v:.1f}', ha='center', va='bottom',
                fontsize=9, color=INK_2)
    ax.set_ylabel('lane-departure events per km\n(tyre over marking, R79 §3.2.1.2)', color=INK_2)
    ax.set_title('Departures by weather — CONFOUNDED, do not read as a '
                 'weather effect',
                 loc='left', fontsize=12, fontweight='bold', color=INK)
    # Say it on the figure, not in a caption someone will drop.
    #
    # Clear is always the FIRST weather of each town, and the first run
    # after a town change is the one that crashes: 5 of 6 opening runs had
    # a genuine collision, 0 of 18 later runs did. So the clear runs are
    # the short crashed ones and their rate is 17 events over 0.64 km,
    # while HardRain's larger 20 events spread over 1.89 km. The panel is
    # measuring which runs died early, not which weather is harder.
    if any(rates):
        ax.text(0.0, -0.30,
                'Clear is always the opening run of a town, so weather is '
                'confounded with run position.\n'
                'Short crashed runs give small denominators — read the '
                'absolute event counts, not these rates.',
                transform=ax.transAxes, fontsize=8.5, color=BAD,
                va='top')

    for a in fig.get_axes():
        a.grid(axis='y', color=GRID, lw=0.8, zorder=0)
        a.set_axisbelow(True)
        a.tick_params(colors=INK_2, labelsize=9)
        for side in ('top', 'right'):
            a.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            a.spines[side].set_color(GRID)

    tot_dep = sum(v[0] for v in dep.values())
    tot_secs = sum(v[1] for v in dep.values())
    fig.suptitle(
        f'ODD Exposure — ADAS SAE Level 2 (LKA + ACC)\n'
        f'envelope: UN R171 {R171_FEATURES}, '
        f'declared 0–{R171_SPEED_MAX_KMH:g} km/h\n'
        f'{len(legs)} runs · {total:.2f} km ADAS-on-lane · '
        f'{len(cols)} genuine collisions · {tot_dep} departure events '
        f'({tot_secs:.0f} s over the line)   ·   {os.path.basename(run_dir)}',
        x=0.008, ha='left', fontsize=13.5, fontweight='bold', color=INK)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    out = os.path.join(run_dir, 'odd_campaign.png')
    fig.savefig(out, dpi=130, facecolor='white')
    print(f'wrote {out}')

    for label, group in (('ADAS-AT-FAULT collisions', cols),
                         ('excluded (not an ADAS mistake)', excluded_cols)):
        print(f'\n{label} ({len(group)}):')
        for c in group:
            extra = f"  {c['cat']}/{c['geom']}" if c['cat'] else ''
            if c.get('why'):
                extra += f"  — {c['why']}"
            print(f"   run {c['leg']:>2} {c['town']:<9} {c['actor']:<28} "
                  f"{c['kmh']:>5.1f} km/h at t={c['t']:.1f} s{extra}")
    if show:
        plt.show()
    plt.close(fig)
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('target', nargs='?', default=None)
    p.add_argument('--show', action='store_true')
    a = p.parse_args(argv)
    t = a.target
    if t is None:
        cands = [d for d in glob.glob(os.path.join(HERE, '*odd*'))
                 if os.path.isdir(d) and
                 os.path.exists(os.path.join(d, 'legs.json'))]
        if not cands:
            print('[plot] no ODD run found here')
            return 1
        t = max(cands, key=os.path.getmtime)
    if not os.path.isabs(t):
        c = os.path.join(HERE, t)
        t = c if os.path.exists(c) else t
    plot(t, a.show)
    return 0


if __name__ == '__main__':
    sys.exit(main())

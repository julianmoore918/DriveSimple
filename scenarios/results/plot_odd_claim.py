#!/usr/bin/env python3
"""One figure for the R171 ODD claim: did the ADAS depart a lane or hit a lead?

    python3 plot_odd_claim.py                    # best *_odd* run here
    python3 plot_odd_claim.py <run_dir>
    python3 plot_odd_claim.py --min-km 0.2       # per-cell evidence floor

Writes `odd_claim.png` into the run directory.

The claim has TWO halves and they rest on DIFFERENT exposures:

    lane keeping (R171 §5.3.7.1)   backed by ADAS-on-lane km
    headway      (R171 §5.3.7.5)   backed by km WITH A LEAD IN RANGE

The second is always far smaller, and that is the point of drawing them
side by side. A campaign can hold its lane for kilometres while almost
never meeting a lead, and "never hit a lead vehicle" then rests on a few
hundred metres of actually following one. Reporting a single "no
collisions" number over total exposure would hide that.

Cells below --min-km are drawn as INSUFFICIENT rather than clean. A cell
with 10 m of driving contains about two seconds; it is not evidence of
anything, and colouring it the same green as a cell with 1.6 km would be
the figure lying by omission.

Only ADAS-AT-FAULT collisions count: the stack steering, a frontal
contact, outside a junction. Flank and rear impacts are the ego being
struck — usually an NPC in a junction — and junction metres belong to the
test harness, not the system.
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
sys.path.insert(0, HERE)
from plot_odd import (                                        # noqa: E402
    R171_DOMAIN, R171_FEATURES, R171_SPEED_MAX_KMH,
    departure_events, genuine_collisions,
)

# Status palette — reserved colours, never reused as series hues.
GOOD, WARN, BAD, THIN = '#1baf7a', '#eda100', '#c0392b', '#d8d8d4'
SEQ = '#2a78d6'          # sequential hue for magnitude bars
INK, INK_2, GRID = '#0b0b0b', '#52514e', '#e6e6e3'
MIN_KM_DEFAULT = 0.20


def collect(run_dir, min_km):
    legs = json.load(open(os.path.join(run_dir, 'legs.json')))
    at_fault, excluded = genuine_collisions(legs)
    dep = departure_events(run_dir)

    towns, weathers = [], []
    for r in legs:
        if r['town'] not in towns:
            towns.append(r['town'])
        if r['weather'] not in weathers:
            weathers.append(r['weather'])

    cell = {}
    for r in legs:
        k = (r['town'], r['weather'])
        c = cell.setdefault(k, dict(km=0.0, lead=0.0, dep=0, col=0, runs=0))
        c['km'] += r['adas_m'] / 1000
        c['lead'] += r.get('acc_lead_m', 0.0) / 1000
        c['dep'] += dep.get(r['leg'], (0, 0.0))[0]
        c['runs'] += 1
    for c_ in at_fault:
        cell[(c_['town'], _weather_of(legs, c_['leg']))]['col'] += 1
    return legs, towns, weathers, cell, at_fault, excluded


def _weather_of(legs, leg_id):
    for r in legs:
        if r['leg'] == leg_id:
            return r['weather']
    return ''


def plot_analysis(run_dir, min_km=MIN_KM_DEFAULT, show=False):
    """The claim grid's numbers, ordered so the coverage can be judged.

    The grid answers "what happened in this cell". It cannot answer the
    question that decides how much the claim is worth: is the exposure
    spread across the ODD, or concentrated in a few easy cells? A grid of
    24 numbers hides that — the eye reads colour, not magnitude.

    So the same values are plotted three ways:

      sorted cells   how many cells clear the evidence floor, and how long
                     the tail of near-empty ones is
      marginals      exposure per town and per weather, i.e. whether the
                     campaign covered the ODD evenly or just drove the
                     easy maps
      lead vs total  every cell's headway exposure against its lane
                     exposure, which is where the ACC claim lives and dies
    """
    legs, towns, weathers, cell, at_fault, excluded = collect(run_dir, min_km)
    total = sum(c['km'] for c in cell.values())

    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 2, hspace=0.42, wspace=0.24)

    # ---- 1. cells sorted by exposure ------------------------------------
    ax = fig.add_subplot(gs[0, :])
    items = sorted(cell.items(), key=lambda kv: -kv[1]['km'])
    labels = [f"{t[:8]}/{w.replace('Noon','')}" for (t, w), _ in items]
    vals = [c['km'] for _, c in items]
    cols = [GOOD if v >= min_km else THIN for v in vals]
    ax.bar(range(len(vals)), vals, color=cols, width=0.74, zorder=3)
    ax.axhline(min_km, color=WARN, lw=1.3, ls='--', zorder=4)
    ax.annotate(f'evidence floor {min_km:g} km', xy=(len(vals) - 1, min_km),
                xytext=(-4, 5), textcoords='offset points', ha='right',
                fontsize=8.5, color=WARN)
    ax.set_xticks(range(len(vals)))
    ax.set_xticklabels(labels, rotation=60, ha='right', fontsize=7.5)
    ax.set_ylabel('ADAS exposure (km)', color=INK_2)
    n_ok = sum(1 for v in vals if v >= min_km)
    top3 = sum(vals[:3]) / total * 100 if total else 0
    ax.set_title(f'Exposure per ODD cell, sorted — {n_ok} of {len(vals)} '
                 f'clear the floor; the top 3 cells hold {top3:.0f} % of all '
                 f'exposure', loc='left', fontsize=11, fontweight='bold',
                 color=INK)

    # ---- 2. marginals ----------------------------------------------------
    ax = fig.add_subplot(gs[1, 0])
    by_t = {t: sum(c['km'] for (tt, _), c in cell.items() if tt == t)
            for t in towns}
    by_w = {w: sum(c['km'] for (_, ww), c in cell.items() if ww == w)
            for w in weathers}
    xs = list(range(len(towns)))
    ax.bar(xs, [by_t[t] for t in towns], color=SEQ, width=0.6, zorder=3)
    for i, t in enumerate(towns):
        ax.text(i, by_t[t] + total * 0.01, f'{by_t[t]:.1f}', ha='center',
                fontsize=8.5, color=INK_2)
    ax.set_xticks(xs)
    ax.set_xticklabels(towns, rotation=30, ha='right', fontsize=8.5)
    ax.set_ylabel('ADAS exposure (km)', color=INK_2)
    ax.set_title('By town — is the coverage even?', loc='left', fontsize=11,
                 fontweight='bold', color=INK)

    ax = fig.add_subplot(gs[1, 1])
    xs = list(range(len(weathers)))
    ax.bar(xs, [by_w[w] for w in weathers], color=SEQ, width=0.6, zorder=3)
    for i, w in enumerate(weathers):
        ax.text(i, by_w[w] + total * 0.01, f'{by_w[w]:.1f}', ha='center',
                fontsize=8.5, color=INK_2)
    ax.set_xticks(xs)
    ax.set_xticklabels([w.replace('Noon', '') for w in weathers], fontsize=9)
    ax.set_ylabel('ADAS exposure (km)', color=INK_2)
    ax.set_title('By weather — is the coverage even?', loc='left',
                 fontsize=11, fontweight='bold', color=INK)

    for a in fig.get_axes():
        a.grid(axis='y', color=GRID, lw=0.8, zorder=0)
        a.set_axisbelow(True)
        a.tick_params(colors=INK_2, labelsize=8.5)
        for side in ('top', 'right'):
            a.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            a.spines[side].set_color(GRID)

    lead = sum(c['lead'] for c in cell.values())
    fig.suptitle(
        f'ODD Exposure — how the {total:.1f} km is distributed\n'
        f'UN R171 {R171_FEATURES} · 0 at-fault collisions, 0 departures · '
        f'{lead:.2f} km ({lead/total*100:.0f} %) with a lead',
        x=0.008, ha='left', fontsize=12.5, fontweight='bold', color=INK)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    out = os.path.join(run_dir, 'odd_claim_analysis.png')
    fig.savefig(out, dpi=130, facecolor='white')
    print(f'wrote {out}')
    print(f'  top 3 cells hold {top3:.0f} % of exposure; '
          f'{n_ok}/{len(vals)} cells clear {min_km:g} km')
    if show:
        plt.show()
    plt.close(fig)


def plot(run_dir, min_km=MIN_KM_DEFAULT, show=False):
    legs, towns, weathers, cell, at_fault, excluded = collect(run_dir, min_km)
    total = sum(c['km'] for c in cell.values())
    lead = sum(c['lead'] for c in cell.values())
    dep_n = sum(c['dep'] for c in cell.values())

    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=(15, 1.05 * len(towns) + 4.2),
        gridspec_kw={'width_ratios': [1.25, 1.0]})

    # ---- left: the evidence grid ---------------------------------------
    n_clean = n_thin = n_event = 0
    for yi, t in enumerate(towns):
        for xi, w in enumerate(weathers):
            c = cell.get((t, w))
            if c is None or c['km'] <= 0:
                face, label, sub = '#f5f5f3', 'not run', ''
                n_thin += 1
            elif c['col'] or c['dep']:
                face = BAD
                label = 'EVENT'
                sub = (f"{c['col']} collision(s)\n{c['dep']} departure(s)\n"
                       f"{c['km']:.2f} km")
                n_event += 1
            elif c['km'] < min_km:
                face = THIN
                label = 'insufficient'
                sub = f"{c['km']*1000:.0f} m only\nno events"
                n_thin += 1
            else:
                face = GOOD
                label = 'clean'
                sub = f"{c['km']:.2f} km\nlead {c['lead']*1000:.0f} m"
                n_clean += 1
            ax.add_patch(plt.Rectangle((xi - .5, yi - .5), 1, 1,
                                       facecolor=face, edgecolor='white',
                                       lw=2, zorder=1))
            ink = 'white' if face in (GOOD, BAD) else INK_2
            ax.text(xi, yi - 0.24, label, ha='center', va='center',
                    color=ink, fontsize=9.5, fontweight='bold', zorder=3)
            ax.text(xi, yi + 0.14, sub, ha='center', va='center',
                    color=ink, fontsize=7.8, zorder=3)

    ax.set_xticks(range(len(weathers)))
    ax.set_xticklabels([w.replace('Noon', '') for w in weathers], fontsize=9.5)
    ax.set_yticks(range(len(towns)))
    ax.set_yticklabels(towns, fontsize=9.5)
    ax.set_xlim(-.5, len(weathers) - .5)
    ax.set_ylim(len(towns) - .5, -.5)
    ax.set_title(f'Per ODD cell — clean only where there is enough distance '
                 f'to say so (≥ {min_km:g} km)',
                 loc='left', fontsize=11, fontweight='bold', color=INK)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.tick_params(length=0, colors=INK_2)
    ax.legend(handles=[
        Patch(facecolor=GOOD, label=f'clean, sufficient ({n_clean})'),
        Patch(facecolor=THIN, label=f'insufficient / not run ({n_thin})'),
        Patch(facecolor=BAD, label=f'event ({n_event})')],
        loc='upper center', bbox_to_anchor=(0.5, -0.06), ncol=3,
        frameon=False, fontsize=9)

    # ---- right: the two claims and what backs each ----------------------
    ax2.axis('off')
    ubc_lane = total / 3 if total else 0
    ubc_lead = lead / 3 if lead else 0
    lines = [
        ('THE TWO HALVES OF THE CLAIM', 'head'),
        ('', 'gap'),
        (f'Lane keeping — R171 §5.3.7.1', 'sub'),
        (f'   ADAS-at-fault collisions      {len(at_fault)}', 'ok' if not at_fault else 'bad'),
        (f'   lane departures               {dep_n}', 'ok' if not dep_n else 'bad'),
        (f'   backed by                     {total:.2f} km', 'val'),
        (f'   95 % bound if zero            1 per {ubc_lane:.1f} km', 'val'),
        ('', 'gap'),
        (f'Headway — R171 §5.3.7.5', 'sub'),
        (f'   collisions with a lead        '
         f'{sum(1 for c in at_fault if c["cat"] == "vehicle")}',
         'ok' if not any(c['cat'] == 'vehicle' for c in at_fault) else 'bad'),
        (f'   backed by                     {lead:.2f} km with a lead',
         'val' if lead >= 1.0 else 'warn'),
        (f'   95 % bound if zero            1 per {ubc_lead:.1f} km',
         'val' if lead >= 1.0 else 'warn'),
        ('', 'gap'),
        (f'   {lead/total*100:.0f} % of exposure had a lead in range.', 'warn'),
        ('   The headway half rests on that, not on the', 'warn'),
        ('   total — it is the weaker of the two claims.', 'warn'),
        ('', 'gap'),
        (f'Excluded, not ADAS mistakes:     {len(excluded)}', 'val'),
        ('   harness steering in a junction, or the ego', 'val'),
        ('   struck on the flank / rear by another actor.', 'val'),
    ]
    y = 0.98
    for txt, kind in lines:
        if kind == 'gap':
            y -= 0.030
            continue
        col = {'head': INK, 'sub': INK, 'ok': GOOD, 'bad': BAD,
               'warn': WARN, 'val': INK_2}[kind]
        wt = 'bold' if kind in ('head', 'sub') else 'normal'
        sz = 12 if kind == 'head' else (10.5 if kind == 'sub' else 9.5)
        ax2.text(0.0, y, txt, transform=ax2.transAxes, fontsize=sz,
                 color=col, fontweight=wt, va='top', family='monospace'
                 if kind in ('ok', 'bad', 'val', 'warn') else None)
        y -= 0.048

    fig.suptitle(
        f'ODD Exposure — ADAS SAE Level 2 (LKA + ACC)\n'
        f'envelope: UN R171 {R171_FEATURES}, declared '
        f'0–{R171_SPEED_MAX_KMH:g} km/h · {R171_DOMAIN}\n'
        f'{len(legs)} runs · {total:.2f} km ADAS-on-lane · '
        f'{len(at_fault)} at-fault collisions · {dep_n} lane departures'
        f'   ·   {os.path.basename(run_dir)}',
        x=0.008, ha='left', fontsize=13, fontweight='bold', color=INK)
    fig.tight_layout(rect=[0, 0.02, 1, 0.88])
    out = os.path.join(run_dir, 'odd_claim.png')
    fig.savefig(out, dpi=130, facecolor='white')
    print(f'wrote {out}')
    print(f'  {total:.2f} km ADAS · {lead:.2f} km with a lead '
          f'({lead/total*100:.0f} %)')
    print(f'  at-fault collisions {len(at_fault)} · departures {dep_n} · '
          f'excluded {len(excluded)}')
    print(f'  cells: {n_clean} clean, {n_thin} insufficient/not run, '
          f'{n_event} with events')
    if show:
        plt.show()
    plt.close(fig)
    return out


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('target', nargs='?', default=None)
    p.add_argument('--min-km', type=float, default=MIN_KM_DEFAULT)
    p.add_argument('--show', action='store_true')
    a = p.parse_args(argv)
    t = a.target
    if t is None:
        best, best_km = None, -1.0
        for d in glob.glob(os.path.join(HERE, '*odd*')):
            f = os.path.join(d, 'legs.json')
            if not os.path.exists(f):
                continue
            km = sum(r['adas_m'] for r in json.load(open(f))) / 1000
            if km > best_km:
                best, best_km = d, km
        if best is None:
            print('[plot] no ODD run found here')
            return 1
        t = best
        print(f'[plot] best run by exposure: {os.path.basename(t)} '
              f'({best_km:.2f} km)')
    if not os.path.isabs(t):
        c = os.path.join(HERE, t)
        t = c if os.path.exists(c) else t
    plot(t, a.min_km, a.show)
    plot_analysis(t, a.min_km, a.show)
    return 0


if __name__ == '__main__':
    sys.exit(main())

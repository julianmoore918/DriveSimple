#!/usr/bin/env python3
"""Check what a curve site's approach actually looks like to a lane detector.

`curve_survey.py` finds arcs. It does not ask what the lane markings do on
the way in, and that turned out to matter more than the arc: t12_r417 has
a lane split shortly before its bend — the left marking peels away to the
left, the right one follows the road right — and UFLD, given both, put the
ego lane somewhere between them. Every speed failed, including 30 km/h,
where the same controller held 0.1 m on t12_r500 (DEBUG §61).

    python3 scenarios/vet_site.py                    # every site in SITES
    python3 scenarios/vet_site.py t12_r417 t12_r500  # named sites
    python3 scenarios/vet_site.py --road 924 --lane -2 --s 56.9 --town Town12

What it reports, walking backwards from the arc entry over the approach:

    junction      metres of the approach inside a junction. Lane markings
                  are usually absent there and CARLA flags the splits and
                  merges it knows about.
    lane_change   road_id / lane_id changes along a walk that should stay
                  on one lane. A split shows up here first.
    neighbours    whether a driving lane exists to the left and right, and
                  whether that changes. A lane that gains or loses a
                  neighbour mid-approach is a fork or a merge, which is
                  what the detector sees as two plausible ego lanes.
    width         lane width variation; a widening lane is usually a
                  gore area.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from curve_adapter import SITES  # noqa: E402

STEP_M = 2.0
DEFAULT_APPROACH_M = 250.0


def _straightest(cur, step, backwards=False):
    cand = cur.previous(step) if backwards else cur.next(step)
    if not cand:
        return None
    cyaw = cur.transform.rotation.yaw
    return min(cand, key=lambda w: abs(
        (w.transform.rotation.yaw - cyaw + 540.0) % 360.0 - 180.0))


def neighbours(wp):
    """(has left driving lane, has right driving lane) in the same direction."""
    import carla
    out = []
    for get in (wp.get_left_lane, wp.get_right_lane):
        n = get()
        out.append(bool(n and n.lane_type == carla.LaneType.Driving
                        and n.lane_id * wp.lane_id > 0))
    return tuple(out)


def vet(cmap, road_id, lane_id, s, approach_m=DEFAULT_APPROACH_M,
        verbose=False):
    """Walk the approach backwards from the arc entry and describe it."""
    entry = cmap.get_waypoint_xodr(road_id, lane_id, s)
    if entry is None:
        return {'error': f'no waypoint at road {road_id} lane {lane_id} s={s}'}

    cur, dist = entry, 0.0
    rows = []
    while dist < approach_m:
        nxt = _straightest(cur, STEP_M, backwards=True)
        if nxt is None:
            break
        cur, dist = nxt, dist + STEP_M
        rows.append(dict(back_m=dist, road=cur.road_id, lane=cur.lane_id,
                         junction=cur.is_junction, width=cur.lane_width,
                         nbr=neighbours(cur)))
    if not rows:
        return {'error': 'no approach found'}

    junction_m = sum(STEP_M for r in rows if r['junction'])
    lane_changes = sum(1 for a, b in zip(rows, rows[1:])
                       if (a['road'], a['lane']) != (b['road'], b['lane']))
    nbr0 = rows[0]['nbr']
    nbr_changes = [r for r in rows if r['nbr'] != nbr0]
    widths = [r['width'] for r in rows]

    if verbose:
        print(f"    {'back[m]':>8} {'road':>6} {'lane':>5} {'junc':>5} "
              f"{'w[m]':>5}  L/R")
        last = None
        for r in rows:
            key = (r['road'], r['lane'], r['junction'], r['nbr'])
            if key != last:
                print(f"    {r['back_m']:>8.0f} {r['road']:>6} {r['lane']:>5} "
                      f"{'yes' if r['junction'] else '-':>5} "
                      f"{r['width']:>5.2f}  "
                      f"{'L' if r['nbr'][0] else '.'}"
                      f"{'R' if r['nbr'][1] else '.'}")
                last = key

    return {
        'approach_m': rows[-1]['back_m'],
        'junction_m': junction_m,
        'lane_changes': lane_changes,
        'neighbour_at_entry': ('L' if nbr0[0] else '.') + ('R' if nbr0[1] else '.'),
        'neighbour_changes': len(nbr_changes),
        'first_neighbour_change_back_m': (nbr_changes[0]['back_m']
                                          if nbr_changes else None),
        'width_min': min(widths), 'width_max': max(widths),
        # A site is clean if the whole approach stays on one lane, out of
        # junctions, with the same lanes either side and a constant width.
        'clean': (junction_m == 0 and lane_changes == 0
                  and not nbr_changes and max(widths) - min(widths) < 0.3),
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('sites', nargs='*', help='site names from curve_adapter.SITES')
    p.add_argument('--road', type=int)
    p.add_argument('--lane', type=int)
    p.add_argument('--s', type=float)
    p.add_argument('--town')
    p.add_argument('--approach-m', type=float, default=DEFAULT_APPROACH_M)
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--verbose', action='store_true',
                   help='print the approach profile, one row per change')
    args = p.parse_args(argv)

    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)
    world = client.get_world()
    town = world.get_map().name.split('/')[-1]

    if args.road is not None:
        targets = [('(explicit)', args.town or town, args.road, args.lane,
                    args.s)]
    else:
        names = args.sites or sorted(SITES)
        targets = [(n, SITES[n].town, SITES[n].road_id, SITES[n].lane_id,
                    SITES[n].entry_s_m) for n in names]

    print(f"[vet] server is in {town}; sites in other towns are skipped\n")
    print(f"{'site':<11} {'junction':>9} {'lane chg':>9} {'nbr':>5} "
          f"{'nbr chg':>8} {'width':>12}  verdict")
    for name, site_town, road, lane, s in targets:
        if site_town != town:
            print(f"{name:<11} {'— not loaded —':>46}")
            continue
        r = vet(world.get_map(), road, lane, s, args.approach_m, args.verbose)
        if 'error' in r:
            print(f"{name:<11} {r['error']}")
            continue
        why = []
        if r['junction_m']:
            why.append(f"{r['junction_m']:.0f} m in a junction")
        if r['lane_changes']:
            why.append(f"{r['lane_changes']} lane/road change(s)")
        if r['neighbour_changes']:
            why.append(f"neighbours change {r['first_neighbour_change_back_m']:.0f} m back")
        if r['width_max'] - r['width_min'] >= 0.3:
            why.append(f"width {r['width_min']:.2f}-{r['width_max']:.2f} m")
        print(f"{name:<11} {r['junction_m']:>7.0f} m {r['lane_changes']:>9} "
              f"{r['neighbour_at_entry']:>5} {r['neighbour_changes']:>8} "
              f"{r['width_min']:>5.2f}-{r['width_max']:<5.2f}  "
              f"{'CLEAN' if r['clean'] else 'dirty: ' + '; '.join(why)}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

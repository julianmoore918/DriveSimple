#!/usr/bin/env python3
"""Catalogue the constant-radius curves in CARLA's stock maps.

UN R171 Annex 4 §4.2.4.1 specifies the curved test section as a clothoid
S-bend: first turn R = 787 m, second turn R = 374 m (opposite direction),
with clothoid transitions between. No CARLA map contains that geometry,
and §4.2.4.1 anticipates exactly this:

    "At the request of the manufacturer and with the agreement of the Type
     Approval Authority, tests may be conducted on a road of different
     curvature, provided this does not change the intention or lower the
     severity of the test."

Severity of a curved-road test is carried by the lateral acceleration the
curve demands (v^2 / R) and by how far the target sits outside a
straight-ahead corridor — not by the radius on its own. So the harness
picks CARLA curves by *measured* radius and reports the resulting demand,
rather than pretending a map has 787 m arcs.

This tool produces the evidence for that choice. For every topology edge
in every requested map it walks the lane, fits a radius, and reports the
straight lead-in available before the curve starts (R171 §4.2.5.2.2.1.2
requires the VUT to be settled in-lane on a straight before curve entry).

    python3 scenarios/curve_survey.py                    # default maps
    python3 scenarios/curve_survey.py --maps Town04 Town06
    python3 scenarios/curve_survey.py --csv sites.csv

Columns
-------
    road/lane       OpenDRIVE ids — this is the site key the scenario
                    scripts take (`--road-id` / `--lane-id`).
    R_mean          arc length / total heading change over the curve [m].
    R_spread        max-min of the per-window radius, as a fraction of
                    R_mean. Low = a genuine constant-radius arc; high =
                    a clothoid or a compound curve, where quoting one
                    radius misleads.
    curve_m         how much curve there is to work with.
    lead_in_m       dead-straight road immediately before curve entry.
                    R171 needs settle_time * v of this; R79's lane-keeping
                    test needs enough to be in steady state at entry.
    v_ay85          speed that puts the lateral demand at 85 % of a
                    3.0 m/s^2 declared aysmax — the R79 Annex 8 §3.2.1
                    window is 80-90 %, so this is the natural test speed
                    for the site. Capped at 130 km/h.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys

# Every stock map with any highway-grade geometry. Town01/02/10HD are
# urban grids whose curves are corner radii (< 30 m), useful only for the
# low-speed end of the R79 matrix; Town11-15 are the large 0.9.15+ maps
# and take minutes to load, so they are opt-in.
DEFAULT_MAPS = ('Town03', 'Town04', 'Town05', 'Town06', 'Town07')

STEP_M = 2.0              # lane-walk resolution
MIN_CURVE_M = 30.0        # shorter than this cannot hold a placement
STRAIGHT_R_M = 3000.0     # radius above which a segment counts as straight
MAX_R_M = 2500.0          # radius above which it is not a "curve" at all
MIN_R_M = 15.0            # below this it is a corner, not a road curve
WINDOW_M = 20.0           # window for the per-window radius spread check
MIN_TURN_RAD = 0.087      # >= 5 deg of actual turn, else it is a straight

AY_MAX_DEFAULT = 3.0      # M1/N1 ceiling, R79 §5.6.2.1.3 table 1
AY_FRACTION = 0.85        # centre of the R79 §3.2.1.1 80-90 % window
V_MAX_KMH = 130.0


def _wrap_deg(d: float) -> float:
    return (d + 540.0) % 360.0 - 180.0


def _straightest(cur, step: float, backwards: bool = False):
    """Next/previous waypoint that continues the current heading.

    Junction entries fan out; taking the smallest heading change keeps the
    walk on the through-road rather than following an exit ramp.
    """
    cand = cur.previous(step) if backwards else cur.next(step)
    if not cand:
        return None
    cyaw = cur.transform.rotation.yaw
    return min(cand, key=lambda w: abs(_wrap_deg(w.transform.rotation.yaw - cyaw)))


def walk(wp, length_m: float, step: float = STEP_M, backwards: bool = False):
    """March along the lane, returning [(s, yaw_deg, waypoint), ...]."""
    out = [(0.0, wp.transform.rotation.yaw, wp)]
    cur, s = wp, 0.0
    while s < length_m:
        nxt = _straightest(cur, step, backwards)
        if nxt is None:
            break
        s += step
        cur = nxt
        out.append((s, cur.transform.rotation.yaw, cur))
    return out


def radius_of(samples) -> tuple[float, float, float]:
    """(signed mean radius, arc length, total heading change [deg]).

    Radius from arc length over total heading change rather than a
    three-point circle fit: heading is what CARLA stores per waypoint, and
    integrating it is immune to the lateral jitter a circle fit picks up
    from waypoint quantisation. Sign follows the turn direction
    (+ve = right, matching the harness's steer convention).
    """
    if len(samples) < 3:
        return float('inf'), 0.0, 0.0
    total = 0.0
    for (_, y0, _), (_, y1, _) in zip(samples, samples[1:]):
        total += _wrap_deg(y1 - y0)
    # Length OF THIS SLICE, not the s-coordinate of its last sample. Using
    # the absolute coordinate scaled every windowed radius by
    # (distance from the walk's start / window length), which inflated a
    # 40 m bend in Town06 to a reported 400 m and made the constant-radius
    # search compare numbers that meant different things at each offset.
    # Caught by CarlaCurveAdapter.connect()'s re-measure, which is the
    # only reason the site table was not published wrong.
    arc = samples[-1][0] - samples[0][0]
    if abs(total) < 1e-6:
        return float('inf'), arc, 0.0
    return arc / math.radians(abs(total)) * (1 if total > 0 else -1), arc, total


def window_radii(samples, window_m: float = WINDOW_M) -> list[float]:
    """Signed radius over a sliding `window_m` window, one per sample."""
    n = max(int(window_m / STEP_M), 3)
    out: list[float] = []
    for i in range(len(samples)):
        # Full windows only. A short tail window integrates a few degrees
        # of heading over a few metres, which reads as a wildly different
        # radius and made the constancy check reject real arcs (and pass
        # near-straight noise as 1400 m "curves").
        if i + n >= len(samples):
            out.append(float('nan'))
            continue
        r, _, _ = radius_of(samples[i:i + n + 1])
        out.append(r)
    return out


def longest_constant_arc(samples, tol: float = 0.15):
    """Longest run of samples whose windowed radius stays within `tol`.

    A CARLA road is one OpenDRIVE road but not one geometry record: the
    highway curves in Town04 open with a spiral, so measuring a radius over
    the whole topology edge mixes the transition into the arc and reports
    a number that exists nowhere on the road (Town04 road 46 came out at
    462 m with the per-window radius varying by 350 % of that).

    R171's reference section is itself clothoid-then-arc, so what the
    scenario needs from a site is the *arc*: a stretch of genuinely
    constant demand long enough to hold the target, with the transition
    counted as lead-in rather than as curve. This finds that stretch.

    Returns (start_index, end_index, signed radius) or None.
    """
    radii = window_radii(samples)
    best = None
    i = 0
    while i < len(radii):
        r0 = radii[i]
        if not math.isfinite(r0) or not (MIN_R_M <= abs(r0) <= MAX_R_M):
            i += 1
            continue
        j = i
        while j + 1 < len(radii):
            r1 = radii[j + 1]
            if (not math.isfinite(r1) or r1 * r0 < 0
                    or abs(abs(r1) - abs(r0)) / abs(r0) > tol):
                break
            j += 1
        if best is None or (j - i) > (best[1] - best[0]):
            # Re-fit over the run itself; the seed window's radius is only
            # the anchor the run grew from.
            r, _, _ = radius_of(samples[i:j + 1])
            if math.isfinite(r):
                best = (i, j, r)
        i = j + 1
    return best


def lead_in_m(entry_wp, max_m: float = 500.0) -> float:
    """Dead-straight road immediately before the curve entry.

    Walks backwards while the accumulated heading change stays inside what
    STRAIGHT_R_M allows over the distance covered — i.e. the same
    "straight" definition used for the radius classification, applied in
    reverse, rather than a lateral-deviation tolerance that would depend
    on how far back you happened to look.
    """
    back = walk(entry_wp, max_m, backwards=True)
    if len(back) < 2:
        return 0.0
    y0 = back[0][1]
    total = 0.0
    prev_yaw = y0
    for s, yaw, _ in back[1:]:
        total += abs(_wrap_deg(yaw - prev_yaw))
        prev_yaw = yaw
        if s > 0 and (s / math.radians(total) if total else float('inf')) \
                < STRAIGHT_R_M:
            return s - STEP_M
    return back[-1][0]


def lane_context(wp) -> tuple[int, int]:
    """(driving lanes to the left, to the right) in the same direction.

    UFLD needs a marking on both sides of the ego lane. A site whose lane
    is the outermost of a carriageway with a shoulder on one side gives
    perception a different problem from one bracketed by traffic lanes,
    and that difference belongs in the site record, not in a surprise.
    """
    import carla
    left = right = 0
    cur = wp
    while True:
        nxt = cur.get_left_lane()
        if (nxt is None or nxt.lane_type != carla.LaneType.Driving
                or nxt.lane_id * cur.lane_id < 0):
            break
        left += 1
        cur = nxt
    cur = wp
    while True:
        nxt = cur.get_right_lane()
        if (nxt is None or nxt.lane_type != carla.LaneType.Driving
                or nxt.lane_id * cur.lane_id < 0):
            break
        right += 1
        cur = nxt
    return left, right


def survey_map(world, town: str, min_curve_m: float,
               walk_m: float = 1200.0) -> list[dict]:
    """One row per constant-radius arc reachable in the map.

    Walks ACROSS road boundaries rather than surveying each topology edge
    in isolation. A CARLA highway curve is not one road: Town04's road 46
    is pure spiral (radius falling from infinity to ~1000 m over 138 m),
    and the arc it leads into is a different road id. Measuring per edge
    reported that spiral as a 462 m constant curve — a radius that exists
    nowhere on the road. Walking through the boundary and then extracting
    the constant-radius run finds the arc itself.

    The same arc is reachable from many starting edges, so sites are
    deduplicated on the arc midpoint (20 m grid) and lane id.
    """
    import carla
    cmap = world.get_map()
    sites: dict = {}

    for begin, _end in cmap.get_topology():
        if begin.lane_type != carla.LaneType.Driving:
            continue

        samples = walk(begin, walk_m)
        if len(samples) < min_curve_m / STEP_M:
            continue

        run = longest_constant_arc(samples)
        if run is None:
            continue
        i0, i1, r = run
        arc = samples[i1][0] - samples[i0][0]
        if arc < min_curve_m:
            continue
        # A 30 m stretch of 1400 m radius bends by 1.2 degrees — that is
        # survey noise on a straight, not a curve, and it dominated the
        # ranking when only radius and length were checked.
        if arc / abs(r) < MIN_TURN_RAD:
            continue
        _, _, heading = radius_of(samples[i0:i1 + 1])
        inner = [abs(x) for x in window_radii(samples[i0:i1 + 1])
                 if math.isfinite(x)]
        spread = (max(inner) - min(inner)) / abs(r) if inner else float('nan')

        entry = samples[i0][2]
        mid = samples[(i0 + i1) // 2][2].transform.location
        key = (round(mid.x / 20.0), round(mid.y / 20.0), entry.lane_id)
        if key in sites and sites[key]['curve_m'] >= arc:
            continue          # same arc, found from a worse starting point

        left, right = lane_context(entry)
        v_ay85 = min(math.sqrt(AY_FRACTION * AY_MAX_DEFAULT * abs(r)) * 3.6,
                     V_MAX_KMH)
        sites[key] = dict(
            town=town,
            # The site key the scenario scripts take. `s` is the
            # OpenDRIVE arc coordinate of the ARC ENTRY along its own
            # road, so map.get_waypoint_xodr(road, lane, s) reproduces
            # this exact point without re-walking the map.
            road_id=entry.road_id,
            lane_id=entry.lane_id,
            entry_s_m=round(entry.s, 1),
            R_m=round(abs(r), 1),
            direction='right' if r > 0 else 'left',
            R_spread=round(spread, 3),
            curve_m=round(arc, 1),
            heading_deg=round(heading, 1),
            lead_in_m=round(lead_in_m(entry), 1),
            lane_width_m=round(entry.lane_width, 2),
            lanes_left=left,
            lanes_right=right,
            junction=bool(entry.is_junction),
            x=round(entry.transform.location.x, 1),
            y=round(entry.transform.location.y, 1),
            v_ay85_kmh=round(v_ay85, 1),
            ay_at_130=round((V_MAX_KMH / 3.6) ** 2 / abs(r), 2),
        )
    return list(sites.values())


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maps', nargs='+', default=list(DEFAULT_MAPS))
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--min-curve-m', type=float, default=MIN_CURVE_M)
    p.add_argument('--min-lead-in-m', type=float, default=60.0,
                   help='Hide sites with less straight lead-in than this. '
                        'A curved-road scenario needs the VUT settled in '
                        'lane before curve entry (R171 §4.2.5.2.2.1.2).')
    p.add_argument('--top', type=int, default=12,
                   help='Rows printed per map. Default 12.')
    p.add_argument('--csv', default=None, help='Write every site here.')
    p.add_argument('--restore', default=None,
                   help='Map to load again when finished, so a survey does '
                        'not leave the server on the last town it visited.')
    args = p.parse_args(argv)

    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(180.0)
    all_sites: list[dict] = []

    for town in args.maps:
        print(f"[survey] loading {town} ...", flush=True)
        try:
            world = client.load_world(town)
        except Exception as exc:
            print(f"[survey] {town}: {exc}", flush=True)
            continue
        sites = survey_map(world, town, args.min_curve_m)
        all_sites.extend(sites)

        show = [s for s in sites if s['lead_in_m'] >= args.min_lead_in_m]
        show.sort(key=lambda s: (-s['lead_in_m'], -s['curve_m']))
        print(f"\n=== {town}: {len(sites)} curved lane sections, "
              f"{len(show)} with >= {args.min_lead_in_m:g} m lead-in ===")
        print(f"{'road':>5} {'lane':>5} {'R[m]':>8} {'dir':>5} {'spread':>7} "
              f"{'curve[m]':>9} {'lead[m]':>8} {'w[m]':>5} {'L/R':>5} "
              f"{'v@85%[km/h]':>12} {'ay@130':>7}")
        for s in show[:args.top]:
            print(f"{s['road_id']:>5} {s['lane_id']:>5} {s['R_m']:>8.1f} "
                  f"{s['direction']:>5} {s['R_spread']:>7.2f} "
                  f"{s['curve_m']:>9.1f} {s['lead_in_m']:>8.1f} "
                  f"{s['lane_width_m']:>5.2f} "
                  f"{str(s['lanes_left']) + '/' + str(s['lanes_right']):>5} "
                  f"{s['v_ay85_kmh']:>12.1f} {s['ay_at_130']:>7.2f}")

    if args.csv and all_sites:
        with open(args.csv, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(all_sites[0].keys()))
            w.writeheader()
            w.writerows(all_sites)
        print(f"\n[survey] {len(all_sites)} sites -> {args.csv}")

    if args.restore:
        print(f"[survey] restoring {args.restore}", flush=True)
        client.load_world(args.restore)
    return 0


if __name__ == '__main__':
    sys.exit(main())

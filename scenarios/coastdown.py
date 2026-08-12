#!/usr/bin/env python3
"""Open-loop coast-down: is the deceleration physics, or the metric?

No ACC, no lead, no ROS. Accelerate to a target speed, then hold
throttle = 0 and brake = 0 and log velocity, gear and rpm every tick.

Deliberately run in SYNCHRONOUS mode at a fine fixed_delta, which the
R171 harness does not use (it runs async because sync regressed forward
motion in the multi-process setup, DEBUG §4). That difference is the
point: if the deceleration needles survive clean, evenly-spaced sampling
then they are the vehicle; if they vanish, they were the sampling.

Restores the world's original settings on exit.
"""
import argparse
import math
import sys

import carla


def dump_physics(pc):
    """The CARLA physics defaults that govern off-throttle drag."""
    rows = [
        ('mass', pc.mass, 'kg'),
        ('drag_coefficient', pc.drag_coefficient, ''),
        ('moi', pc.moi, 'engine inertia'),
        ('damping_rate_full_throttle', pc.damping_rate_full_throttle, ''),
        ('damping_rate_zero_throttle_clutch_engaged',
         pc.damping_rate_zero_throttle_clutch_engaged, '<- off-throttle drag'),
        ('damping_rate_zero_throttle_clutch_disengaged',
         pc.damping_rate_zero_throttle_clutch_disengaged, ''),
        ('use_gear_autobox', pc.use_gear_autobox, ''),
        ('gear_switch_time', pc.gear_switch_time, 's'),
        ('clutch_strength', pc.clutch_strength, ''),
        ('final_ratio', pc.final_ratio, ''),
    ]
    print('--- get_physics_control() ---')
    for k, v, note in rows:
        print(f'  {k:<46} {str(v):>8}  {note}')
    print(f'  forward_gears ({len(pc.forward_gears)}):')
    for i, g in enumerate(pc.forward_gears):
        print(f'    gear {i+1}: ratio {g.ratio:.3f}  down_ratio {g.down_ratio:.2f} '
              f' up_ratio {g.up_ratio:.2f}')
    for i, w in enumerate(pc.wheels):
        print(f'  wheel {i}: damping_rate {w.damping_rate}  radius {w.radius} '
              f' max_brake_torque {w.max_brake_torque}')
    print()


def speed_kmh(actor):
    v = actor.get_velocity()
    return 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


def centred_decel(ts, vs, half_window_s):
    """Same estimator the R171 KPI uses: centred least-squares slope."""
    out = []
    for i, ti in enumerate(ts):
        win = [(t, v) for t, v in zip(ts, vs) if abs(t - ti) <= half_window_s]
        if len(win) < 3:
            out.append(0.0)
            continue
        n = len(win)
        mt = sum(t for t, _ in win) / n
        mv = sum(v for _, v in win) / n
        den = sum((t - mt) ** 2 for t, _ in win)
        out.append(-sum((t - mt) * (v - mv) for t, v in win) / den if den else 0.0)
    return out


def run(world, bp, spawn, v0_kmh, fixed_delta, damping=None):
    ego = world.spawn_actor(bp, spawn)
    try:
        world.tick()
        if damping is not None:
            pc = ego.get_physics_control()
            pc.damping_rate_zero_throttle_clutch_engaged = damping
            ego.apply_physics_control(pc)
            world.tick()
        # --- spin up to v0 under throttle ---
        for _ in range(2000):
            world.tick()
            if speed_kmh(ego) >= v0_kmh:
                break
            ego.apply_control(carla.VehicleControl(throttle=0.85, brake=0.0))
        # --- coast: nothing commanded at all ---
        ctl = carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False)
        ts, vs, gears, rpms = [], [], [], []
        t0 = world.get_snapshot().timestamp.elapsed_seconds
        while True:
            ego.apply_control(ctl)
            world.tick()
            snap = world.get_snapshot().timestamp
            t = snap.elapsed_seconds - t0
            v = speed_kmh(ego) / 3.6
            ts.append(t)
            vs.append(v)
            gears.append(ego.get_control().gear)
            try:
                rpms.append(ego.get_physics_control().max_rpm and
                            ego.get_vehicle_telemetry_data().engine_rpm)
            except Exception:
                rpms.append(float('nan'))
            if v < 0.5 or t > 30.0:
                break
        return ts, vs, gears, rpms
    finally:
        ego.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--speeds', default='30,50',
                    help='km/h to coast down from (comma separated)')
    ap.add_argument('--fixed-delta', type=float, default=0.01,
                    help='sync-mode physics step. Default 0.01 s.')
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--vehicle', default='vehicle.dodge.charger_2020')
    ap.add_argument('--damping', type=float, default=None,
                    help='override damping_rate_zero_throttle_clutch_engaged')
    ap.add_argument('--table', action='store_true',
                    help='emit a COAST_DECEL table for controller_node.py')
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    original = world.get_settings()

    try:
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = args.fixed_delta
        s.max_substeps = 16
        s.max_substep_delta_time = 0.00625
        world.apply_settings(s)
        client.get_trafficmanager().set_synchronous_mode(True)

        bp = world.get_blueprint_library().find(args.vehicle)
        spawn = world.get_map().get_spawn_points()[80]
        spawn.location.z += 0.5

        probe = world.spawn_actor(bp, spawn)
        world.tick()
        dump_physics(probe.get_physics_control())
        probe.destroy()
        world.tick()

        for v0 in [float(x) for x in args.speeds.split(',')]:
            ts, vs, gears, rpms = run(world, bp, spawn, v0,
                                      args.fixed_delta, args.damping)
            print(f'=== coast-down from {v0:.0f} km/h '
                  f'(sync, fixed_delta={args.fixed_delta}s, '
                  f'{len(ts)} samples) ===')
            # Report on the same estimator the KPI uses, at several widths,
            # so metric sensitivity is visible next to the physics.
            for hw in (0.05, 0.12, 0.25, 0.50):
                d = centred_decel(ts, vs, hw)
                print(f'   peak decel, centred +/-{hw:.2f}s window : '
                      f'{max(d):5.2f} m/s^2')
            print(f'   mean decel over the coast           : '
                  f'{(vs[0] - vs[-1]) / (ts[-1] - ts[0]):5.2f} m/s^2')
            print(f'   distance to rest                    : '
                  f'{sum(0.5*(vs[i]+vs[i-1])*(ts[i]-ts[i-1]) for i in range(1,len(ts))):5.1f} m')
            d12 = centred_decel(ts, vs, 0.12)
            print(f'   {"t":>6} {"v_kmh":>7} {"decel":>7} {"gear":>5}')
            step = max(1, len(ts) // 25)
            for i in range(0, len(ts), step):
                print(f'   {ts[i]:>6.2f} {vs[i]*3.6:>7.2f} {d12[i]:>7.2f} '
                      f'{gears[i]:>5}')
            if args.table:
                # COAST_DECEL is a v -> decel map read by the controller's
                # coast-aware split. Bin the smoothed trace by speed and
                # take the MEDIAN in each bin.
                #
                # Median, not mean: a bin that a downshift passes through
                # gets a short, large transient, and the mean carries it
                # into what is supposed to be a steady-state map. Measured
                # on the first attempt, the 5.5 and 8.5 m/s bins came out
                # at 2.33 and 2.24 against ~0.7 either side -- those are
                # the 2->1 and 3->2 shifts, not drag at those speeds. The
                # split needs what the vehicle SUSTAINS; the transient is
                # real but belongs in the peak metric, not here.
                d = centred_decel(ts, vs, 0.25)
                bins = {}
                for v, a in zip(vs, d):
                    b = round(v - 0.5) + 0.5
                    if b >= 0.5 and a > 0:
                        bins.setdefault(b, []).append(a)
                print('   COAST_DECEL candidate (mean decel per 1 m/s bin):')
                print('       ', ', '.join(
                    f'({b:.1f}, {sorted(v)[len(v)//2]:.2f})'
                    for b, v in sorted(bins.items())))
            gs = sorted(set(gears))
            print(f'   gears used: {gs}')
            for g in gs:
                sp = [vs[i]*3.6 for i in range(len(ts)) if gears[i] == g]
                if sp:
                    print(f'     gear {g}: {min(sp):5.1f} - {max(sp):5.1f} km/h')
            print()
    finally:
        world.apply_settings(original)
        try:
            client.get_trafficmanager().set_synchronous_mode(False)
        except Exception:
            pass
        print('world settings restored')


if __name__ == '__main__':
    sys.exit(main() or 0)

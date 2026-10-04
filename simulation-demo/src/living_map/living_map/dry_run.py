"""Dry run: the complete Living Map mission WITHOUT Gazebo / ROS.

All real mission nodes run unchanged on a stand-in for rclpy, with a kinematic
"fake Gazebo": skid-steer robots with wheel slip (encoders drift), IMU gyro with bias,
2D lidar ray casting in the tunnel model (other robots are obstacles too), velocity-controlled
Flix A/B with ToF (the downward ToF sees the water surface of the flooded line C).

    python3 -m living_map.dry_run --scenario all            # real time + 2D window
    python3 -m living_map.dry_run --scenario all --fast --no-view   # test mode
"""
import argparse
import json
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from living_map import fake_ros  # noqa: E402

try:
    import numpy as np
except Exception:
    np = None


def set_quat(q, yaw, roll=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q.x, q.y, q.z, q.w = sr * cy, sr * sy, cr * sy, cr * cy


class Caster:
    """Vectorised 2D ray casting against the tunnel free-space model."""

    def __init__(self, C):
        self.C = C
        rects = list(C.TUNNEL_RECTS) + [C.OUTSIDE_RECT]
        # LM_GEOM_SHIFT=d: the real south branch (X2 -> victim) lies d metres east of the radio /
        # map model - reproduces real tiles that differ from the conservative corridor model
        sh = float(os.environ.get('LM_GEOM_SHIFT', '0') or 0)
        if sh:
            rects = [r for r in rects if not (r[0] == 68 and r[1] == 72 and r[2] < 0)]
            rects += [(68 + sh, 72 + sh, -30.0, 2.0), (68.0, 72.0, -2.0, 10.0)]
        self.rects = rects
        self.R = np.array(rects, dtype=float) if np is not None else None
        self.circles = []            # (x, y, r) of the ground robots (set every step)
        # LM_ROUGH=1: rough real-tile walls - rock bumps 0.2-0.7 m into the galleries every few
        # metres (the real SubT tiles are not straight 4 m boxes)
        self.solids = self._bumps() if os.environ.get('LM_ROUGH') == '1' else []
        self.S = np.array(self.solids, dtype=float) if (np is not None and self.solids) else None

    def _in_rects(self, x, y):
        return any(r[0] - 1e-6 <= x <= r[1] + 1e-6 and r[2] - 1e-6 <= y <= r[3] + 1e-6
                   for r in self.rects)

    def _bumps(self):
        rng = random.Random(11)
        out = []
        for x0, x1, y0, y1 in self.C.TUNNEL_RECTS:
            horiz = (x1 - x0) >= (y1 - y0)
            a0, a1 = (x0, x1) if horiz else (y0, y1)
            for side in (0, 1):
                a = a0 + rng.uniform(1.0, 3.0)
                while a < a1 - 1.0:
                    L, d = rng.uniform(0.5, 2.0), rng.uniform(0.2, 0.7)
                    if horiz:
                        w = y0 if side == 0 else y1
                        box = (a, a + L, w, w + d) if side == 0 else (a, a + L, w - d, w)
                        out_pt = [(a + L / 2, w - 0.3 if side == 0 else w + 0.3)]
                    else:
                        w = x0 if side == 0 else x1
                        box = (w, w + d, a, a + L) if side == 0 else (w - d, w, a, a + L)
                        out_pt = [(w - 0.3 if side == 0 else w + 0.3, a + L / 2)]
                    # only on a real wall (not across a junction opening or a portal):
                    # rock just behind both ends of the bump
                    ends = [(a + 0.05, out_pt[0][1]) if horiz else (out_pt[0][0], a + 0.05),
                            (a + L - 0.05, out_pt[0][1]) if horiz else (out_pt[0][0], a + L - 0.05)]
                    wall = all(not self._in_rects(px, py) for px, py in out_pt + ends)
                    if wall and box[0] > 0.5:
                        out.append(box)
                    a += L + rng.uniform(2.0, 5.0)
        return out

    def free(self, x, y):
        return self._in_rects(x, y) and not any(
            b[0] <= x <= b[1] and b[2] <= y <= b[3] for b in self.solids)

    def cast(self, x, y, yaw, angles, rmax, step=0.1):
        if np is None:
            return [float('inf')] * len(angles)
        a = yaw + np.asarray(angles)
        r = np.arange(0.15, rmax, step)
        px = x + np.outer(np.cos(a), r)
        py = y + np.outer(np.sin(a), r)
        R = self.R
        inside = ((px[..., None] >= R[:, 0] - 1e-6) & (px[..., None] <= R[:, 1] + 1e-6) &
                  (py[..., None] >= R[:, 2] - 1e-6) & (py[..., None] <= R[:, 3] + 1e-6)).any(axis=2)
        if self.S is not None:
            S = self.S
            rock = ((px[..., None] >= S[:, 0]) & (px[..., None] <= S[:, 1]) &
                    (py[..., None] >= S[:, 2]) & (py[..., None] <= S[:, 3])).any(axis=2)
            inside = inside & ~rock
        blocked = ~inside
        first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), -1)
        out = np.where(first >= 0, r[np.clip(first, 0, len(r) - 1)], np.inf)
        for cx, cy, cr in self.circles:                 # other robots: ray-circle intersection
            if math.hypot(cx - x, cy - y) < 0.05 or math.hypot(cx - x, cy - y) > rmax + cr:
                continue
            ca, sa = np.cos(a), np.sin(a)
            fx, fy = x - cx, y - cy
            b = fx * ca + fy * sa
            c = fx * fx + fy * fy - cr * cr
            disc = b * b - c
            t = -b - np.sqrt(np.maximum(disc, 0.0))
            hit = (disc > 0) & (t > 0.15)
            out = np.where(hit & (t < out), t, out)
        return out.tolist()


class FakeGazebo:
    def __init__(self, C, perturb=False):
        self.C = C
        self.rng = random.Random(7)
        self.cast = Caster(C)
        self.pert = perturb
        self.bots = {}
        for name in C.GROUND:
            sp = C.SPAWN[name]
            self.bots[name] = {'x': sp[0], 'y': sp[1], 'yaw': sp[3], 'v': 0.0, 'w': 0.0,
                               'cv': 0.0, 'cw': 0.0, 'spawn': sp, 'tipped': False,
                               'ox': 0.0, 'oy': 0.0, 'oyaw': 0.0,
                               'bias': self.rng.choice((-1, 1)) * C.SENS['gyro_bias']}
            fake_ros.BUS.subs.setdefault(f'/model/{name}/cmd_vel', []).append(
                lambda m, n=name: self._cmd(n, m))
        self.fly = {}
        for name in C.FLIXES:
            sp = C.SPAWN[name]
            self.fly[name] = {'x': sp[0], 'y': sp[1], 'z': sp[2], 'yaw': sp[3], 'cmd': None,
                              'bias': self.rng.choice((-1, 1)) * C.SENS['gyro_bias']}
            fake_ros.BUS.subs.setdefault(f'/model/{name}/cmd_vel', []).append(
                lambda m, n=name: self.fly[n].__setitem__('cmd', m))
        fake_ros.BUS.subs.setdefault('/lm/gz/cmd', []).append(self._gz)
        self.t_pub = 0.0
        self.t_scan = 0.0
        self.t_tof = 0.0

    def _cmd(self, name, m):
        b = self.bots[name]
        b['cv'] = max(-1.5, min(1.5, m.linear.x))
        b['cw'] = max(-1.5, min(1.5, m.angular.z))

    def _gz(self, m):
        m = json.loads(m.data)
        if m['op'] == 'set_pose' and m['name'] in self.bots:
            self.bots[m['name']]['tipped'] = True

    def step(self, dt, t):
        C = self.C
        for b in self.bots.values():
            if b['tipped']:
                continue
            k = min(1.0, dt / (0.35 if self.pert else 0.1))
            b['v'] += k * (b['cv'] - b['v'])
            b['w'] += k * (b['cw'] - b['w'])
            # skid-steer: true motion differs from what the wheels (encoders) report
            slip_v = C.SENS['enc_slip_v'] * (2.0 if self.pert else 1.0)
            slip_w = C.SENS['enc_slip_w']
            vt = b['v'] * (1.0 - slip_v)
            wt = b['w'] * (1.0 - slip_w) + (0.02 * abs(b['v']) if self.pert else 0.0)   # terrain-induced yaw while driving
            b['yaw'] += wt * dt
            nx = b['x'] + vt * math.cos(b['yaw']) * dt
            ny = b['y'] + vt * math.sin(b['yaw']) * dt
            others = [(o['x'], o['y']) for o in self.bots.values() if o is not b]
            ok = lambda px, py: (all(self.cast.free(px + ex, py + ey)
                                     for ex, ey in ((0.3, 0), (-0.3, 0), (0, 0.3), (0, -0.3)))
                                 and all(math.hypot(px - ox, py - oy) > 0.62 or
                                         math.hypot(px - ox, py - oy) > math.hypot(b['x'] - ox, b['y'] - oy)
                                         for ox, oy in others))
            if ok(nx, ny):
                b['x'], b['y'] = nx, ny
            elif ok(nx, b['y']):              # slide along the wall like a real contact
                b['x'] = nx
            elif ok(b['x'], ny):
                b['y'] = ny
            b['wt'] = wt
            # encoder odometry (what DiffDrive reports): wheel speeds, no slip knowledge
            b['oyaw'] += b['w'] * dt
            b['ox'] += b['v'] * math.cos(b['oyaw']) * dt
            b['oy'] += b['v'] * math.sin(b['oyaw']) * dt
        for name, f in self.fly.items():
            c = f['cmd']
            if c is not None:
                cy, sy = math.cos(f['yaw']), math.sin(f['yaw'])
                f['x'] += (cy * c.linear.x - sy * c.linear.y) * dt
                f['y'] += (sy * c.linear.x + cy * c.linear.y) * dt
                f['z'] += (c.linear.z - 0.01) * dt           # velocity control + gravity sag
                f['yaw'] += c.angular.z * dt
                f['wt'] = c.angular.z
            else:
                f['wt'] = 0.0
            floor = self.surface(f['x'], f['y'])
            f['z'] = max(f['z'], floor + 0.02)
            if f['x'] > 0.0 and f['y'] < -50:          # line C roof (ramps, sump, air pocket)
                f['z'] = min(f['z'], C.c_floor_z(f['x']) + C.ROOF_CLEAR - 0.05)
        if t - self.t_pub >= 0.02:
            self.t_pub = t
            self.publish_state()
        if t - self.t_scan >= 0.2:
            self.t_scan = t
            self.publish_scans()
        if t - self.t_tof >= 0.1:
            self.t_tof = t
            self.publish_tof()

    def surface(self, x, y):
        """What is under a flying robot: the floor, or the water of the flooded sump (line C)."""
        if x > 0.0 and y < -50:
            return max(self.C.c_floor_z(x), self.C.WATER_Z)
        return 0.0

    def publish_state(self):
        O = sys.modules['nav_msgs.msg'].Odometry
        I = sys.modules['sensor_msgs.msg'].Imu
        C = self.C
        for name, b in self.bots.items():
            m = O()
            m.pose.pose.position.x, m.pose.pose.position.y = b['ox'], b['oy']
            set_quat(m.pose.pose.orientation, b['oyaw'])
            m.twist.twist.linear.x = b['v'] if not b['tipped'] else 0.0
            m.twist.twist.angular.z = b['w'] if not b['tipped'] else 0.0
            fake_ros.BUS.publish(f'/model/{name}/odometry', m)
            g = O()
            g.pose.pose.position.x, g.pose.pose.position.y = b['x'], b['y']
            set_quat(g.pose.pose.orientation, b['yaw'], 1.35 if b['tipped'] else 0.0)
            fake_ros.BUS.publish(f'/model/{name}/ground_truth', g)
            im = I()
            im.angular_velocity.z = b.get('wt', 0.0) + b['bias'] + self.rng.gauss(0, C.SENS['gyro_sigma'])
            fake_ros.BUS.publish(f'/{name}/imu', im)
        for name, f in self.fly.items():
            g = O()
            g.pose.pose.position.x, g.pose.pose.position.y, g.pose.pose.position.z = f['x'], f['y'], f['z']
            set_quat(g.pose.pose.orientation, f['yaw'])
            fake_ros.BUS.publish(f'/model/{name}/ground_truth', g)
            im = I()
            im.angular_velocity.z = f.get('wt', 0.0) + f['bias'] + self.rng.gauss(0, C.SENS['gyro_sigma'])
            fake_ros.BUS.publish(f'/{name}/imu', im)

    def publish_scans(self):
        L = sys.modules['sensor_msgs.msg'].LaserScan
        for name, b in self.bots.items():
            self.cast.circles = [(o['x'], o['y'], 0.3) for n2, o in self.bots.items() if n2 != name]
            m = L()
            n = 181
            m.angle_min, m.angle_max = -2.356, 2.356
            m.angle_increment = (m.angle_max - m.angle_min) / (n - 1)
            m.range_min, m.range_max = 0.1, 12.0
            angles = [m.angle_min + i * m.angle_increment for i in range(n)]
            rs = self.cast.cast(b['x'], b['y'], b['yaw'], angles, 12.0)
            m.ranges = [r + self.rng.gauss(0, 0.01) if r != float('inf') else r for r in rs]
            if self.pert:
                # smoke / dust: isolated spurious returns near the gas leak (as Gazebo's particle
                # noise does), which the robots' lidar dust filter has to reject
                g = self.C.GAS_LEAK
                if math.hypot(b['x'] - g['x'], b['y'] - g['y']) < 8.0:
                    m.ranges = [self.rng.uniform(0.3, min(r, 6.0)) if self.rng.random() < 0.15 else r
                                for r in m.ranges]
            fake_ros.BUS.publish(f'/{name}/scan', m)
        self.cast.circles = []

    def publish_tof(self):
        L = sys.modules['sensor_msgs.msg'].LaserScan
        for name in self.C.FLIXES:
            f = self.fly[name]
            d = L()
            d.range_min, d.range_max = 0.05, 4.0
            d.ranges = [f['z'] - self.surface(f['x'], f['y']) - self.C.FLIX_TOF_DOWN_Z
                        + self.rng.gauss(0, self.C.SENS['tof_sigma'])]
            fake_ros.BUS.publish(f'/{name}/tof_down', d)
            h = L()
            h.angle_min, h.angle_max, h.angle_increment = -1.5708, 1.5708, 1.5708
            h.range_min, h.range_max = 0.05, 4.0
            h.ranges = self.cast.cast(f['x'], f['y'], f['yaw'], [-1.5708, 0.0, 1.5708], 4.0)
            fake_ros.BUS.publish(f'/{name}/tof', h)


NODES = (('writer_node:Writer', {'robot': 'writer_a'}), ('writer_node:Writer', {'robot': 'writer_b'}),
         ('executor_node:Executor', {'robot': 'executor'}),
         ('flix_node:Flix', {'robot': 'flix_a'}), ('flix_node:Flix', {'robot': 'flix_b'}),
         ('beacon_network:BeaconNetwork', {}),
         ('outside_network:OutsideNetwork', {'ona': 'A'}),
         ('outside_network:OutsideNetwork', {'ona': 'B'}),
         ('outside_network:OutsideNetwork', {'ona': 'C'}),
         ('command_post:CommandPost', {}), ('environment:Environment', {}),
         ('gz_actions:GzActions', {}), ('director_node:Director', {}))
ROBOT_TOPICS = ('writer_a', 'writer_b', 'executor', 'flix_a', 'flix_b')


def run(scenario='nominal', fast=False, speed=1.0, until=1500.0, view=True, port=8765,
        quiet=False, linger=True, perturb=False):
    fake_ros.install(params={'scenario': scenario, 'dry_run': True, 'use_sim_time': True,
                             'port': port}, quiet=quiet)
    import importlib
    from living_map import config as C
    nodes = []
    base = dict(fake_ros.STATE['params'])
    for spec, extra in NODES:
        mod, cls = spec.split(':')
        fake_ros.STATE['params'] = {**base, 'robot': '', 'ona': '', **extra}
        nodes.append(getattr(importlib.import_module(f'living_map.{mod}'), cls)())
    fake_ros.STATE['params'] = dict(base, robot='', ona='')
    if view:
        from living_map.dataflow_view import DataflowView
        nodes.append(DataflowView())
        print(f'\n  2D dataflow window: http://localhost:{port}\n', flush=True)

    rec = {'cp': None, 'flows': [], 'poses': {}, 'beacons': None, 'camera': [], 'trace': []}
    B = fake_ros.BUS.subs
    B.setdefault('/lm/cp/state', []).append(lambda m: rec.__setitem__('cp', json.loads(m.data)))
    B.setdefault('/lm/flow', []).append(lambda m: rec['flows'].append(json.loads(m.data)))
    for n in ROBOT_TOPICS:
        B.setdefault(f'/lm/pose/{n}', []).append(
            lambda m, n=n: rec['poses'].__setitem__(n, json.loads(m.data)))
    B.setdefault('/lm/beacons', []).append(lambda m: rec.__setitem__('beacons', json.loads(m.data)))

    def cam(m):
        d = json.loads(m.data)
        if not rec['camera'] or rec['camera'][-1][1] != d['phase']:
            rec['camera'].append((round(fake_ros.STATE['t_ns'] / 1e9, 1), d['phase']))
    B.setdefault('/lm/camera', []).append(cam)

    gz = FakeGazebo(C, perturb)
    inject = []
    for spec in filter(None, os.environ.get('LM_INJECT', '').split(',')):
        who, ti, dx, dy = spec.split(':')
        inject.append([who, float(ti), float(dx), float(dy), False])
    dt = 0.01
    t = 0.0
    wall0 = time.time()
    max_err = {}
    while t < until:
        t += dt
        fake_ros.STATE['t_ns'] = int(round(t * 1e9))
        gz.step(dt, t)
        for inj in inject:
            if not inj[4] and t >= inj[1]:
                inj[4] = True
                for nd in nodes:
                    if getattr(nd, 'rname', None) == inj[0]:
                        nd.ekf.f.x[0] += inj[2]
                        nd.ekf.f.x[1] += inj[3]
        fake_ros.fire_timers()
        fake_ros.BUS.drain()
        for n, p in rec['poses'].items():
            if 'err' in p:
                max_err[n] = max(max_err.get(n, 0.0), p['err'])
        if int(round(t * 100)) % int(os.environ.get('LM_TRACE_CS', '200')) == 0:
            for n, p in rec['poses'].items():
                rec['trace'].append((round(t, 1), n, round(p['x'], 2), round(p['y'], 2),
                                     round(p.get('ex', 0), 2), round(p.get('ey', 0), 2),
                                     p.get('state'), p.get('note') or p.get('src') or '',
                                     round(p.get('yaw', 0), 3), round(p.get('eyaw', 0), 3),
                                     round(p.get('z', 0), 2), round(p.get('ez', 0), 2)))
        if rec['camera'] and rec['camera'][-1][1] == 'cutaway':
            if 'cut_t' not in rec:
                rec['cut_t'] = t
            elif t - rec['cut_t'] > 3.0:
                break
        if not fast:
            lag = t / speed - (time.time() - wall0)
            if lag > 0:
                time.sleep(lag)
    rec['t_end'] = t
    rec['max_err'] = max_err
    rec['gui'] = next((n.gui_log for n in nodes if type(n).__name__ == 'Director'), [])
    if view and linger and not fast:
        print('\nMission finished - window stays live. Ctrl+C to quit.', flush=True)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    return rec


def main():
    ap = argparse.ArgumentParser(description='Living Map dry run (no Gazebo)')
    ap.add_argument('--scenario', default='nominal')
    ap.add_argument('--fast', action='store_true')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--until', type=float, default=1500.0)
    ap.add_argument('--no-view', action='store_true')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--perturb', action='store_true', help='robustness test: lag, extra slip, drift')
    ap.add_argument('--report', default='')
    a = ap.parse_args()
    try:
        rec = run(a.scenario, a.fast, a.speed, a.until, not a.no_view, a.port, a.quiet,
                  perturb=a.perturb or os.environ.get('LM_PERTURB') == '1')
    except KeyboardInterrupt:
        return
    cp = rec['cp'] or {}
    if a.report:
        kinds = {}
        for f in rec['flows']:
            kinds[f['kind']] = kinds.get(f['kind'], 0) + 1
        heal = [f for f in rec['flows'] if f['kind'] == 'heal_plan']
        with open(a.report, 'w') as fh:
            json.dump({'cp': cp, 'flow_kinds': kinds, 'heal': heal, 't_end': rec['t_end'],
                       'beacons': rec['beacons'], 'poses': rec['poses'],
                       'camera': rec['camera'], 'gui': rec['gui'], 'max_err': rec['max_err'],
                       'trace': rec['trace'],
                       'flows': [f for f in rec['flows'] if f['kind'] in (
                           'drop', 'csi_drop', 'presence', 'fusion', 'writer_lost', 'junction',
                           'release', 'heal_plan', 'relinked', 'rockfall', 'collapse', 'glitch',
                           'ona_beacon')]},
                      fh, indent=1)
    print('\n==== RESULT ====')
    print('t_end   :', round(rec['t_end'], 1))
    print('phase   :', cp.get('phase'))
    print('entrance:', cp.get('entrance'), [(e['id'], e['state'], e.get('dist')) for e in cp.get('entrances', [])])
    print('writers :', cp.get('writer'))
    print('flix    :', cp.get('drone'))
    print('executor:', cp.get('executor'))
    for e in cp.get('events', []):
        print(f"event   : {e['key']:5s} {e['type']:7s} {e['status']:15s} "
              f"conf {e.get('conf', 0):.2f} (cam {e.get('cam', 0):.2f} csi {e.get('csi', 0):.2f}) "
              f"{', '.join(e.get('sources', []))} {e.get('note', '')}")
    print('camera  :', rec['camera'])
    print('max est. error (m):', {k: round(v, 2) for k, v in rec['max_err'].items()})


if __name__ == '__main__':
    main()

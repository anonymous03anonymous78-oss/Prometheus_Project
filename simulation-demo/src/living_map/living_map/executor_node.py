"""Executor (Explorer X1): receives a mission built from the information already stored in the
beacons (it does not need the Writer that collected it, nor a complete exploration), briefed
through an Outside Network Area as soon as a victim hypothesis is on the living map. It uses
the beacon network for localization (beacon fixes), route reconstruction (the briefed trail,
followed by RF gradient fused with the beacons' coordinates) and target identification (its
camera finds the person at the hypothesis - a point for a camera find, a region for a CSI-only
presence). On its way it re-measures unconfirmed gas events. At each person it confirms the
detection, delivers first aid (oxygen self-rescuer + two-way radio from its kit) and waits for
the Command Post to send it to the next one: most confident first, then the nearest."""
import math

from . import config as C
from .common import spin_node
from .geometry import rssi
from .ground import GroundRobot
from .packet import decode_briefing

ARRIVE_RSSI = rssi(C.EXEC_ARRIVE_R)


class Executor(GroundRobot):
    def __init__(self):
        super().__init__('executor', seed=23, publishes=('/lm/executor/uplink', '/lm/pose/executor'))
        self.sub('/lm/rf/executor', self.on_rf)
        self.sub('/lm/executor/downlink', self.on_downlink)
        self.sub('/lm/sensors/executor', self.on_sensors)
        self.state = 'wait_start'
        self.heard = {}
        self.grad = {}
        self.sensors = {}
        self.path = []
        self.idx = 0
        self.victim = None
        self.target_id = None
        self.verify = []
        self.lost_since = None
        self.wait_since = None
        self.heading_src = ''
        self.grad_dir = None
        self.briefings = 0
        self.aid_until = 0.0
        self.go_after_aid = False                # next briefing received during the first aid
        self.confirmed = []
        self.measure = None
        self.radius = 0.0
        self.conf = 0.0
        self.eid = 0
        self.create_timer(0.1, self.step)

    # ------------------------------------------------------------ inputs
    def on_sensors(self, m):
        self.sensors = m
        self.apply_gps(m)

    def on_rf(self, m):
        self.heard = {b['id']: b for b in m.get('beacons', [])}
        self.peers = m.get('peers', [])
        a = 0.3
        for bid, b in self.heard.items():
            w = 10.0 * math.log10(max(b['fresh'], 1e-3))   # trail strength decays with age
            gx = (b['f'] - b['b']) / (2 * C.EXEC_ANTENNA_OFFSET)
            gy = (b['l'] - b['r']) / (2 * C.EXEC_ANTENNA_OFFSET)
            c = b['c'] + w
            g = self.grad.get(bid)
            if g is None:
                self.grad[bid] = [gx, gy, c, b['c']]
            else:
                g[0] += a * (gx - g[0])
                g[1] += a * (gy - g[1])
                g[2] += a * (c - g[2])
                g[3] += a * (b['c'] - g[3])     # raw signal: distance to the beacon

    def on_downlink(self, m):
        raw = bytes.fromhex(m['hex'])
        br = decode_briefing(raw)
        self.briefings += 1
        cur = self.path_id()
        self.path = [(bid, x, y) for bid, x, y in br['path']]
        self.victim = br['victim']
        self.target_id = br['target']
        done = {v['id']: v['done'] for v in self.verify}
        self.verify = [{'id': bid, 'type': et, 'x': x, 'y': y, 'done': done.get(bid, False)}
                       for bid, et, x, y in br['verify']]
        ids = [p[0] for p in self.path]
        if m['kind'] == 'briefing' and self.state in ('wait_briefing', 'wait_start',
                                                       'awaiting_next', 'first_aid'):
            self.idx = 0
            # briefed while still giving first aid: finish it, then leave at once
            self.go_after_aid = self.state == 'first_aid'
            if not self.go_after_aid:
                self.state = 'en_route'
            self.measure = None
            if hasattr(self, '_search'):
                del self._search
            self.conf, self.radius = br['conf'], br['radius']
            self.get_logger().info(f'briefed ({len(raw)} B): path {ids}, person at '
                                   f'({self.victim[0]:.1f},{self.victim[1]:.1f}) +/- {self.radius} m, '
                                   f'confidence {self.conf:.0%}'
                                   + (' - leaving once first aid is done' if self.go_after_aid else ''))
        else:
            if cur in ids:
                self.idx = ids.index(cur)
            if self.state == 'waiting_chain' and self.idx < len(self.path):
                self.state = 'en_route'
            self.get_logger().info(f'briefing update: path {ids}')

    def path_id(self):
        return self.path[self.idx][0] if 0 <= self.idx < len(self.path) else None

    # ------------------------------------------------------------ helpers
    def uplink(self, ev, subject, x, y, flags=0, **kw):
        self.eid = self.eid % 127 + 1
        pkt = {'id': subject, 'type': ev, 'parent': 0, 'flags': flags, 'src': C.SRC_ID['executor'],
               'eid': self.eid, 'x': x, 'y': y, 'heading': math.degrees(self.est[2]),
               't_ms': self.now_ms()}
        pkt.update(kw)
        self.pub('/lm/executor/uplink', {'from': 'executor', 'wx': self.truth.x,
                                         'wy': self.truth.y, 'pkt': pkt})

    def step(self):
        if not self.truth.ok:
            return
        if self.state == 'wait_start':
            self.stop()
            if self.started():
                self.gps_fix()
                self.state = 'wait_briefing'
            return
        if self.state in ('wait_briefing', 'on_scene', 'awaiting_next'):
            self.stop()
            if self.state == 'awaiting_next':
                self.heading_src = 'first aid delivered - waiting for the next miner'
            return
        if self.state == 'first_aid':
            self.stop()
            self.heading_src = 'first aid: oxygen self-rescuer + radio delivered' + (
                f' - next: B{self.target_id}' if self.go_after_aid else '')
            if self.now() >= self.aid_until:
                self.state = 'en_route' if self.go_after_aid else 'awaiting_next'
                self.go_after_aid = False
            return
        self.check_verify()
        if self.state == 'en_route':
            self.follow_trail()
        elif self.state == 'waiting_chain':
            self.stop()
            self.heading_src = 'holding at chain end - waiting for the Writer to extend it'
            if self.now() - self.wait_since > 180.0:
                self.get_logger().warn('no chain extension - final approach on coordinates')
                self.state = 'final_approach'
        elif self.state == 'final_approach':
            self.final_approach()

    def follow_trail(self):
        x, y, th = self.est
        if self.idx >= len(self.path):
            vx, vy = self.victim
            last = self.path[-1] if self.path else None
            if last is None or math.hypot(vx - last[1], vy - last[2]) <= C.EXEC_SIGHT_R + self.radius:
                self.state = 'final_approach'
            else:
                self.state = 'waiting_chain'
                self.wait_since = self.now()
            return
        bid, bx, by = self.path[self.idx]
        g = self.grad.get(bid) if bid in self.heard else None
        # arrival and range use the raw signal (an old trail is weaker as a trail, but the
        # beacon is just as close)
        if g is not None and g[3] >= ARRIVE_RSSI:
            self.get_logger().info(f'reached B{bid}')
            # beacon fix: re-anchor to the Writer's map at every beacon reached (RSSI range)
            d_rf = 10 ** ((C.RF_P1M - g[3]) / (10 * C.RF_N))
            h = math.atan2(by - y, bx - x)
            self.ekf.landmark(self.now(), bx - d_rf * math.cos(h), by - d_rf * math.sin(h), 0.35)
            self.idx += 1
            self.lost_since = None
            return
        dx, dy = bx - x, by - y
        dcoord = math.hypot(dx, dy)
        coord_h = math.atan2(dy, dx)
        if g is not None:
            self.lost_since = None
            gh = th + math.atan2(g[1], g[0])
            self.grad_dir = gh
            hx = 0.6 * math.cos(gh) + 0.4 * math.cos(coord_h)
            hy = 0.6 * math.sin(gh) + 0.4 * math.sin(coord_h)
            d_est = 10 ** ((C.RF_P1M - g[3]) / (10 * C.RF_N))
            tx, ty = x + min(d_est, 6.0) * hx / max(math.hypot(hx, hy), 1e-6), \
                y + min(d_est, 6.0) * hy / max(math.hypot(hx, hy), 1e-6)
            self.heading_src = f'RF gradient + coordinates -> B{bid}'
            self.drive_to(tx, ty, C.EXECUTOR_SPEED, near_turn=d_est < 4.0)
            return
        self.grad_dir = None
        if self.lost_since is None:
            self.lost_since = self.now()
        for j in range(self.idx + 1, len(self.path)):
            if self.path[j][0] in self.heard and self.now() - self.lost_since > 1.5:
                self.get_logger().warn(f'B{bid} silent -> skipping to B{self.path[j][0]}')
                self.idx = j
                self.lost_since = None
                return
        if dcoord < C.EXEC_ARRIVE_R:
            self.idx += 1
            return
        self.heading_src = f'coordinates B{bid} (not heard)'
        self.drive_to(bx, by, C.EXECUTOR_SPEED * 0.8, near_turn=dcoord < 4.0)

    def final_approach(self):
        """Target identification: go to the hypothesis; once the camera sees the person, close in
        to the stand-off distance and confirm. If nothing is seen there, search the hypothesis
        region (a CSI presence is a region, not a point); nobody found -> not confirmed."""
        x, y, th = self.est
        if self.measure is not None:
            self.stop()
            self.heading_src = 'camera: identifying the person'
            for v in self.sensors.get('victims') or []:
                self.measure['conf'] = max(self.measure['conf'], v['conf'])
            if self.now() - self.measure['t0'] > C.CV_DWELL:
                self.confirm(self.measure)
            return
        vs = self.sensors.get('victims') or []
        v = min(vs, key=lambda d: d['range']) if vs else None
        lim = 6.0 + self.radius
        if v and (math.hypot(v['range'] * math.cos(th + v['bearing']) + x - self.victim[0],
                             v['range'] * math.sin(th + v['bearing']) + y - self.victim[1]) > lim
                  or any(math.hypot(v['range'] * math.cos(th + v['bearing']) + x - cx,
                                    v['range'] * math.sin(th + v['bearing']) + y - cy) < 2.5
                         for cx, cy in self.confirmed)):
            v = None                                  # another person (already handled)
        if v:
            a = th + v['bearing']
            vx, vy = x + v['range'] * math.cos(a), y + v['range'] * math.sin(a)
            self.heading_src = f'person in the camera at {v["range"]:.1f} m - closing in'
            if v['range'] <= C.VICTIM_STANDOFF:
                self.stop()
                self.measure = {'t0': self.now(), 'x': vx, 'y': vy, 'conf': v['conf'],
                                'range': v['range']}
                return
            far = v['range'] > C.EXEC_CLOSE_R
            self.drive_to(vx, vy, C.EXECUTOR_SPEED if far else C.EXEC_CLOSE_SPEED, centring=False)
            return
        if not hasattr(self, '_search'):
            ex, ey = self.victim
            h = th
            r = max(3.0, float(self.radius))
            self._search = [(ex, ey), (ex + r * math.cos(h), ey + r * math.sin(h)),
                            (ex - r * math.cos(h), ey - r * math.sin(h))] * 3
            self._si = 0
        if self._si >= len(self._search):
            self.stop()
            self.get_logger().warn('nobody found at the hypothesis')
            ex, ey = self.victim
            self.uplink(C.EV_FALSE_POSITIVE, self.target_id or 0, ex, ey, conf=0.0, a=0,
                        b=C.EV_VICTIM)
            self.state = 'awaiting_next'
            return
        tx, ty = self._search[self._si]
        self.heading_src = 'searching the hypothesis region' if self._si else \
            'final approach to the person'
        if self.drive_to(tx, ty, C.EXECUTOR_SPEED * 0.8, centring=False) < 0.6:
            self._si += 1

    def confirm(self, meas):
        self.uplink(C.EV_VICTIM_CONFIRMED, self.target_id or 0, meas['x'], meas['y'],
                    a=10.0 * meas['range'], b=C.CAM_RGBD, conf=meas['conf'])
        self.get_logger().info(f'person identified ({meas["conf"]:.0%}) - first aid')
        self.confirmed.append((meas['x'], meas['y']))
        self.measure = None
        self.aid_until = self.now() + C.FIRST_AID_S
        self.go_after_aid = False
        self.state = 'first_aid'

    def check_verify(self):
        """Re-measure each unconfirmed gas event on the route: keep the highest reading while
        within 4 m of the event, decide once the Executor has passed it."""
        x, y, _ = self.est
        gas = self.sensors.get('gas', 0.0)
        for v in self.verify:
            if v['done']:
                continue
            d = math.hypot(v['x'] - x, v['y'] - y)
            if d <= 4.0:
                v['max'] = max(v.get('max', 0.0), gas)
                v['near'] = True
                continue
            at_target = self.state in ('first_aid', 'awaiting_next') or self.measure is not None
            if not v.get('near') or (d < 5.0 and not at_target):
                continue
            v['done'] = True
            g = v.get('max', 0.0)
            if g < 0.5 * C.GAS_ALARM:
                self.uplink(C.EV_FALSE_POSITIVE, v['id'], v['x'], v['y'], a=g * 10.0,
                            b=C.EV_HAZARD)
                self.get_logger().info(f'B{v["id"]} gas re-measured {g:.2f}% -> false positive')
            else:
                self.uplink(C.EV_HAZARD, v['id'], v['x'], v['y'], a=g * 10.0,
                            b=self.sensors.get('o2', 20.9) * 10.0 - 150.0,
                            conf=min(1.0, g / (2.0 * C.GAS_ALARM)))

    def publish_pose(self):
        self.pub('/lm/pose/executor', self.pose_msg({
            'state': self.state, 'target': self.path_id(), 'path': [p[0] for p in self.path],
            'idx': self.idx, 'grad': self.grad_dir, 'src': self.heading_src,
            'briefings': self.briefings, 'confirmed': len(self.confirmed),
            'robot': 'executor'}))


def main(args=None):
    spin_node(Executor, args)


if __name__ == '__main__':
    main()

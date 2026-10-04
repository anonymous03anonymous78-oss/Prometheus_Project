"""Environment: ground-truth hazards, simulated mission sensors and scripted incidents.

Mission sensors (models driven by simulator truth; everything else the robots use is a real
Gazebo sensor):
  Writers A/B : CH4 + O2 detector (hazardous area), computer vision person detector on the
                RGB-D camera (4 m, line of sight) with its detection confidence
  Executor    : same detectors (target identification on arrival, gas re-measurement)
  Flix A/B    : portal camera (classifies an opening it is inside: clear / flooded / collapsed,
                reads the portal sign) and thermal + RGB camera person detector (25 m, line of
                sight, +/-70 deg FOV) with its detection confidence
The Wi-Fi CSI channel (the RF victim sensing) is modelled with the radio medium in
beacon_network.py.
Incidents: false-positive gas glitch, rockfall on a relay beacon, roof collapse on Writer B.
"""
import math

from nav_msgs.msg import Odometry

from . import config as C
from .common import LMNode, OdomTracker, spin_node, wrap
from .geometry import dist, los, plan_relocation

ROBOTS = C.GROUND + C.FLIXES


def gas_at(x, y):
    g = C.GAS_LEAK
    d2 = (x - g['x']) ** 2 + (y - g['y']) ** 2
    return g['peak'] * math.exp(-d2 / (2 * g['sigma'] ** 2))


class Environment(LMNode):
    def __init__(self):
        super().__init__('environment', publishes=(
            '/lm/env/knockout', '/lm/env/writer_hit', '/lm/gz/cmd', '/lm/scenario')
            + tuple(f'/lm/sensors/{n}' for n in ROBOTS))
        self.trk = {}
        for name in ROBOTS:
            self.trk[name] = OdomTracker(C.SPAWN[name])
            self.create_subscription(Odometry, f'/model/{name}/ground_truth',
                                     self.trk[name].update, 20)
        self.sub('/lm/beacons', self.on_beacons)
        for n in C.GROUND:
            self.sub(f'/lm/pose/{n}', lambda m, n=n: self._state(n, m))
        self.beacons = []
        self.states = {}
        self.exec_path = []
        self.glitch_used = False
        self.knocked = False
        self.collapsed = False
        self.rock_n = 0
        import random
        self.rng = random.Random(5)
        self.create_timer(0.2, self.sense)
        self.create_timer(0.5, self.incidents)
        self.create_timer(2.0, self.publish_scenario)

    def on_beacons(self, m):
        self.beacons = m.get('beacons', [])

    def _state(self, name, m):
        self.states[name] = m.get('state', '')
        if name == 'executor':
            self.exec_path = m.get('path') or []

    # ------------------------------------------------------------ sensors
    def victims_in_mine(self):
        return list(C.VICTIMS)

    def detect(self, tr, victim, rng, fov=None, model=C.CV_GROUND):
        d = dist((tr.x, tr.y), (victim['x'], victim['y']))
        if d > rng:
            return None
        # line of sight to the body (a point 0.6 m in front of it, toward the observer)
        k = min(0.6, d) / max(d, 1e-6)
        tx, ty = victim['x'] + (tr.x - victim['x']) * k, victim['y'] + (tr.y - victim['y']) * k
        if not los((tr.x, tr.y), (tx, ty)):
            return None
        b = wrap(math.atan2(victim['y'] - tr.y, victim['x'] - tr.x) - tr.yaw)
        if fov is not None and abs(b) > fov:
            return None
        # person detector score (what the CV model outputs for this frame): falls with range
        conf = model[0] - model[1] * d + self.rng.gauss(0.0, C.SENS['cv_sigma'])
        return {'range': d, 'bearing': b, 'conf': round(max(0.3, min(0.98, conf)), 3)}

    def gps(self, tr):
        """RTK-GPS (ONA base station corrections) - only outside, under open sky."""
        if tr.x > -0.5:
            return None
        from .packet import world_to_gps
        lat, lon = world_to_gps(tr.x + self.rng.gauss(0, 0.05), tr.y + self.rng.gauss(0, 0.05))
        return {'lat': lat, 'lon': lon, 'sigma': 0.1}

    def ground_sensors(self, name, tr):
        g = gas_at(tr.x, tr.y)
        if (name in C.WRITERS and self.flags['false_positive'] and not self.glitch_used
                and dist((tr.x, tr.y), (C.FP_GLITCH['x'], C.FP_GLITCH['y'])) <= C.FP_GLITCH['r']):
            g = C.FP_GLITCH['value']
            self.glitch_used = True
            self.flow('glitch', x=tr.x, y=tr.y, value=g, robot=name)
        vics = [d for d in (self.detect(tr, v, C.VICTIM_DETECT_R) for v in self.victims_in_mine()) if d]
        return {'gas': round(g, 3), 'o2': round(20.9 - 0.25 * g, 2), 'gps': self.gps(tr),
                'victims': vics, 'victim': vics[0] if vics else None}

    def portal_reading(self, f):
        """Camera + ToF reading when Flix is inside an opening of the hillside."""
        for pid, e in C.ENTRANCES.items():
            if 0.3 < f.x < C.FLIX_INSPECT_DEPTH + 1.5 and abs(f.y - e['y']) < 2.5:
                st = e['state']
                detail = {'clear': 'gallery clear ahead (ToF 4 m free), no rockfall, no water',
                          'flooded': 'the gallery ramps down into water a few metres inside (flooded sump)',
                          'collapsed': 'roof fall 2.5 m inside the portal, rubble to the roof'}[st]
                return {'id': pid, 'state': st, 'detail': detail, 'sign': f'PORTAL {pid}'}
        return None

    def sense(self):
        for n in C.GROUND:
            tr = self.trk[n]
            if tr.ok:
                self.pub(f'/lm/sensors/{n}', self.ground_sensors(n, tr))
        for n in C.FLIXES:
            f = self.trk[n]
            if not f.ok:
                continue
            m = {'victims': [], 'portal': self.portal_reading(f), 'gps': self.gps(f)}
            if f.x > 0.0:
                for v in self.victims_in_mine():
                    d = self.detect(f, v, C.FLIX_THERMAL_R, C.FLIX_THERMAL_FOV, C.CV_FLIX)
                    if d:
                        m['victims'].append(d)
            m['victim'] = m['victims'][0] if m['victims'] else None
            self.pub(f'/lm/sensors/{n}', m)

    # ------------------------------------------------------------ incidents
    def spawn_rocks(self, x, y, n, spread, model='rock', z0=2.2):
        for i in range(n):
            self.rock_n += 1
            a = 2.4 * i
            r = spread * (0.3 + 0.7 * ((i * 37) % 10) / 10.0)
            self.pub('/lm/gz/cmd', {'op': 'spawn', 'name': f'rock_{self.rock_n}', 'model': model,
                                    'x': x + r * math.cos(a), 'y': y + r * math.sin(a),
                                    'z': z0 + 0.4 * i, 'yaw': a})

    def pick_knockout(self, ex):
        """Relay beacon on the Executor's briefed route, ahead of it, whose loss needs the
        largest successful relocation (or at least a re-link)."""
        route = set(self.exec_path or [])
        view = {b['id']: {'x': b['x'], 'y': b['y'], 'parent': b['parent'], 'alive': b['alive']}
                for b in self.beacons}
        best = None
        for b in self.beacons:
            if not b['alive'] or b['btype'] != 'trail' or b['x'] < ex + 6.0:
                continue
            if route and b['id'] not in route:
                continue
            children = [k for k, v in view.items() if v['parent'] == b['id'] and v['alive']]
            if not children:
                continue
            moves = []
            for c in children:
                plan = plan_relocation(view, c, b['id'])
                if plan is None:
                    moves = None
                    break
                moves.append(plan[3])
            if moves and (best is None or max(moves) > best[1]):
                best = (b['id'], max(moves))
        return best

    def incidents(self):
        e = self.trk['executor']
        if (self.flags['beacon_knockout'] and not self.knocked and e.ok
                and self.states.get('executor') == 'en_route' and e.x >= C.KNOCKOUT_MIN_X):
            pick = self.pick_knockout(e.x)
            if pick is not None:
                self.knocked = True
                bid = pick[0]
                b = next(v for v in self.beacons if v['id'] == bid)
                self.flow('rockfall', id=bid, x=b['x'], y=b['y'])
                self.get_logger().warn(f'ROCKFALL on B{bid} (heal needs {pick[1]:.1f} m move)')
                self.spawn_rocks(b['x'], b['y'], 1, 0.0, 'rock_small')
                self.spawn_rocks(b['x'], b['y'] + 1.2, 5, 0.8, 'rock_small')

                def kill():
                    self.pub('/lm/env/knockout', {'id': bid})
                    self.pub('/lm/gz/cmd', {'op': 'remove', 'name': f'beacon_{bid}'})
                self.after(0.9, kill)
        name = C.COLLAPSE_VICTIM
        w = self.trk[name]
        if (self.flags['writer_destroyed'] and not self.collapsed and w.ok
                and self.states.get(name, '') == 'exploring'
                and 60.0 < w.x < 80.0 and w.y < C.WRITER_COLLAPSE_Y):
            self.collapsed = True
            wx, wy = w.x, w.y
            self.flow('collapse', x=wx, y=wy, robot=name)
            self.get_logger().warn(f'ROOF COLLAPSE on {name}')
            self.spawn_rocks(wx, wy, 8, 0.7)
            self.after(0.7, lambda: self.pub('/lm/env/writer_hit', {'x': wx, 'y': wy, 'robot': name}))
            self.after(1.2, lambda: self.pub('/lm/gz/cmd', {
                'op': 'set_pose', 'name': name, 'x': wx, 'y': wy, 'z': 0.3,
                'roll': 1.35, 'yaw': w.yaw}))

    def publish_scenario(self):
        self.pub('/lm/scenario', {'name': self.scenario, 'flags': self.flags,
                                  'text': C.SCENARIO_TEXT[self.scenario]})


def main(args=None):
    spin_node(Environment, args)


if __name__ == '__main__':
    main()

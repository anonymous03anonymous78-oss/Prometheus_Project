"""Outside Network Area stations (ONA-A, ONA-B, ONA-C): the only bridges between the mine and
the world.

Each station: communication computer, directional (parabolic) antenna to the relay satellite
(+ LTE backup mast), solar panels + high-capacity battery, optical-fibre cable to the first
beacon of its portal (ONA-C: to its own first beacon, placed at the flooded portal C). All
stations are linked to the mission server / Command Post through the relay satellite.

Uplink : fibre (entrance beacon) -> decode the 32 bytes -> frame translation: the Writer's
         local SLAM coordinates are tied to the entrance beacon, whose position in the global
         frame is surveyed (RTK-GPS at the portal) -> GPS (lat, lon, alt) -> satellite ->
         server / Command Post.
Downlink: Command Post -> satellite -> GPS -> site frame -> binary briefing -> fibre -> mesh.
Outside-zone wireless: GPS fixes of robots at the staging area, drone reports, GO, lessons.
Energy : solar panels charge the battery bank, which powers the station and the chargers of the
         robots between missions (Writers parked at the station, Flix pads).
"""
import math

from . import config as C
from .common import LMNode, spin_node
from .geometry import dist
from .packet import (MSG_BRIEF, MSG_BRIEF_UPDATE, decode, encode_briefing, fields_hex,
                     gps_to_local, local_to_gps, value_text)


class OutsideNetwork(LMNode):
    def __init__(self):
        super().__init__('ona_b', publishes=(
            '/lm/sat/uplink', '/lm/fiber/downlink', '/lm/wireless/from_ona', '/lm/ona/state',
            '/lm/server/sync', '/lm/gz/cmd'))
        self.letter = str(self.get_parameter('ona').value or 'B')
        self.pos = C.ONAS[self.letter]['pos']
        # site frame origin surveyed with GPS by the Outside Network Area itself
        self.anchor = (C.GEO_LAT0, C.GEO_LON0, C.GEO_AZ_DEG)
        self.fixes = {}
        self.translated = 0
        self.last = None
        E = C.ONA_ENERGY
        self.soc = E['soc0']                       # battery bank state of charge
        self.charging = {}                         # robot -> charger load (W)
        self.poses = {}
        for n in C.GROUND + C.FLIXES:
            self.sub(f'/lm/pose/{n}', lambda m, n=n: self.poses.__setitem__(n, m))
        self.sub('/lm/fiber/uplink', self.on_fiber_up)
        self.sub('/lm/wireless/to_ona', self.on_wireless)
        self.sub('/lm/sat/downlink', self.on_sat_down)
        self.create_timer(1.0, self.publish_state)
        self.create_timer(1.0, self.energy)
        self.t_beam = -1e9

    def mine(self, m):
        return (m.get('ona') or 'B') == self.letter

    def beam(self, down=False):
        """Gazebo: flash the dish -> satellite beam (rate-limited)."""
        if self.now() - self.t_beam < 2.5:
            return
        self.t_beam = self.now()
        self.pub('/lm/gz/cmd', {'op': 'beam', 'ona': self.letter, 'down': down})

    # ------------------------------------------------------------ uplink
    def on_fiber_up(self, env):
        if env.get('ona', 'B') != self.letter:
            return
        raw = bytes.fromhex(env['hex'])
        pkt = decode(raw)
        lat, lon = local_to_gps(pkt['x'], pkt['y'], *self.anchor)
        alt = C.GEO_ALT0 + pkt.get('z', 0.0)
        self.translated += 1
        self.last = {'mid': env['mid'], 'id': pkt['id'], 'type': pkt['type'],
                     'mx': round(pkt['x'], 3), 'my': round(pkt['y'], 3), 'mz': round(pkt['z'], 2),
                     'lat': round(lat, 7), 'lon': round(lon, 7), 'alt': round(alt, 2),
                     'utc': pkt['utc'], 'txt': value_text(pkt)}
        self.flow('translate', mid=env['mid'], pkt=pkt, lat=lat, lon=lon, alt=alt,
                  fields=fields_hex(raw), hex=env['hex'], hops=env['hops'], ona=self.letter,
                  refresh=env.get('refresh', False), txt=value_text(pkt))
        self.flow('sat_up', mid=env['mid'], pkt=pkt, ona=self.letter)
        self.beam()
        out = {'kind': 'packet', 'mid': env['mid'], 'hex': env['hex'], 'pkt': pkt,
               'lat': lat, 'lon': lon, 'alt': alt, 'hops': env['hops'], 'path': env['path'],
               'origin': env['origin'], 'ona': self.letter, 'refresh': env.get('refresh', False)}
        self.after(C.SAT_DELAY, lambda: self.pub('/lm/sat/uplink', out))

    def on_wireless(self, m):
        """Outside zone: the ONA nearest to the robot handles its radio message."""
        lat, lon = m.get('lat'), m.get('lon')
        if lat is not None:
            x, y = gps_to_local(lat, lon, *self.anchor)
            near = min(C.ONAS, key=lambda k: dist(C.ONAS[k]['pos'], (x, y)))
            if near != self.letter:
                return
        elif self.letter != 'B':
            return
        k = m.get('kind')
        if k == 'gps_fix':
            self.fixes[m['robot']] = (m['lat'], m['lon'], m['az'])
        self.flow('wireless_in', frm=m.get('robot', m.get('from', '?')), msg=k, ona=self.letter)
        self.flow('sat_up', msg=k, ona=self.letter)
        self.beam()
        out = dict(m, ona=self.letter)
        self.after(C.SAT_DELAY, lambda: self.pub('/lm/sat/uplink', out))

    # ------------------------------------------------------------ energy
    def energy(self):
        """Solar panels -> battery bank -> station load + robot chargers. Robots parked at this
        station (Writers on its charger, Flix on a pad it powers) recharge between missions."""
        E = C.ONA_ENERGY
        hour = (C.MISSION_EPOCH_UTC + self.now()) / 3600.0 % 24.0 + 1.0     # local time (UTC+1)
        sun = max(0.0, math.sin(math.pi * (hour - 6.0) / 13.0))
        solar = 1000.0 * E['solar_kwp'] * sun * 0.8
        self.charging = {}
        for n, p in self.poses.items():
            if not p.get('charging'):
                continue
            here = (p.get('x', 0.0), p.get('y', 0.0))
            near = min(C.ONAS, key=lambda k: dist(C.ONAS[k]['pos'], here))
            if near == self.letter and p.get('battery', 1.0) < 0.999:
                self.charging[n] = E['charger_w']['flix' if n in C.FLIXES else 'ground']
        load = E['base_w'] + sum(self.charging.values())
        self.solar_w, self.load_w = solar, load
        self.soc = max(0.0, min(1.0, self.soc + (solar - load) / (3.6e6 * E['battery_kwh'])))

    # ------------------------------------------------------------ downlink
    def on_sat_down(self, m):
        target = m.get('ona') or ('B' if m.get('kind') not in ('go', 'flix_task', 'lessons')
                                  else None)
        if target is None:
            # outside wireless: the ONA nearest to the addressee's portal / staging area
            target = m.get('via') or 'B'
        if target != self.letter:
            return
        self.after(C.SAT_DELAY, lambda: self.handle_down(m))

    def handle_down(self, m):
        k = m.get('kind')
        self.beam(down=True)
        if k in ('go', 'flix_task', 'lessons'):
            to = m.get('to', 'all')
            self.flow('wireless_out', to=to, msg=k, ona=self.letter)
            self.pub('/lm/wireless/from_ona', m)
            return
        if k == 'entrance_beacon':
            # the station crew lays the fibre to the portal and places the first beacon there
            self.flow('ona_beacon', ona=self.letter, x=C.ONA_C_BEACON[0], y=C.ONA_C_BEACON[1])
            self.pub('/lm/fiber/downlink', dict(m, ona=self.letter))
            return
        if k == 'release_claims':
            self.flow('fiber_down', msg=k, size=4, ona=self.letter)
            self.pub('/lm/fiber/downlink', dict(m, ona=self.letter))
            return
        if k in ('briefing', 'briefing_update'):
            a = self.anchor
            path = [(p['id'],) + gps_to_local(p['lat'], p['lon'], *a) for p in m['path']]
            victim = gps_to_local(m['victim']['lat'], m['victim']['lon'], *a)
            verify = [(v['id'], v['type']) + gps_to_local(v['lat'], v['lon'], *a)
                      for v in m.get('verify', [])]
            off, dyaw = (0.0, 0.0), 0.0          # all robots run in the site frame
            mt = MSG_BRIEF if k == 'briefing' else MSG_BRIEF_UPDATE
            raw = encode_briefing(mt, m['target'], path, victim, verify, off, dyaw,
                                  m.get('conf', 0.0), m.get('radius', 0.0))
            self.flow('translate_down', msg=k, size=len(raw), hex=raw.hex(),
                      path=[p[0] for p in path], ona=self.letter)
            self.flow('fiber_down', msg=k, size=len(raw), ona=self.letter)
            self.pub('/lm/fiber/downlink', {'kind': k, 'hex': raw.hex(), 'size': len(raw),
                                            'ona': self.letter})

    def publish_state(self):
        self.pub('/lm/ona/state', {'ona': self.letter, 'pos': self.pos, 'anchor': self.anchor,
                                   'fixes': self.fixes, 'translated': self.translated,
                                   'last': self.last,
                                   'energy': {'soc': round(self.soc, 4),
                                              'solar_w': round(getattr(self, 'solar_w', 0.0)),
                                              'load_w': round(getattr(self, 'load_w', 0.0)),
                                              'charging': self.charging}})


def main(args=None):
    spin_node(OutsideNetwork, args)


if __name__ == '__main__':
    main()

"""Beacon network: firmware of every dropped beacon + the radio medium between them
(ESP-NOW links and the Wi-Fi CSI sensing channel).

- 32-byte records relayed hop by hop along the dynamic parent chain over ESP-NOW
  (Beacon N -> N-1 -> ... -> entrance beacon -> optical fibre -> its Outside Network Area
  station). The first beacon of every entrance is fibered: A and B (dropped by the Writers),
  C (placed by the ONA-C station crew at the flooded portal). The mesh is one network.
- PERSISTENT SPATIAL MEMORY: every beacon stores the event records written into it (event
  identifier, type, position, timestamp, detection confidence, source) and re-broadcasts them
  with their original timestamp (message aging), so they stay available if their Writer fails.
- Unicast ACK failure (dead / out-of-range parent) -> store-and-forward.
- Self-healing: an orphaned beacon re-links to another routed neighbour, or plans a
  relocation, drives there, reports ROUTE_HEALED + BEACON_LOST and flushes held messages.
- Junction beacons keep the DFS table of their junction (exits: open / claimed / explored /
  seen closed) and flood it to the whole mesh, so the Writers share one topological map.
- "Where to go": every beacon knows the next beacon toward the exit (set by the Writer that
  dropped it) and the next beacon toward the most confident victim deeper in the mine
  (propagated back along the trail when an event is written).
- Wi-Fi CSI presence sensing (independent of the ESP-NOW topology): ESP32-S3 CSI nodes dropped
  by the slow Writers receive Wi-Fi from the beacons and the Writers (sources); each node streams
  its CSI to an aggregator beacon that runs the AI presence classifier every window and writes
  a PRESENCE record (region + confidence). The Flix ESP32-S3 is a flying CSI node: it gets the
  CSI of its links to the nearby sources while it hovers.
- Downlink (briefings, claim releases) from the entrances into the mesh.
- Publishes the RF view of each robot (links, heard beacons + their tables, peers, CSI).
"""
import math
import random

from nav_msgs.msg import Odometry

from . import config as C
from .common import LMNode, OdomTracker, spin_node
from .geometry import (csi_classify, csi_link_ok, csi_link_score, dist, freshness, link_ok,
                       plan_relocation, route_ok, route_to_entrance, rssi, subtree)
from .packet import decode, encode, value_text

MODEL_FOR = {'entrance': 'beacon_entrance', 'victim': 'beacon_victim', 'gas': 'beacon_gas',
             'trail': 'beacon_trail', 'junction': 'beacon_junction'}
EVENT_RANK_GAS = 9
TRACKED = C.GROUND + C.FLIXES
KIND_OF = {C.EV_VICTIM: 'victim', C.EV_PRESENCE: 'presence', C.EV_HAZARD: 'hazard',
           C.EV_VICTIM_CONFIRMED: 'confirmed', C.EV_FALSE_POSITIVE: 'not_found'}


class BeaconNetwork(LMNode):
    def __init__(self):
        super().__init__('beacon_network', publishes=(
            '/lm/fiber/uplink', '/lm/executor/downlink', '/lm/beacons', '/lm/gz/cmd')
            + tuple(f'/lm/rf/{n}' for n in TRACKED))
        self.rng = random.Random(42)
        self.beacons = {}
        self.mid = 0
        self.eids = {}                                 # source id -> next event id (records it creates)
        self.csi = {}                                  # CSI node id -> node
        self.csi_track = {}                            # aggregator beacon -> classifier track
        self.csi_links = []                            # last window's links (2D window)
        self.flix_csi = {}                             # Flix -> its CSI links while hovering
        self._csi_ok = {}
        # radio physics uses the simulator's ground truth positions of the transmitters
        self.trk = {}
        for n in TRACKED:
            self.trk[n] = OdomTracker(C.SPAWN[n])
            self.create_subscription(Odometry, f'/model/{n}/ground_truth', self.trk[n].update, 20)
        self.est = {}                                  # robots' own broadcast estimates (peers)
        for n in TRACKED:
            self.sub(f'/lm/pose/{n}', lambda m, n=n: self.est.__setitem__(n, m))
        self.sub('/lm/writer/drop', self.on_drop)
        self.sub('/lm/writer/junction', self.on_junction)
        self.sub('/lm/writer/uplink', self.on_robot_uplink)
        self.sub('/lm/writer/csi', self.on_csi_drop)
        self.sub('/lm/executor/uplink', self.on_robot_uplink)
        self.sub('/lm/flix/uplink', self.on_robot_uplink)
        self.sub('/lm/fiber/downlink', self.on_fiber_down)
        self.sub('/lm/env/knockout', self.on_knockout)
        self.create_timer(0.2, self.rf_views)
        self.create_timer(0.1, self.rf_executor)
        self.create_timer(0.5, self.supervise)
        self.create_timer(0.2, self.animate)
        self.create_timer(0.5, self.publish_state)
        self.create_timer(1.0, self.refresh_events)
        self.create_timer(C.CSI_WINDOW, self.csi_scan)

    # ------------------------------------------------------------ helpers
    def world_view(self):
        return {k: {'x': b['x'], 'y': b['y'], 'parent': b['parent'], 'alive': b['alive']}
                for k, b in self.beacons.items()}

    def pos(self, bid):
        b = self.beacons[bid]
        return (b['x'], b['y'])

    def next_eid(self, src, csi=False):
        """Event identifier for a record this network creates: infrastructure records (src 0)
        and the records of a Writer's CSI network (128..255, its own records are 1..127)."""
        lo = 128 if csi else 1
        e = self.eids.get((src, csi), lo)
        self.eids[(src, csi)] = lo if e >= lo + 127 else e + 1
        return e

    def new_env(self, pkt, origin, prio=False):
        self.mid += 1
        pkt = dict(pkt)
        if not pkt.get('eid'):
            pkt['eid'] = self.next_eid(pkt.get('src', 0))
        raw = encode(pkt)
        return {'mid': self.mid, 'hex': raw.hex(), 'pkt': decode(raw), 'origin': origin,
                'hops': 0, 'path': [origin], 'prio': prio}

    def routed(self):
        """{beacon id: True if its route to an entrance is live hop by hop}, cached per tick."""
        now = self.now()
        if getattr(self, '_routed_t', None) != now:
            view = self.world_view()
            memo = {}
            self._routed = {k: route_ok(view, k, memo) for k in view}
            self._routed_t = now
        return self._routed

    def root_ona(self, bid):
        r = route_to_entrance(self.world_view(), bid)
        if not r:
            return None
        return self.beacons[r[-1]].get('ona')

    def best_entry(self, wx, wy):
        """Strongest beacon in range of (wx, wy): one with a live route first; otherwise one
        with a pointer route (its messages are then held there until the chain heals)."""
        view = self.world_view()
        live = self.routed()
        best = None
        for k, b in self.beacons.items():
            if not b['alive'] or b['relocating']:
                continue
            if route_to_entrance(view, k) is None:
                continue
            if link_ok((wx, wy), (b['x'], b['y'])):
                r = (live.get(k, False), rssi(dist((wx, wy), (b['x'], b['y']))))
                if best is None or r > best[1]:
                    best = (k, r)
        return None if best is None else best[0]

    def new_beacon(self, bid, **kw):
        b = {'id': bid, 'btype': 'trail', 'owner': '', 'x': 0.0, 'y': 0.0, 'mx': 0.0, 'my': 0.0,
             'mz': 0.0, 'parent': C.PARENT_ONA, 'alive': True, 'relocating': False,
             't_drop': self.now(), 'held': [], 'orphan_since': None, 'healing': False,
             'target': None, 'next_try': 0.0, 'relocated': False, 'events': [],
             'last_refresh': self.now(), 'next_out': 0, 'next_in': 0, 'bearing_in': 0.0,
             'dist_in': 0.0, 'in_event': None, 'junction': None, 'ona': None,
             'can_move': True, 'heading': 0.0}
        b.update(kw)
        self.beacons[bid] = b
        return b

    # ------------------------------------------------------------ drops
    def on_drop(self, m):
        bid = m['id']
        btype = m['btype']
        b = self.new_beacon(
            bid, btype=btype, owner=m.get('from', ''), x=m['wx'], y=m['wy'], mx=m['mx'],
            my=m['my'], parent=m['parent'], t_drop=m['t_ms'] / 1000.0,
            next_out=m.get('next_out', 0), heading=m['heading'],
            ona=m.get('entrance') if btype == 'entrance' else None,
            can_move=btype != 'entrance', last_refresh=m['t_ms'] / 1000.0)
        if m.get('junction'):
            b['junction'] = {k: dict(e) for k, e in m['junction'].items()}
            b['links'] = {}
        self.pub('/lm/gz/cmd', {'op': 'spawn', 'name': f'beacon_{bid}',
                                'model': MODEL_FOR.get(btype, 'beacon_trail'),
                                'x': m['wx'], 'y': m['wy'], 'z': 0.01,
                                'yaw': math.radians(m['heading'])})
        self.flow('drop', id=bid, btype=btype, parent=m['parent'], x=m['wx'], y=m['wy'],
                  mx=m['mx'], my=m['my'], by=m.get('from', ''), next_out=b['next_out'],
                  stock=m.get('stock'))
        for p in m['packets']:
            if p['type'] in KIND_OF:
                self.store_event(bid, p)
        for i, p in enumerate(m['packets']):
            p = dict(p)
            p.update(self.wtg_fields(bid))
            self.after(0.15 * i + 0.05, lambda p=p, bid=bid: self.originate(bid, p))

    def place_ona_beacon(self, m):
        """ONA-C station crew: the station's fibre ends in its own first beacon, placed at the
        flooded portal C (no Writer enters there)."""
        bid = C.ONA_BEACON_ID
        if bid in self.beacons:
            return
        x, y = C.ONA_C_BEACON
        self.new_beacon(bid, btype='entrance', owner='ONA-C crew', x=x, y=y, mx=x, my=y,
                        parent=C.PARENT_ONA, ona=m.get('entrance', 'C'), can_move=False)
        self.pub('/lm/gz/cmd', {'op': 'spawn', 'name': f'beacon_{bid}', 'model': 'beacon_entrance',
                                'x': x, 'y': y, 'z': 0.01, 'yaw': 0.0})
        self.flow('drop', id=bid, btype='entrance', parent=C.PARENT_ONA, x=x, y=y, mx=x, my=y,
                  by='ONA-C crew', next_out=0)
        self.after(0.2, lambda: self.originate(bid, {
            'id': bid, 'type': C.EV_ENTRANCE, 'src': C.SRC_ID['ona'], 'x': x, 'y': y,
            'heading': 0.0, 't_ms': self.now_ms()}))

    # ------------------------------------------------------------ stored events (spatial memory)
    def store_event(self, bid, pkt):
        """A beacon keeps the event records written into it (re-broadcast with their original
        timestamp). The same event (source, event id) updated -> replaced, not duplicated."""
        b = self.beacons.get(bid)
        kind = KIND_OF.get(pkt['type'])
        if b is None or kind is None:
            return
        pkt = dict(pkt)
        key = (pkt.get('src', 0), pkt.get('eid', 0), pkt['type'])
        rec = {'kind': kind, 'x': pkt['x'], 'y': pkt['y'], 'conf': pkt.get('conf', 0.0),
               'r': float(pkt.get('a', 0)) if kind == 'presence' else 0.0, 'pkt': pkt, 'key': key,
               'no_ground': bool(pkt.get('flags', 0) & C.FL_NO_GROUND)}
        b['events'] = [e for e in b['events'] if e['key'] != key] + [rec]
        b['events'] = b['events'][-6:]
        self.update_where_to_go()

    def originate(self, bid, pkt, refresh=False, prio=False):
        if bid not in self.beacons or not self.beacons[bid]['alive']:
            return
        pkt = dict(pkt)
        pkt['parent'] = self.beacons[bid]['parent']
        env = self.new_env(pkt, bid, prio)
        env['refresh'] = refresh
        self.flow('tx', at=bid, mid=env['mid'], hex=env['hex'], pkt=env['pkt'], refresh=refresh,
                  txt=value_text(env['pkt']))
        self.forward(env, bid)

    def on_robot_uplink(self, m):
        who = m.get('from', 'robot')
        tr = self.trk.get(who)
        wx, wy = (tr.x, tr.y) if tr is not None and tr.ok else (m['wx'], m['wy'])
        entry = self.best_entry(wx, wy)
        pkt = dict(m['pkt'])
        if entry is None:
            tries = m.get('_tries', 0)
            if tries % 30 == 0:           # log the first failure, then every 30 s
                self.flow('rf_fail', frm=who, x=wx, y=wy)
                self.get_logger().warn(f'{who} uplink: no beacon in range, retrying every 1 s')
            m = dict(m, _tries=tries + 1)
            self.after(1.0, lambda: self.on_robot_uplink(m))
            return
        pkt['parent'] = entry
        env = self.new_env(pkt, who)
        self.flow('rf_up', frm=who, to=entry, x=wx, y=wy, mid=env['mid'],
                  hex=env['hex'], pkt=env['pkt'], txt=value_text(env['pkt']))
        if pkt['type'] in KIND_OF:
            # the record is written into a beacon: the one it is about if the robot is by it,
            # otherwise the beacon that received it
            sub = pkt['id']
            store = sub if (sub in self.beacons and self.beacons[sub]['alive']
                            and link_ok((wx, wy), self.pos(sub))) else entry
            self.store_event(store, env['pkt'])
        self.after(C.HOP_DELAY, lambda: self.forward(env, entry))

    # ------------------------------------------------------------ junction tables (DFS)
    def on_junction(self, m):
        """A Writer wrote / updated the DFS table of a junction beacon."""
        b = self.beacons.get(m['id'])
        if b is None:
            return
        old = b.get('junction') or {}
        rank = {'open': 0, 'claimed': 1, 'seen': 2, 'done': 2}
        new = {k: dict(e) for k, e in old.items()}
        who = m.get('from')
        for k, e in m['exits'].items():
            prev = old.get(k)
            if prev is None or m.get('release') or e.get('by') == who and prev.get('by') in (who, '') \
                    or rank.get(e.get('st'), 0) > rank.get(prev.get('st'), 0):
                new[k] = dict(e)
        b['junction'] = new
        links = dict(b.get('links') or {})
        links.update({k: v for k, v in (m.get('links') or {}).items()})
        b['links'] = links
        self.flow('junction', id=m['id'], by=m.get('from'), exits=new, x=b['x'], y=b['y'],
                  claim=m.get('claim'))
        # JUNCTION packet: exits E, N, W, S packed 2 bits each (0 rock, 1 open, 2 claimed,
        # 3 explored / seen closed) + which exits are claimed by Writer A (bits) in valueB
        code, by_a = 0, 0
        for n, deg in enumerate((0, 90, 180, -90)):
            e = new.get(str(deg))
            v = 0 if e is None else {'open': 1, 'claimed': 2}.get(e['st'], 3)
            code |= v << (2 * n)
            if e and e.get('by') == 'writer_a' and e['st'] == 'claimed':
                by_a |= 1 << n
        self.originate(b['id'], {'id': b['id'], 'type': C.EV_JUNCTION,
                                 'src': C.SRC_ID.get(m.get('from'), 0), 'eid': m.get('eid', 0),
                                 'x': b['mx'], 'y': b['my'], 'a': code, 'b': by_a,
                                 't_ms': self.now_ms(), **self.wtg_fields(b['id'])})

    def release_claims(self, robot):
        """Server order (through an ONA): a lost Writer's claimed exits become open again."""
        n = 0
        for b in self.beacons.values():
            for k, e in (b.get('junction') or {}).items():
                if e.get('st') == 'claimed' and e.get('by') == robot:
                    e['st'] = 'open'
                    e['by'] = ''
                    n += 1
        self.flow('release', robot=robot, n=n)
        self.get_logger().info(f'claims of {robot} released ({n} exits open again)')

    def mesh_junctions(self):
        """Junction tables flooded through the mesh (ESP-NOW broadcast, ~1 s)."""
        out = []
        for k, b in self.beacons.items():
            if b['alive'] and b.get('junction'):
                out.append({'id': k, 'mx': b['mx'], 'my': b['my'], 'exits': b['junction'],
                            'links': b.get('links') or {}, 'owner': b['owner']})
        return out

    # ------------------------------------------------------------ "where to go"
    def best_event(self, b):
        """The event this beacon points the trail to: the most confident victim record (camera
        or CSI) stored in it, else a hazard. (rank, x, y): lower rank = more urgent."""
        best = None
        for e in b.get('events') or []:
            if e['kind'] in ('victim', 'presence'):
                r = 1.0 - e['conf']
            elif e['kind'] == 'hazard':
                r = EVENT_RANK_GAS
            else:
                continue
            if best is None or r < best[0]:
                best = (r, e['x'], e['y'])
        return best

    def update_where_to_go(self):
        """Every beacon points to the most urgent event deeper along the trail (nextIn) - the
        trail tree is the 'nextOut' (toward the exit) pointers set by the Writers."""
        kids = {}
        for k, b in self.beacons.items():
            if b['alive'] and b['next_out'] and b['next_out'] in self.beacons:
                kids.setdefault(b['next_out'], []).append(k)
        memo = {}

        def best(k, depth=0):
            if k in memo:
                return memo[k]
            b = self.beacons[k]
            cand = None
            ev = self.best_event(b) if b['alive'] else None
            if ev is not None:
                r, ex, ey = ev
                cand = (round(r, 1), math.hypot(ex - b['mx'], ey - b['my']), 0,
                        math.degrees(math.atan2(ey - b['my'], ex - b['mx'])))
            if depth < 60:
                for c in kids.get(k, []):
                    sub = best(c, depth + 1)
                    if sub is None:
                        continue
                    cb = self.beacons[c]
                    d = math.hypot(cb['mx'] - b['mx'], cb['my'] - b['my'])
                    s = (sub[0], sub[1] + d, c, math.degrees(math.atan2(cb['my'] - b['my'],
                                                                        cb['mx'] - b['mx'])))
                    if cand is None or s[:2] < cand[:2]:
                        cand = s
            memo[k] = cand
            return cand
        for k, b in self.beacons.items():
            r = best(k)
            if r is None:
                b['next_in'], b['bearing_in'], b['dist_in'], b['in_rank'] = 0, 0.0, 0.0, None
            else:
                b['in_rank'], b['dist_in'], b['next_in'], b['bearing_in'] = r[0], r[1], r[2], r[3]

    def wtg_fields(self, bid):
        b = self.beacons.get(bid)
        if b is None:
            return {}
        return {'next_in': b['next_in'], 'bearing_in': b['bearing_in'], 'dist_in': b['dist_in'],
                'next_out': b['next_out'], 'z': b.get('mz', 0.0)}

    # ------------------------------------------------------------ uplink relay
    def forward(self, env, at):
        b = self.beacons.get(at)
        if b is None or not b['alive']:
            self.flow('lost', at=at, mid=env['mid'])
            return
        if b['relocating']:
            self.hold(env, at)
            return
        parent = b['parent']
        hop = C.HOP_DELAY / 3 if env.get('prio') else C.HOP_DELAY
        if parent == C.PARENT_ONA:
            ona = b.get('ona') or 'B'
            self.flow('fiber_up', frm=at, mid=env['mid'], hex=env['hex'], pkt=env['pkt'],
                      hops=env['hops'], ona=ona)
            env2 = dict(env, ona=ona)
            env2['path'] = env['path'] + [f'ONA-{ona}']
            self.after(C.FIBER_DELAY, lambda: self.pub('/lm/fiber/uplink', env2))
            return
        p = self.beacons.get(parent)
        if p is None or not p['alive'] or not link_ok(self.pos(at), self.pos(parent)):
            self.hold(env, at)
            return
        self.flow('hop', frm=at, to=parent, mid=env['mid'], hex=env['hex'], pkt=env['pkt'],
                  prio=env.get('prio', False))

        def arrive():
            q = self.beacons.get(parent)
            if q is None or not q['alive']:
                self.flow('ack_fail', frm=at, to=parent, mid=env['mid'])
                self.hold(env, at)
                return
            env2 = dict(env)
            env2['hops'] = env['hops'] + 1
            env2['path'] = env['path'] + [parent]
            self.forward(env2, parent)
        self.after(hop, arrive)

    def hold(self, env, at):
        b = self.beacons[at]
        b['held'].append(env)
        self.flow('hold', at=at, mid=env['mid'], n=len(b['held']))

    def flush(self, bid, k=0):
        b = self.beacons[bid]
        envs, b['held'] = b['held'], []
        for env in envs:
            pkt = dict(env['pkt'])
            pkt['flags'] = pkt['flags'] | C.FL_STORED_FWD
            raw = encode(pkt)
            env2 = dict(env)
            env2['hex'] = raw.hex()
            env2['pkt'] = decode(raw)
            self.flow('flush', at=bid, mid=env['mid'])
            self.after(0.1 + 0.15 * k, lambda e=env2, a=bid: self.forward(e, a))
            k += 1
        return k

    def flush_all(self):
        k = 0
        for bid, b in self.beacons.items():
            if b['alive'] and b['held']:
                k = self.flush(bid, k)

    # ------------------------------------------------------------ downlink
    def on_fiber_down(self, m):
        k = m.get('kind')
        if k == 'release_claims':
            self.after(C.FIBER_DELAY, lambda: self.release_claims(m['robot']))
            return
        if k == 'entrance_beacon':
            self.after(C.FIBER_DELAY, lambda: self.place_ona_beacon(m))
            return
        self.after(C.FIBER_DELAY, lambda: self.route_down(m))

    def route_down(self, m):
        ex = self.trk['executor']
        if ex.ok and ex.x < -0.5:
            # Executor still outside (staging area): the ONA's outside radio reaches it directly
            self.flow('wireless_out', to='executor', msg=m.get('kind'), ona=m.get('ona', 'B'))
            self.after(C.HOP_DELAY, lambda: self.pub('/lm/executor/downlink', m))
            return
        entry = self.best_entry(ex.x, ex.y)
        if entry is None:
            self.flow('rf_fail', frm='network', to='executor')
            self.after(2.0, lambda: self.route_down(m))
            return
        path = list(reversed(route_to_entrance(self.world_view(), entry)))
        self.mid += 1
        mid = self.mid
        ona = self.beacons[path[0]].get('ona') or 'B'

        def step(i):
            if i + 1 < len(path):
                a, b = path[i], path[i + 1]
                if not self.beacons[b]['alive'] or self.beacons[b]['relocating']:
                    self.after(1.0, lambda: step(i))
                    return
                self.flow('hop_down', frm=a, to=b, mid=mid, msg=m.get('kind'))
                self.after(C.HOP_DELAY, lambda: step(i + 1))
            else:
                self.flow('rf_down', frm=path[i], to='executor', mid=mid, msg=m.get('kind'),
                          hex=m.get('hex'))
                self.after(C.HOP_DELAY, lambda: self.pub('/lm/executor/downlink', m))
        self.flow('fiber_down_in', to=path[0], mid=mid, msg=m.get('kind'), ona=ona)
        step(0)

    # ------------------------------------------------------------ failures / healing
    def on_knockout(self, m):
        bid = m['id']
        b = self.beacons.get(bid)
        if b is None or not b['alive']:
            return
        b['alive'] = False
        lost = len(b['held'])
        b['held'] = []
        self.flow('beacon_dead', id=bid, x=b['x'], y=b['y'], lost=lost)
        self.get_logger().warn(f'B{bid} destroyed by rockfall')
        # the trail's "toward the exit" pointers bypass the lost beacon (its neighbours knew
        # where it pointed), so the escape route stays continuous
        for k, q in self.beacons.items():
            if q['alive'] and q['next_out'] == bid:
                q['next_out'] = b['next_out']
        self.update_where_to_go()

    def supervise(self):
        now = self.now()
        live = self.routed()
        for bid, b in list(self.beacons.items()):
            if b['alive'] and b['held'] and not b['relocating'] and live.get(bid):
                self.flush(bid)            # store-and-forward: the route is live again
        for bid, b in list(self.beacons.items()):
            if not b['alive'] or b['relocating'] or b['parent'] == C.PARENT_ONA:
                continue
            p = self.beacons.get(b['parent'])
            if p is not None and p['alive']:
                if not p['relocating'] and not link_ok(self.pos(bid), self.pos(b['parent'])):
                    # parent alive but its link is lost (range / geometry): dynamic routing picks
                    # another routed neighbour in range, otherwise the beacon relocates
                    if b.get('link_lost_since') is None:
                        b['link_lost_since'] = now
                        self.flow('heartbeat_lost', at=bid, parent=b['parent'])
                    elif now - b['link_lost_since'] >= C.HEARTBEAT_TIMEOUT and now >= b['next_try']:
                        if not self.reparent(bid):
                            self.start_heal(bid, b['parent'])
                    continue
                b['link_lost_since'] = None
                b['orphan_since'] = None
                continue
            if b['orphan_since'] is None:
                b['orphan_since'] = now
                self.flow('heartbeat_lost', at=bid, parent=b['parent'])
                continue
            if now - b['orphan_since'] >= C.HEARTBEAT_TIMEOUT and now >= b['next_try']:
                if not self.reparent(bid, dead=True):
                    self.start_heal(bid, b['parent'])

    def reparent(self, bid, dead=False):
        """Re-link to the strongest routed neighbour in radio range (no move needed)."""
        b = self.beacons[bid]
        view = self.world_view()
        sub = subtree(view, bid)
        live = self.routed()
        best = None
        for k, q in self.beacons.items():
            if k in sub or not q['alive'] or q['relocating'] or k == b['parent']:
                continue
            if not live.get(k) or not link_ok(self.pos(bid), self.pos(k)):
                continue
            r = rssi(dist(self.pos(bid), self.pos(k)))
            if best is None or r > best[1]:
                best = (k, r)
        if best is None:
            if not b['can_move']:
                b['next_try'] = self.now() + 3.0       # fixed beacon: wait for a new neighbour
            return False if b['can_move'] else True
        old = b['parent']
        b['dead_parent'] = old
        b['new_parent'] = best[0]
        b['target'] = None
        b['link_lost_since'] = None
        b['orphan_since'] = None
        self.get_logger().info(f'B{bid}: link to B{old} lost -> re-linked to B{best[0]}')
        b['parent'] = best[0]
        self.flow('relinked', at=bid, parent=best[0], x=b['x'], y=b['y'])
        self.originate(bid, {'id': bid, 'type': C.EV_ROUTE_HEALED, 'x': b['mx'],
                             'y': b['my'], 't_ms': self.now_ms(), **self.wtg_fields(bid)})
        if dead and old in self.beacons and not self.beacons[old]['alive']:
            self.after(0.15, lambda: self.originate(bid, {
                'id': old, 'type': C.EV_BEACON_LOST, 'x': self.beacons[old]['mx'],
                'y': self.beacons[old]['my'], 't_ms': self.now_ms()}))
        self.after(0.4, self.flush_all)
        return True

    def start_heal(self, bid, dead_id):
        b = self.beacons[bid]
        if not b['can_move']:
            b['next_try'] = self.now() + 3.0
            return
        plan = plan_relocation(self.world_view(), bid, dead_id)
        if plan is None:
            b['next_try'] = self.now() + 5.0
            self.flow('heal_failed', at=bid, dead=dead_id)
            self.get_logger().error(f'B{bid}: no relocation restores the chain')
            return
        x, y, new_parent, move = plan
        b['dead_parent'] = dead_id
        b['new_parent'] = new_parent
        if move < 0.05:
            self.flow('heal_plan', at=bid, dead=dead_id, new_parent=new_parent, x=x, y=y, move=0.0)
            self.finish_heal(bid)
            return
        b['relocating'] = True
        b['target'] = (x, y)
        self.flow('heal_plan', at=bid, dead=dead_id, new_parent=new_parent, x=x, y=y,
                  move=round(move, 2))
        self.get_logger().info(f'B{bid}: relocating {move:.1f} m to re-link with B{new_parent}')

    def animate(self):
        step = C.RELOCATE_SPEED * 0.2
        for bid, b in self.beacons.items():
            if not b['relocating']:
                continue
            tx, ty = b['target']
            d = dist((b['x'], b['y']), (tx, ty))
            yaw = math.atan2(ty - b['y'], tx - b['x']) if d > 1e-3 else b.get('yaw_move', 0.0)
            b['yaw_move'] = yaw                  # the beacon faces its direction of travel
            if d <= step:
                nx, ny = tx, ty
            else:
                nx = b['x'] + (tx - b['x']) * step / d
                ny = b['y'] + (ty - b['y']) * step / d
            b['mx'] += nx - b['x']
            b['my'] += ny - b['y']
            b['x'], b['y'] = nx, ny
            self.pub('/lm/gz/cmd', {'op': 'set_pose', 'name': f'beacon_{bid}',
                                    'x': nx, 'y': ny, 'z': 0.01, 'yaw': yaw})
            if d <= step:
                b['relocating'] = False
                self.finish_heal(bid)

    def finish_heal(self, bid):
        b = self.beacons[bid]
        b['parent'] = b['new_parent']
        b['orphan_since'] = None
        b['relocated'] = b['relocated'] or b.get('target') is not None
        self.flow('relinked', at=bid, parent=b['parent'], x=b['x'], y=b['y'])
        t = self.now_ms()
        fl = C.FL_RELOCATED if b.get('target') is not None else 0
        self.originate(bid, {'id': bid, 'type': C.EV_ROUTE_HEALED, 'flags': fl, 'x': b['mx'],
                             'y': b['my'], 't_ms': t, **self.wtg_fields(bid)})
        b['link_lost_since'] = None
        dead = self.beacons.get(b['dead_parent'])
        if dead is not None and not dead['alive']:      # only a destroyed parent is 'lost'
            self.after(0.15, lambda: self.originate(bid, {
                'id': b['dead_parent'], 'type': C.EV_BEACON_LOST,
                'x': dead['mx'], 'y': dead['my'], 't_ms': t}))
        self.after(0.4, self.flush_all)

    def refresh_events(self):
        """Message aging: beacons re-broadcast every stored event record every EVENT_REFRESH s
        with the ORIGINAL timestamp and event identifier, so every receiver can compute the
        event's age and recognise it; the 'where to go' fields are the beacon's current ones."""
        now = self.now()
        for bid, b in self.beacons.items():
            if b['alive'] and b['events'] and now - b['last_refresh'] >= C.EVENT_REFRESH:
                b['last_refresh'] = now
                for i, e in enumerate(b['events']):
                    p = dict(e['pkt'])
                    p['id'] = bid if p['id'] >= 200 else p['id']
                    p.update(self.wtg_fields(bid))
                    self.after(0.15 * i, lambda p=p, bid=bid: self.originate(bid, p, refresh=True))

    # ------------------------------------------------------------ Wi-Fi CSI sensing
    def on_csi_drop(self, m):
        """A slow Writer dropped an ESP32-S3 CSI node: it streams its CSI to the nearest beacon in
        Wi-Fi reach (the cluster's aggregator, which runs the AI classifier)."""
        nid = m['id']
        n = {'id': nid, 'x': m['wx'], 'y': m['wy'], 'mx': m['mx'], 'my': m['my'],
             'owner': m['from'], 'agg': None, 't_drop': m['t_ms'] / 1000.0, 'why': m.get('why', '')}
        self.csi[nid] = n
        self.pub('/lm/gz/cmd', {'op': 'spawn', 'name': f'csi_{nid}', 'model': 'csi_node',
                                'x': m['wx'], 'y': m['wy'], 'z': 0.01, 'yaw': m.get('yaw', 0.0)})
        self.attach_csi(n)
        self.flow('csi_drop', id=nid, x=m['wx'], y=m['wy'], mx=m['mx'], my=m['my'], by=m['from'],
                  agg=n['agg'], stock=m.get('stock'), why=m.get('why', ''))

    def attach_csi(self, n):
        best = None
        for k, b in self.beacons.items():
            if not b['alive'] or b['relocating']:
                continue
            d = dist((n['x'], n['y']), self.pos(k))
            if d <= C.CSI_AGG_R and link_ok((n['x'], n['y']), self.pos(k)) and \
                    (best is None or d < best[0]):
                best = (d, k)
        if best is None:
            return
        n['agg'] = best[1]
        size = sum(1 for q in self.csi.values() if q['agg'] == best[1])
        src = C.SRC_ID.get(n['owner'], 0)
        self.originate(best[1], {'id': best[1], 'type': C.EV_CSI_NODE, 'src': src,
                                 'eid': self.next_eid(src, csi=True), 'x': n['mx'], 'y': n['my'],
                                 'a': n['id'], 'b': size, 'conf': 1.0, 't_ms': self.now_ms()})

    def sources(self):
        """Wi-Fi sources of the CSI network: every live relay beacon and the Writers."""
        out = [(f'B{k}', self.pos(k), (b['mx'], b['my'])) for k, b in self.beacons.items()
               if b['alive'] and not b['relocating']]
        for w in C.WRITERS:
            tr, e = self.trk[w], self.est.get(w) or {}
            if tr.ok and tr.x > 0.0 and e.get('state') not in ('destroyed', None):
                out.append((w, (tr.x, tr.y), (e.get('ex', tr.x), e.get('ey', tr.y))))
        return out

    def link_scores(self, rx, srcs):
        """CSI links from the sources to a receiver at rx (truth): [(name, src_map_xy, score)]."""
        out = []
        for name, p, pm in srcs:
            if dist(p, rx) > C.CSI_LINK_R:
                continue
            key = (name, round(p[0], 1), round(p[1], 1), round(rx[0], 1), round(rx[1], 1))
            ok = self._csi_ok.get(key)
            if ok is None:
                ok = csi_link_ok(p, rx)
                if len(self._csi_ok) > 20000:
                    self._csi_ok.clear()
                self._csi_ok[key] = ok
            if ok:
                out.append((name, p, pm, csi_link_score(p, rx, C.VICTIMS, self.rng)))
        return out

    def csi_scan(self):
        """One CSI window: every aggregator classifies the CSI of its nodes' links; two windows
        in a row above the threshold -> PRESENCE record (region + confidence) in that beacon."""
        srcs = self.sources()
        links_view = []
        for n in self.csi.values():
            if n['agg'] is None or not self.beacons.get(n['agg'], {}).get('alive', False):
                n['agg'] = None
                self.attach_csi(n)
        by_agg = {}
        for n in self.csi.values():
            if n['agg'] is not None:
                by_agg.setdefault(n['agg'], []).append(n)
        for agg, nodes in by_agg.items():
            fired = []
            scores = []
            for n in nodes:
                for name, p, pm, sc in self.link_scores((n['x'], n['y']), srcs):
                    scores.append(sc)
                    links_view.append([round(p[0], 2), round(p[1], 2), round(n['x'], 2),
                                       round(n['y'], 2), round(sc, 2)])
                    if sc > 0.25:
                        fired.append((sc, pm, (n['mx'], n['my'])))
            p = csi_classify(scores)
            tr = self.csi_track.setdefault(agg, {'n': 0, 'p': 0.0, 'reported': 0.0, 'eid': None,
                                                 'windows': []})
            tr['p'] = p
            tr['windows'] = (tr['windows'] + [round(p, 2)])[-6:]
            tr['n'] = tr['n'] + 1 if p >= C.CSI_THRESHOLD else 0
            if tr['n'] < C.CSI_CONFIRM or not fired or p < tr['reported'] + 0.1:
                continue
            # region = where the disturbed links are (Phase 1: the region, not the position)
            wsum = sum(f[0] for f in fired)
            cx = sum(f[0] * (f[1][0] + f[2][0]) / 2 for f in fired) / wsum
            cy = sum(f[0] * (f[1][1] + f[2][1]) / 2 for f in fired) / wsum
            r = max([C.CSI_REGION_R] + [math.hypot(f[1][0] - f[2][0], f[1][1] - f[2][1]) / 2
                                        for f in fired])
            owner = nodes[0]['owner']
            src = C.SRC_ID.get(owner, 0)
            if tr['eid'] is None:
                tr['eid'] = self.next_eid(src, csi=True)
            tr['reported'] = p
            pkt = {'id': agg, 'type': C.EV_PRESENCE, 'src': src, 'eid': tr['eid'],
                   'x': cx, 'y': cy, 'a': min(255, round(r)), 'b': len(fired), 'conf': p,
                   't_ms': self.now_ms()}
            self.flow('presence', at=agg, x=cx, y=cy, r=round(r, 1), conf=round(p, 2),
                      links=len(fired), owner=owner, nodes=[n['id'] for n in nodes])
            self.get_logger().info(f'B{agg} CSI classifier: presence {p:.0%} in a {r:.0f} m region '
                                   f'around ({cx:.1f},{cy:.1f}) - {len(fired)} links')
            self.store_event(agg, decode(encode(pkt)))
            self.originate(agg, pkt)
            ax, ay = self.pos(agg)
            self.pub('/lm/gz/cmd', {'op': 'pulse', 'kind': 'csi', 'x': ax, 'y': ay, 'z': 0.3})
        # the Flix as a flying CSI node: its links to the sources while it hovers
        for f in C.FLIXES:
            tr = self.trk[f]
            e = self.est.get(f) or {}
            if not tr.ok or tr.x <= 0.5 or e.get('state') not in ('escort', 'to_victim', 'inspect_c'):
                self.flix_csi[f] = {}
                continue
            srcs_f = [s for s in srcs if s[0].startswith('B') or s[0] == C.PAIR[f]]
            out = []
            for name, p, pm, sc in self.link_scores((tr.x, tr.y), srcs_f):
                out.append({'src': name, 'mx': round(pm[0], 2), 'my': round(pm[1], 2),
                            'score': round(sc, 3)})
                links_view.append([round(p[0], 2), round(p[1], 2), round(tr.x, 2), round(tr.y, 2),
                                   round(sc, 2)])
            self.flix_csi[f] = {'t': round(self.now(), 2), 'links': out}
        self.csi_links = links_view[-120:]

    # ------------------------------------------------------------ RF views
    def heard_list(self, w):
        live = self.routed()
        links = []
        for k, b in self.beacons.items():
            if not b['alive']:
                continue
            p = (b['x'], b['y'])
            if link_ok(w, p):
                # heartbeat content: id, stored coordinates, route state, whether the beacon has
                # moved itself, its trail pointers and (junction beacons) its DFS table
                links.append({'id': k, 'rssi': round(rssi(dist(w, p)), 2), 'link': True,
                              'routed': bool(live.get(k)), 'mx': b['mx'], 'my': b['my'],
                              'btype': b['btype'], 'owner': b['owner'],
                              'moved': bool(b['relocated'] or b['relocating']),
                              'next_out': b['next_out'], 'next_in': b['next_in'],
                              'junction': b.get('junction')})
        return links

    def peers(self, name, w):
        out = []
        for n, tr in self.trk.items():
            if n == name or not tr.ok or n in C.FLIXES:
                continue
            e = self.est.get(n)
            if e is None or not link_ok(w, (tr.x, tr.y)):
                continue
            out.append({'name': n, 'ex': e.get('ex'), 'ey': e.get('ey'), 'eyaw': e.get('eyaw'),
                        'state': e.get('state')})
        return out

    def mesh_marks(self):
        """Victim records already in the beacons (camera finds and CSI presence regions), flooded
        through the mesh: robots do not report the same person twice, and the Writers' utility
        function steers toward people seen only by a Flix or only by CSI."""
        out = []
        for k, b in self.beacons.items():
            if not b['alive']:
                continue
            for e in b['events']:
                if e['kind'] in ('victim', 'presence', 'confirmed'):
                    out.append({'id': k, 'kind': e['kind'], 'x': e['x'], 'y': e['y'], 'r': e['r'],
                                'conf': e['conf'], 'src': e['pkt'].get('src', 0),
                                'no_ground': e['no_ground'], 'at_victim': b['btype'] == 'victim'})
        return out

    def rf_views(self):
        mj = self.mesh_junctions()
        marks = self.mesh_marks()
        for n in C.WRITERS + C.FLIXES:
            tr = self.trk[n]
            if not tr.ok:
                continue
            w = (tr.x, tr.y)
            heard = self.heard_list(w)
            self.pub(f'/lm/rf/{n}', {'links': heard, 'peers': self.peers(n, w),
                                     'mesh': mj if n in C.WRITERS else [],
                                     'marks': marks if heard else [],
                                     'csi': self.flix_csi.get(n, {}) if n in C.FLIXES else {}})

    def rf_executor(self):
        tr = self.trk['executor']
        if not tr.ok:
            return
        ex, ey, yaw = tr.x, tr.y, tr.yaw
        o = C.EXEC_ANTENNA_OFFSET
        c, s = math.cos(yaw), math.sin(yaw)
        pts = {'c': (ex, ey), 'f': (ex + o * c, ey + o * s), 'b': (ex - o * c, ey - o * s),
               'l': (ex - o * s, ey + o * c), 'r': (ex + o * s, ey - o * c)}
        now = self.now()
        out = []
        for k, b in self.beacons.items():
            if not b['alive']:
                continue
            p = (b['x'], b['y'])
            if not link_ok((ex, ey), p):
                continue
            e = {'id': k, 'fresh': round(freshness(now - b['t_drop']), 4), 'mx': b['mx'],
                 'my': b['my'], 'next_in': b['next_in'], 'dist_in': b['dist_in'],
                 'next_out': b['next_out']}
            for key, q in pts.items():
                e[key] = rssi(dist(q, p)) + self.rng.gauss(0.0, C.RF_NOISE_DB)
            out.append(e)
        self.pub('/lm/rf/executor', {'beacons': out, 'peers': self.peers('executor', (ex, ey))})

    def publish_state(self):
        now = self.now()
        live = self.routed()
        out = []
        for k, b in sorted(self.beacons.items()):
            out.append({'id': k, 'btype': b['btype'], 'owner': b['owner'],
                        'x': round(b['x'], 3), 'y': round(b['y'], 3),
                        'mx': round(b['mx'], 3), 'my': round(b['my'], 3), 'parent': b['parent'],
                        'alive': b['alive'], 'relocating': b['relocating'],
                        'relocated': b['relocated'], 'ona': b.get('ona'),
                        'orphan': b['orphan_since'] is not None, 'held': len(b['held']),
                        'routed': bool(live.get(k)), 'next_out': b['next_out'],
                        'next_in': b['next_in'], 'bearing_in': round(b['bearing_in'], 1),
                        'dist_in': round(b['dist_in'], 1), 'junction': b.get('junction'),
                        'events': [{'kind': e['kind'], 'conf': round(e['conf'], 2),
                                    'x': round(e['x'], 2), 'y': round(e['y'], 2), 'r': e['r'],
                                    'src': e['pkt'].get('src', 0), 'eid': e['pkt'].get('eid', 0)}
                                   for e in b['events']],
                        'csi_agg': sorted(n['id'] for n in self.csi.values() if n['agg'] == k),
                        'fresh': round(freshness(now - b['t_drop']), 4),
                        'target': b['target'] if b['relocating'] else None})
        csi = [{'id': n['id'], 'x': round(n['x'], 2), 'y': round(n['y'], 2), 'agg': n['agg'],
                'owner': n['owner'], 'p': round(self.csi_track.get(n['agg'], {}).get('p', 0.0), 2)}
               for n in self.csi.values()]
        self.pub('/lm/beacons', {'beacons': out, 't': now, 'csi': csi, 'csi_links': self.csi_links,
                                 'flix_csi': self.flix_csi})


def main(args=None):
    spin_node(BeaconNetwork, args)


if __name__ == '__main__':
    main()

"""Writer (slow ground writer, Robotika X2) - explores the UNKNOWN mine on its own.

No map and no route are given: the Writer only knows the portal it was assigned (found by
Flix) and the miners' last known position (cap-lamp tag). It localises with SLAM (encoders,
gyro, visual odometry, lidar scan matching, gallery lock, loop closures on its beacons) and runs
the exploration loop of the technical report:

    Sense -> Localize -> Map -> Identify frontiers -> Evaluate -> Select -> Navigate

  * gallery travel : follows the gallery axis, centred by the lidar;
  * junction       : side openings seen by the lidar for several scans -> it creeps to the
                     side gallery's centre line, then reads every exit (forward / left / right):
                     'open' (a FRONTIER: the lidar sees > 10 m into unexplored gallery), or 'seen'
                     (its end wall is visible: a short stub explored by sight), and drops a
                     JUNCTION beacon holding the table (a junction is a mapping event);
  * dead end       : rock ahead and no side opening -> the branch is 'explored', backtrack;
  * modified DFS   : at each junction every open exit is scored by the UTILITY FUNCTION
                       U = gain + relevance - distance - energy + accessibility
                     (expected information gain from the lidar depth, mission relevance: toward
                     the last known position / people seen only by a Flix or by CSI / lessons,
                     path distance, traversal energy vs battery, accessibility of the exit);
                     the best one is CLAIMED and explored; when no open exit is left, the Writer
                     backtracks to the previous junction (stack).

The two Writers share one map through the beacons: every junction table (and its claims) is
flooded through the mesh, so a Writer arriving at a junction found by the other one takes only
exits nobody claimed. When its own DFS is finished the Writer evaluates the open exits anywhere
in the shared map (e.g. a lost Writer's branches, released by the server) before leaving.

Beacons: BEACON_STOCK relay beacons, dropped when the link to the last beacon drops
significantly, on the spacing guard, or on a relevant event (victim, hazard, junction).
Wi-Fi CSI: CSI_STOCK ESP32-S3 CSI nodes, dropped at junctions and branch ends, and every
CSI_DENSE_SPACING m near the last known position; the Writer and the beacons are their Wi-Fi
sources. Events: CH4 detector (hazardous area), camera person detector (victim, with its
confidence). When the stock is empty or the DFS is complete, the Writer leaves the mine by the
NEAREST known exit and parks on its ONA station's charger. If destroyed it simply falls silent:
its records stay in the beacons.
"""
import math

from . import config as C
from .common import spin_node, wrap
from .geometry import dist, rssi, shortest_path
from .ground import GroundRobot

KEYS = ('0', '90', '180', '-90')


def key_of(rad):
    d = int(round(math.degrees(rad) / 90.0)) * 90
    d = ((d + 180) % 360) - 180
    return str(d if d != -180 else 180)


def rad_of(key):
    return math.radians(float(key))


def back_key(key):
    return key_of(rad_of(key) + math.pi)


class Writer(GroundRobot):
    def __init__(self):
        super().__init__('writer_b', seed=11, publishes=('/lm/writer/drop', '/lm/writer/uplink',
                                                         '/lm/writer/junction', '/lm/writer/csi'))
        self.sub(f'/lm/sensors/{self.rname}', self.on_sensors)
        self.sub(f'/lm/rf/{self.rname}', self.on_rf)
        self.sub('/lm/wireless/from_ona', self.on_wireless)
        self.sub('/lm/env/writer_hit', self.on_hit)
        self.robot_id = C.ID_ROBOT[self.rname]
        self.src = C.SRC_ID[self.rname]
        self.eid = 0
        clo, chi = C.CSI_IDS[self.rname]
        self.next_csi, self.csi_hi = clo, chi
        self.csi_stock = C.CSI_STOCK
        self.csi_nodes = []          # where this Writer dropped its CSI nodes (own estimate)
        self.marks = []
        lo, hi = C.BEACON_IDS[self.rname]
        self.next_id = lo
        self.id_hi = hi
        self.stock = C.BEACON_STOCK
        self.state = 'wait_start'
        self.mode = ''
        self.entrance = None
        self.portal = None
        self.sensors = {}
        self.rf = []
        self.mesh = []
        self.beacons = []            # beacons this Writer dropped (positions = its own estimate)
        self.gas_reported = []
        self.victims_done = []       # victim positions already measured / reported
        self._peak = {}
        self._last_parent = None
        self._link_low = rssi(0.85 * C.RF_RANGE)
        self._close_rssi = rssi(1.2)
        self._weak_drops = 0
        self.last_passed = 0         # beacon last passed on the trail (for "nextOut")
        self.lessons = {}
        self._gas_first = None
        # DFS
        self.nodes = {}              # node id (= junction beacon id) -> {x, y, exits, links, own}
        self.stack = []
        self.edge = None             # {'from': nid, 'key': k, 'sx', 'sy'} while on a gallery
        self.axis = 0.0
        self.transit = []            # list of node ids to drive through (backtrack / transit)
        self.after_transit = None
        self._open_n = 0
        self._dead_t = None
        self._approach = None
        self._cv = None
        self.explored_m = 0.0
        self.loop = 'Sense'
        self.utility = []            # last frontier evaluation (2D window)
        self._last_xy = None
        self.exit_target = None
        self.dfs_log = []
        self._guard_due = None
        self.create_timer(0.1, self.step)
        self.create_timer(C.HEARTBEAT_S, self.heartbeat)

    # ------------------------------------------------------------ inputs
    def on_sensors(self, m):
        self.sensors = m
        self.apply_gps(m)

    def on_rf(self, m):
        self.rf = m.get('links', [])
        self.peers = m.get('peers', [])
        self.mesh = m.get('mesh', [])
        if m.get('marks'):
            self.marks = m['marks']
        now = self.now()
        # trail position: the beacon the Writer is passing (nextOut of the next drop)
        for l in self.rf:
            if l['rssi'] >= self._close_rssi:
                self.last_passed = l['id']
        # loop closure on the Writer's own beacons when it passes them again
        own = {b['id']: b for b in self.beacons}
        for l in self.rf:
            b = own.get(l['id'])
            if b is not None and l.get('moved'):
                b['moved'] = True
                self._peak.pop(l['id'], None)
            if b is None or b.get('moved') or now - b.get('t', now) < 25.0:
                continue
            if l['rssi'] >= self._close_rssi:
                prev = self._peak.get(l['id'])
                if prev is None or l['rssi'] > prev[0]:
                    ex, ey, _ = self.est
                    self._peak[l['id']] = (l['rssi'], ex, ey)
            elif l['id'] in self._peak:
                _, px, py = self._peak.pop(l['id'])
                ex, ey, _ = self.est
                self.ekf.landmark(now, ex + (b['x'] - px), ey + (b['y'] - py), 0.6)
                self.get_logger().info(f'loop closure on B{l["id"]}')

    def next_eid(self):
        self.eid = self.eid % 127 + 1           # 1..127: this Writer's own records
        return self.eid

    def on_wireless(self, m):
        k = m.get('kind')
        if k == 'lessons' and m.get('to') in (self.rname, 'all'):
            self.lessons = m.get('lessons', {})
            self.get_logger().info(f'lessons from the mission server: {len(self.lessons.get("text", []))}')
        if k == 'go' and m.get('to') == self.rname and self.state == 'wait_go':
            self.entrance = m['entrance']
            self.portal = tuple(m['portal'])
            self.lkp = tuple(m.get('lkp', C.LAST_KNOWN))
            self.state = 'to_portal'
            self.get_logger().info(f'GO: assigned portal {self.entrance} at {self.portal}')

    def on_hit(self, m):
        if self.state == 'destroyed' or m.get('robot') != self.rname:
            return
        # destroyed: it simply falls silent (no more heartbeats). Everything it found is already
        # stored in the beacons; the server notices the missed heartbeats.
        self.state = 'destroyed'
        self.stop()
        self.get_logger().warn('Writer destroyed - silent from now on')

    def heartbeat(self):
        if self.state in ('exploring', 'leaving'):
            x, y, _ = self.est
            self.send_uplink(C.EV_STATUS, x, y, a=self.stock, b=min(255, int(self.explored_m / 2)),
                             conf=self.battery)

    # ------------------------------------------------------------ helpers
    def send_uplink(self, ev, x, y, **kw):
        th = self.est[2]
        pkt = {'id': kw.pop('subject', self.robot_id), 'type': ev, 'parent': 0,
               'flags': kw.pop('flags', 0), 'src': self.src, 'eid': self.next_eid(), 'x': x, 'y': y,
               'heading': math.degrees(th), 't_ms': self.now_ms(),
               'next_out': self.last_passed}
        pkt.update(kw)
        self.pub('/lm/writer/uplink', {'from': self.rname, 'wx': self.truth.x,
                                       'wy': self.truth.y, 'pkt': pkt})

    def known_beacons(self):
        """Every beacon position this Writer knows: its own drops + all heard ones."""
        pts = [(b['x'], b['y']) for b in self.beacons]
        pts += [(l['mx'], l['my']) for l in self.rf]
        pts += [(n['mx'], n['my']) for n in self.mesh]
        return pts

    def nearest_beacon_dist(self):
        x, y, _ = self.est
        return min((dist(p, (x, y)) for p in self.known_beacons()), default=1e9)

    def best_link(self):
        best = None
        for l in self.rf:
            if l.get('link') and l.get('routed', True):
                if best is None or l['rssi'] > best['rssi']:
                    best = l
        return best

    def choose_parent(self, mx=None, my=None):
        best = self.best_link()
        if best is not None:
            return best['id']
        if self.beacons and mx is not None:
            last = self.beacons[-1]
            if dist((last['x'], last['y']), (mx, my)) <= C.RF_NLOS_RANGE:
                return last['id']
        return self._last_parent or (self.beacons[-1]['id'] if self.beacons else C.PARENT_ONA)

    def drop(self, btype, event=None, junction=None):
        """Drop one relay beacon (decrements the stock). Returns its id or None."""
        if self.stock <= 0 or self.next_id > self.id_hi:
            return None
        x, y, th = self.est
        back = C.DROP_BEHIND if btype not in ('junction',) else 0.0
        mx = x - back * math.cos(th)
        my = y - back * math.sin(th)
        wx, wy = self.truth_behind(back)
        bid = self.next_id
        self.next_id += 1
        self.stock -= 1
        parent = C.PARENT_ONA if btype == 'entrance' else self.choose_parent(mx, my)
        t = self.now_ms()
        head = math.degrees(th)
        nxt = 0 if btype == 'entrance' else self.last_passed
        ptype = {'entrance': C.EV_ENTRANCE, 'junction': C.EV_JUNCTION}.get(btype, C.EV_PLACEMENT)
        pkts = [{'id': bid, 'type': ptype, 'parent': parent, 'src': self.src,
                 'eid': self.next_eid(), 'x': mx, 'y': my, 'heading': head, 't_ms': t,
                 'next_out': nxt}]
        ev_meta = None
        if event is not None:
            pk = {'id': bid, 'parent': parent, 'src': self.src, 'eid': self.next_eid(),
                  'heading': head, 't_ms': t, 'next_out': nxt}
            pk.update(event['pkt'])
            pkts.append(pk)
            ev_meta = {'kind': event['kind'], 'x': event['pkt']['x'], 'y': event['pkt']['y'],
                       'conf': event['pkt'].get('conf', 0.0)}
        self.beacons.append({'id': bid, 'x': mx, 'y': my, 'type': btype, 't': self.now()})
        self.pub('/lm/writer/drop', {
            'from': self.rname, 'id': bid, 'btype': btype, 'wx': wx, 'wy': wy, 'mx': mx,
            'my': my, 'heading': head, 'parent': parent, 't_ms': t, 'packets': pkts,
            'next_out': nxt, 'entrance': self.entrance, 'event': ev_meta,
            'junction': junction, 'stock': self.stock})
        self.last_passed = bid
        self.get_logger().info(f'dropped B{bid} ({btype}) parent={parent} nextOut={nxt} '
                               f'stock={self.stock} est ({mx:.2f},{my:.2f}) true ({wx:.2f},{wy:.2f})')
        return bid

    def drop_csi(self, why):
        """Drop one ESP32-S3 Wi-Fi CSI sensing node (it streams its CSI to a nearby beacon)."""
        if self.csi_stock <= 0 or self.next_csi > self.csi_hi:
            return None
        x, y, th = self.est
        if any(math.hypot(x - a, y - b) < 5.0 for a, b in self.csi_nodes):
            return None
        wx, wy = self.truth_behind(0.2)
        nid = self.next_csi
        self.next_csi += 1
        self.csi_stock -= 1
        self.csi_nodes.append((x, y))
        self.pub('/lm/writer/csi', {'from': self.rname, 'id': nid, 'wx': wx, 'wy': wy, 'mx': x,
                                    'my': y, 'yaw': self.truth.yaw, 't_ms': self.now_ms(),
                                    'stock': self.csi_stock, 'why': why})
        self.log_dfs(f'CSI node N{nid} dropped ({why}), {self.csi_stock} left')
        return nid

    # ------------------------------------------------------------ lessons (mission memory)
    def in_zone(self, kind, extra=0.0):
        x, y, _ = self.est
        for z in self.lessons.get(kind, []):
            if math.hypot(x - z['x'], y - z['y']) <= z['r'] + extra:
                return z
        return None

    def guard_spacing(self):
        return C.GUARD_SPACING_RISK if self.in_zone('rockfall') else C.GUARD_SPACING

    def exit_bias(self, node, key):
        """Prior from past missions: survivors were found in this direction before."""
        b = 0.0
        a = rad_of(key)
        for v in self.lessons.get('victim_prior', []):
            d = math.hypot(v['x'] - node['x'], v['y'] - node['y'])
            if d < 1.0:
                continue
            ang = math.atan2(v['y'] - node['y'], v['x'] - node['x'])
            if math.cos(wrap(ang - a)) > 0.7 and d < 40.0:
                b += 0.3 * v.get('w', 1.0)
        return b

    # ------------------------------------------------------------ DFS map
    def node_table(self, nid):
        n = self.nodes[nid]
        return {k: dict(e) for k, e in n['exits'].items()}

    def publish_node(self, nid, claim=None, release=False):
        n = self.nodes[nid]
        self.pub('/lm/writer/junction', {'from': self.rname, 'id': nid, 'exits': n['exits'],
                                         'links': n['links'], 'claim': claim, 'release': release,
                                         'eid': self.next_eid()})

    def merge_mesh(self):
        """Adopt junction tables flooded through the mesh (the other Writer's discoveries and
        its claims)."""
        for j in self.mesh:
            nid = j['id']
            n = self.nodes.get(nid)
            if n is None:
                self.nodes[nid] = {'x': j['mx'], 'y': j['my'], 'exits': {k: dict(e) for k, e in
                                                                    j['exits'].items()},
                                   'links': {k: int(o) for k, o in (j.get('links') or {}).items()},
                                   'own': False}
                continue
            for k, o in (j.get('links') or {}).items():
                n['links'].setdefault(k, int(o))
            for k, e in j['exits'].items():
                mine = n['exits'].get(k)
                if mine is None:
                    n['exits'][k] = dict(e)
                elif e.get('by') != self.rname:
                    # the mesh is the shared truth for exits not driven by this Writer
                    if mine.get('st') in ('open', 'claimed') and mine.get('by') != self.rname:
                        n['exits'][k] = dict(e)
                    elif mine.get('st') == 'open' and e.get('st') in ('claimed', 'done', 'seen'):
                        n['exits'][k] = dict(e)

    def node_near(self, x, y, r=C.DFS_KNOWN_NODE_R, exclude=None):
        best = None
        for nid, n in self.nodes.items():
            if nid == exclude:
                continue
            d = math.hypot(n['x'] - x, n['y'] - y)
            if d <= r and (best is None or d < best[0]):
                best = (d, nid)
        return None if best is None else best[1]

    def graph(self):
        adj = {}
        for nid, n in self.nodes.items():
            for k, other in n.get('links', {}).items():
                if other in self.nodes:
                    o = self.nodes[other]
                    d = math.hypot(o['x'] - n['x'], o['y'] - n['y'])
                    adj.setdefault(nid, {})[other] = d
                    adj.setdefault(other, {})[nid] = d
        return adj

    def link(self, a, ka, b, kb):
        self.nodes[a].setdefault('links', {})[ka] = b
        self.nodes[b].setdefault('links', {})[kb] = a

    def read_exits(self, arrive_key):
        """Read the junction's exits with the lidar (forward / left / right of the arrival axis)."""
        x, y, th = self.est
        a0 = rad_of(arrive_key)
        exits = {back_key(arrive_key): {'st': 'done', 'by': self.rname}}
        for turn in (0.0, math.pi / 2, -math.pi / 2):
            k = key_of(a0 + turn)
            rel = wrap(rad_of(k) - th)
            if max(self.free_along(rel, 6.0), min(self.sight(rel, 0.17), 6.0)) < C.DFS_OPEN:
                continue                        # rock on that side (no gallery)
            # closed stub only when every ray in the sector ends at a wall short of the
            # junction's arm length; any doubt (a long ray) -> explore it
            depth = max(self.free_along(rel, C.DFS_LIDAR_RANGE, 0.3), self.sight(rel))
            st = 'seen' if depth < C.DFS_SEEN_CLOSED else 'open'
            # accessibility from the lidar: does a wide footprint pass into the exit?
            acc = 1.0 if self.free_along(rel, 5.0, 0.7) >= 3.0 else 0.6
            exits[k] = {'st': st, 'by': self.rname if st == 'seen' else '',
                        'd': round(min(depth, C.DFS_LIDAR_RANGE), 1), 'w': acc}
        return exits

    def log_dfs(self, txt):
        self.dfs_log = (self.dfs_log + [f'{self.now():.0f}s {txt}'])[-12:]
        self.get_logger().info(txt)

    # ------------------------------------------------------------ main loop
    def step(self):
        if self.state == 'destroyed' or not self.truth.ok:
            return
        x, y, th = self.est
        if self._last_xy is not None and self.state == 'exploring':
            self.explored_m += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        if self.state == 'wait_start':
            self.stop()
            if self.started():
                self.gps_fix()
                self.state = 'wait_go'
            return
        if self.state in ('wait_go', 'parked'):
            self.stop()
            return
        if self.state == 'to_portal':
            px, py = self.portal
            if math.hypot(-4.0 - x, py - y) > 1.2 and x < -3.0:
                self.drive_to(-4.0, py, C.WRITER_SPEED)
                return
            if self.drive_to(1.0, py, C.WRITER_SPEED * 0.6, near_turn=True) < 0.5:
                bid = self.drop('entrance', junction={'0': {'st': 'claimed', 'by': self.rname},
                                                      '180': {'st': 'done', 'by': 'outside'}})
                self.nodes[bid] = {'x': x, 'y': y, 'exits': {'0': {'st': 'claimed', 'by': self.rname},
                                                             '180': {'st': 'done', 'by': 'outside'}},
                                   'links': {}, 'own': True, 'entrance': self.entrance}
                self.stack = [bid]
                self.start_edge(bid, '0')
                self.state = 'exploring'
                self.log_dfs(f'entrance {self.entrance}: B{bid} fibered to ONA-{self.entrance}, '
                             f'DFS starts east')
            return
        if self.state == 'leaving':
            self.leave()
            return
        if self.state != 'exploring':
            self.stop()
            return
        self.merge_mesh()
        if self._cv is not None:
            self.cv_step()
            return
        self.check_events()
        if self._cv is not None:
            return
        if self.transit:
            self.transit_step()
            return
        self.seg_ref = None
        if self.edge is not None:
            self.gallery_step()

    # ------------------------------------------------------------ gallery travel
    def start_edge(self, nid, key):
        n = self.nodes[nid]
        self.edge = {'from': nid, 'key': key, 'sx': n['x'], 'sy': n['y']}
        self.axis = rad_of(key)
        self._open_n = 0
        self._dead_t = None
        self._approach = None

    def side_open(self, rel):
        return self.free_along(rel, 6.0) >= C.DFS_OPEN

    def gallery_step(self):
        x, y, th = self.est
        h = self.axis
        e = self.edge
        travelled = math.hypot(x - e['sx'], y - e['sy'])
        rel_f = wrap(h - th)
        rel_l = wrap(h + math.pi / 2 - th)
        rel_r = wrap(h - math.pi / 2 - th)
        if abs(rel_f) < 0.6:
            open_l, open_r = self.side_open(rel_l), self.side_open(rel_r)
        else:
            open_l = open_r = False               # still turning into the gallery
        # 1. arrival at a junction already known (own map or shared by the other Writer)
        known = self.node_near(x, y, exclude=e['from'])
        if known is not None and travelled > C.DFS_NODE_SPACING:
            n = self.nodes[known]
            if math.hypot(n['x'] - x, n['y'] - y) < 1.5 or (open_l or open_r):
                if self.creep_to_centre(rel_f, rel_l if open_l else rel_r, n):
                    self.arrive(known)
                return
        # 2. a new junction seen by the lidar (side gallery) - confirmed over several scans
        if (open_l or open_r) and travelled > C.DFS_NODE_SPACING:
            self._open_n += 1
        else:
            self._open_n = 0
        if self._open_n >= 3 or self._approach is not None:
            side = rel_l if open_l else (rel_r if open_r else None)
            if self._approach is None:
                self._approach = {'t': self.now(), 'x': x, 'y': y}
            if side is None:
                if math.hypot(x - self._approach['x'], y - self._approach['y']) > 5.0:
                    self._approach = None          # false alarm (gallery widening)
                    self._open_n = 0
                else:
                    self.drive_to(x + 2.0 * math.cos(h), y + 2.0 * math.sin(h), 0.4, near_turn=True)
                return
            if self.creep_to_centre(rel_f, side, None):
                self.arrive(None)
            return
        # 3. dead end: rock ahead, no side opening - confirmed over 1 s of scans (dust or smoke
        #    gives passing returns, rock stays)
        if self.free_along(rel_f, 12.0) < C.DFS_DEAD_END and not (open_l or open_r) \
                and travelled > 2.0:
            if self._dead_t is None:
                self._dead_t = self.now()
            if self.now() - self._dead_t >= 1.0:
                self._dead_t = None
                self.dead_end()
            else:
                self.stop()
            return
        self._dead_t = None
        # 4. keep going along the gallery; relay drops on spacing / weak link; denser Wi-Fi CSI
        #    sensing near the miners' last known position
        self.loop = 'Navigate'
        self.relay_drops(near_junction=(open_l or open_r))
        lx, ly = getattr(self, 'lkp', C.LAST_KNOWN)
        if math.hypot(lx - x, ly - y) <= C.CSI_DENSE_R and travelled > 3.0 and \
                min((math.hypot(x - a, y - b) for a, b in self.csi_nodes), default=99.0) \
                >= C.CSI_DENSE_SPACING:
            self.drop_csi('dense sensing near the last known position')
        tx, ty = x + 5.0 * math.cos(h), y + 5.0 * math.sin(h)
        # stay on the gallery's line from the node (removes along-track wobble on long runs)
        self.drive_to(tx, ty, C.WRITER_SPEED)

    def creep_to_centre(self, rel_f, rel_side, node):
        """Creep along the gallery until the robot is on the side gallery's centre line
        (measured from the side gallery's two walls in the scan). True when there."""
        x, y, th = self.est
        uc = self.arm_centre(rel_f, rel_side)
        if node is not None and uc is None:
            uc_n = (node['x'] - x) * math.cos(self.axis) + (node['y'] - y) * math.sin(self.axis)
            uc = uc_n
        if uc is not None and abs(uc) < 0.6:
            return True
        if uc is None:
            ap = self._approach or {'x': x, 'y': y}
            if math.hypot(x - ap['x'], y - ap['y']) > 4.0:
                return True
            uc = 1.0
        h = self.axis
        s = max(-2.0, min(2.0, uc))
        self.drive_to(x + s * math.cos(h), y + s * math.sin(h), 0.35, near_turn=True)
        return False

    def arm_centre(self, a_in, a_out):
        """Along-gallery offset of the side gallery's centre line from the robot (robot frame
        bearings: a_in = approach axis, a_out = side gallery). None if both walls not seen."""
        cu, su = math.cos(a_in), math.sin(a_in)
        cv, sv = math.cos(a_out), math.sin(a_out)
        us = []
        for px, py in self.scan_xy:
            v = px * cv + py * sv
            if 2.8 < v < 9.0:
                us.append(px * cu + py * su)
        if len(us) < 12:
            return None
        us.sort()
        lo, hi = us[int(0.05 * len(us))], us[int(0.95 * len(us)) - 1]
        if not 2.5 < hi - lo < 7.0:
            return None
        return (lo + hi) / 2.0

    # ------------------------------------------------------------ DFS events
    def arrive(self, known):
        """At a junction centre, coming along self.edge."""
        x, y, th = self.est
        e = self.edge
        arr_key = e['key']
        self.edge = None
        self._approach = None
        self._open_n = 0
        src = self.nodes[e['from']]
        self._guard_due = None
        if known is not None:
            self.last_passed = known
            n = self.nodes[known]
            # junction beacon = map anchor: re-anchor on it (loop closure / map merge)
            self.ekf.landmark(self.now(), n['x'], n['y'], 0.8 if not n.get('own') else 0.5)
            bk = back_key(arr_key)
            n['exits'][bk] = {'st': 'done', 'by': self.rname}
            self.link(e['from'], arr_key, known, bk)
            if known in self.stack:
                # cycle inside this Writer's own exploration: this gallery is explored
                src['exits'][arr_key] = {'st': 'done', 'by': self.rname}
                self.publish_node(e['from'])
                self.publish_node(known)
                self.log_dfs(f'loop: B{e["from"]} -> B{known} already in my stack, backtrack')
                self.go_to_nodes([e['from']], then='decide')
                return
            self.log_dfs(f'reached junction B{known} ({"shared by the other Writer" if not n.get("own") else "known"})')
            self.publish_node(known)
            self.stack.append(known)
            self.decide()
            return
        self.loop = 'Map'
        exits = self.read_exits(arr_key)
        bid = self.drop('junction', junction=exits)
        if bid is None:                      # no beacon left to mark the junction
            self.log_dfs('junction found but beacon stock empty -> leaving by the nearest exit')
            self.start_leaving()
            return
        self.nodes[bid] = {'x': x, 'y': y, 'exits': exits, 'links': {}, 'own': True}
        self.link(e['from'], arr_key, bid, back_key(arr_key))
        desc = ', '.join(f'{k}:{v["st"]}' for k, v in exits.items())
        self.log_dfs(f'new junction B{bid} at ({x:.1f},{y:.1f}): {desc}')
        self.publish_node(bid)
        self.drop_csi(f'junction B{bid}')
        self.stack.append(bid)
        self.decide()

    def dead_end(self):
        e = self.edge
        self.edge = None
        src = self.nodes[e['from']]
        src['exits'][e['key']] = {'st': 'done', 'by': self.rname}
        self.publish_node(e['from'])
        self.log_dfs(f'dead end after {self.explored_m:.0f} m explored - branch {e["key"]} of '
                     f'B{e["from"]} done, backtrack')
        self.drop_csi('branch end')
        self.go_to_nodes([e['from']], then='decide')

    def decide(self):
        """DFS decision at the junction on top of the stack."""
        if self.stock <= 0:
            self.log_dfs('beacon stock empty -> leaving by the nearest exit')
            self.start_leaving()
            return
        nid = self.stack[-1]
        n = self.nodes[nid]
        self.loop = 'Identify frontiers'
        cands = []
        back = self.exit_distance(nid)
        for k, ex in n['exits'].items():
            if ex.get('st') != 'open':
                continue
            u = self.exit_utility(n, k, ex, 0.0, back)
            if u is not None:
                cands.append((u['U'], k, u))
        if cands:
            self.loop = 'Evaluate'
            cands.sort(key=lambda c: -c[0])
            self.utility = [dict(c[2], node=nid, exit=c[1], chosen=(i == 0))
                            for i, c in enumerate(cands)]
            k = cands[0][1]
            n['exits'][k] = {'st': 'claimed', 'by': self.rname}
            self.publish_node(nid, claim=k)
            self.loop = 'Select'
            self.log_dfs(f'B{nid} frontiers: ' + ' | '.join(
                f'{c[1]}deg U={c[0]:+.2f} (gain {c[2]["gain"]:.2f}, rel {c[2]["rel"]:.2f}, '
                f'energy {c[2]["energy"]:.2f}, acc {c[2]["acc"]:.1f})' for c in cands)
                + f' -> claim {k}deg')
            self.start_edge(nid, k)
            return
        # nothing open here: backtrack - unless nothing below on my stack is still open, then go
        # straight to the nearest open exit of the shared map (or leave): no useless walk back
        self.stack.pop()
        below_open = any(e.get('st') == 'open' for nid2 in self.stack
                         for e in self.nodes[nid2]['exits'].values())
        if self.stack and not below_open:
            for nid2 in self.stack:
                for k2, o2 in self.nodes[nid2]['links'].items():
                    if o2 in self.stack or o2 == nid:
                        self.nodes[nid2]['exits'][k2] = {'st': 'done', 'by': self.rname}
                self.publish_node(nid2)
            self.stack = []
        if self.stack:
            prev = self.stack[-1]
            pk = next((k for k, o in self.nodes[prev]['links'].items() if o == nid), None)
            if pk is not None:
                self.nodes[prev]['exits'][pk] = {'st': 'done', 'by': self.rname}
                self.publish_node(prev)
            self.log_dfs(f'B{nid} finished - backtrack to B{prev}')
            self.go_to_nodes([prev], then='decide')
            return
        # own DFS complete: any open exit left anywhere in the shared map?
        tgt = self.frontier()
        if tgt is not None:
            path = tgt[1]
            self.log_dfs(f'my DFS is complete; open exit left at B{path[-1]} in the shared map '
                         f'-> going there ({tgt[0]:.0f} m)')
            self.go_to_nodes(path[1:], then='frontier')
            self._frontier_node = path[-1]
            return
        self.log_dfs('DFS complete: no open exit left in the shared map')
        self.start_leaving(complete=True)

    def frontier(self):
        """Open exits anywhere in the shared map, evaluated with the same utility function
        (the path distance now counts)."""
        x, y, _ = self.est
        here = self.node_near(x, y, r=6.0)
        if here is None:
            return None
        adj = self.graph()
        best = None
        evals = []
        for nid, n in self.nodes.items():
            for k, ex in n['exits'].items():
                if ex.get('st') != 'open':
                    continue
                d, path = shortest_path(adj, here, nid)
                if d is None:
                    continue
                u = self.exit_utility(n, k, ex, d, self.exit_distance(nid))
                if u is None:
                    continue
                evals.append(dict(u, node=nid, exit=k, chosen=False))
                if best is None or u['U'] > best[2]:
                    best = (d, path, u['U'])
        if evals:
            evals.sort(key=lambda e: -e['U'])
            evals[0]['chosen'] = True
            self.utility = evals[:6]
        return None if best is None else best[:2]

    # ------------------------------------------------------------ frontier utility
    def exit_distance(self, nid):
        """Graph distance from a junction to the nearest known fibered portal."""
        adj = self.graph()
        best = None
        for ent in self.entrance_nodes():
            d, _ = shortest_path(adj, nid, ent)
            if d is not None and (best is None or d < best):
                best = d
        return 60.0 if best is None else best

    def exit_utility(self, n, k, ex, path_d, back_d):
        """Utility of exploring exit k of junction n (technical report, 2.2):
        expected information gain, mission relevance, distance, traversal energy cost,
        accessibility. None if the battery cannot cover going there and coming back out."""
        W = C.UTILITY_W
        a = rad_of(k)
        # expected information gain: the deeper the lidar sees unexplored gallery, the more map
        gain = 0.6 + 0.4 * min(1.0, ex.get('d', C.DFS_LIDAR_RANGE) / C.DFS_LIDAR_RANGE)
        # mission relevance: toward the miners' last known position, toward people on the map
        # that no Writer has reached yet (seen only by a Flix or by CSI), lessons from the server
        lx, ly = getattr(self, 'lkp', C.LAST_KNOWN)
        rel = 0.5 * (1.0 + math.cos(wrap(a - math.atan2(ly - n['y'], lx - n['x']))))
        rel += self.exit_bias(n, k)
        for mk in self.marks:
            if mk.get('at_victim') or mk.get('kind') == 'confirmed' or mk.get('no_ground'):
                continue
            d = math.hypot(mk['x'] - n['x'], mk['y'] - n['y'])
            if 3.0 < d < 45.0 and math.cos(wrap(math.atan2(mk['y'] - n['y'], mk['x'] - n['x']) - a)) > 0.7:
                rel += 0.6
                break
        rel = min(rel, 2.0)
        dist_t = path_d / 50.0
        # traversal energy: to the exit, ~20 m of new gallery, back out to the nearest portal
        need = (path_d + 20.0 + back_d) * C.ENERGY_PER_M
        if need > self.battery:
            return None
        energy = need / max(0.05, self.battery)
        # accessibility: exit width seen by the lidar, known hazard in that direction
        acc = ex.get('w', 1.0)
        for g in self.gas_reported:
            if math.hypot(g[0] - n['x'], g[1] - n['y']) < 12.0 and \
                    math.cos(wrap(math.atan2(g[1] - n['y'], g[0] - n['x']) - a)) > 0.7:
                acc -= 0.3
        U = (W['gain'] * gain + W['relevance'] * rel - W['distance'] * dist_t
             - W['energy'] * energy + W['access'] * acc)
        return {'U': round(U, 3), 'gain': round(gain, 3), 'rel': round(rel, 3),
                'dist': round(path_d, 1), 'energy': round(energy, 3), 'acc': round(acc, 2)}

    # ------------------------------------------------------------ transit along known nodes
    def go_to_nodes(self, nodes, then):
        self.transit = list(nodes)
        self.after_transit = then
        x, y, _ = self.est
        self._seg_from = self.node_near(x, y, r=6.0)

    def segment(self, a, b):
        """(start, end) of the straight gallery between two linked junctions of the shared map
        while the robot is well inside it, else None."""
        if a is None or a not in self.nodes or b not in self.nodes:
            return None
        if b not in self.nodes[a].get('links', {}).values() and a not in self.nodes[b].get('links', {}).values():
            return None
        p, q = self.nodes[a], self.nodes[b]
        L = math.hypot(q['x'] - p['x'], q['y'] - p['y'])
        x, y, _ = self.est
        along = ((x - p['x']) * (q['x'] - p['x']) + (y - p['y']) * (q['y'] - p['y'])) / max(L, 1e-6)
        if L < 8.0 or not 3.0 < along < L - 3.0:
            return None
        return (p['x'], p['y']), (q['x'], q['y'])

    def transit_step(self):
        x, y, _ = self.est
        nid = self.transit[0]
        n = self.nodes[nid]
        self.seg_ref = self.segment(getattr(self, '_seg_from', None), nid)
        self.relay_drops(near_junction=math.hypot(n['x'] - x, n['y'] - y) < 7.0)
        if self.drive_to(n['x'], n['y'], C.WRITER_SPEED, near_turn=math.hypot(n['x'] - x, n['y'] - y) < 3.0) < 1.0:
            self.ekf.landmark(self.now(), n['x'], n['y'], 0.6)
            self.last_passed = nid
            self._seg_from = nid
            self.seg_ref = None
            self.transit.pop(0)
            if self.transit:
                return
            then = self.after_transit
            self.after_transit = None
            if then == 'decide':
                self.decide()
            elif then == 'frontier':
                self.stack = [self._frontier_node]
                self.decide()
            elif then == 'exit':
                self.state = 'leaving'

    # ------------------------------------------------------------ relay drops
    def relay_drops(self, near_junction=False):
        """Relay beacon when the chain needs one: spacing guard (deferred up to GUARD_DEFER m in
        case a junction comes, whose beacon then does the job), weak link, or a learned
        risk zone ahead."""
        best = self.best_link()
        if best is not None:
            self._last_parent = best['id']
        heard = max((l['rssi'] for l in self.rf if l.get('link')), default=None)
        weak = heard is None or heard < self._link_low
        if not weak:
            self._weak_drops = 0
        near = self.nearest_beacon_dist()
        x, y, _ = self.est
        guard = False
        if near >= self.guard_spacing():
            if self._guard_due is None:
                self._guard_due = (x, y)
            guard = math.hypot(x - self._guard_due[0], y - self._guard_due[1]) >= C.GUARD_DEFER
        else:
            self._guard_due = None
        weak_drop = weak and near > C.WEAK_DROP_MIN and self._weak_drops < C.WEAK_DROP_MAX
        pre = (self.in_zone('collapse', extra=4.0) is not None and not self.in_zone('collapse')
               and near > 6.0)
        if (guard or weak_drop or pre) and not (near_junction and not weak):
            if weak_drop and not guard:
                self._weak_drops += 1
            self._guard_due = None
            if self.stock > 0:
                self.drop('trail')
                if pre:
                    self.log_dfs('lesson: roof-collapse zone ahead - beacon dropped before it')
            if self.stock <= 0:
                self.log_dfs('beacon stock empty -> leaving by the nearest exit')
                self.start_leaving()

    # ------------------------------------------------------------ events
    def check_events(self):
        s = self.sensors
        if not s:
            return
        x, y, th = self.est
        here = (x, y)
        gas = s.get('gas', 0.0)
        if gas >= C.GAS_ALARM and all(dist(here, g) > 6.0 for g in self.gas_reported):
            if self.lessons.get('gas_confirm'):
                # lesson: this sensor type gave false positives - confirm with a second sample
                if self._gas_first is None:
                    self._gas_first = self.now()
                    return
                if self.now() - self._gas_first < 1.0:
                    return
            self._gas_first = None
            self.gas_reported.append(here)
            o2 = s.get('o2', 20.9)
            ev = {'kind': 'gas', 'pkt': {'type': C.EV_HAZARD, 'x': x, 'y': y,
                                         'a': gas * 10.0, 'b': o2 * 10.0 - 150.0,
                                         'conf': min(1.0, gas / (2.0 * C.GAS_ALARM))}}
            if self.drop('gas', ev) is None:
                self.send_uplink(C.EV_HAZARD, x, y, a=gas * 10.0, b=o2 * 10.0 - 150.0,
                                 conf=min(1.0, gas / (2.0 * C.GAS_ALARM)))
        elif gas < C.GAS_ALARM:
            self._gas_first = None
        for v in s.get('victims') or []:
            a = th + v['bearing']
            vx, vy = x + v['range'] * math.cos(a), y + v['range'] * math.sin(a)
            if any(dist((vx, vy), p) < 3.0 for p in self.victims_done):
                continue
            # person in the camera: pause a moment so the detector fuses several frames
            self._cv = {'x': vx, 'y': vy, 't0': self.now(), 'conf': v['conf'], 'range': v['range']}
            self.stop()
            self.loop = 'Sense'
            self.log_dfs(f'camera: person at {v["range"]:.1f} m (detector {v["conf"]:.0%})')
            return

    def cv_step(self):
        """Camera dwell: best detector confidence over the frames, then the VICTIM record."""
        self.stop()
        c = self._cv
        for v in self.sensors.get('victims') or []:
            x, y, th = self.est
            a = th + v['bearing']
            vx, vy = x + v['range'] * math.cos(a), y + v['range'] * math.sin(a)
            if math.hypot(vx - c['x'], vy - c['y']) < 2.0 and v['conf'] > c['conf']:
                c.update({'x': vx, 'y': vy, 'conf': v['conf'], 'range': v['range']})
        if self.now() - c['t0'] < C.CV_DWELL:
            return
        self._cv = None
        self.victims_done.append((c['x'], c['y']))
        ev = {'type': C.EV_VICTIM, 'x': c['x'], 'y': c['y'], 'a': 10.0 * c['range'],
              'b': C.CAM_RGBD, 'conf': c['conf']}
        # a victim beacon already at this person (the other Writer's)? the observation is
        # written into it; otherwise this Writer drops the victim beacon (trail end)
        mark = None
        for l in self.rf:
            if l.get('btype') == 'victim' and math.hypot(l['mx'] - c['x'], l['my'] - c['y']) < 4.0:
                mark = l['id']
        if mark is not None:
            self.send_uplink(C.EV_VICTIM, c['x'], c['y'], subject=mark,
                             **{k: v for k, v in ev.items() if k not in ('type', 'x', 'y')})
            self.log_dfs(f'person already marked by B{mark}: camera observation added')
            return
        self.log_dfs(f'VICTIM: camera {c["conf"]:.0%} at ({c["x"]:.1f},{c["y"]:.1f}) - victim beacon')
        if self.drop('victim', {'kind': 'victim', 'pkt': ev}) is None:
            self.send_uplink(C.EV_VICTIM, c['x'], c['y'],
                             **{k: v for k, v in ev.items() if k not in ('type', 'x', 'y')})

    # ------------------------------------------------------------ leaving the mine
    def entrance_nodes(self):
        out = []
        for nid, n in self.nodes.items():
            if n.get('entrance') or ('180' in n['exits'] and n['exits']['180'].get('by') == 'outside'):
                out.append(nid)
        for j in self.mesh:
            e = j['exits'].get('180')
            if e and e.get('by') == 'outside' and j['id'] not in out:
                out.append(j['id'])
        return out

    def start_leaving(self, complete=False):
        """Nearest exit = shortest path in the junction graph to any known fibered portal."""
        x, y, _ = self.est
        here = self.node_near(x, y, r=40.0)
        adj = self.graph()
        best = None
        for ent in self.entrance_nodes():
            if here is None:
                break
            d, path = shortest_path(adj, here, ent)
            if d is None:
                continue
            d += math.hypot(self.nodes[here]['x'] - x, self.nodes[here]['y'] - y)
            if best is None or d < best[0]:
                best = (d, path)
        if best is None:
            ent = self.entrance_nodes()[:1] or [self.stack[0] if self.stack else None]
            best = (0.0, [here] + ent if here is not None else ent)
        path = [p for p in best[1] if p is not None]
        ent = self.nodes[path[-1]]
        self.exit_target = path[-1]
        letter = ent.get('entrance') or ('A' if ent['y'] > 20 else 'B')
        self.exit_letter = letter
        self.send_uplink(C.EV_EXPLORATION_COMPLETE if complete else C.EV_WRITER_EXIT, x, y,
                         a=self.stock, b=min(255, int(self.explored_m / 2)), conf=self.battery)
        self.log_dfs(f'leaving by portal {letter} (nearest exit, {best[0]:.0f} m), '
                     f'{self.stock} beacons left')
        self.edge = None
        self.go_to_nodes(path, then='exit')

    def leave(self):
        px, py = C.EXIT_PARK.get(getattr(self, 'exit_letter', 'B'), (-6.0, -4.0))
        x, y, _ = self.est
        ent = self.nodes.get(self.exit_target)
        if x > -1.0 and ent is not None:
            self.drive_to(-2.0, ent['y'], C.WRITER_SPEED)
            return
        if self.drive_to(px, py, C.WRITER_SPEED) < 1.0:
            self.stop()
            self.state = 'parked'
            self.charging = True
            self.log_dfs(f'out of the mine, on the charger of ONA-{self.exit_letter} '
                         f'(battery {self.battery:.0%})')

    def publish_pose(self):
        x, y, _ = self.est
        lead = None
        if self.state == 'exploring' and self.edge is not None:
            lead = (x + C.FLIX_LEAD * math.cos(self.axis), y + C.FLIX_LEAD * math.sin(self.axis))
        nodes = {str(k): {'x': round(n['x'], 2), 'y': round(n['y'], 2), 'exits': n['exits'],
                          'own': n.get('own', False), 'links': n.get('links', {})}
                 for k, n in self.nodes.items()}
        self.pub(f'/lm/pose/{self.rname}', self.pose_msg({
            'state': self.state, 'beacons': len(self.beacons), 'stock': self.stock,
            'entrance': self.entrance, 'axis': self.axis, 'lead': lead,
            'stack': list(self.stack), 'edge': self.edge, 'transit': list(self.transit),
            'nodes': nodes, 'dfs_log': self.dfs_log, 'explored_m': round(self.explored_m, 1),
            'camera': self._cv is not None, 'robot': self.rname, 'csi_stock': self.csi_stock,
            'loop': self.loop, 'utility': self.utility}))


def main(args=None):
    spin_node(Writer, args)


if __name__ == '__main__':
    main()

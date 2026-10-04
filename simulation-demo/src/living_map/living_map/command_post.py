"""Command Post + mission server.

Command Post: portal discovery results (from both Flix) -> assignment (one Writer + its Flix per
clear portal; a flooded portal is inspected by a Flix and gets its ONA's own first beacon;
collapsed portals rejected), live map in GPS, event log, VICTIM DETECTION FUSION (camera and
Wi-Fi CSI observations of the same person combined into one victim hypothesis with a fused
confidence), the Executor's briefings, Writer watchdog (missed heartbeats), claim release for a
lost Writer.

Executor: briefed AS SOON AS a victim hypothesis is known and the trail reaches it (it does not
wait for the Writers to finish), then sent from one to the next: MOST CONFIDENT FIRST, then the
NEAREST; the route is the shortest path on the trail graph built from every beacon's "toward
the exit" pointer. Briefing updates follow any change on the route (relocated beacon, new gas
event to re-measure).

Mission server (shared by every Outside Network Area): MISSION MEMORY. Every finished mission is
stored with where it came from (ONA-A/B/C). Before a new mission the server turns all stored
missions into lessons with simple Bayesian estimates, and the ONAs brief the Writers with them:
  * rockfall zones   (beacon losses per zone, Gamma-Poisson)  -> denser beacon chain there
  * collapse zones   (Writers lost per zone)                  -> beacon before the zone,
                                                                 shorter silence watchdog
  * CH4 sensor false-positive rate (Beta-Binomial)            -> Writers confirm with a 2nd
                                                                 sample, Executor re-measures
  * where survivors were found (refuges / dead ends)          -> DFS explores toward them first
  * robot speeds                                               -> Executor ETA
"""
import json
import math
import os

from . import config as C
from .common import LMNode, spin_node
from .geometry import fuse_confidence, los, seg_dist as _seg_dist, shortest_path
from .packet import value_text


def obs_name(p):
    """Detection source of a record: robot + sensing modality."""
    who = C.SRC_NAME.get(p.get('src', 0), '?')
    if p['type'] == C.EV_VICTIM:
        return f'{who} camera'
    if p['type'] == C.EV_VICTIM_CONFIRMED:
        return 'Executor camera (on site)'
    if p.get('src') in (C.SRC_ID['flix_a'], C.SRC_ID['flix_b']):
        return f'{who} Wi-Fi CSI (flying node)'
    return f'{who} Wi-Fi CSI nodes'


def conf_class(c):
    """Executor order: confidence first, in 5 % classes; within a class, the nearest."""
    return round(c * 20.0) / 20.0


# ====================================================================== mission memory
class MissionMemory:
    def __init__(self, logger=None):
        self.log = logger
        mode = os.environ.get('LM_MEMORY', '')
        here = os.path.dirname(os.path.abspath(__file__))
        self.seed = os.path.join(here, 'memory_seed.json')
        if not os.path.isfile(self.seed):
            try:
                from ament_index_python.packages import get_package_share_directory
                self.seed = os.path.join(get_package_share_directory('living_map'), 'memory_seed.json')
            except Exception:
                pass
        if mode == 'off':
            self.path = None
        elif mode in ('', 'live'):
            self.path = os.path.expanduser('~/living_map_runs/mission_memory.json')
        elif mode == 'seed':
            self.path = None
        else:
            self.path = os.path.expanduser(mode)
        self.missions = []
        self.writable = bool(self.path) and mode != 'seed'
        self.load(mode)

    def load(self, mode):
        src = None
        if mode == 'off':
            return
        if self.path and os.path.isfile(self.path):
            src = self.path
        elif os.path.isfile(self.seed):
            src = self.seed
        if src is None:
            return
        try:
            with open(src) as f:
                self.missions = json.load(f).get('missions', [])
        except Exception:
            self.missions = []

    def save(self, mission):
        self.missions.append(mission)
        if not self.writable:
            return False
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, 'w') as f:
                json.dump({'missions': self.missions}, f, indent=1)
            return True
        except Exception:
            return False

    @staticmethod
    def _zones(points, size=C.MEMORY_ZONE):
        """Cluster event positions: a point joins a zone whose centre is within `size` metres."""
        zones = []
        for p in points:
            for z in zones:
                if math.hypot(z['sx'] / z['n'] - p['x'], z['sy'] / z['n'] - p['y']) <= size:
                    break
            else:
                z = {'n': 0, 'sx': 0.0, 'sy': 0.0, 'onas': set()}
                zones.append(z)
            z['n'] += 1
            z['sx'] += p['x']
            z['sy'] += p['y']
            z['onas'].add(p.get('ona', '?'))
        return {i: z for i, z in enumerate(zones)}

    def lessons(self):
        M = len(self.missions)
        out = {'missions': M, 'onas': sorted({o for m in self.missions for o in m.get('onas', [])}),
               'rockfall': [], 'collapse': [], 'victim_prior': [], 'text': []}
        if M == 0:
            out['gas_fp'] = C.GAS_FP_PRIOR[0] / sum(C.GAS_FP_PRIOR)
            out['gas_confirm'] = False
            out['text'].append('no past mission stored yet: default behaviour')
            return out
        for kind, key, words in (('rockfall', 'beacon_lost', 'beacon(s) lost to rockfall'),
                                 ('collapse', 'writer_lost', 'Writer(s) lost to roof collapse')):
            pts = [dict(p, ona=m.get('onas', ['?'])[0]) for m in self.missions for p in m.get(key, [])]
            for _, e in sorted(self._zones(pts).items()):
                rate = (0.5 + e['n']) / (1.0 + M)          # Gamma(0.5, 1) prior -> posterior mean
                if rate < 0.3:
                    continue
                x, y = e['sx'] / e['n'], e['sy'] / e['n']
                out[kind].append({'x': round(x, 1), 'y': round(y, 1), 'r': C.MEMORY_RISK_R,
                                  'rate': round(rate, 2), 'n': e['n']})
                act = ('beacon spacing 18 -> 11 m there' if kind == 'rockfall'
                       else 'drop a beacon before it, silence watchdog 90 -> 45 s')
                out['text'].append(f'{kind} zone near ({x:.0f}, {y:.0f}): {e["n"]} {words} in {M} '
                                   f'missions (rate {rate:.2f}/mission) -> {act}')
        fp = sum(m.get('gas_fp', 0) for m in self.missions)
        real = sum(m.get('gas_real', 0) for m in self.missions)
        a, b = C.GAS_FP_PRIOR[0] + fp, C.GAS_FP_PRIOR[1] + real
        out['gas_fp'] = round(a / (a + b), 3)
        out['gas_confirm'] = out['gas_fp'] > C.GAS_FP_CONFIRM
        out['text'].append(f'CH4 sensor: {fp} false / {real} real alarms -> P(false alarm) = '
                           f'{out["gas_fp"]:.2f}' + (' -> Writers confirm with a second sample, '
                                                     'Executor re-measures' if out['gas_confirm']
                                                     else ' (no change)'))
        vz = self._zones([dict(v, ona=m.get('onas', ['?'])[0]) for m in self.missions
                          for v in m.get('victims', [])])
        for _, e in vz.items():
            w = e['n'] / M
            out['victim_prior'].append({'x': round(e['sx'] / e['n'], 1),
                                        'y': round(e['sy'] / e['n'], 1), 'w': round(w, 2)})
        if vz:
            out['text'].append(f'survivors found in {len(vz)} places before (refuges / dead ends) '
                               f'-> DFS explores toward them first')
        sp = [m['exec_speed'] for m in self.missions if m.get('exec_speed')]
        if sp:
            out['exec_speed'] = round(sum(sp) / len(sp), 3)
            out['text'].append(f'Executor average speed underground {out["exec_speed"]:.2f} m/s '
                               f'-> ETA for each miner')
        return out


# ====================================================================== command post
class CommandPost(LMNode):
    def __init__(self):
        super().__init__('command_post', publishes=('/lm/sat/downlink', '/lm/cp/state'))
        self.memory = MissionMemory(self.get_logger())
        self.lessons = self.memory.lessons()
        self.beacons = {}
        self.events = {}
        self.n_victims = 0
        self.log = []
        self.phase = 'standby'
        self.fixes = set()
        self.entrances = {}               # discovered openings by portal id
        self.reports = set()              # Flix that reported their sweep
        self.assign = {}                  # writer -> portal
        self.briefing = None
        self.first_briefing = None
        self.exec_target = None           # victim key the Executor is going to
        self.exec_busy = False
        self.exec_at = None               # beacon where the Executor is (after a victim)
        self.visited = []
        self.update_pending = False
        self.writer = {w: {'status': 'at staging area', 'last_rx': None, 'lost': False,
                           'done': False, 'beacons': 0} for w in C.WRITERS}
        self.flix = {f: 'on its pad' for f in C.FLIXES}
        self.executor_status = 'at staging area, waiting for a briefing'
        self.rx = 0
        self.csi_nodes = {}
        self.obs_seen = {}                # (source, event id, type) -> hypothesis key
        self.mission_saved = False
        self.stats = {'gas_fp': 0, 'gas_real': 0, 'beacon_lost': [], 'writer_lost': [],
                      'victims': [], 't_exec_start': None, 'exec_dist': 0.0}
        self.sub('/lm/sat/uplink', self.on_uplink)
        self.create_timer(1.0, self.publish_state)
        for t in self.lessons['text']:
            self.note(f'Mission server lesson ({self.lessons["missions"]} missions): {t}', 'info')

    def note(self, text, level='info'):
        self.log.append({'t': round(self.now(), 1), 'text': text, 'level': level})
        self.log = self.log[-120:]
        self.get_logger().info(text)

    def downlink(self, m):
        self.flow('sat_down', msg=m['kind'], ona=m.get('ona') or m.get('via'))
        self.pub('/lm/sat/downlink', m)

    # ------------------------------------------------------------ uplink handling
    def on_uplink(self, m):
        self.rx += 1
        k = m.get('kind')
        if k == 'gps_fix':
            self.note(f"GPS fix from {C.LABEL.get(m['robot'], m['robot'])} at the staging area "
                      f"(ONA-{m.get('ona')}): {m['lat']:.6f}, {m['lon']:.6f}")
            self.fixes.add(m['robot'])
        elif k == 'drone_status':
            f = m.get('robot', m.get('from'))
            if m.get('status') == 'scouting':
                self.flix[f] = 'sweeping the hillside'
                if self.phase == 'standby':
                    self.phase = 'discovery'
                    self.note('Flix A and Flix B airborne - sweeping the hillside for openings')
            elif m.get('status') == 'landed':
                self.flix[f] = 'landed on its pad'
                self.note(f'{C.LABEL[f]} back on its pad')
            elif m.get('status') == 'emergency':
                self.flix[f] = 'battery empty - emergency landing'
                self.note(f'{C.LABEL[f]}: battery empty, emergency landing (its finds are on the map)', 'alert')
        elif k == 'portal_found':
            p = m['portal']
            self.entrances[p['id']] = p
            self.note(f"{C.LABEL.get(m.get('robot'), 'Flix')} found an opening at y={p['y']:.1f} "
                      f"(sign {p['id']}): {p['state'].upper()} - {p['detail']}",
                      'alert' if p['state'] != 'clear' else 'info')
        elif k == 'drone_report':
            f = m.get('robot', m.get('from'))
            self.reports.add(f)
            self.flix[f] = 'sweep done, hovering'
            for p in m.get('portals', []):
                self.entrances.setdefault(p['id'], p)
            if len(self.reports) == len(C.FLIXES):
                self.dispatch()
        elif k == 'packet':
            self.on_packet(m)
        self.flow('cp_rx', msg=k, mid=m.get('mid'), ona=m.get('ona'))

    # ------------------------------------------------------------ discovery -> dispatch
    def dispatch(self):
        tx, ty = C.LAST_KNOWN
        ents = sorted(self.entrances.values(), key=lambda p: -p['y'])
        for p in ents:
            p['dist'] = round(math.hypot(tx - p['x'], ty - p['y']), 1)
        clear = [p for p in ents if p['state'] == 'clear']
        txt = ', '.join(f"{p['id']} {p['state']}" for p in ents)
        self.note(f'Hillside survey complete: {len(ents)} openings found ({txt})', 'ok')
        for p in ents:
            if p['state'] == 'collapsed':
                self.note(f"Portal {p['id']} rejected: collapsed ({p['detail']})", 'alert')
        self.phase = 'writers_exploring'
        # lessons first: every Writer is briefed with the server's mission memory
        self.downlink({'kind': 'lessons', 'to': 'all', 'via': 'B', 'lessons': self.lessons})
        # a flooded portal: no ground robot can go in; the Flix that found it inspects it (low
        # over the water) before its escort task, and the portal's ONA station places its own
        # fibered first beacon at the portal so that the Flix can write what it finds there
        flooded = {p['by']: p for p in ents if p['state'] == 'flooded'}
        for p in flooded.values():
            ona = p['id'] if p['id'] in C.ONAS else 'B'
            self.note(f"Portal {p['id']} flooded: no ground robot can enter -> {C.LABEL[p['by']]} "
                      f"inspects it over the water; ONA-{ona} places its fibered first beacon "
                      f"at the portal", 'alert')
            self.downlink({'kind': 'entrance_beacon', 'entrance': p['id'], 'ona': ona})
        for w, p in zip(C.WRITERS, clear):
            self.assign[w] = p['id']
            ona = p['id'] if p['id'] in C.ONAS else 'B'
            self.writer[w]['status'] = f"entering portal {p['id']} (DFS)"
            self.writer[w]['last_rx'] = None        # watchdog starts with its first record
            self.note(f"GO -> {C.LABEL[w]} + {C.LABEL[C.PAIR[w]]} through portal {p['id']} "
                      f"(fiber to ONA-{ona})", 'ok')
            self.downlink({'kind': 'go', 'to': w, 'entrance': p['id'], 'via': 'B',
                           'portal': [p['x'], p['y']], 'lkp': list(C.LAST_KNOWN)})
            f = C.PAIR[w]
            task = {'kind': 'flix_task', 'to': f, 'entrance': p['id'],
                    'via': 'A' if f == 'flix_a' else 'B', 'portal': [p['x'], p['y']]}
            fp = flooded.get(f)
            if fp is not None:
                task['inspect'] = {'entrance': fp['id'], 'portal': [fp['x'], fp['y']]}
                self.flix[f] = (f'inspecting flooded portal {fp["id"]}, then fast writer with '
                                f'{C.LABEL[w]} (portal {p["id"]})')
            else:
                self.flix[f] = f'fast writer with {C.LABEL[w]} (portal {p["id"]})'
            self.downlink(task)

    # ------------------------------------------------------------ packets
    def victim_key(self, x, y, r=5.0):
        best = None
        for k, e in self.events.items():
            if e['type'] == 'VICTIM':
                d = math.hypot(e['mx'] - x, e['my'] - y)
                if d < r + e.get('r', 0.0) and (best is None or d < best[0]):
                    best = (d, k)
        return None if best is None else best[1]

    def writer_of(self, p):
        r = C.ROBOT_OF_ID.get(p['id'])
        if r in C.WRITERS:
            return r
        for w, (lo, hi) in C.BEACON_IDS.items():
            if w in C.WRITERS and lo <= p['id'] <= hi:
                return w
        r = C.SRC_OF_ID.get(p.get('src', 0))
        return r if r in C.WRITERS else None

    def on_packet(self, m):
        p = m['pkt']
        t, bid = p['type'], p['id']
        ev = C.EV_NAMES.get(t, str(t))
        w = self.writer_of(p)
        if w is not None and p.get('src') == C.SRC_ID[w] and not m.get('refresh') \
                and p.get('eid', 0) < 128:
            self.writer[w]['last_rx'] = self.now()          # anything the Writer itself wrote
        if t in (C.EV_ENTRANCE, C.EV_PLACEMENT, C.EV_JUNCTION):
            known = bid in self.beacons
            role = {C.EV_ENTRANCE: 'entrance', C.EV_JUNCTION: 'junction'}.get(t, 'trail')
            b = self.beacons.get(bid, {})
            if known and b.get('role') in ('victim', 'gas', 'entrance', 'junction'):
                role = b['role'] if not (role == 'junction' and b['role'] == 'trail') else role
            owner = w or ('ONA-C crew' if p.get('src') == C.SRC_ID['ona'] else '')
            b.update({'id': bid, 'lat': m['lat'], 'lon': m['lon'], 'alt': m.get('alt'),
                      'parent': p['parent'], 'alive': b.get('alive', True), 'role': role,
                      't_ms': p['t_ms'], 'relocated': b.get('relocated', False),
                      'next_out': p.get('next_out', 0), 'owner': owner, 'ona': m.get('ona')})
            b['mx'], b['my'] = p['x'], p['y']
            if t == C.EV_JUNCTION:
                b['exits_code'] = p['a']
            self.beacons[bid] = b
            if not known:
                if w is not None:
                    self.writer[w]['beacons'] += 1
                if t == C.EV_ENTRANCE and w is None:
                    self.note(f"B{bid} ENTRANCE beacon of the flooded portal placed by the ONA-"
                              f"{m.get('ona')} crew - fibre to ONA-{m.get('ona')}")
                elif t == C.EV_ENTRANCE:
                    self.note(f"B{bid} ENTRANCE beacon at portal {self.assign.get(w, '?')} - fiber to "
                              f"ONA-{m.get('ona')} ({C.LABEL.get(w, '')})")
                else:
                    self.note(f"B{bid} {role} placed by {C.LABEL.get(w, '?')}, parent="
                              f"{'ONA' if p['parent'] == 0 else 'B%d' % p['parent']}, "
                              f"toward exit B{p.get('next_out', 0)}, hops={m['hops']}"
                              + (' (junction = mapping event)' if role == 'junction' else ''))
                self.schedule_update()
            elif t == C.EV_JUNCTION and not m.get('refresh'):
                self.writer_status_from(w, f'DFS at junction B{bid}')
        elif t == C.EV_CSI_NODE:
            nid = p['a']
            if nid not in self.csi_nodes:
                self.csi_nodes[nid] = {'id': nid, 'agg': bid, 'mx': p['x'], 'my': p['y'],
                                       'owner': C.SRC_OF_ID.get(p.get('src', 0), '')}
                self.note(f"CSI node N{nid} ({C.SRC_NAME.get(p.get('src', 0))}) streams to B{bid}: "
                          f"{p['b']} node(s) in that cluster - Wi-Fi CSI sensing grows locally")
        elif t in (C.EV_VICTIM, C.EV_PRESENCE):
            self.on_detection(m, p)
        elif t == C.EV_HAZARD:
            key = f'H{bid}'
            if p.get('src') == C.SRC_ID['executor'] and key in self.events:
                if self.events[key]['status'] == 'CONFIRMED':
                    return
                self.events[key]['status'] = 'CONFIRMED'
                self.stats['gas_real'] += 1
                self.note(f'HAZARD at B{bid} confirmed by the Executor ({value_text(p)})', 'alert')
            elif key in self.events:
                self.events[key]['last_rx'] = round(self.now(), 1)
            else:
                self.events[key] = {'key': key, 'beacon': bid, 'type': 'HAZARD', 'lat': m['lat'],
                                    'lon': m['lon'], 'mx': p['x'], 'my': p['y'], 'alt': m.get('alt'),
                                    't_ms': p['t_ms'], 'status': 'REPORTED', 'conf': p['conf'],
                                    'value': value_text(p),
                                    'sources': [f"{p.get('source', 'Writer')} CH4 detector"],
                                    'note': 'gas', 'last_rx': round(self.now(), 1)}
                if bid in self.beacons:
                    self.beacons[bid]['role'] = 'gas'
                self.note(f'HAZARDOUS AREA at B{bid}: {value_text(p)} (alarm {C.GAS_ALARM} %)', 'alert')
                self.schedule_update()
        elif t in (C.EV_EXPLORATION_COMPLETE, C.EV_WRITER_EXIT):
            w = C.ROBOT_OF_ID.get(bid)
            if w in self.writer:
                why = 'DFS complete' if t == C.EV_EXPLORATION_COMPLETE else 'beacon stock empty'
                self.writer[w]['done'] = True
                self.writer[w]['battery'] = p['conf']
                self.writer[w]['status'] = (f'{why} - leaving by the nearest exit to its ONA charger '
                                            f'({p["a"]} beacons left)')
                self.note(f'{C.LABEL[w]}: {why}, {p["a"]} beacons left, ~{2 * p["b"]} m explored, '
                          f'battery {p["conf"]:.0%} - leaving the mine', 'ok')
        elif t in (C.EV_VICTIM_CONFIRMED, C.EV_FALSE_POSITIVE) and \
                (p.get('src', 0), p.get('eid', 0), t) in self.obs_seen:
            pass                                        # re-broadcast of a record already handled
        elif t == C.EV_VICTIM_CONFIRMED:
            self.obs_seen[(p.get('src', 0), p.get('eid', 0), t)] = self.exec_target
            self.on_confirmed(m, p)
        elif t == C.EV_FALSE_POSITIVE:
            self.obs_seen[(p.get('src', 0), p.get('eid', 0), t)] = self.exec_target
            if p['b'] == C.EV_VICTIM:
                self.on_not_found(m, p)
            else:
                key = f'H{bid}'
                if key in self.events:
                    self.events[key]['status'] = 'FALSE POSITIVE'
                self.stats['gas_fp'] += 1
                self.note(f'Executor re-measured gas at B{bid}: clean air -> FALSE POSITIVE '
                          f'(stored in the mission memory)', 'ok')
        elif t == C.EV_ROUTE_HEALED:
            b = self.beacons.get(bid)
            if b:
                b.update({'lat': m['lat'], 'lon': m['lon'], 'mx': p['x'], 'my': p['y'],
                          'parent': p['parent'], 'relocated': bool(p['flags'] & C.FL_RELOCATED)})
            moved = 'relocated and ' if p['flags'] & C.FL_RELOCATED else ''
            self.note(f"Self-healing: B{bid} {moved}re-linked to B{p['parent']}", 'ok')
            self.schedule_update()
        elif t == C.EV_BEACON_LOST:
            b = self.beacons.get(bid)
            if b:
                b['alive'] = False
                self.stats['beacon_lost'].append({'x': round(b['mx'], 1), 'y': round(b['my'], 1)})
            self.note(f"B{bid} reported LOST by B{p['parent']}", 'alert')
            self.schedule_update()
        elif t == C.EV_STATUS:
            w = C.ROBOT_OF_ID.get(bid)
            if w in self.writer:
                self.writer[w]['pos'] = (round(p['x'], 1), round(p['y'], 1))
                self.writer[w]['stock'] = p['a']
                self.writer[w]['explored'] = 2 * p['b']
                self.writer[w]['battery'] = p['conf']
        if p['flags'] & C.FL_STORED_FWD:
            self.note(f'  ({ev} from B{bid} was held and forwarded after the chain healed)')

    def writer_status_from(self, w, txt):
        if w in self.writer and not self.writer[w]['lost'] and not self.writer[w]['done']:
            self.writer[w]['status'] = txt

    def writer_lost(self, w, status, pos=None):
        if self.writer[w]['lost']:
            return
        self.writer[w]['lost'] = True
        self.writer[w]['status'] = status
        if pos:
            self.stats['writer_lost'].append({'x': round(pos[0], 1), 'y': round(pos[1], 1)})
        self.note(f'{C.LABEL[w]} lost - {status}', 'alert')
        self.note(f'Server: releasing the branches claimed by {C.LABEL[w]} - the other Writer may '
                  f'take them over; everything it found stays in the beacons', 'alert')
        self.flow('writer_lost', robot=w)
        self.downlink({'kind': 'release_claims', 'robot': w, 'ona': 'B'})

    # ------------------------------------------------------------ victim detection fusion
    def on_detection(self, m, p):
        """Camera (VICTIM: a position) and Wi-Fi CSI (PRESENCE: a region) observations of the same
        person are combined into ONE victim hypothesis: best confidence per modality, the two
        modalities fused as independent evidence."""
        okey = (p.get('src', 0), p.get('eid', 0), p['type'])
        cam = p['type'] == C.EV_VICTIM
        name = obs_name(p)
        key = self.obs_seen.get(okey)
        new = False
        if key is None:
            x, y = p['x'], p['y']
            r = 0.0 if cam else float(p['a'])
            best = None
            for k, e in self.events.items():
                if e['type'] != 'VICTIM':
                    continue
                d = math.hypot(e['mx'] - x, e['my'] - y)
                lim = (5.0 if e['r'] == 0.0 else e['r'] + 2.0) if cam else r + 2.0 + e['r'] * 0.5
                if d < lim and (best is None or d < best[0]):
                    best = (d, k)
            key = best[1] if best else None
            if key is None:
                self.n_victims += 1
                key = f'V{self.n_victims}'
                self.events[key] = {
                    'key': key, 'type': 'VICTIM', 'lat': m['lat'], 'lon': m['lon'],
                    'alt': m.get('alt'), 'mx': x, 'my': y, 'r': r, 't_ms': p['t_ms'],
                    'status': 'REPORTED', 'cam': 0.0, 'csi': 0.0, 'conf': 0.0, 'obs': [],
                    'sources': [], 'beacon': None, 'stored': [], 'no_ground': False, 'note': '',
                    'ona': m.get('ona'), 'last_rx': round(self.now(), 1), 'found_by': name}
                self.stats['victims'].append({'x': round(x, 1), 'y': round(y, 1)})
                new = True
            else:
                new = False
            self.obs_seen[okey] = key
            e = self.events[key]
            if cam and e['r'] > 0.0:
                # a camera find inside a CSI region: the hypothesis is now localised
                e.update({'mx': x, 'my': y, 'r': 0.0, 'lat': m['lat'], 'lon': m['lon'],
                          'alt': m.get('alt')})
            e['obs'].append({'name': name, 'kind': 'camera' if cam else 'csi', 'conf': p['conf'],
                             'src': okey[0], 'eid': okey[1], 't': round(p['t_ms'] / 1000.0, 1),
                             'region': 0 if cam else p['a'], 'links': 0 if cam else p['b']})
            if name not in e['sources']:
                e['sources'].append(name)
        e = self.events[key]
        for o in e['obs']:
            if (o['src'], o['eid']) == okey[:2] and o['kind'] == ('camera' if cam else 'csi'):
                o['conf'] = max(o['conf'], p['conf'])
        if cam:
            e['cam'] = max(e['cam'], p['conf'])
        else:
            e['csi'] = max(e['csi'], p['conf'])
        old = e['conf']
        e['conf'] = round(fuse_confidence(e['cam'], e['csi']), 3)
        e['last_rx'] = round(self.now(), 1)
        bid = p['id']
        if bid in self.beacons:
            if bid not in e['stored']:
                e['stored'].append(bid)
            if self.beacons[bid].get('role') in ('trail', 'victim') and cam and \
                    p.get('src') in (C.SRC_ID['writer_a'], C.SRC_ID['writer_b']) and \
                    math.hypot(self.beacons[bid]['mx'] - p['x'], self.beacons[bid]['my'] - p['y']) < 5.0:
                self.beacons[bid]['role'] = 'victim'
                e['beacon'] = bid
        if p['flags'] & C.FL_NO_GROUND and not e['no_ground']:
            e['no_ground'] = True
            e['note'] = 'beyond the flooded sump - not reachable by ground robots (divers / pumping)'
        if new and okey not in getattr(self, '_noted', set()):
            self._noted = getattr(self, '_noted', set()) | {okey}
            what = 'camera' if cam else f'Wi-Fi CSI presence ({p["a"]} m region)'
            self.note(f'VICTIM HYPOTHESIS {key}: {what} by {name}, confidence {p["conf"]:.0%} at '
                      f'{m["lat"]:.6f}, {m["lon"]:.6f}' + (f' ({e["note"]})' if e['note'] else ''), 'alert')
            self.flow('fusion', key=key, conf=e['conf'], cam=e['cam'], csi=e['csi'], new=True,
                      x=e['mx'], y=e['my'], r=e['r'], by=name, no_ground=e['no_ground'])
            self.maybe_brief()
        elif okey not in getattr(self, '_noted', set()):
            self._noted = getattr(self, '_noted', set()) | {okey}
            self.note(f'{key}: + {name} {p["conf"]:.0%} -> fused confidence {old:.0%} -> '
                      f'{e["conf"]:.0%} (camera {e["cam"]:.0%}, CSI {e["csi"]:.0%})', 'ok')
            self.flow('fusion', key=key, conf=e['conf'], cam=e['cam'], csi=e['csi'], new=False,
                      x=e['mx'], y=e['my'], r=e['r'], by=name, old=old)
            self.schedule_update()

    def on_confirmed(self, m, p):
        key = self.exec_target or self.victim_key(p['x'], p['y'])
        if key in self.events:
            e = self.events[key]
            e['status'] = 'CONFIRMED'
            e['cam'] = max(e['cam'], p['conf'])
            e['conf'] = round(fuse_confidence(e['cam'], e['csi']), 3)
            e['obs'].append({'name': obs_name(p), 'kind': 'camera', 'conf': p['conf'],
                             'src': p.get('src', 0), 'eid': p.get('eid', 0),
                             't': round(p['t_ms'] / 1000.0, 1), 'region': 0, 'links': 0})
            if 'Executor camera (on site)' not in e['sources']:
                e['sources'].append('Executor camera (on site)')
            if e['r'] > 0.0:
                e.update({'mx': p['x'], 'my': p['y'], 'r': 0.0, 'lat': m['lat'], 'lon': m['lon']})
            self.visited.append(key)
        self.executor_status = f'at {key}: person identified - first aid (oxygen self-rescuer + radio)'
        self.note(f"Executor CONFIRMED {key} ({value_text(p)}); delivering a self-rescuer (SCSR) "
                  f"and a radio", 'ok')
        self.release_executor(key)

    def on_not_found(self, m, p):
        key = self.exec_target
        if key in self.events:
            self.events[key]['status'] = 'NOT FOUND'
            self.events[key]['note'] = 'nobody found at the hypothesis by the Executor camera'
            self.visited.append(key)
        self.executor_status = f'{key}: nobody found there - next hypothesis'
        self.note(f'Executor: nobody at {key} (uncertain detection resolved on site)', 'alert')
        self.release_executor(key, aid=False)

    def release_executor(self, key, aid=True):
        self.exec_at = (self.briefing or {}).get('target') or (self.events.get(key) or {}).get('beacon')
        if self.briefing:
            self.exec_odo = getattr(self, 'exec_odo', 0.0) + self.briefing.get('dist', 0.0)
            self.exec_time = getattr(self, 'exec_time', 0.0) + max(1.0, self.now() - self.briefing['t'])
        self.exec_busy = False
        self.exec_target = None
        # the next briefing goes out now: the Executor receives it during the first aid and
        # leaves as soon as the aid is done (no extra stop in front of the victim)
        self._aid_until = self.now() + (C.FIRST_AID_S if aid else 0.0)
        self.after(0.5, self.maybe_brief)

    # ------------------------------------------------------------ trail graph + briefing
    def trail_graph(self):
        adj = {}
        for k, b in self.beacons.items():
            n = b.get('next_out', 0)
            if n and n in self.beacons and 'mx' in b and 'mx' in self.beacons[n]:
                o = self.beacons[n]
                d = math.hypot(o['mx'] - b['mx'], o['my'] - b['my'])
                adj.setdefault(k, {})[n] = d
                adj.setdefault(n, {})[k] = d
        return adj

    def victim_beacon(self, e):
        """Trail end for a hypothesis: its victim beacon, else the nearest beacon by it, else (a
        person seen from afar, e.g. by a Flix, before any Writer got there) the nearest beacon in
        line of sight on the map: the Executor covers the last metres on its camera."""
        if e.get('beacon') in self.beacons and self.beacons[e['beacon']].get('alive', True):
            return e['beacon']
        best = None
        for k, b in self.beacons.items():
            if 'mx' not in b or not b.get('alive', True):
                continue
            d = math.hypot(b['mx'] - e['mx'], b['my'] - e['my'])
            near = d < 8.0 + e.get('r', 0.0)
            if not near and (d > C.EXEC_SIGHT_R or not los((b['mx'], b['my']), (e['mx'], e['my']))):
                continue
            if best is None or d < best[0]:
                best = (d, k)
        return None if best is None else best[1]

    def plan(self, e):
        adj = self.trail_graph()
        tgt = self.victim_beacon(e)
        if tgt is None:
            return None
        if self.exec_at is not None and self.exec_at in self.beacons:
            d, path = shortest_path(adj, self.exec_at, tgt)
            return (d, path) if d is not None else None
        sx, sy = C.SPAWN['executor'][:2]
        best = None
        for k, b in self.beacons.items():
            if b['role'] != 'entrance':
                continue
            d, path = shortest_path(adj, k, tgt)
            if d is None:
                continue
            d += math.hypot(b['mx'] - sx, b['my'] - sy)
            if best is None or d < best[0]:
                best = (d, path)
        return best

    def candidates(self):
        """Hypotheses the Executor can reach on the beacon trail: most confident first (5 %
        classes), then the nearest."""
        out = []
        for k, e in self.events.items():
            if e['type'] != 'VICTIM' or e['no_ground'] or e['status'] != 'REPORTED' \
                    or e['conf'] < C.CONF_MIN_EXEC:
                continue
            pl = self.plan(e)
            if pl is None:
                continue
            out.append((-conf_class(e['conf']), pl[0], k, pl))
        out.sort()
        return out

    def maybe_brief(self):
        """Executor free + a reachable victim hypothesis -> brief it toward the most confident
        one (the nearest among equally confident ones)."""
        if self.exec_busy or self.phase in ('standby', 'discovery'):
            return
        c = self.candidates()
        if not c:
            if self.visited:
                self.executor_status = (f'with {self.visited[-1]} (first aid) - no other reachable '
                                        f'miner yet')
                self.check_complete()
            return
        cc, d, key, (dd, path) = c[0]
        e = self.events[key]
        others = ', '.join(f'{k} {self.events[k]["conf"]:.0%} / {d2:.0f} m' for _, d2, k, _ in c[1:])
        same = [k for c2, _, k, _ in c[1:] if c2 == cc]
        self.exec_target = key
        self.exec_busy = True
        why = (f'most confident ({e["conf"]:.0%})' + (', nearest in its class' if same else '')
               + (f' before {others}' if others else ''))
        self.cand_log = (getattr(self, 'cand_log', []) + [
            {'t': round(self.now(), 1), 'chosen': key,
             'cands': [[k, self.events[k]['conf'], round(d2, 1)] for _, d2, k, _ in c]}])[-20:]
        self.brief(key, path, update=False, why=why, dist=d)

    def verify_list(self, path):
        """Unconfirmed gas events on the Executor's route: to be re-measured on the way."""
        pts = [(self.beacons[i]['mx'], self.beacons[i]['my']) for i in path if 'mx' in self.beacons[i]]
        verify = []
        for ev in self.events.values():
            if ev['type'] == 'HAZARD' and ev['status'] == 'REPORTED' and len(pts) > 1:
                d = min(_seg_dist((ev['mx'], ev['my']), a, b) for a, b in zip(pts[:-1], pts[1:]))
                if d <= 3.0:
                    verify.append({'id': ev['beacon'], 'type': C.EV_HAZARD,
                                   'lat': ev['lat'], 'lon': ev['lon']})
        return verify

    def brief(self, key, path, update=False, why='', dist=0.0):
        e = self.events[key]
        verify = self.verify_list(path)
        first = self.first_briefing is None
        kind = 'briefing_update' if update else 'briefing'
        msg = {'kind': kind, 'target': path[-1], 'ona': 'B', 'conf': e['conf'],
               'radius': e.get('r', 0.0),
               'path': [{'id': i, 'lat': self.beacons[i]['lat'], 'lon': self.beacons[i]['lon']}
                        for i in path],
               'victim': {'lat': e['lat'], 'lon': e['lon']}, 'verify': verify}
        speed = self.lessons.get('exec_speed', 0.7)
        self.briefing = {'t': round(self.now(), 1), 'target': path[-1], 'path': path,
                         'verify': [v['id'] for v in verify], 'update': update, 'victim': key,
                         'conf': e['conf'], 'why': why, 'dist': round(dist, 1),
                         'eta': round(dist / max(0.2, speed)),
                         'writers': {w: self.writer[w]['status'] for w in C.WRITERS}}
        if not update:
            if first:
                self.first_briefing = dict(self.briefing)
                self.stats['t_exec_start'] = self.now()
                self.phase = 'executor_briefed'
            aid = self.now() < getattr(self, '_aid_until', 0.0) and self.visited
            self.executor_status = (f'first aid at {self.visited[-1]}, then -> {key}' if aid else
                                    f'briefed -> {key}') + f' (confidence {e["conf"]:.0%})'
            self.note(f"BRIEFING -> Executor: {key} ({why}), route "
                      f"{' > '.join('B%d' % i for i in path)} ({dist:.0f} m, ETA ~{self.briefing['eta']} s)"
                      + ('' if not first else ' - Writers still exploring')
                      + (' - leaves as soon as the first aid is done' if aid else ''), 'ok')
        else:
            self.note(f"BRIEFING UPDATE -> Executor: route {' > '.join('B%d' % i for i in path)}", 'ok')
        self.downlink(msg)

    def schedule_update(self):
        if self.briefing is None or not self.exec_busy or self.update_pending:
            if not self.exec_busy:
                self.maybe_brief()
            return
        self.update_pending = True

        def go():
            self.update_pending = False
            key = self.exec_target
            if key is None or not self.exec_busy:
                return
            pl = self.plan(self.events[key])
            if pl is None:
                return
            path = pl[1]
            # keep the Executor's start: replan from the briefing's first beacon
            if self.briefing and self.briefing['path'] and self.briefing['path'][0] in self.beacons:
                d, p2 = shortest_path(self.trail_graph(), self.briefing['path'][0], path[-1])
                if p2:
                    path = p2
            self._upd_why = self.briefing.get('why', '')
            new_v = [v['id'] for v in self.verify_list(path)]
            if path != self.briefing['path'] or new_v != self.briefing['verify'] or \
                    any(self.beacons[i].get('relocated') for i in path):
                d = sum(math.hypot(self.beacons[a]['mx'] - self.beacons[b]['mx'],
                                   self.beacons[a]['my'] - self.beacons[b]['my'])
                        for a, b in zip(path[:-1], path[1:]))
                self.brief(key, path, update=True, why=self._upd_why, dist=d)
        self.after(0.6, go)

    # ------------------------------------------------------------ watchdog + mission end
    def watchdog(self, now):
        for w, s in self.writer.items():
            if s['lost'] or s['done'] or s['last_rx'] is None:
                continue
            lim = C.WRITER_SILENCE
            last = self.beacons.get(max((k for k, b in self.beacons.items()
                                         if b.get('owner') == w), default=-1))
            if last and any(math.hypot(last['mx'] - z['x'], last['my'] - z['y']) < z['r'] + 10
                            for z in self.lessons.get('collapse', [])):
                lim = C.WRITER_SILENCE_RISK
            if now - s['last_rx'] >= lim:
                pos = s.get('pos')
                self.writer_lost(w, f'NO CONTACT for {lim:.0f} s ({lim / C.HEARTBEAT_S:.0f} missed '
                                    f'heartbeats) - presumed lost'
                                    + (f', last STATUS at ({pos[0]}, {pos[1]})' if pos else ''), pos)

    def check_complete(self):
        if self.mission_saved:
            return
        writers_out = all(s['lost'] or s['done'] for s in self.writer.values())
        open_v = [k for k, e in self.events.items() if e['type'] == 'VICTIM' and not e['no_ground']
                  and e['status'] == 'REPORTED']
        if writers_out and open_v and not self.exec_busy:
            # the exploration is over and the Executor has nobody it can reach: what is left
            # stays on the map for the next mission (incomplete exploration / uncertain detection)
            for k in open_v:
                e = self.events[k]
                if e.get('note'):
                    continue
                if e['conf'] < C.CONF_MIN_EXEC:
                    e['note'] = f'uncertain ({e["conf"]:.0%}): below the Executor threshold - next mission'
                elif self.plan(e) is None:
                    e['note'] = 'no beacon trail reaches this hypothesis - next mission'
                else:
                    continue
                self.note(f'{k}: {e["note"]}', 'alert')
            open_v = [k for k in open_v if self.plan(self.events[k]) is not None
                      and self.events[k]['conf'] >= C.CONF_MIN_EXEC]
        if not writers_out or open_v or self.exec_busy:
            return
        for k, e in self.events.items():
            if e['type'] == 'VICTIM' and e['no_ground'] and not e.get('flagged'):
                e['flagged'] = True
                self.note(f'{k}: {e["note"]} - reported to the rescue team', 'alert')
        self.phase = 'mission_complete'
        self.mission_saved = True
        mission = {'utc': int(self.utc()), 'scenario': self.scenario,
                   'onas': sorted({b.get('ona') for b in self.beacons.values() if b.get('ona')}),
                   'beacon_lost': self.stats['beacon_lost'], 'writer_lost': self.stats['writer_lost'],
                   'gas_fp': self.stats['gas_fp'],
                   'gas_real': sum(1 for e in self.events.values()
                                   if e['type'] == 'HAZARD' and e['status'] != 'FALSE POSITIVE'),
                   'victims': self.stats['victims'],
                   'exec_speed': round(self.exec_odo / self.exec_time, 3)
                   if getattr(self, 'exec_odo', 0) else None}
        saved = self.memory.save(mission)
        self.note(f'MISSION COMPLETE - stored in the mission server memory '
                  f'({len(self.memory.missions)} missions{"" if saved else ", not written: read-only"}) '
                  f'for the next mission at any ONA', 'ok')
        self.end_summary(saved)

    def end_summary(self, saved):
        """Mission result, shown in the 2D window and as a banner in the terminal: the
        simulation has nothing left to do (it keeps running until Ctrl+C)."""
        vic = [e for e in self.events.values() if e['type'] == 'VICTIM']
        haz = [e for e in self.events.values() if e['type'] == 'HAZARD']
        t0 = C.MISSION_START_DELAY
        self.summary = {
            't': round(self.now(), 1), 'mission_s': round(self.now() - t0, 1),
            'confirmed': sorted(e['key'] for e in vic if e['status'] == 'CONFIRMED'),
            'not_found': sorted(e['key'] for e in vic if e['status'] == 'NOT FOUND'),
            'rescue_team': sorted(e['key'] for e in vic if e['no_ground']),
            'next_mission': sorted(e['key'] for e in vic if not e['no_ground']
                                   and e['status'] == 'REPORTED'),
            'gas_real': sum(1 for e in haz if e['status'] != 'FALSE POSITIVE'),
            'gas_fp': sum(1 for e in haz if e['status'] == 'FALSE POSITIVE'),
            'beacons': len(self.beacons), 'beacons_lost': len(self.stats['beacon_lost']),
            'writers_lost': len(self.stats['writer_lost']), 'stored': bool(saved)}
        s = self.summary
        m, sec = divmod(int(s['mission_s']), 60)
        rows = [f"mission time   {m} min {sec:02d} s (sim time {s['t']:.0f} s)",
                f"victims        {len(s['confirmed'])} confirmed by the Executor"
                + (f" ({', '.join(s['confirmed'])})" if s['confirmed'] else ''),
                f"               {len(s['rescue_team'])} reported to the rescue team (no ground access)"
                + (f" ({', '.join(s['rescue_team'])})" if s['rescue_team'] else '')]
        if s['not_found']:
            rows.append(f"               {len(s['not_found'])} not found on site ({', '.join(s['not_found'])})")
        if s['next_mission']:
            rows.append(f"               {len(s['next_mission'])} left for the next mission "
                        f"({', '.join(s['next_mission'])})")
        rows += [f"gas            {s['gas_real']} confirmed, {s['gas_fp']} false positive(s)",
                 f"beacons        {s['beacons']} placed, {s['beacons_lost']} lost; "
                 f"Writers lost: {s['writers_lost']}",
                 'the robots have finished their tasks: nothing else will happen.',
                 'press Ctrl+C in this terminal to stop the simulation.']
        bar = '=' * 72
        self.get_logger().info('\n'.join(['', bar, '  MISSION COMPLETE', bar]
                                         + ['  ' + r for r in rows] + [bar]))

    def publish_state(self):
        now = self.now()
        self.watchdog(now)
        if self.phase not in ('standby', 'discovery'):
            self.maybe_brief()
        self.check_complete()
        ev = []
        for e in self.events.values():
            e2 = dict(e)
            e2['age'] = round(now - e['t_ms'] / 1000.0, 1)
            e2['fresh'] = round(math.exp(-max(0.0, e2['age']) / C.TRAIL_TAU), 3)
            ev.append(e2)
        hyps = sorted([e for e in ev if e['type'] == 'VICTIM'],
                      key=lambda e: (e['no_ground'], e['status'] != 'REPORTED', -conf_class(e['conf'])))
        self.pub('/lm/cp/state', {
            'phase': self.phase, 'events': ev, 'beacons': list(self.beacons.values()),
            'log': self.log[-40:], 'briefing': self.briefing, 'first_briefing': self.first_briefing,
            'entrances': sorted(self.entrances.values(), key=lambda p: -p['y']),
            'assign': self.assign, 'writers': self.writer,
            'writer': '; '.join(f"{C.LABEL[w]}: {s['status']}" for w, s in self.writer.items()),
            'executor': self.executor_status, 'flix': self.flix,
            'drone': '; '.join(f'{C.LABEL[f]}: {s}' for f, s in self.flix.items()),
            'rx': self.rx, 't': now,
            'hypotheses': [{'key': e['key'], 'conf': e['conf'], 'cam': e['cam'], 'csi': e['csi'],
                            'status': e['status'], 'no_ground': e['no_ground'],
                            'sources': e['sources']} for e in hyps],
            'csi_nodes': list(self.csi_nodes.values()), 'cand_log': getattr(self, 'cand_log', []),
            'visited': self.visited, 'exec_target': self.exec_target,
            'summary': getattr(self, 'summary', None),
            'server': {'missions': self.lessons['missions'], 'onas': self.lessons['onas'],
                       'lessons': self.lessons['text'], 'gas_fp': self.lessons.get('gas_fp'),
                       'rockfall': self.lessons['rockfall'], 'collapse': self.lessons['collapse'],
                       'victim_prior': self.lessons['victim_prior'], 'saved': self.mission_saved},
            'entrance': next(iter(self.assign.values()), None)})


def main(args=None):
    spin_node(CommandPost, args)


if __name__ == '__main__':
    main()

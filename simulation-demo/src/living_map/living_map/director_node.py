"""Camera director: drives the Gazebo GUI camera through the story, then removes the hillside
and the tunnel shells for a cutaway overview. Every shot has a caption (shown in the 2D window).

  1. intro: the site, the three Outside Network Area stations and the relay satellite
  2. Flix B sweeping the hillside (it finds B, the collapsed D and the flooded C)
  3. ONA-B: GO and the lessons go out through the dish -> satellite link
  4. Flix B flying low over the water of the flooded portal C toward the air pocket
  5. Writer B exploring (frontiers + DFS + utility; main shot until the Executor is briefed)
  6. Executor from its briefing until the mission is complete (main shot); while it stays
     with a miner because no other one is reachable yet, the camera follows the Writer still
     exploring and comes back to the Executor when it is briefed again
     event cuts on the way: a Flix spotting a person with its camera, a Writer dropping its
     first Wi-Fi CSI node, a CSI presence detection, Writer A taking a junction found by
     Writer B, a roof collapse (the Writer falls silent), a rockfall and the neighbour beacon
     relocating itself, Flix B finding the miner beyond the sump
  7. MISSION COMPLETE: cutaway, hill + tiles removed, top view of everything (the end)

Following uses the GUI's own follow mode (/gui/follow + /gui/follow/offset of the GzScene3D 3D
view, follow gain set in the world): the camera glides behind the robot on every rendered frame. A change of target
is a quick cut (/gui/move_to/pose to a point behind the new target, kept inside the gallery),
then follow. Static shots (intro, wreck, rockfall, cutaway) use /gui/move_to/pose. All GUI
requests go through one ordered queue, so follow / offset / move_to never overtake each other.
"""
import math
import queue
import shutil
import subprocess
import threading
import time

from . import config as C
from .common import LMNode, spin_node
from .geometry import inside_free

# follow offset in the target's own frame (x forward, y left, z up), metres
# (the GUI glides there with the follow gain set in the world: GzScene3D <camera_follow>)
FOLLOW = {'flix_a': (-1.0, 0.0, 0.35), 'flix_b': (-1.0, 0.0, 0.35), 'writer_a': (-1.5, 0.0, 0.8),
          'writer_b': (-1.5, 0.0, 0.8), 'executor': (-1.8, 0.0, 1.0)}
FLOODED_MAX = 24.0                       # s on Flix B over the flooded sump before the Writers
C_STATES = ('c_go', 'inspect_c', 'c_out')
BEACON_FOLLOW = (2.0, 0.0, 0.9)          # ahead of a relocating beacon (its +x = direction of travel)
INTRO_POSE = (-40.0, -24.0, 15.0, (0.0, 0.0, 3.0))          # overlooking hill + staging area
ROCKFALL_HOLD = 4.5                      # s on the rockfall before following the relocating neighbour
ROCKFALL_MAX = 22.0                      # s, then back to the Executor in any case
EXEC_WAIT_CUT = 10.0                     # s the Executor waits with a miner before the camera
                                         # goes to the Writer still exploring (back when re-briefed)


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def cutaway_names(world_kind='fuel'):
    """Models removed for the final cutaway: the hill and the gallery shells (roofs/walls)."""
    if world_kind == 'basic':
        return ['hillside', 'galleries_roof', 'galleries_walls', 'adit_d']
    names = [f'tile_{t[0]}' for t in C.TILES]
    names += [f'blocker_{i}' for i in range(len(C.BLOCKERS))]
    return names + ['hillside', 'adit_d']


def _inside_gallery(x, y, margin):
    """Inside the modelled galleries with a margin from the walls, or outside the mine.
    (Tested on the union of the gallery rectangles, so a joint between two of them is not a wall.)"""
    if x < -0.5:
        return True
    return all(inside_free(x + dx, y + dy)
               for dx, dy in ((0.0, 0.0), (margin, 0.0), (-margin, 0.0), (0.0, margin), (0.0, -margin)))


def chase_point(tx, ty, yaw, back, margin=0.6):
    """Point `back` metres behind (tx, ty), pulled toward the target until it is inside the
    gallery (never behind rock)."""
    cx, cy = tx - back * math.cos(yaw), ty - back * math.sin(yaw)
    if not inside_free(tx, ty) and tx > -0.5:
        return cx, cy
    n = max(1, int(back / 0.1))
    last = (tx, ty)
    for i in range(1, n + 1):
        px, py = tx + (cx - tx) * i / n, ty + (cy - ty) * i / n
        if not _inside_gallery(px, py, margin):
            break
        last = (px, py)
    return last


class Director(LMNode):
    def __init__(self):
        super().__init__('director', publishes=('/lm/camera', '/lm/gz/cmd'))
        self.declare_parameter('dry_run', False)
        self.declare_parameter('world_kind', 'fuel')
        self.declare_parameter('camera_auto', True)
        self.dry = bool(self.get_parameter('dry_run').value)
        self.world_kind = str(self.get_parameter('world_kind').value)
        self.auto = bool(self.get_parameter('camera_auto').value)
        self.ign = shutil.which('ign')
        self.q = queue.Queue()                   # ordered GUI requests
        self.gui_log = []                        # (service, request) - for the dry-run tests
        if not self.dry and self.ign is not None and self.auto:
            threading.Thread(target=self.worker, daemon=True).start()
        self.states, self.poses = {}, {}
        self.cp = {}
        self.beacons = {}
        for n in FOLLOW:
            self.sub(f'/lm/pose/{n}', lambda m, n=n: self.on_pose(n, m))
        self.sub('/lm/cp/state', lambda m: self.__setattr__('cp', m))
        self.sub('/lm/beacons', self.on_beacons)
        self.sub('/lm/flow', self.on_flow)
        self.phase = 'intro'
        self.t_phase = 0.0
        self.target = None
        self.caption = 'The site: hillside, staging area, three Outside Network Areas, relay satellite'
        self.intro_sent = False
        self.rock = None                         # rockfall cut state
        self.event = None                        # current event cut {target, until, caption}
        self.pending = []                        # queued event cuts
        self.seen = set()                        # one-shot cuts already done
        self.following = None                    # (model, offset) the GUI camera follows
        self.t_follow = 0.0
        self.t_last_cut = -99.0
        self.exec_wait_t0 = None                 # since when the Executor waits with a miner
        self.create_timer(0.5, self.tick)

    def on_pose(self, n, m):
        if n == 'executor':
            if m.get('state') != 'awaiting_next':
                self.exec_wait_t0 = None
            elif self.exec_wait_t0 is None:
                self.exec_wait_t0 = self.now()
        self.states[n] = m.get('state', '')
        self.poses[n] = m

    def on_beacons(self, m):
        self.beacons = {b['id']: b for b in m.get('beacons', [])}

    # ------------------------------------------------------------ GUI camera (ordered queue)
    def call(self, service, reqtype, req, pause=0.0):
        self.gui_log.append((service, req))
        self.q.put((service, reqtype, req, pause))

    def worker(self):
        while True:
            service, reqtype, req, pause = self.q.get()
            for attempt in range(3):
                try:
                    r = subprocess.run([self.ign, 'service', '-s', service, '--reqtype', reqtype,
                                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000',
                                        '--req', req], capture_output=True, text=True, timeout=4)
                    ok = r.returncode == 0 and 'true' in r.stdout
                except Exception as e:
                    self.get_logger().warn(f'camera service {service}: {e}')
                    ok = False
                if ok or service != '/gui/move_to/pose' or not self.q.empty():
                    break
                time.sleep(1.0)
            if pause:
                time.sleep(pause)                # let a 0.5 s move_to animation finish

    def unfollow(self):
        self.following = None
        self.call('/gui/follow', 'ignition.msgs.StringMsg', 'data: ""')

    def move_to(self, x, y, z, roll, pitch, yaw):
        """Static camera pose (0.5 s GUI animation). Stops following first."""
        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
        self.unfollow()
        self.call('/gui/move_to/pose', 'ignition.msgs.GUICamera',
                  f'pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
                  f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}',
                  pause=0.6)

    def look_from(self, cx, cy, cz, lx, ly, lz):
        yaw = math.atan2(ly - cy, lx - cx)
        pitch = math.atan2(cz - lz, max(0.05, math.hypot(lx - cx, ly - cy)))
        self.move_to(cx, cy, cz, 0.0, pitch, yaw)

    def follow(self, name, offset):
        """GUI follow mode: smooth, updated on every rendered frame."""
        self.following = (name, offset)
        self.t_follow = self.now()
        self.call('/gui/follow', 'ignition.msgs.StringMsg', f'data: "{name}"')
        ox, oy, oz = offset
        self.call('/gui/follow/offset', 'ignition.msgs.Vector3d', f'x: {ox}, y: {oy}, z: {oz}')

    def target_z(self, name, p):
        return p.get('z', 0.0) if name.startswith('flix') else 0.15

    def cut_to(self, name):
        """Quick cut to a point behind the robot (inside its gallery), then follow it."""
        p = self.poses.get(name)
        off = FOLLOW[name]
        if p:
            tz = self.target_z(name, p)
            cx, cy = chase_point(p['x'], p['y'], p['yaw'], abs(off[0]))
            self.look_from(cx, cy, tz + off[2], p['x'], p['y'], tz)
        self.follow(name, off)

    def set_phase(self, phase, target=None, cut=True, caption=None):
        self.phase = phase
        self.t_phase = self.now()
        self.target = target
        self.t_last_cut = self.now()
        if caption is not None:
            self.caption = caption
        self.get_logger().info(f'camera -> {phase}' + (f' ({target})' if target else ''))
        if cut and target in FOLLOW:
            self.cut_to(target)
        self.publish_camera()

    def publish_camera(self):
        p = self.poses.get(self.target) if self.target in FOLLOW else None
        note = (p or {}).get('note') or (p or {}).get('src') or ''
        self.pub('/lm/camera', {'phase': self.phase, 'target': self.target,
                                'caption': self.caption, 'detail': note})

    # ------------------------------------------------------------ main shot + event cuts
    def main_target(self):
        cpp = self.cp.get('phase')
        if cpp != 'mission_complete' and self.exec_wait_t0 is not None \
                and self.now() - self.exec_wait_t0 > EXEC_WAIT_CUT:
            for w in ('writer_b', 'writer_a'):
                if self.states.get(w) in ('to_portal', 'exploring', 'leaving'):
                    return w, (f'The Executor stays with the miner (first aid) - no other reachable '
                               f'hypothesis yet. Meanwhile {C.LABEL[w]} keeps exploring; the camera '
                               f'goes back to the Executor when it is briefed again')
        if cpp in ('executor_briefed', 'mission_complete') or self.states.get('executor') in (
                'en_route', 'final_approach', 'first_aid', 'awaiting_next', 'waiting_chain'):
            return 'executor', ('Executor: briefed through the ONA, follows the beacon trail to each '
                                'victim hypothesis - most confident first, then the nearest')
        for w in ('writer_b', 'writer_a'):
            if self.states.get(w) in ('to_portal', 'exploring', 'leaving'):
                return w, (f'{C.LABEL[w]} explores the unknown mine (frontiers, modified DFS, '
                           f'utility function), dropping beacons and Wi-Fi CSI nodes')
        return 'writer_b', 'Writers at the staging area'

    def queue_cut(self, key, target, dur, caption, still=None, once=True):
        if once and key in self.seen:
            return
        self.seen.add(key)
        self.pending.append({'target': target, 'dur': dur, 'caption': caption, 'still': still})

    def on_flow(self, m):
        k = m.get('kind')
        if k == 'rockfall' and self.rock is None and self.phase not in ('intro', 'sweep'):
            self.rockfall(m['id'], m['x'], m['y'])
            return
        if self.rock is not None and self.phase == 'rockfall':
            if k == 'heal_plan' and m.get('dead') == self.rock['bid']:
                self.rock['orphan'] = m['at']
                self.rock['moving'] = m.get('move', 0.0) > 0.0
            elif k == 'relinked' and m.get('at') == self.rock.get('orphan'):
                self.rock['t_relinked'] = self.now()
            return
        if k == 'fusion' and m.get('new') and str(m.get('by', '')).startswith('Flix') \
                and not m.get('no_ground'):
            f = 'flix_a' if m['by'].startswith('Flix A') else 'flix_b'
            if self.states.get(f) not in C_STATES:
                self.queue_cut(f'find{m["key"]}', f, 9.0, f'{C.LABEL[f]} (fast writer) spots a person '
                               f'with its camera ({m.get("cam", 0):.0%}) and writes the VICTIM record '
                               f'into the nearest beacon', once=True)
        elif k == 'csi_drop' and m.get('by') in FOLLOW:
            w = m['by']
            self.queue_cut(f'csi_{w}', w, 8.0, f'{C.LABEL[w]} drops an ESP32-S3 Wi-Fi CSI node: the '
                           f'beacons and the Writer are its Wi-Fi sources, B{m.get("agg")} aggregates',
                           once=True)
        elif k == 'presence':
            self.queue_cut('presence', None, 7.0, f'Wi-Fi CSI: the AI classifier of B{m["at"]} reports '
                           f'human presence ({m["conf"]:.0%}) in a {m["r"]:.0f} m region',
                           still=(m['x'], m['y']), once=True)
        elif k == 'junction' and m.get('by') == 'writer_a' and m.get('claim') is not None:
            b = self.beacons.get(m['id'], {})
            if b.get('owner') == 'writer_b':
                self.queue_cut('shared', 'writer_a', 12.0, 'Writer A reaches a junction mapped by '
                               'Writer B: it reads the claims in the beacon and takes a free branch')
        elif k == 'collapse':
            self.queue_cut('collapse', None, 7.0, f'Roof collapse on {C.LABEL[m.get("robot", "writer_b")]}: '
                           'it falls silent - its finds are already in the beacons',
                           still=(m['x'], m['y']), once=True)
        elif k == 'release':
            self.queue_cut('release', 'writer_a', 10.0, 'Server released the lost Writer\'s branches: '
                           'Writer A takes them over', once=True)

    def rockfall(self, bid, bx, by):
        """Cut to the beacon hit by the rockfall, seen from the side of its downstream neighbour."""
        child = next((b for b in self.beacons.values()
                      if b.get('parent') == bid and b.get('alive')), None)
        e = self.poses.get('executor')
        if child is not None:
            yaw = math.atan2(by - child['y'], bx - child['x'])     # neighbour -> hit beacon
        elif e:
            yaw = math.atan2(e['y'] - by, e['x'] - bx)             # Executor side
        else:
            yaw = 0.0
        self.rock = {'bid': bid, 'orphan': None, 'moving': False, 'following': False,
                     't_relinked': None}
        self.event = None
        self.set_phase('rockfall', f'beacon_{bid}', cut=False,
                       caption=f'Rockfall destroys relay beacon B{bid} on the Executor\'s route')
        cx, cy = chase_point(bx, by, yaw, 4.5)
        self.look_from(cx, cy, 1.6, bx, by, 0.3)

    def rockfall_tick(self, t, dt):
        r = self.rock
        if r['orphan'] is not None and r['moving'] and not r['following'] and dt >= ROCKFALL_HOLD:
            r['following'] = True
            self.target = f'beacon_{r["orphan"]}'
            self.caption = f'Self-healing: B{r["orphan"]} drives itself back to re-link the chain'
            self.follow(self.target, BEACON_FOLLOW)
            self.get_logger().info(f'camera -> following B{r["orphan"]} relocating')
        done = r['t_relinked'] is not None and t - r['t_relinked'] >= 2.5 and dt >= ROCKFALL_HOLD + 1.5
        if done or dt >= ROCKFALL_MAX:
            tgt, cap = self.main_target()
            self.set_phase('executor' if tgt == 'executor' else 'writers', tgt, caption=cap)

    def start_event(self, ev):
        self.event = dict(ev, t0=self.now())
        if ev.get('still') is not None and len(ev['still']) == 4:
            x, y, z, yaw = ev['still']                  # a robot, seen from behind and above
            self.set_phase('event', None, cut=False, caption=ev['caption'])
            cx, cy = chase_point(x, y, yaw, 3.2)
            self.look_from(cx, cy, z + 1.1, x, y, z)
        elif ev.get('still') is not None:
            x, y = ev['still']
            self.set_phase('event', None, cut=False, caption=ev['caption'])
            cx, cy = chase_point(x, y, 0.8, 3.0)
            self.look_from(cx, cy, 1.5, x, y, 0.2)
        else:
            self.set_phase('event', ev['target'], caption=ev['caption'])

    # ------------------------------------------------------------ story
    def tick(self):
        t = self.now()
        dt = t - self.t_phase
        s = self.states
        cpp = self.cp.get('phase')
        if self.phase == 'intro':
            if not self.intro_sent:
                self.intro_sent = True
                x, y, z, (lx, ly, lz) = INTRO_POSE
                self.look_from(x, y, z, lx, ly, lz)
            if self.started():
                self.set_phase('sweep', 'flix_b', caption='Flix A and Flix B sweep the hillside: the '
                               'side ToF finds every opening, the camera classifies it')
        elif self.phase == 'sweep':
            if cpp not in (None, 'standby', 'discovery') and dt > 3.0:
                ox, oy = C.ONAS['B']['pos']
                sx, sy, sz = C.SAT_POS
                dx, dy = ox - 2.4, oy - 1.2                     # dish of ONA-B
                h = math.atan2(sy - dy, sx - dx)
                self.set_phase('ona', None, cut=False, caption='ONA-B (PC + satellite dish): GO, '
                               'portal assignment and the server\'s lessons go out via the relay satellite')
                # from behind the dish, looking up the beam toward the satellite
                self.look_from(dx - 9.0 * math.cos(h), dy - 9.0 * math.sin(h), 2.5,
                               dx + 0.25 * (sx - dx), dy + 0.25 * (sy - dy), 0.25 * sz)
        elif self.phase == 'ona':
            if dt > 6.0:
                if s.get('flix_b') in C_STATES:
                    self.set_phase('flooded', 'flix_b', caption='Flooded portal C: Flix B flies low over '
                                   'the water of the sump toward the air pocket (camera + CSI); '
                                   'ONA-C placed its fibered first beacon at the portal')
                else:
                    tgt, cap = self.main_target()
                    self.set_phase('writers', tgt, caption=cap)
        elif self.phase == 'flooded':
            if dt > FLOODED_MAX or s.get('flix_b') not in C_STATES + ('to_victim',):
                tgt, cap = self.main_target()
                self.set_phase('writers', tgt, caption=cap)
        elif self.phase == 'rockfall':
            self.rockfall_tick(t, dt)
        elif self.phase == 'event':
            if dt >= self.event['dur']:
                self.event = None
                tgt, cap = self.main_target()
                self.set_phase('executor' if tgt == 'executor' else 'writers', tgt, caption=cap)
        elif self.phase in ('writers', 'executor'):
            tgt, cap = self.main_target()
            fb = self.poses.get('flix_b') or {}
            if fb.get('in_c') and s.get('flix_b') == 'to_victim':
                self.queue_cut('c_find', 'flix_b', 9.0, 'Flix B found a miner in the air pocket beyond '
                               'the flooded sump: it carries the record out to ONA-C\'s beacon '
                               '(not reachable by ground robots)', once=True)
            if self.pending and t - self.t_last_cut > 6.0:
                self.start_event(self.pending.pop(0))
            elif tgt != self.target:
                self.set_phase('executor' if tgt == 'executor' else 'writers', tgt, caption=cap)
            elif cpp == 'mission_complete':
                if not hasattr(self, '_done_t'):
                    self._done_t = t
                if t - self._done_t > 6.0:
                    self.cutaway()
        # re-assert the follow every few seconds: harmless for the GUI (no jump), and it catches
        # a GUI that was still loading, or a camera moved by hand
        if self.following is not None and t - self.t_follow >= 4.0:
            self.follow(*self.following)
        self.publish_camera()

    def cutaway(self):
        self.set_phase('cutaway', caption='MISSION COMPLETE - cutaway: the living map of the whole '
                       'mine. The simulation is over: press Ctrl+C in the terminal to stop it.')
        for n in cutaway_names(self.world_kind):
            self.pub('/lm/gz/cmd', {'op': 'remove', 'name': n})
        # top view over the whole site: staging area, all lines, flooded sump, every miner
        self.move_to(45.0, -6.0, 185.0, 0.0, 1.5607, 1.5708)     # north up, east right


def main(args=None):
    spin_node(Director, args)


if __name__ == '__main__':
    main()

"""Flix A / Flix B (fast writers, 65 g ESP32-S3 quadcopters - github.com/okalachev/flix).

1. PORTAL DISCOVERY - no list of entrances is given. Each Flix sweeps half of the hillside
   face at 3 m from the rock, its side ToF ranger looking at the face. A stretch of "no
   return" at least 1.5 m wide is an opening; Flix flies into it with its light (camera + front
   ToF), reads the portal sign and classifies it (clear / flooded / collapsed), comes back out
   and continues the sweep. Each Flix reports its openings to its Outside Network Area.
2. REGIONS GROUND ROBOTS CANNOT REACH - after the sweep, Flix B flies into the flooded portal
   C, low over the water of the sump, to the air pocket beyond. It looks for people with its
   camera and, where a Wi-Fi source is in reach, with Wi-Fi CSI; it carries what it found until
   it hears a beacon (ONA-C's fibered first beacon at the portal) and writes it there, flagged
   "not reachable by ground robots".
3. FAST WRITER - each Flix is paired with a ground Writer. It flies ahead of its Writer along
   the gallery the Writer is exploring (the Writer shares its DFS heading on the radio), but
   never beyond radio reach of a routed beacon. At every junction it spins once: its camera
   looks down every branch and its ESP32-S3 works as a FLYING CSI NODE (Wi-Fi CSI of its links
   to the nearby beacons and its Writer -> AI presence classifier for that region). When the
   camera sees a person it flies to them for a closer look (+ a CSI window there) and writes the
   VICTIM record (detection confidence) into the nearest beacon: the miner is on the living map
   before the slow Writer arrives. Battery: it returns to its pad (charged by its ONA station)
   before its endurance runs out.

Motion: scripted body-frame velocity commands (Gazebo VelocityControl), closed-loop on its
own EKF. Sensors: gyro (IMU), optical flow (model), downward ToF laser + 3 side/front ToF.
"""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

from . import config as C
from .common import LMNode, OdomTracker, spin_node, wrap
from .estimation import Body3DEKF, NoisySensor
from .geometry import csi_classify, rssi
from .packet import world_to_gps

LINK_OK = rssi(C.RF_RANGE - C.FLIX_RF_MARGIN)
ENDURANCE = 540.0            # s of flight (250 mAh 1S) - returns with a reserve


class Flix(LMNode):
    def __init__(self):
        super().__init__('flix_b', publishes=('/lm/wireless/to_ona', '/lm/flix/uplink'))
        self.rname = str(self.get_parameter('robot').value or 'flix_b')
        self.writer = C.PAIR[self.rname]
        sp = C.SPAWN[self.rname]
        self.sp = sp
        n = self.rname
        self.cmd = self.create_publisher(Twist, f'/model/{n}/cmd_vel', 10)
        self.ekf = Body3DEKF(sp[0], sp[1], sp[2], sp[3])
        self.truth = OdomTracker(sp)
        self.flow_s = NoisySensor(31 + C.FLIXES.index(n), 0.02)
        self._gt = None
        self.tof_down = None
        self.tof = {'r': 99.0, 'f': 99.0, 'l': 99.0}
        self.create_subscription(Imu, f'/{n}/imu', self.on_imu, 50)
        self.create_subscription(LaserScan, f'/{n}/tof_down', self.on_tof_down, 10)
        self.create_subscription(LaserScan, f'/{n}/tof', self.on_tof, 10)
        self.create_subscription(Odometry, f'/model/{n}/ground_truth', self.on_truth, 30)
        self.sub(f'/lm/sensors/{n}', self.on_sensors)
        self.sub(f'/lm/rf/{n}', self.on_rf)
        self.sub('/lm/wireless/from_ona', self.on_wireless)
        self.sub(f'/lm/pose/{self.writer}', self.on_writer)
        self.state = 'wait_start'
        self.t_state = 0.0
        self.t_takeoff = None
        self.sweep_y0, self.sweep_y1 = C.FLIX_SWEEP[n]
        self.sweep_dir = 1.0 if self.sweep_y1 > self.sweep_y0 else -1.0
        self.sweep_y = self.sweep_y0
        self.gap = None               # current stretch of "no return" along the face
        self.openings = []            # discovered openings (y centre, width)
        self.inspections = []
        self.cur = None               # opening being inspected
        self.entrance = None
        self.portal = None
        self.links = []
        self.sensors = {}
        self.wpose = {}
        self.last_good = None
        self.crumbs = []              # own path inside (for the way home)
        self.src = C.SRC_ID[n]
        self.eid = 0
        self.marked = []              # miners already reported by this Flix
        self.marks = []
        self.csi = {}                 # last CSI window of its links (flying CSI node)
        self.csi_windows = []         # windows collected during the current scan
        self.inspect = None           # flooded portal to inspect before the escort task
        self.in_c = False
        self.target_victim = None
        self.scanned = set()          # positions of the junctions already scanned (thermal)
        self.spin = None
        self.track_est, self.track_true, self._track_t = [], [], 0.0
        self.note = ''
        self.create_timer(0.1, self.step)
        self.create_timer(0.2, self.publish_pose)

    # ------------------------------------------------------------ sensors
    def on_imu(self, msg):
        self.ekf.gyro(self.now(), msg.angular_velocity.z, C.SENS['gyro_sigma'] + 0.002)

    def on_tof_down(self, msg):
        if msg.ranges and msg.range_min < msg.ranges[0] < msg.range_max:
            self.tof_down = msg.ranges[0] + C.FLIX_TOF_DOWN_Z     # sensor sits under the frame
            self.ekf.height(self.now(), self.tof_down, C.SENS['tof_sigma'] + 0.01)

    def on_tof(self, msg):
        vals = list(msg.ranges)
        if len(vals) >= 3:
            fix = [v if msg.range_min < v < msg.range_max else 99.0 for v in vals]
            self.tof = {'r': fix[0], 'f': fix[len(fix) // 2], 'l': fix[-1]}

    def on_truth(self, msg):
        """Truth only drives the optical-flow sensor MODEL (and the 2D window)."""
        self.truth.update(msg)
        t = self.now()
        p = (self.truth.x, self.truth.y, self.truth.z, self.truth.yaw)
        if self._gt is None:
            self._gt = (t, p)
            return
        if t - self._gt[0] >= 0.05:
            t0, p0 = self._gt
            dt = t - t0
            dx, dy = p[0] - p0[0], p[1] - p0[1]
            c, s = math.cos(p0[3]), math.sin(p0[3])
            bx, by = dx * c + dy * s, -dx * s + dy * c
            h = max(0.3, self.tof_down or 0.3)
            sig = C.SENS['flow_sigma'] * h * (1.5 if self.in_c else 1.0)    # water: poor texture
            if (self.tof_down or 0) > 0.08:
                self.ekf.body_vel(t, self.flow_s.v(bx / dt, sig), self.flow_s.v(by / dt, sig),
                                  sig + 0.02)
            self._gt = (t, p)

    def on_sensors(self, m):
        self.sensors = m
        fix = m.get('gps')
        if fix and self.state not in ('wait_start', 'landed'):
            from .packet import gps_to_local
            x, y = gps_to_local(fix['lat'], fix['lon'], C.GEO_LAT0, C.GEO_LON0, C.GEO_AZ_DEG)
            self.ekf.gps(self.now(), x, y, fix['sigma'])

    def on_rf(self, m):
        self.links = m.get('links', [])
        if m.get('marks'):
            self.marks = m['marks']
        cs = m.get('csi') or {}
        if cs.get('t') is not None and cs.get('t') != self.csi.get('t'):
            self.csi = cs
            if self.scanning():
                self.csi_windows.append(cs)
        kb = getattr(self, 'known_b', {})
        for l in self.links:                   # trail memory: every beacon heard so far
            kb[l['id']] = (l['mx'], l['my'], l.get('next_out', 0))
        self.known_b = kb
        # beacon fix: passing a beacon re-anchors the estimate to the beacon's mapped position
        # (bounds the optical-flow / gyro drift inside the mine). The strongest signal marks the
        # closest approach: once the signal falls again, the estimate is shifted by the offset
        # between the beacon and where the Flix believed it was at that moment (no early bias).
        now = self.now()
        x, y, _, _ = self.ekf.pose
        fix_t = self._fix_t = getattr(self, '_fix_t', {})
        pk = self._pk = getattr(self, '_pk', {})
        for l in self.links:
            if l.get('moved'):
                pk.pop(l['id'], None)
                continue
            r, p = l['rssi'], pk.get(l['id'])
            if r < rssi(3.0):
                pk.pop(l['id'], None)
                continue
            if p is None or r > p[0]:
                pk[l['id']] = (r, x, y)
            elif r < p[0] - 1.5 and p[0] >= rssi(1.5) and now - fix_t.get(l['id'], -1e9) > 8.0:
                fix_t[l['id']] = now
                pk.pop(l['id'], None)
                self.ekf.gps(now, x + l['mx'] - p[1], y + l['my'] - p[2], 0.5, keep_yaw=True)

    def on_writer(self, m):
        self.wpose = m

    def on_wireless(self, m):
        if m.get('kind') == 'flix_task' and m.get('to') == self.rname and self.state == 'await_task':
            self.entrance = m['entrance']
            self.portal = tuple(m['portal'])
            self.inspect = m.get('inspect')
            if self.inspect:
                self.set_state('c_go', f'flooded portal {self.inspect["entrance"]}: flying in over the water')
                self.get_logger().info(f'task: inspect flooded portal {self.inspect["entrance"]}, then '
                                       f'fast writer with {self.writer}')
            else:
                self.set_state('to_portal')
            self.get_logger().info(f'task: fast writer with {self.writer} through portal {self.entrance}')

    # ------------------------------------------------------------ helpers
    def set_state(self, s, note=''):
        if s in ('landed', 'emergency') and self.state not in ('landed', 'emergency'):
            self._bat_land = (self.now(), max(0.0, self.endurance_left()))   # motors off
        self.state = s
        self.t_state = self.now()
        if note:
            self.note = note

    def battery(self):
        """Battery shown in the 2D window: drains in flight, frozen after an emergency landing,
        recharges on the pad's contacts (full in 10 min)."""
        bl = getattr(self, '_bat_land', None)
        if bl is None or self.state not in ('landed', 'emergency'):
            return max(0.0, self.endurance_left()) / ENDURANCE
        if self.state == 'emergency':
            return bl[1] / ENDURANCE
        return min(1.0, bl[1] / ENDURANCE + (self.now() - bl[0]) / 600.0)

    def next_eid(self):
        self.eid = self.eid % 127 + 1
        return self.eid

    def scanning(self):
        return self.spin is not None or \
            (self.state == 'to_victim' and getattr(self, '_look_t', None) is not None)

    def alt(self):
        """Flight height above the surface under it (ToF): low over the flooded sump (roof)."""
        return C.FLIX_ALT_WATER if self.in_c else C.FLIX_ALT

    def send(self, obj):
        obj['from'] = self.rname
        obj['robot'] = self.rname
        obj['t_ms'] = self.now_ms()
        x, y, _, _ = self.ekf.pose
        obj.setdefault('lat', world_to_gps(x, y)[0])
        obj.setdefault('lon', world_to_gps(x, y)[1])
        self.pub('/lm/wireless/to_ona', obj)

    def fly(self, tx, ty, tz, vmax, face=True, centring=False, yaw=None):
        x, y, z, cyaw = self.ekf.pose
        ex, ey, ez = tx - x, ty - y, tz - z
        vx, vy = 0.9 * ex, 0.9 * ey
        n = math.hypot(vx, vy)
        if n > vmax:
            vx, vy = vx * vmax / n, vy * vmax / n
        vz = max(-0.8, min(0.8, 1.2 * ez))
        c, s = math.cos(cyaw), math.sin(cyaw)
        bx, by = c * vx + s * vy, -s * vx + c * vy
        if centring and self.tof['l'] < 3.9 and self.tof['r'] < 3.9 and \
                self.tof['l'] + self.tof['r'] < 5.5:            # both walls seen: one gallery
            by += max(-0.5, min(0.5, 0.5 * (self.tof['l'] - self.tof['r']) / 2.0))
        elif centring and min(self.tof['l'], self.tof['r']) < 0.8:
            by += 0.3 if self.tof['r'] < self.tof['l'] else -0.3     # too close to one wall
        if self.tof['f'] < 1.0 and bx > 0:
            # rock ahead (cutting a corner): stop forward, slide toward the more open side
            bx = 0.0
            if centring:
                by = 0.5 if self.tof['l'] > self.tof['r'] else -0.5
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = bx, by, vz
        if yaw is not None:
            tw.angular.z = max(-1.5, min(1.5, 1.8 * wrap(yaw - cyaw)))
        elif face and n > 0.4:
            tw.angular.z = max(-1.5, min(1.5, 1.8 * wrap(math.atan2(ey, ex) - cyaw)))
        self.cmd.publish(tw)
        return math.sqrt(ex * ex + ey * ey + ez * ez)

    def hover(self, yaw_rate=0.0):
        x, y, z, _ = self.ekf.pose
        self.fly(x, y, self.alt() if self.state != 'landed' else z, 0.0, face=False)
        if yaw_rate:
            tw = Twist()
            tw.angular.z = yaw_rate
            tw.linear.z = max(-0.8, min(0.8, 1.2 * (self.alt() - z)))
            self.cmd.publish(tw)

    def best_link(self):
        best = None
        for l in self.links:
            if l.get('routed') and (best is None or l['rssi'] > best['rssi']):
                best = l
        return best

    def reserve(self):
        """Battery needed to fly home along the breadcrumbs (+ outside leg + margin)."""
        L = sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(self.crumbs[:-1], self.crumbs[1:]))
        if self.crumbs:
            x, y, _, _ = self.ekf.pose
            L += math.hypot(x - self.crumbs[-1][0], y - self.crumbs[-1][1])
        return 60.0 + (L + 18.0) / 1.0

    def endurance_left(self):
        if self.t_takeoff is None:
            return ENDURANCE
        return ENDURANCE - (self.now() - self.t_takeoff)

    # ------------------------------------------------------------ behaviour
    def step(self):
        if not self.truth.ok:
            return
        t = self.now()
        st = self.state
        sp = self.sp
        if st not in ('wait_start', 'landed', 'landing', 'returning', 'home', 'emergency', 'c_out') \
                and self.endurance_left() < self.reserve():
            v = self.target_victim if st == 'to_victim' else None
            ex, ey, _, _ = self.ekf.pose
            if v and math.hypot(v['x'] - ex, v['y'] - ey) < 8.0 and \
                    self.endurance_left() > self.reserve() - 25.0:
                pass            # seconds from the miner: report him first (inside the 60 s margin)
            elif self.in_c:
                self.get_logger().warn('battery reserve reached in the flooded line - flying out')
                self.c_home = True
                self.set_state('c_out', 'battery reserve: flying out over the water')
            else:
                self.get_logger().warn('battery reserve reached - returning to the pad')
                self.set_state('returning', 'battery reserve: flying home')
        if st not in ('wait_start', 'landed', 'emergency', 'landing') and self.endurance_left() <= 0.0:
            self.get_logger().error('battery empty - emergency landing where it is')
            self.set_state('emergency', 'battery empty: landed in place (beacon trail keeps its finds)')
            self.send({'kind': 'drone_status', 'status': 'emergency'})
        st = self.state
        if st == 'wait_start':
            self.ekf.still(t)
            self.cmd.publish(Twist())
            if self.started():
                self.send({'kind': 'drone_status', 'status': 'scouting'})
                self.t_takeoff = t
                self.set_state('takeoff', 'take-off')
        elif st == 'takeoff':
            if self.fly(sp[0], sp[1], C.FLIX_ALT, 1.0, face=False) < 0.15:
                self.set_state('to_sweep', 'to the hillside face')
        elif st == 'to_sweep':
            yaw = math.pi / 2 if self.sweep_dir > 0 else -math.pi / 2
            d = self.fly(C.FLIX_SWEEP_X, self.sweep_y0, C.FLIX_ALT, C.FLIX_SPEED_OUT,
                         yaw=yaw if math.hypot(C.FLIX_SWEEP_X - self.ekf.pose[0],
                                               self.sweep_y0 - self.ekf.pose[1]) < 4.0 else None)
            if d < 0.3 and abs(wrap(self.ekf.pose[3] - yaw)) < 0.1:
                self.set_state('sweep', 'sweeping the hillside face (side ToF on the rock)')
        elif st == 'sweep':
            self.sweep_step()
        elif st == 'inspect_go':
            if self.fly(-4.0, self.cur['y'], C.FLIX_ALT, C.FLIX_SPEED_OUT, yaw=0.0) < 0.3:
                self.set_state('inspect_in', f'opening at y={self.cur["y"]:.1f}: flying in')
                self._prog = (t, 99.0)
        elif st == 'inspect_in':
            d = self.fly(C.FLIX_INSPECT_DEPTH, self.cur['y'], C.FLIX_ALT, C.FLIX_SPEED_IN,
                         centring=True, yaw=0.0)
            if d < self._prog[1] - 0.2:
                self._prog = (t, d)
            if d < 0.3 or t - self._prog[0] > 3.0:          # front ToF stops it (rubble)
                self.set_state('inspect', 'looking with the camera + ToF')
        elif st == 'inspect':
            self.hover()
            if t - self.t_state >= C.FLIX_INSPECT_S:
                r = self.sensors.get('portal') or {}
                x, y, _, _ = self.ekf.pose
                rec = {'id': r.get('id', '?'), 'x': 0.0, 'y': round(self.cur['y'], 2),
                       'state': r.get('state', 'unknown'), 'detail': r.get('detail', ''),
                       'width': round(self.cur['w'], 2), 'depth_seen': round(max(0.0, x), 2),
                       'tof_front': round(min(self.tof['f'], 99.0), 2), 'by': self.rname}
                self.inspections.append(rec)
                self.send({'kind': 'portal_found', 'portal': rec})
                self.get_logger().info(f'opening {rec["id"]} at y={rec["y"]}: {rec["state"]} - '
                                       f'{rec["detail"]}')
                self.set_state('inspect_out', f'portal {rec["id"]}: {rec["state"]}')
        elif st == 'inspect_out':
            if self.fly(C.FLIX_SWEEP_X, self.cur['y'], C.FLIX_ALT, C.FLIX_SPEED_IN, yaw=0.0) < 0.4:
                self.sweep_y = self.cur['y'] + self.sweep_dir * (self.cur['w'] / 2 + 0.8)
                self.cur = None
                self.set_state('sweep', 'sweeping the hillside face')
        elif st == 'await_task':
            self.hover()
        elif st == 'to_portal':
            px, py = self.portal
            if self.fly(-3.0, py, C.FLIX_ALT, C.FLIX_SPEED_OUT) < 0.4:
                self.set_state('wait_chain', 'waiting for the first beacon at the portal')
        elif st == 'wait_chain':
            self.hover()
            b = self.best_link()
            if b is not None and b['rssi'] >= LINK_OK and self.wpose.get('state') == 'exploring':
                self.set_state('escort', f'flying ahead of {C.LABEL[self.writer]}')
        elif st == 'c_go':
            px, py = self.inspect['portal']
            yaw = 0.0 if math.hypot(-3.0 - self.ekf.pose[0], py - self.ekf.pose[1]) < 4.0 else None
            if self.fly(-3.0, py, C.FLIX_ALT, C.FLIX_SPEED_OUT, yaw=yaw) < 0.4:
                self.in_c = True
                self.crumbs = [(-1.0, py)]
                self.set_state('inspect_c', 'flooded line C: low over the water (ToF on the surface)')
        elif st == 'inspect_c':
            self.inspect_c()
        elif st == 'c_out':
            self.c_out()
        elif st == 'escort':
            self.escort()
        elif st == 'reanchor':
            self.reanchor()
        elif st == 'to_victim':
            self.to_victim()
        elif st == 'returning':
            self.return_home()
        elif st == 'landing':
            if self.fly(sp[0], sp[1], 0.0, 0.3, face=False) < 0.08 or t - self.t_state > 15:
                self.cmd.publish(Twist())
                self.set_state('landed', 'landed on its pad')
                self.send({'kind': 'drone_status', 'status': 'landed'})
        elif st == 'landed':
            self.ekf.still(t)
            self.cmd.publish(Twist())
        elif st == 'emergency':
            x, y, z, _ = self.ekf.pose
            if z > 0.05:
                self.fly(x, y, 0.0, 0.0, face=False)
            else:
                self.cmd.publish(Twist())

    # ------------------------------------------------------------ portal discovery
    def face_tof(self):
        """Side ToF looking at the hillside (east): right beam when flying north."""
        return self.tof['r'] if self.sweep_dir > 0 else self.tof['l']

    def sweep_step(self):
        x, y, _, _ = self.ekf.pose
        yaw = math.pi / 2 if self.sweep_dir > 0 else -math.pi / 2
        target = self.sweep_y1
        d = self.fly(C.FLIX_SWEEP_X, target, C.FLIX_ALT, C.FLIX_SWEEP_SPEED, yaw=yaw)
        r = self.face_tof()
        _, _, _, cyaw = self.ekf.pose
        if self.now() - self.t_state < 0.6 or abs(wrap(cyaw - yaw)) > 0.12:
            self.gap = None                            # side ToF not yet facing the rock
            return
        if r > 3.8:                                    # no return: the face is open here
            if self.gap is None:
                self.gap = {'y0': y, 'n': 0}
            self.gap['n'] += 1
            self.gap['y1'] = y
        elif self.gap is not None:
            w = abs(self.gap['y1'] - self.gap['y0'])
            yc = (self.gap['y0'] + self.gap['y1']) / 2.0
            self.gap = None
            if w >= C.FLIX_OPENING_MIN_W and all(abs(o['y'] - yc) > 3.0 for o in self.openings):
                self.cur = {'y': yc, 'w': w}
                self.openings.append(self.cur)
                self.get_logger().info(f'opening detected in the face at y={yc:.1f} ({w:.1f} m wide)')
                self.set_state('inspect_go', f'opening at y={yc:.1f} ({w:.1f} m): inspecting')
                return
        if d < 0.5:
            self.send({'kind': 'drone_report', 'portals': self.inspections,
                       'swept': [self.sweep_y0, self.sweep_y1]})
            self.get_logger().info(f'sweep done: {len(self.inspections)} openings')
            self.set_state('await_task', 'sweep reported - waiting for a task')

    # ------------------------------------------------------------ fast writer
    def marked_near(self, vx, vy):
        """Person already seen by a camera (this Flix, or any record in the beacons)? A CSI
        presence region alone does not count: the Flix goes to look."""
        if any(math.hypot(vx - a, vy - b) < 6.0 for a, b in self.marked):
            return True
        for mk in self.marks:
            if mk.get('kind') in ('victim', 'confirmed') and math.hypot(mk['x'] - vx, mk['y'] - vy) < 6.0:
                return True
        return False

    # ------------------------------------------------------------ flooded portal (line C)
    def inspect_c(self):
        """Fly east along the flooded gallery, low over the water (height from the downward ToF
        on the surface, roof above), side ToF centring, camera on."""
        x, y, z, yaw = self.ekf.pose
        py = self.inspect['portal'][1]
        if not self.crumbs or math.hypot(self.crumbs[-1][0] - x, self.crumbs[-1][1] - y) > 2.0:
            self.crumbs.append((x, y))
        for v in self.sensors.get('victims') or []:
            a = yaw + v['bearing']
            vx, vy = x + v['range'] * math.cos(a), y + v['range'] * math.sin(a)
            if not self.marked_near(vx, vy):
                self.target_victim = {'x': vx, 'y': vy, 'range': v['range'], 'conf': v['conf']}
                self.set_state('to_victim', f'camera: person at {v["range"]:.0f} m beyond the sump')
                return
        if x >= C.FLIX_INSPECT_END or self.tof['f'] < 1.2:
            self.set_state('c_out', 'end of the flooded line: flying back out')
            return
        self.note = 'over the flooded sump: camera + CSI (no Wi-Fi source in reach)' \
            if not (self.csi.get('links')) else 'over the flooded sump'
        self.fly(x + 4.0, py, self.alt(), C.FLIX_SPEED_IN, centring=True, yaw=0.0)

    def c_out(self):
        """Back along the flooded gallery and out of the portal: done once outside under open
        sky (RTK-GPS fix again), so the estimate drift over the water does not matter."""
        x, y, z, yaw = self.ekf.pose
        py = self.inspect['portal'][1]
        if x > -2.5 or not self.sensors.get('gps'):
            self.in_c = x > -0.5
            self.fly(min(x - 4.0, -4.0), py, self.alt(), C.FLIX_SPEED_IN, centring=x > 1.0, yaw=math.pi)
            return
        self.in_c = False
        self.inspect = None
        self.crumbs = []
        if getattr(self, 'c_home', False) or self.endurance_left() < 240.0:
            self.set_state('returning', 'line C inspected - battery low: back to the pad')
            return
        self.set_state('to_portal', f'line C inspected - joining {C.LABEL[self.writer]}')

    def gallery_lock(self):
        """Side ToF rangers against the gallery its Writer mapped: the gallery centre line goes
        through the Writer's junction along its exploration axis; the two wall distances give
        Flix's position across the gallery (removes the optical-flow / heading drift sideways)."""
        edge = self.wpose.get('edge') or {}
        ax = self.wpose.get('axis')
        if ax is None or 'sx' not in edge:
            return
        x, y, z, yaw = self.ekf.pose
        l, r = self.tof['l'], self.tof['r']
        if l > 3.9 or r > 3.9 or l + r > 5.0:      # both walls seen: one gallery
            return
        d = wrap(yaw - ax)
        if min(abs(d), abs(abs(d) - math.pi)) > 0.45:      # beams still on the side walls
            return
        g = ax + math.pi / 2
        along = (x - edge['sx']) * math.cos(ax) + (y - edge['sy']) * math.sin(ax)
        lat = -(x - edge['sx']) * math.sin(ax) + (y - edge['sy']) * math.cos(ax)
        if along < 4.0 or along > 40.0 or abs(lat) > 3.0:
            return
        c = edge['sx'] * math.cos(g) + edge['sy'] * math.sin(g)
        off = (r - l) / 2.0 * math.cos(wrap(yaw + math.pi / 2 - g))
        self.ekf.line(self.now(), g, c + off, 0.25)
        return True

    def segment_lock(self, tx, ty):
        """Between two linked junctions of the Writer's map (one straight gallery), e.g. while the
        Writer backtracks: side-ToF lock across that gallery. Returns the gallery heading toward
        (tx, ty) to hold, or None when not on such a segment."""
        x, y, _, _ = self.ekf.pose
        nodes = self.wpose.get('nodes') or {}
        best = None
        for k, n in nodes.items():
            for o in (n.get('links') or {}).values():
                m = nodes.get(str(o))
                if m is None or int(o) <= int(k):
                    continue
                ax = math.atan2(m['y'] - n['y'], m['x'] - n['x'])
                L = math.hypot(m['x'] - n['x'], m['y'] - n['y'])
                along = (x - n['x']) * math.cos(ax) + (y - n['y']) * math.sin(ax)
                lat = -(x - n['x']) * math.sin(ax) + (y - n['y']) * math.cos(ax)
                if L >= 6.0 and 2.0 <= along <= L - 2.0 and abs(lat) < 3.0 and \
                        (best is None or abs(lat) < best[0]):
                    best = (abs(lat), (n['x'], n['y']), (m['x'], m['y']))
        if best is None:
            return None
        ax = self.trail_lock(best[1], best[2])
        if ax is None:
            return None
        return ax if math.cos(math.atan2(ty - y, tx - x) - ax) >= 0.0 else wrap(ax + math.pi)

    def escort(self):
        locked = self.gallery_lock()
        x, y, z, yaw = self.ekf.pose
        if not self.crumbs or math.hypot(self.crumbs[-1][0] - x, self.crumbs[-1][1] - y) > 2.0:
            # back where it already flew: cut the loop, the way home stays the shortest
            for i, (cx, cy) in enumerate(self.crumbs[:-3]):
                if math.hypot(cx - x, cy - y) < 2.5:
                    self.crumbs = self.crumbs[:i]
                    break
            self.crumbs.append((x, y))
        ws = self.wpose.get('state', '')
        if ws in ('leaving', 'parked', 'destroyed'):
            self.set_state('returning', f'{C.LABEL[self.writer]} {ws}: flying home')
            return
        if x < 1.5 and self.portal is not None:
            # still outside (it joins its Writer late, e.g. after the flooded portal): in through
            # its portal first, never straight at the hillside
            py = self.portal[1]
            tx = 3.0 if abs(y - py) < 1.0 else -3.0
            self.note = f'flying in through portal {self.entrance}'
            self.fly(tx, py, C.FLIX_ALT, C.FLIX_SPEED_IN if abs(y - py) < 1.0 else C.FLIX_SPEED_OUT,
                     centring=abs(y - py) < 1.0)
            return
        # camera: a person not yet on the map?
        for v in self.sensors.get('victims') or []:
            a = yaw + v['bearing']
            vx, vy = x + v['range'] * math.cos(a), y + v['range'] * math.sin(a)
            if not self.marked_near(vx, vy):
                self.target_victim = {'x': vx, 'y': vy, 'range': v['range'], 'conf': v['conf']}
                self.set_state('to_victim', f'camera: person at {v["range"]:.0f} m - flying there')
                self.get_logger().info(f'camera: person at ({vx:.1f},{vy:.1f}), '
                                       f'{v["range"]:.1f} m ({v["conf"]:.0%})')
                return
        b = self.best_link()
        leash_ok = b is not None and b['rssi'] >= LINK_OK
        if leash_ok:
            self.last_good = (x, y)
        # junction scan: when both side rangers see no wall (a junction), spin once so the
        # thermal camera looks down every branch
        if self.spin is not None:
            if self.now() - self.spin < 5.0:
                self.hover(yaw_rate=1.3)
                return
            self.spin = None
            self.csi_report()
        edge = self.wpose.get('edge') or {}
        jid = edge.get('from')
        nodes = self.wpose.get('nodes') or {}
        if self.tof['l'] > 3.8 and self.tof['r'] > 3.8 and x > 2.0 and \
                all(math.hypot(x - a, y - b) > 8.0 for a, b in self.scanned):
            self.scanned.add((x, y))
            self.spin = self.now()
            self.csi_windows = []
            self.note = 'junction: 360 deg camera scan + Wi-Fi CSI (flying CSI node)'
            return
        lead = self.wpose.get('lead')
        wx, wy = self.wpose.get('ex', x), self.wpose.get('ey', y)
        hold = None
        if lead is None:
            tx, ty = wx, wy
        else:
            tx, ty = lead
            ax = self.wpose.get('axis', 0.0)
            hold = ax          # in the Writer's gallery: nose along it, side ToF square to walls
            # off the Writer's gallery line (it just turned at a junction): go via the junction
            if jid is not None and str(jid) in nodes:
                n = nodes[str(jid)]
                off = abs(-(x - n['x']) * math.sin(ax) + (y - n['y']) * math.cos(ax))
                if off > 1.0:
                    tx, ty = n['x'], n['y']           # not yet in the Writer's new gallery
                    hold = None
            if math.cos(math.atan2(ty - y, tx - x) - ax) < 0.5:
                hold = None                           # lead behind / to the side: face it
        if not leash_ok and self.last_good is not None:
            # radio leash: never beyond reach of a routed beacon
            tx, ty = self.last_good
            self.note = 'radio leash: holding at the last point with a good link'
        else:
            self.note = f'ahead of {C.LABEL[self.writer]}'
            hold = hold if math.hypot(tx - x, ty - y) > 1.0 else None
        if not locked:
            seg = self.segment_lock(tx, ty)           # Writer backtracking / in transit
            if hold is None and seg is not None and math.hypot(tx - x, ty - y) > 1.0:
                hold = seg
        if not leash_ok and self.last_good is not None:
            hold = None
        d = self.fly(tx, ty, C.FLIX_ALT, C.FLIX_SPEED_IN, centring=True, yaw=hold)
        self.check_stall(d)

    # ------------------------------------------------------------ re-anchoring on a beacon
    def check_stall(self, d):
        """No progress toward where it wants to go for 6 s (typically: blocked by a corner it
        does not expect because its optical-flow estimate lags along the gallery): find the
        signal peak of a nearby beacon of the map and re-anchor on it."""
        now = self.now()
        pr = getattr(self, '_esc_prog', None)
        if pr is None or d < pr[1] - 0.5 or d < 2.0:
            self._esc_prog = (now, d)
            return
        if now - pr[0] < 6.0:
            return
        self._esc_prog = None
        tried = getattr(self, '_reanchored', {})
        cands = [l for l in self.links if not l.get('moved') and l['rssi'] >= rssi(5.0)
                 and now - tried.get(l['id'], -1e9) > 40.0]
        if not cands:
            return
        b = max(cands, key=lambda l: l['rssi'])
        tried[b['id']] = now
        self._reanchored = tried
        self._ra = {'id': b['id'], 'mx': b['mx'], 'my': b['my'], 'k': 0, 't': now,
                    'best': (b['rssi'],) + tuple(self.ekf.pose[:2])}
        self.set_state('reanchor', f'stalled: searching the signal peak of B{b["id"]} to re-anchor')
        self.get_logger().info(f'stalled - re-anchoring on B{b["id"]}')

    def reanchor(self):
        """Square spiral around the beacon's map position until its signal says the Flix is over
        it (then the beacon's coordinates are its position), or until the spiral is done (then
        the strongest point seen is taken as the closest approach)."""
        ra = self._ra
        x, y, _, _ = self.ekf.pose
        now = self.now()
        lk = next((l for l in self.links if l['id'] == ra['id']), None)
        if lk is not None and lk['rssi'] > ra['best'][0]:
            ra['best'] = (lk['rssi'], x, y)
        if lk is not None and lk['rssi'] >= rssi(1.2):
            self.ekf.gps(now, ra['mx'], ra['my'], 0.5, keep_yaw=True)
            self.get_logger().info(f're-anchored on B{ra["id"]}')
            self.set_state('escort', f're-anchored on B{ra["id"]}')
            return
        pts = [(0.0, 0.0)] + [(r * math.cos(a * math.pi / 4), r * math.sin(a * math.pi / 4))
                              for r in (1.2, 2.4, 3.4) for a in range(8)]
        if ra['k'] >= len(pts) or now - self.t_state > 60.0:
            if ra['best'][0] >= rssi(2.0):     # closest approach seen: shift by that offset
                self.ekf.gps(now, x + ra['mx'] - ra['best'][1], y + ra['my'] - ra['best'][2], 1.0,
                             keep_yaw=True)
            self.set_state('escort', f'B{ra["id"]} search done')
            return
        tx, ty = ra['mx'] + pts[ra['k']][0], ra['my'] + pts[ra['k']][1]
        if self.fly(tx, ty, C.FLIX_ALT, 0.8, face=False) < 0.3 or now - ra['t'] > 3.0:
            ra['k'] += 1
            ra['t'] = now

    def to_victim(self):
        """Close in for a better camera view (confidence grows), hover for a CSI window, then
        write the VICTIM record (and PRESENCE if the CSI classifier agrees)."""
        x, y, z, yaw = self.ekf.pose
        v = self.target_victim
        b = self.best_link()
        if not self.in_c and (b is None or b['rssi'] < rssi(C.RF_RANGE - 1.0)):
            # out of reach of the chain: report from where we are (held until a beacon hears it)
            self.report_victim(v)
            self.set_state('escort')
            return
        # refine the position / confidence with every new detection of the same person
        for det in self.sensors.get('victims') or []:
            a = yaw + det['bearing']
            vx, vy = x + det['range'] * math.cos(a), y + det['range'] * math.sin(a)
            if math.hypot(vx - v['x'], vy - v['y']) < 7.0 and det['range'] < v.get('range', 99) + 0.5:
                v.update({'x': vx, 'y': vy, 'range': det['range'], 'conf': max(v['conf'], det['conf'])})
        d = math.hypot(v['x'] - x, v['y'] - y)
        look = getattr(self, '_look_t', None)
        if look is None and d > 1.3 and self.now() - self.t_state < 25.0:
            self.fly(v['x'], v['y'], self.alt(), C.FLIX_SPEED_IN, centring=d > 4.0)
            return
        if look is None:
            self._look_t = self.now()
            self.csi_windows = []
            self.note = 'hovering by the person: camera + Wi-Fi CSI window'
        if self.now() - self._look_t < 2.2 * C.CSI_WINDOW:
            self.hover()
            return
        self._look_t = None
        self.report_victim(v)
        self.csi_report(force_region=(v['x'], v['y']))
        if self.in_c:
            self.set_state('c_out', 'miner found beyond the sump - flying out to report him')
        else:
            self.set_state('escort', 'person reported in the nearest beacon')

    def report_victim(self, v):
        """VICTIM record: position, camera confidence, source = this Flix. Beyond the flooded
        sump it is flagged: not reachable by ground robots."""
        self.marked.append((v['x'], v['y']))
        x, y, z, yaw = self.ekf.pose
        pkt = {'id': C.ID_ROBOT[self.rname], 'type': C.EV_VICTIM, 'parent': 0,
               'flags': C.FL_NO_GROUND if self.in_c else 0, 'src': self.src,
               'eid': self.next_eid(), 'x': v['x'], 'y': v['y'], 'heading': math.degrees(yaw),
               'a': 10.0 * v.get('range', 0.0), 'b': C.CAM_THERMAL, 'conf': v.get('conf', 0.7),
               't_ms': self.now_ms()}
        self.get_logger().info(f'VICTIM ({v["x"]:.1f},{v["y"]:.1f}) camera {v.get("conf", 0):.0%}'
                               + (' - beyond the flooded sump' if self.in_c else ''))
        self.pub('/lm/flix/uplink', {'from': self.rname, 'wx': self.truth.x, 'wy': self.truth.y,
                                     'pkt': pkt})

    def csi_report(self, force_region=None):
        """Flying CSI node: classify the windows collected while hovering / spinning; presence in
        two windows -> PRESENCE record for this region."""
        wins = [w for w in self.csi_windows if w.get('links')]
        self.csi_windows = []
        if len(wins) < C.CSI_CONFIRM:
            return
        ps = [csi_classify([l['score'] for l in w['links']]) for w in wins]
        if min(ps[-C.CSI_CONFIRM:]) < C.CSI_THRESHOLD:
            return
        p = sum(ps[-C.CSI_CONFIRM:]) / C.CSI_CONFIRM
        x, y, _, yaw = self.ekf.pose
        fired = [l for l in wins[-1]['links'] if l['score'] > 0.25]
        if not fired:
            return
        wsum = sum(l['score'] for l in fired)
        cx = sum(l['score'] * (l['mx'] + x) / 2 for l in fired) / wsum
        cy = sum(l['score'] * (l['my'] + y) / 2 for l in fired) / wsum
        r = max([C.CSI_REGION_R] + [math.hypot(l['mx'] - x, l['my'] - y) / 2 for l in fired])
        pkt = {'id': C.ID_ROBOT[self.rname], 'type': C.EV_PRESENCE, 'parent': 0,
               'flags': C.FL_NO_GROUND if self.in_c else 0, 'src': self.src,
               'eid': self.next_eid(), 'x': cx, 'y': cy, 'heading': math.degrees(yaw),
               'a': min(255, round(r)), 'b': len(fired), 'conf': p, 't_ms': self.now_ms()}
        self.get_logger().info(f'flying CSI node: presence {p:.0%} in a {r:.0f} m region '
                               f'({len(fired)} links)')
        self.pub('/lm/flix/uplink', {'from': self.rname, 'wx': self.truth.x, 'wy': self.truth.y,
                                     'pkt': pkt})

    def trail_lock(self, a, b):
        """On the way home between two consecutive trail beacons (in line of sight, so on one
        straight gallery): Flix holds the gallery heading (side ToF square to the walls) and the
        side ToF give its position across that gallery. Returns the heading to hold, or None
        near the ends of the segment (corners, junctions)."""
        x, y, z, yaw = self.ekf.pose
        ax = math.atan2(b[1] - a[1], b[0] - a[0])
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L < 6.0:
            return None
        along = (x - a[0]) * math.cos(ax) + (y - a[1]) * math.sin(ax)
        lat = -(x - a[0]) * math.sin(ax) + (y - a[1]) * math.cos(ax)
        if along < 2.0 or along > L - 2.0 or abs(lat) > 4.0:
            return None
        l, r = self.tof['l'], self.tof['r']
        d = wrap(yaw - ax)
        if l <= 3.9 and r <= 3.9 and l + r <= 5.0 and min(abs(d), abs(abs(d) - math.pi)) <= 0.45:
            g = ax + math.pi / 2
            c = a[0] * math.cos(g) + a[1] * math.sin(g)
            off = (r - l) / 2.0 * math.cos(wrap(yaw + math.pi / 2 - g))
            self.ekf.line(self.now(), g, c + off, 0.25)
        return ax

    def return_home(self):
        """Home along the beacon trail: each beacon's "toward the exit" pointer (nextOut) gives
        the next beacon to fly to, down to the entrance beacon (nextOut = 0), then out of the
        portal to the pad. The same pointers would lead a walking survivor out."""
        x, y, _, _ = self.ekf.pose
        sp = self.sp
        if x < -1.0:
            if self.fly(sp[0], sp[1], C.FLIX_ALT, C.FLIX_SPEED_OUT) < 0.3:
                self.set_state('landing', 'landing on its pad')
            return
        kb = getattr(self, 'known_b', {})
        hb = getattr(self, '_home_b', None)
        if hb == 0:                                   # entrance reached: out through ITS portal
            py = getattr(self, '_exit_y', self.portal[1] if self.portal else 0.0)
            self.fly(-3.0, py, C.FLIX_ALT, C.FLIX_SPEED_IN, centring=True)
            return
        if hb is not None and hb not in kb:
            # next beacon never heard by this Flix (e.g. the other Writer's entrance beacon):
            # its position is in the Writer's shared junction map
            node = (self.wpose.get('nodes') or {}).get(str(hb))
            if node is not None:
                ent = ((node.get('exits') or {}).get('180') or {}).get('by') == 'outside'
                kb[hb] = (node['x'], node['y'], 0 if ent else -1)
        if hb is None or hb not in kb:
            prev = getattr(self, '_home_prev', None)
            cands = [l for l in self.links if l['id'] != prev]
            if not cands:
                u = getattr(self, '_home_u', None)
                if u is None:
                    self.hover()
                else:      # keep going the way the trail went until a beacon is heard
                    self.fly(x + 4.0 * u[0], y + 4.0 * u[1], C.FLIX_ALT, C.FLIX_SPEED_IN,
                             centring=True)
                return
            hb = max(cands, key=lambda l: l['rssi'])['id']
            self._home_b, self._home_prev = hb, None
            self._home_prog = (self.now(), 1e9)
        tx, ty, nxt = kb[hb]
        prev = getattr(self, '_home_prev', None)
        hold = None
        if prev is not None and prev in kb:
            hold = self.trail_lock(kb[prev][:2], (tx, ty))
        d0 = math.hypot(tx - x, ty - y)
        if d0 > 2.0:
            self._home_u = ((tx - x) / d0, (ty - y) / d0)     # approach direction
        lk = next((l for l in self.links if l['id'] == hb), None)
        if d0 < 1.5 and lk is not None and not lk.get('moved') and getattr(self, '_home_u', None):
            # "there" on the estimate, but the beacon's signal says it is still metres ahead
            # (optical-flow scale drift along the gallery): RF range fix along the approach
            d_rf = 10 ** ((C.RF_P1M - lk['rssi']) / (10 * C.RF_N))
            if 2.5 < d_rf < 8.0 and self.now() - getattr(self, '_rf_fix_t', -1e9) > 3.0:
                ux, uy = self._home_u
                self.ekf.gps(self.now(), tx - d_rf * ux, ty - d_rf * uy, 1.0)
                self._rf_fix_t = self.now()
        d = self.fly(tx, ty, C.FLIX_ALT, C.FLIX_SPEED_IN, centring=True, yaw=hold)
        self.note = f'flying home along the trail: B{hb} -> toward the exit B{nxt}'
        if d < self._home_prog[1] - 0.3:
            self._home_prog = (self.now(), d)
        stalled = self.now() - self._home_prog[0] > 6.0
        if d < 1.0 or (stalled and nxt in kb):
            if nxt == 0:
                self._exit_y = ty
            self._home_prev, self._home_b = hb, nxt
            self._home_prog = (self.now(), 1e9)
        elif stalled and prev in kb and self.now() - self._home_prog[0] > 10.0:
            # stuck behind a corner (left the last beacon too early): back to it, then retry
            self._home_prev, self._home_b = None, prev
            self._home_prog = (self.now(), 1e9)

    def publish_pose(self):
        x, y, z, yaw = self.ekf.pose
        t = self.now()
        if t - self._track_t >= 1.0:
            self._track_t = t
            self.track_est = (self.track_est + [(round(x, 2), round(y, 2))])[-300:]
            self.track_true = (self.track_true + [(round(self.truth.x, 2), round(self.truth.y, 2))])[-300:]
        self.pub(f'/lm/pose/{self.rname}', {
            'x': self.truth.x, 'y': self.truth.y, 'z': self.truth.z, 'yaw': self.truth.yaw,
            'ex': x, 'ey': y, 'ez': z, 'eyaw': yaw, 'robot': self.rname,
            'err': round(math.hypot(x - self.truth.x, y - self.truth.y), 3),
            'sigma': round(self.ekf.f.sigma_xy(), 3), 'state': self.state, 'note': self.note,
            'entrance': self.entrance, 'inspections': self.inspections,
            'battery': round(self.battery(), 3), 'in_c': self.in_c,
            'charging': self.state == 'landed',
            'openings': self.openings, 'sweep_y': self.sweep_y,
            'track_est': self.track_est[-120:], 'track_true': self.track_true[-120:], 't': t})


def main(args=None):
    spin_node(Flix, args)


if __name__ == '__main__':
    main()

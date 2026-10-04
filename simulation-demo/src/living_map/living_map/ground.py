"""Common machinery for the ground robots (Writers A/B = Robotika X2, Executor = Explorer X1):
sensor fusion (EKF / SLAM front end), lidar processing, reactive corridor centring, keep-right
when two robots meet in a gallery, motion commands and the battery model."""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

from . import config as C
from .common import LMNode, OdomTracker, wrap, yaw_from_quat
from .estimation import GroundEKF, NoisySensor, corridor_axis, icp, scan_points
from .packet import world_to_gps


class GroundRobot(LMNode):
    def __init__(self, default_name, seed, publishes=()):
        super().__init__(default_name, publishes=tuple(publishes) + ('/lm/wireless/to_ona',))
        name = str(self.get_parameter('robot').value or default_name)
        self.rname = name
        spawn = C.SPAWN[name]
        self.spawn = spawn
        seed = seed + 7 * C.GROUND.index(name)
        self.half_width = 0.35 if name.startswith('writer') else 0.45
        self.peers = []                             # other robots heard on the radio (their estimates)
        self.cmd_pub = self.create_publisher(Twist, f'/model/{name}/cmd_vel', 10)
        # GPS + compass fix at the staging area initialises the EKF in the site frame
        self.ekf = GroundEKF(spawn[0], spawn[1], spawn[3])
        self.truth = OdomTracker(spawn)             # simulator ground truth (NOT used to navigate)
        self.vo_sensor = NoisySensor(seed, C.SENS['vo_scale'])
        self._gt_last = None
        self._vo_t = -1.0
        self._vo_v = 0.0
        self._stuck_since = None
        self._recover_until = 0.0
        self._recover_turn = 0.0
        self.front_min = 99.0
        self.left = 99.0
        self.right = 99.0
        self._prev_scan = None
        self._prev_scan_t = None
        self._scan_n = 0
        self._gal = None
        self._gal_cand = []
        self.seg_ref = None                         # mapped gallery (two junctions) to lock on
        self._front_prev = 99.0
        self.scan_xy = []
        self.blocked_since = None
        self.scan = None
        self.track_est = []
        self.track_true = []
        self._track_t = 0.0
        self.battery = 1.0                          # fraction of a full charge
        self.charging = False                       # on an ONA station charger
        self._bat_xy = None
        self.create_timer(0.5, self.battery_step)
        self.create_subscription(Odometry, f'/model/{name}/odometry', self.on_encoders, 20)
        self.create_subscription(Imu, f'/{name}/imu', self.on_imu, 50)
        self.create_subscription(LaserScan, f'/{name}/scan', self.on_scan, 5)
        self.create_subscription(Odometry, f'/model/{name}/ground_truth', self.on_truth, 20)
        self.create_timer(0.2, self.publish_pose)

    # ------------------------------------------------------------ sensors
    def on_encoders(self, msg):
        v = msg.twist.twist.linear.x
        t = self.now()
        slipping = (t - self._vo_t < 0.4) and abs(v - self._vo_v) > max(0.08, 0.3 * abs(v))
        self.ekf.encoders(t, v, msg.twist.twist.angular.z, slipping)

    def on_imu(self, msg):
        self.ekf.gyro(self.now(), msg.angular_velocity.z, C.SENS['gyro_sigma'] + 0.002)

    def on_truth(self, msg):
        """Truth feeds only (a) the visual-odometry sensor MODEL and (b) the 2D window."""
        self.truth.update(msg)
        t = self.now()
        p = (self.truth.x, self.truth.y, self.truth.yaw)
        if self._gt_last is None:
            self._gt_last = (t, p)
            return
        if t - self._gt_last[0] >= 0.1:
            t0, p0 = self._gt_last
            dt = t - t0
            dx, dy = p[0] - p0[0], p[1] - p0[1]
            v = (dx * math.cos(p0[2]) + dy * math.sin(p0[2])) / dt
            w = wrap(p[2] - p0[2]) / dt
            vo_v = self.vo_sensor.v(v, C.SENS['vo_sigma_v'])
            vo_w = self.vo_sensor.g(w, C.SENS['vo_sigma_w'])
            self.ekf.vo(t, vo_v, vo_w, C.SENS['vo_sigma_v'] + C.SENS['vo_scale'] * abs(v),
                        C.SENS['vo_sigma_w'] + 0.01)
            self._vo_v, self._vo_t = vo_v, t
            self._gt_last = (t, p)

    @staticmethod
    def dust_filter(ranges, rmin, rmax, amin=0.0, inc=0.0):
        """Airborne dust, smoke or gas particles give isolated returns; rock gives continuous
        surfaces. A return is kept if a neighbouring ray (+/-2) sees a surface at a similar
        range, or if it is collinear with its neighbours (a wall seen at a grazing angle).
        Removed returns become 'no return' (inf)."""
        n = len(ranges)
        out = [float('inf')] * n
        ok = [rmin < r < rmax for r in ranges]
        xy = [(r * math.cos(amin + i * inc), r * math.sin(amin + i * inc)) if ok[i] else None
              for i, r in enumerate(ranges)]
        for i, r in enumerate(ranges):
            if not ok[i]:
                continue
            tol = 0.2 + 0.08 * r
            keep = False
            for j in (i - 1, i + 1, i - 2, i + 2):
                if 0 <= j < n and ok[j] and abs(ranges[j] - r) < tol:
                    keep = True
                    break
            if not keep and inc > 0:
                px, py = xy[i]
                for a, b in ((i - 1, i + 1), (i - 2, i + 2), (i - 2, i - 1), (i + 1, i + 2)):
                    if 0 <= a < n and 0 <= b < n and ok[a] and ok[b]:
                        (ax, ay), (bx, by) = xy[a], xy[b]
                        L = math.hypot(bx - ax, by - ay)
                        if L > 1e-6 and abs((bx - ax) * (ay - py) - (ax - px) * (by - ay)) / L < 0.05 + 0.01 * r:
                            keep = True
                            break
            if keep:
                out[i] = r
        return out

    def on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        ranges = self.dust_filter(list(msg.ranges), msg.range_min, msg.range_max,
                                  msg.angle_min, msg.angle_increment)
        fmin, lmin, rmin = 99.0, 99.0, 99.0
        pts = []
        for i, r in enumerate(ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
            if abs(a) <= 0.35:
                fmin = min(fmin, r)
            elif 1.2 <= a <= 1.95:
                lmin = min(lmin, r)
            elif -1.95 <= a <= -1.2:
                rmin = min(rmin, r)
        # an obstacle ahead must be seen in two consecutive scans (no stop on a stray return)
        self.front_min = max(fmin, self._front_prev)
        self._front_prev = fmin
        self.left, self.right = lmin, rmin
        self.scan = (msg.angle_min, msg.angle_increment, ranges, msg.range_min, msg.range_max)
        self.scan_xy = pts
        # lidar scan matching every 2nd scan
        self._scan_n += 1
        if self._scan_n % 2:
            return
        pts = scan_points(ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                          msg.range_max)
        t = self.now()
        if pts is not None and self._prev_scan is not None:
            dt = t - self._prev_scan_t
            if dt > 0.05:
                dx, dy, dth, fwd_ok, ok = icp(self._prev_scan, pts)
                if ok:
                    self.ekf.icp(t, dx / dt, dth / dt, fwd_ok)
        if pts is not None:
            self._prev_scan, self._prev_scan_t = pts, t
            self.gallery_lock(t, pts)

    def gallery_lock(self, t, pts):
        """Scan-to-map inside a straight drift: when the robot enters a straight gallery it
        records the gallery's wall direction and centre line in its own map; while it stays
        in that gallery, every scan re-measures heading and cross-gallery position against
        them (heading and lateral drift stop growing between junctions)."""
        ax = corridor_axis(pts)
        x, y, th = self.est
        steady = abs(self.ekf.vel[1]) < 0.2
        if self.seg_ref is not None and ax is not None and ax[3] <= 0.1 and steady:
            # on a gallery of the shared map (between two linked junction beacons): lock on
            # the mapped gallery line itself (map-based relocalisation: heading and position
            # across the gallery stay tied to the junction beacons, they cannot drift)
            (sx, sy), (ex, ey) = self.seg_ref
            G = wrap(math.atan2(ey - sy, ex - sx) + math.pi / 2)
            Cc = sx * math.cos(G) + sy * math.sin(G)
            a_n, sp, sn, _ = ax
            mid = (sp + sn) / 2.0
            dA = wrap(wrap(th + a_n) - G)
            flip = abs(dA) > math.pi / 2
            d = wrap(dA - math.pi) if flip else dA
            if abs(d) < 0.12 and 2.5 < sp - sn < 7.0:
                self.ekf.gallery(t, th - d, G, Cc - (-mid if flip else mid), 0.03, 0.3)
            return
        if ax is None or ax[3] > 0.1 or not steady:
            self._gal_cand = []
            if self._gal is not None and t - self._gal[2] > 1.5:
                self._gal = None                    # junction, chamber or dead end: unlock
            return
        a_n, sp, sn, _ = ax
        A = wrap(th + a_n)                           # wall normal in the map frame
        mid, width = (sp + sn) / 2.0, sp - sn
        if self._gal is None:
            # map the gallery from several consistent scans before trusting it
            self._gal_cand.append((A, x * math.cos(A) + y * math.sin(A) + mid, width))
            c = self._gal_cand[-5:]
            if len(c) == 5:
                A0 = c[0][0]
                dev = [wrap(a - A0) for a, _, _ in c]
                if max(dev) - min(dev) < 0.02 and max(w for _, _, w in c) - min(w for _, _, w in c) < 0.3:
                    G = wrap(A0 + sum(dev) / 5.0)
                    self._gal = [G, sum(p for _, p, _ in c) / 5.0, t, sum(w for _, _, w in c) / 5.0]
                    self._gal_cand = []
            return
        G, Cc, _, wid = self._gal
        dA = wrap(A - G)
        flip = abs(dA) > math.pi / 2
        d = wrap(dA - math.pi) if flip else dA
        if abs(d) > 0.12 or abs(width - wid) > 1.0:
            self._gal = None                         # turned into another gallery
            return
        self._gal[2] = t
        self.ekf.gallery(t, th - d, G, Cc - (-mid if flip else mid), 0.01, 0.05)

    def battery_step(self):
        """Battery model: drains with the distance driven (plus an idle draw); recharges on the
        charger of an Outside Network Area station (solar + battery bank) after the mission."""
        x, y, _ = self.est
        if self.charging:
            self.battery = min(1.0, self.battery + 0.5 / C.CHARGE_FULL_S)
        elif self.started():
            d = 0.0 if self._bat_xy is None else min(1.0, math.hypot(x - self._bat_xy[0], y - self._bat_xy[1]))
            self.battery = max(0.0, self.battery - d * C.ENERGY_PER_M
                               - 0.5 * C.GROUND_IDLE_DRAIN / C.GROUND_ENDURANCE)
        self._bat_xy = (x, y)

    def apply_gps(self, m):
        fix = m.get('gps') if m else None
        if fix:
            from .packet import gps_to_local
            x, y = gps_to_local(fix['lat'], fix['lon'], C.GEO_LAT0, C.GEO_LON0, C.GEO_AZ_DEG)
            self.ekf.gps(self.now(), x, y, fix['sigma'])

    # ------------------------------------------------------------ motion
    @property
    def est(self):
        return self.ekf.pose

    def stop(self):
        self.cmd_pub.publish(Twist())

    def gap_direction(self, rel, d, half_width=None):
        """Width-aware free-space steering: for candidate headings around the goal bearing,
        compute how far the robot's footprint can travel before touching rock (from the lidar
        scan) and pick the heading with the best progress toward the goal."""
        if self.scan is None or len(self.scan_xy) < 10:
            return rel
        reach = min(d, 8.0)

        def free(a):
            return self.free_along(a, reach, half_width)
        if free(rel) >= reach - 1e-6:
            return rel
        best_s, best_a = -1.0, rel
        for k in range(-18, 19):                 # +/- 90 deg in 5 deg steps
            a = rel + math.radians(5 * k)
            s = free(a) * math.cos(a - rel)
            if s > best_s + 1e-6:
                best_s, best_a = s, a
        return best_a

    def sight(self, a, half=0.21):
        """Longest lidar ray within +/- half of bearing a (robot frame); no return = beyond range.
        A gallery that continues shows at least one long ray even with rough walls, a robot a
        little off its centre line or a few degrees of heading error."""
        if self.scan is None:
            return 0.0
        amin, inc, ranges, rmin, rmax = self.scan
        best = 0.0
        for i, r in enumerate(ranges):
            if abs(wrap(amin + i * inc - a)) > half:
                continue
            best = max(best, r if rmin < r < rmax else 99.0)
        return best

    def free_along(self, a, reach=8.0, half_width=None):
        """How far the robot's footprint can travel along bearing a (robot frame) before
        touching a lidar return (dust-filtered)."""
        hw = self.half_width if half_width is None else half_width
        ca, sa = math.cos(a), math.sin(a)
        best = reach
        for px, py in self.scan_xy:
            along = px * ca + py * sa
            if along <= 0.0:
                continue
            if abs(-px * sa + py * ca) < hw:
                best = min(best, along - 0.35)
        return max(0.0, best)

    def drive_to(self, tx, ty, vmax, centring=True, near_turn=False):
        """Go to (tx, ty) on the EKF estimate; lidar keeps the robot centred in tunnels and
        steers around rock when the estimate has drifted."""
        x, y, th = self.est
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        err = wrap(desired - th)
        if abs(err) < 1.6:
            err = self.gap_direction(err, max(d, 1.2))
        keep = self.keep_right_offset()
        if centring and not near_turn and abs(err) < 0.35 and self.left < 4.5 and self.right < 4.5 \
                and self.left + self.right < 6.0:
            lateral = (self.left - self.right) / 2.0      # >0: robot is right of centre
            err += max(-0.3, min(0.3, 0.35 * (lateral - keep)))
        tw = Twist()
        now = self.now()
        if now < self._recover_until:              # stuck recovery: back off, turn to open side
            tw.linear.x = -0.3
            tw.angular.z = self._recover_turn
            self.cmd_pub.publish(tw)
            return d
        tw.angular.z = max(-1.0, min(1.0, 2.0 * err))
        if abs(err) < 0.6:
            tw.linear.x = min(vmax * math.cos(err) ** 2, 0.8 * d + 0.08)
        if tw.linear.x > 0.2 and now - self._vo_t < 0.4 and abs(self._vo_v) < 0.05:
            if self._stuck_since is None:
                self._stuck_since = now
            elif now - self._stuck_since > 3.0:
                self._stuck_since = None
                self._recover_until = now + 3.0
                self._recover_turn = 0.4 if self.left > self.right else -0.4
                self.get_logger().warn('stuck (vision says no motion) - backing off')
        else:
            self._stuck_since = None
        if self.front_min < 0.5:
            if self.blocked_since is None:
                self.blocked_since = self.now()
            if self.now() - self.blocked_since < 4.0:
                tw.linear.x = 0.0
            else:
                tw.linear.x = min(tw.linear.x, 0.2)
        else:
            self.blocked_since = None
        self.cmd_pub.publish(tw)
        return d

    def keep_right_offset(self):
        """Traffic rule in a gallery: when another robot is heard ahead and coming closer, both
        keep 0.8 m right of the centre line so they pass each other."""
        x, y, th = self.est
        for p in self.peers:
            if p.get('ex') is None:
                continue
            dx, dy = p['ex'] - x, p['ey'] - y
            along = dx * math.cos(th) + dy * math.sin(th)
            lat = -dx * math.sin(th) + dy * math.cos(th)
            if 0.0 < along < 8.0 and abs(lat) < 2.5:
                return 0.8
        return 0.0

    def gps_fix(self):
        lat, lon = world_to_gps(self.spawn[0], self.spawn[1])
        self.pub('/lm/wireless/to_ona', {'kind': 'gps_fix', 'robot': self.rname, 'lat': lat,
                                         'lon': lon, 'az': C.GEO_AZ_DEG - math.degrees(self.spawn[3]),
                                         't_ms': self.now_ms()})

    def truth_behind(self, back):
        """Physical drop point (behind the robot, simulator truth) for Gazebo + radio physics."""
        return (self.truth.x - back * math.cos(self.truth.yaw),
                self.truth.y - back * math.sin(self.truth.yaw))

    def pose_msg(self, extra):
        x, y, th = self.est
        t = self.now()
        if t - self._track_t >= 1.0:
            self._track_t = t
            self.track_est.append((round(x, 2), round(y, 2)))
            self.track_true.append((round(self.truth.x, 2), round(self.truth.y, 2)))
            self.track_est = self.track_est[-600:]
            self.track_true = self.track_true[-600:]
        err = math.hypot(x - self.truth.x, y - self.truth.y)
        m = {'x': self.truth.x, 'y': self.truth.y, 'yaw': self.truth.yaw,
             'ex': x, 'ey': y, 'eyaw': th, 'err': round(err, 3),
             'sigma': round(self.ekf.f.sigma_xy(), 3), 'stats': self.ekf.stats,
             'battery': round(self.battery, 3), 'charging': self.charging,
             'track_est': self.track_est[-120:], 'track_true': self.track_true[-120:], 't': t}
        m.update(extra)
        return m

    def publish_pose(self):
        pass

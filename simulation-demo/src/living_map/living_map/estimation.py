"""State estimation for GPS-denied robots: EKF + lidar scan matching.

Ground robots (Writer, Executor) - 6-state EKF  [x, y, theta, v, w, gyro_bias]
    predict : unicycle model
    updates : wheel encoders (v, w)      - real Gazebo DiffDrive odometry (skid-steer slip)
              IMU gyro (w + bias)        - real Gazebo IMU sensor (noise + bias)
              visual odometry (v, w)     - sensor MODEL (true motion + scale error + noise)
              lidar scan matching (v, w) - real lidar scans, point-to-point ICP with
                                           degeneracy check (long straight tunnels)
              zero-velocity update       - when the wheels are still (learns gyro bias)

Aerial (Flix) - 9-state EKF [x, y, z, yaw, vbx, vby, vz, w, gyro_bias]
    Flix : gyro (real IMU) + optical flow (MODEL) + downward ToF laser (real 1-beam lidar,
           height above the floor or the water surface) + side ToF gallery locks + beacon fixes

Pure Python matrices (n <= 9); numpy only for ICP (optional).
"""
import math
import random

try:
    import numpy as np
except Exception:          # ICP disabled without numpy
    np = None


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class EKF:
    def __init__(self, x0, p0):
        self.n = len(x0)
        self.x = list(x0)
        self.P = [[p0[i] if i == j else 0.0 for j in range(self.n)] for i in range(self.n)]

    def predict(self, fx, F, q):
        n = self.n
        self.x = fx
        FP = [[sum(F[i][k] * self.P[k][j] for k in range(n) if F[i][k] != 0.0)
               for j in range(n)] for i in range(n)]
        P = [[sum(FP[i][k] * F[j][k] for k in range(n) if F[j][k] != 0.0)
              for j in range(n)] for i in range(n)]
        for i in range(n):
            P[i][i] += q[i]
        self.P = P

    def update(self, h, z, r, angle=False, gate=False, consider=()):
        """Scalar linear measurement z = h . x + noise(r). Returns normalized innovation^2.
        gate=True rejects 5-sigma outliers (used for slip-prone odometry sources).
        consider: states this measurement must not change (Schmidt "consider" update; their
        uncertainty is still accounted for in the gain of the other states)."""
        n = self.n
        pred = sum(h[i] * self.x[i] for i in range(n) if h[i])
        y = z - pred
        if angle:
            y = wrap(y)
        Ph = [sum(self.P[i][k] * h[k] for k in range(n) if h[k]) for i in range(n)]
        S = sum(h[i] * Ph[i] for i in range(n) if h[i]) + r
        if S <= 0:
            return 0.0
        nis = y * y / S
        if gate and nis > 25.0:
            return nis
        K = [Ph[i] / S for i in range(n)]
        if consider:
            for i in range(n):
                if i not in consider:
                    self.x[i] += K[i] * y
            for i in range(n):
                row = self.P[i]
                for j in range(n):
                    if not (i in consider and j in consider):
                        row[j] -= Ph[i] * Ph[j] / S
            return nis
        for i in range(n):
            self.x[i] += K[i] * y
        hP = [sum(h[k] * self.P[k][j] for k in range(n) if h[k]) for j in range(n)]
        for i in range(n):
            ki = K[i]
            if ki:
                row = self.P[i]
                for j in range(n):
                    row[j] -= ki * hP[j]
        return nis

    def sigma_xy(self):
        return math.sqrt(max(0.0, self.P[0][0] + self.P[1][1]))


def _e(n, *idx):
    h = [0.0] * n
    for i in idx:
        h[i] = 1.0
    return h


class GroundEKF:
    """[x, y, th, v, w, b]"""
    X, Y, TH, V, W, B = range(6)

    def __init__(self, x, y, th):
        self.f = EKF([x, y, th, 0.0, 0.0, 0.0], [1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-4])
        self.t = None
        self.stats = {'enc': 0, 'imu': 0, 'vo': 0, 'icp': 0, 'icp_v_rejected': 0, 'zupt': 0}
        self.dist_since_fix = 0.0

    def _predict(self, t):
        if self.t is None:
            self.t = t
            return
        dt = t - self.t
        if dt < 0.05:          # predict at <= 20 Hz; faster measurements update the current state
            return
        dt = min(dt, 0.2)
        self.t = t
        x, y, th, v, w, b = self.f.x
        c, s = math.cos(th), math.sin(th)
        self.dist_since_fix += abs(v) * dt
        fx = [x + v * c * dt, y + v * s * dt, wrap(th + w * dt), v, w, b]
        F = [[1, 0, -v * s * dt, c * dt, 0, 0],
             [0, 1, v * c * dt, s * dt, 0, 0],
             [0, 0, 1, 0, dt, 0],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]]
        q = [1e-6 * dt, 1e-6 * dt, 1e-7 * dt, (1.5 * dt) ** 2, (2.5 * dt) ** 2, (1e-5) ** 2 * dt]
        self.f.predict(fx, F, q)

    def encoders(self, t, v, w, slipping=False):
        self._predict(t)
        self.stats['enc'] += 1
        if slipping:                       # wheels disagree with vision: slip / stuck
            self.stats['slip'] = self.stats.get('slip', 0) + 1
            return
        self.f.update(_e(6, self.V), v, (0.05 + 0.08 * abs(v)) ** 2, gate=True)
        # skid-steer: wheel-derived yaw rate is unreliable while turning
        self.f.update(_e(6, self.W), w, (0.3 + 0.5 * abs(w)) ** 2, gate=True)
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            self.stats['zupt'] += 1
            self.f.update(_e(6, self.V), 0.0, 1e-6)
            self.f.update(_e(6, self.W), 0.0, 1e-6)

    def gyro(self, t, wz, sigma):
        self._predict(t)
        self.stats['imu'] += 1
        self.f.update(_e(6, self.W, self.B), wz, sigma ** 2)

    # VO and scan matching deliver yaw RATES averaged over their frame interval (0.1-0.4 s),
    # i.e. delayed with respect to the gyro. While the robot turns, that delay would be read
    # as gyro bias, so their yaw-rate weight drops with the turn rate: the gyro carries the
    # turns, and the bias learned at standstill (ZUPT) is kept.
    def vo(self, t, v, w, sv, sw):
        self._predict(t)
        self.stats['vo'] += 1
        self.f.update(_e(6, self.V), v, sv ** 2)
        self.f.update(_e(6, self.W), w, (sw + 0.3 * abs(w)) ** 2)

    def icp(self, t, v, w, v_ok):
        self._predict(t)
        self.stats['icp'] += 1
        self.f.update(_e(6, self.W), w, (0.06 + 0.5 * abs(w)) ** 2, gate=True)
        if v_ok:
            self.f.update(_e(6, self.V), v, 0.06 ** 2, gate=True)
        else:
            self.stats['icp_v_rejected'] += 1

    def landmark(self, t, x, y, sigma):
        """Position update from a beacon of the Living Map (the robot passes over a beacon whose
        coordinates are known in the Writer's map frame): keeps every robot consistent with
        the map the Writer built - no prior map of the mine is used."""
        self._predict(t)
        self.stats['beacon_fix'] = self.stats.get('beacon_fix', 0) + 1
        # odometry scale errors are correlated (not white): admit ~2.5 % of the distance
        # travelled since the last fix as position uncertainty before fusing the fix
        infl = (0.025 * self.dist_since_fix) ** 2
        self.f.P[0][0] += infl
        self.f.P[1][1] += infl
        self.dist_since_fix = 0.0
        self.f.update(_e(6, self.X), x, sigma ** 2)
        self.f.update(_e(6, self.Y), y, sigma ** 2)

    def gallery(self, t, th, g, p, s_th, s_p):
        """Scan-to-map gallery lock: heading from the side-wall direction and position across
        the gallery (along its normal g) from the wall distances, both relative to the gallery
        the robot itself mapped when it entered it. Removes drift inside straight drifts."""
        self._predict(t)
        self.stats['gallery'] = self.stats.get('gallery', 0) + 1
        self.f.update(_e(6, self.TH), th, s_th ** 2, angle=True)
        h = [math.cos(g), math.sin(g), 0.0, 0.0, 0.0, 0.0]
        self.f.update(h, p, s_p ** 2)

    def gps(self, t, x, y, sigma):
        """RTK-GPS fix - only available outside the mine."""
        self._predict(t)
        self.stats['gps'] = self.stats.get('gps', 0) + 1
        self.f.update(_e(6, self.X), x, sigma ** 2)
        self.f.update(_e(6, self.Y), y, sigma ** 2)

    @property
    def pose(self):
        return self.f.x[0], self.f.x[1], self.f.x[2]

    @property
    def vel(self):
        return self.f.x[3], self.f.x[4]


class Body3DEKF:
    """[x, y, z, yaw, vbx, vby, vz, w, b] for Flix."""

    def __init__(self, x, y, z, yaw):
        self.f = EKF([x, y, z, yaw, 0, 0, 0, 0, 0], [1e-4, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2, 1e-2, 1e-4])
        self.t = None
        self.t_still = -1e9

    def _predict(self, t):
        if self.t is None:
            self.t = t
            return
        dt = t - self.t
        if dt < 0.019:          # fast yawing drone: integrate every gyro sample
            return
        dt = min(dt, 0.2)
        self.t = t
        x, y, z, yaw, bx, by, vz, w, b = self.f.x
        c, s = math.cos(yaw), math.sin(yaw)
        fx = [x + (bx * c - by * s) * dt, y + (bx * s + by * c) * dt, z + vz * dt,
              wrap(yaw + w * dt), bx, by, vz, w, b]
        F = [[1, 0, 0, (-bx * s - by * c) * dt, c * dt, -s * dt, 0, 0, 0],
             [0, 1, 0, (bx * c - by * s) * dt, s * dt, c * dt, 0, 0, 0],
             [0, 0, 1, 0, 0, 0, dt, 0, 0],
             [0, 0, 0, 1, 0, 0, 0, dt, 0],
             [0, 0, 0, 0, 1, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 1, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 0, 0, 0, 1]]
        a = 3.0 * dt
        # optical flow / DVL scale error: position uncertainty grows with the distance flown,
        # so a sideways wall fix corrects the position instead of being blamed on the heading
        qp = 1e-6 * dt + (0.1 * math.hypot(bx, by)) ** 2 * dt
        q = [qp, qp, 1e-6 * dt, 1e-7 * dt, a * a, a * a, a * a, (3 * dt) ** 2,
             (2e-4) ** 2 * dt]
        self.f.predict(fx, F, q)

    def gyro(self, t, wz, sigma):
        self._predict(t)
        if t - self.t_still < 0.15:          # still() runs every 0.1 s control tick
            # standing still (on the pad / at the surface): the gyro reads its own bias -
            # the calibration every drone does before take-off
            self.f.update(_e(9, 8), wz, sigma ** 2)
            return
        self.f.update(_e(9, 7, 8), wz, sigma ** 2)

    def body_vel(self, t, vx, vy, sigma):          # optical flow / DVL
        self._predict(t)
        self.f.update(_e(9, 4), vx, sigma ** 2)
        self.f.update(_e(9, 5), vy, sigma ** 2)

    def vz(self, t, vz, sigma):
        self._predict(t)
        self.f.update(_e(9, 6), vz, sigma ** 2)

    def height(self, t, z, sigma):                 # ToF over known floor / pressure depth
        self._predict(t)
        self.f.update(_e(9, 2), z, sigma ** 2)

    def gps(self, t, x, y, sigma, keep_yaw=False):
        # the gyro bias is calibrated on the pad / surface (still) and by the gyro itself: a
        # position fix (GPS, a beacon's mapped position) must not re-tune it, or that fix's own
        # error would turn into a steady heading drift for the rest of the flight. keep_yaw: a
        # large along-track correction inside the mine must not rotate the heading either.
        self._predict(t)
        cons = (3, 8) if keep_yaw else (8,)
        self.f.update(_e(9, 0), x, sigma ** 2, consider=cons)
        self.f.update(_e(9, 1), y, sigma ** 2, consider=cons)

    def still(self, t):
        self._predict(t)
        self.t_still = t
        for i in (4, 5, 6, 7):
            self.f.update(_e(9, i), 0.0, 1e-6)

    def line(self, t, g, p, sigma):
        """Position across a gallery (along its normal g) measured from the side walls."""
        self._predict(t)
        h = [math.cos(g), math.sin(g), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.f.update(h, p, sigma ** 2, consider=(8,))

    @property
    def pose(self):
        return self.f.x[0], self.f.x[1], self.f.x[2], self.f.x[3]


# ------------------------------------------------------------------ lidar scan matching
def scan_points(ranges, angle_min, angle_inc, rmin, rmax, max_pts=180):
    if np is None:
        return None
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin) & (r < min(rmax, 25.0))
    r, a = r[ok], a[ok]
    if len(r) < 20:
        return None
    step = max(1, len(r) // max_pts)
    r, a = r[::step], a[::step]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def icp(prev, cur, iters=12, max_d=0.6):
    """Point-to-point ICP. Returns (dx, dy, dth, forward_observable, ok): motion of the
    robot from the previous scan to the current one, expressed in the previous frame."""
    if np is None or prev is None or cur is None:
        return 0.0, 0.0, 0.0, False, False
    R = np.eye(2)
    t = np.zeros(2)
    src = cur.copy()
    matched = None
    for _ in range(iters):
        moved = src @ R.T + t
        d2 = ((moved[:, None, :] - prev[None, :, :]) ** 2).sum(axis=2)
        j = d2.argmin(axis=1)
        dmin = np.sqrt(d2[np.arange(len(moved)), j])
        m = dmin < max_d
        if m.sum() < 15:
            return 0.0, 0.0, 0.0, False, False
        A, B = moved[m], prev[j[m]]
        ca, cb = A.mean(axis=0), B.mean(axis=0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        Ri = Vt.T @ U.T
        if np.linalg.det(Ri) < 0:
            Vt[1, :] *= -1
            Ri = Vt.T @ U.T
        ti = cb - Ri @ ca
        R = Ri @ R
        t = Ri @ t + ti
        matched = B
    # degeneracy: normals of matched map points; weak constraint direction = long tunnel axis
    obs_forward = False
    if matched is not None and len(matched) > 20:
        nrm = []
        for k in range(0, len(matched), 3):
            p = matched[k]
            dd = ((matched - p) ** 2).sum(axis=1)
            nb = matched[np.argsort(dd)[:6]]
            c = np.cov((nb - nb.mean(axis=0)).T)
            w, v = np.linalg.eigh(c)
            nrm.append(v[:, 0])
        N = np.array(nrm)
        C = N.T @ N
        w, v = np.linalg.eigh(C)
        weak = v[:, 0]
        ratio = w[0] / max(w[1], 1e-9)
        obs_forward = ratio > 0.08 or abs(weak[0]) < 0.5   # x = forward in robot frame
    th = math.atan2(R[1, 0], R[0, 0])
    moved = cur @ R.T + t
    d2 = ((moved[:, None, :] - prev[None, :, :]) ** 2).sum(axis=2)
    res = np.sqrt(d2.min(axis=1))
    inl = res < max_d
    rms = float(np.sqrt((res[inl] ** 2).mean())) if inl.sum() else 9.0
    good = inl.mean() > 0.6 and rms < 0.12
    return float(t[0]), float(t[1]), th, obs_forward, good


def corridor_axis(pts, max_range=8.0):
    """Gallery geometry from one scan (robot frame): returns (a_n, s_pos, s_neg, ratio) or
    None. a_n = direction of the side-wall normal (the gallery axis is a_n + pi/2, mod pi),
    s_pos / s_neg = distances of the two side walls along that normal, ratio = straightness
    (small = both walls straight and parallel). Used for the scan-to-map gallery lock."""
    if np is None or pts is None:
        return None
    r = np.hypot(pts[:, 0], pts[:, 1])
    P = pts[r < max_range]
    if len(P) < 40:
        return None
    nrm = []
    for k in range(0, len(P), 2):
        dd = ((P - P[k]) ** 2).sum(axis=1)
        nb = P[np.argsort(dd)[:7]]
        if ((nb - P[k]) ** 2).sum(axis=1).max() > 0.8 ** 2:
            continue                                   # isolated point (corner, opening)
        c = np.cov((nb - nb.mean(axis=0)).T)
        w, v = np.linalg.eigh(c)
        nrm.append(v[:, 0])
    if len(nrm) < 20:
        return None
    N = np.array(nrm)
    w, v = np.linalg.eigh(N.T @ N)
    n = v[:, 1]                                        # dominant wall normal (coarse)
    ratio = float(w[0] / max(w[1], 1e-9))
    s = P @ n
    L, R = P[s > 0.3], P[s < -0.3]
    if len(L) < 12 or len(R) < 12:
        return None
    # both side walls must be straight lines: most points close to the wall's median offset
    sl, sr = L @ n, R @ n
    il, ir = np.abs(sl - np.median(sl)) < 0.15, np.abs(sr - np.median(sr)) < 0.15
    if il.mean() < 0.75 or ir.mean() < 0.75 or il.sum() < 10 or ir.sum() < 10:
        return None
    L, R = L[il], R[ir]
    # refine: joint line fit of the two walls (common direction, long baseline)
    Q = np.vstack([L - L.mean(axis=0), R - R.mean(axis=0)])
    w2, v2 = np.linalg.eigh(Q.T @ Q)
    d = v2[:, 1]
    if w2[0] / max(w2[1], 1e-9) > 0.01:
        return None                                    # walls not parallel / not straight
    n2 = np.array([-d[1], d[0]])
    if n2 @ n < 0:
        n2 = -n2
    ext = lambda A: float(np.ptp(A @ d))
    if ext(L) < 3.0 or ext(R) < 3.0:
        return None                                    # too short to define a gallery
    return math.atan2(n2[1], n2[0]), float((L @ n2).mean()), float((R @ n2).mean()), ratio


class NoisySensor:
    """Sensor models driven by simulator truth (VO, optical flow, DVL, depth)."""

    def __init__(self, seed, scale_err=0.0):
        self.rng = random.Random(seed)
        self.scale = 1.0 + scale_err * (1 if self.rng.random() > 0.5 else -1)

    def v(self, true_v, sigma):
        return true_v * self.scale + self.rng.gauss(0.0, sigma)

    def g(self, value, sigma):
        return value + self.rng.gauss(0.0, sigma)

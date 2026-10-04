"""Tunnel line-of-sight, radio model (ESP-NOW links + Wi-Fi CSI sensing channel) and the
self-healing relocation planner."""
import math
import os

from . import config as C

_EPS = 1e-6


def inside_free(x, y):
    for (x0, x1, y0, y1) in C.TUNNEL_RECTS:
        if x0 - _EPS <= x <= x1 + _EPS and y0 - _EPS <= y <= y1 + _EPS:
            return True
    x0, x1, y0, y1 = C.OUTSIDE_RECT
    return x0 <= x <= x1 + _EPS and y0 <= y <= y1


SNAP = 2.5      # real SubT galleries are wider / less regular than the 4 m model corridors


def snap_free(p):
    """Nearest point of the modelled free space to p if within SNAP metres, else p.
    Radio endpoints (robots, beacons dropped at real positions) near a real tunnel wall may sit
    just outside the conservative 4 m corridor model."""
    if inside_free(p[0], p[1]):
        return p
    best, bd = p, SNAP
    for (x0, x1, y0, y1) in list(C.TUNNEL_RECTS) + [C.OUTSIDE_RECT]:
        q = (min(max(p[0], x0), x1), min(max(p[1], y0), y1))
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        if d <= bd:
            best, bd = q, d
    return best


# test hook (dry-run stress test): LM_RADIO_HOLE="x0,x1,y0,y1" makes every radio line of sight
# from inside that box fail - reproduces a real gallery that differs from the corridor model
_HOLE = [float(v) for v in os.environ.get('LM_RADIO_HOLE', '').split(',') if v.strip()]


def _in_hole(p):
    return len(_HOLE) == 4 and _HOLE[0] <= p[0] <= _HOLE[1] and _HOLE[2] <= p[1] <= _HOLE[3]


def los(a, b, step=0.1):
    """True if the straight segment a->b stays inside free space (no rock in between)."""
    a, b = snap_free(a), snap_free(b)
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    n = max(1, int(d / step))
    for i in range(n + 1):
        t = i / n
        if not inside_free(a[0] + t * dx, a[1] + t * dy):
            return False
    return True


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rssi(d):
    return C.RF_P1M - 10.0 * C.RF_N * math.log10(max(d, 0.1))


_LINK_CACHE = {}


def link_ok(a, b, rng=None):
    """Radio link a<->b: within range and in line of sight, or short enough (RF_NLOS_RANGE) to
    get through without line of sight. Cached: beacons only move while self-healing."""
    rng = C.RF_RANGE if rng is None else rng
    key = (round(a[0], 2), round(a[1], 2), round(b[0], 2), round(b[1], 2), rng)
    ok = _LINK_CACHE.get(key)
    if ok is None:
        d = dist(a, b)
        hole = _HOLE and (_in_hole(a) or _in_hole(b))
        ok = d <= rng and (d <= C.RF_NLOS_RANGE or (not hole and los(a, b)))
        if len(_LINK_CACHE) > 50000:
            _LINK_CACHE.clear()
        _LINK_CACHE[key] = ok
    return ok


def freshness(age):
    return math.exp(-max(0.0, age) / C.TRAIL_TAU)


# ------------------------------------------------------------------ routing
def route_to_entrance(beacons, bid):
    """List of beacon ids from bid up to the entrance (inclusive), or None if broken."""
    seen = set()
    path = []
    cur = bid
    while True:
        b = beacons.get(cur)
        if b is None or not b['alive'] or cur in seen:
            return None
        seen.add(cur)
        path.append(cur)
        if b['parent'] == C.PARENT_ONA:
            return path
        cur = b['parent']


def route_ok(beacons, bid, memo=None):
    """True if the pointer route bid -> entrance exists AND every hop on it is a live radio link.
    beacons: {id: {'x','y','parent','alive'}}. memo: optional dict shared between calls."""
    memo = {} if memo is None else memo
    path = []
    cur = bid
    seen = set()
    ok = None
    while True:
        if cur in memo:
            ok = memo[cur]
            break
        b = beacons.get(cur)
        if b is None or not b['alive'] or cur in seen:
            ok = False
            break
        seen.add(cur)
        path.append(cur)
        if b['parent'] == C.PARENT_ONA:
            ok = True
            break
        p = beacons.get(b['parent'])
        if p is None or not p['alive'] or not link_ok((b['x'], b['y']), (p['x'], p['y'])):
            ok = False
            break
        cur = b['parent']
    for k in path:
        memo[k] = ok
    return ok


def subtree(beacons, bid):
    out = {bid}
    changed = True
    while changed:
        changed = False
        for k, b in beacons.items():
            if b['parent'] in out and k not in out:
                out.add(k)
                changed = True
    return out


def _sample_polyline(pts, step=0.25):
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        d = dist(a, b)
        n = max(1, int(d / step))
        for i in range(1, n + 1):
            t = i / n
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def plan_relocation(beacons, orphan_id, dead_id):
    """Self-healing planner.

    The orphan (child of the dead beacon) searches along the path it knows back
    towards the entrance (orphan -> dead beacon -> dead beacon's parent) for the
    closest point from which it can reach a routed beacon while still keeping
    all its own children in range. Returns (x, y, new_parent_id, move_dist) or None.
    beacons: {id: {'x','y','parent','alive'}} in world frame.
    """
    orphan = beacons[orphan_id]
    dead = beacons[dead_id]
    sub = subtree(beacons, orphan_id)
    memo = {}
    cands = [k for k, b in beacons.items()
             if b['alive'] and k not in sub and k != dead_id and route_ok(beacons, k, memo)]
    children = [k for k, b in beacons.items() if b['alive'] and b['parent'] == orphan_id]
    poly = [(orphan['x'], orphan['y']), (dead['x'], dead['y'])]
    gp = beacons.get(dead['parent'])
    if gp is not None:
        poly.append((gp['x'], gp['y']))
    lim = C.RF_RANGE - C.HEAL_MARGIN
    start = (orphan['x'], orphan['y'])
    for p in _sample_polyline(poly):
        ok_children = all(link_ok(p, (beacons[c]['x'], beacons[c]['y']), lim) for c in children)
        if not ok_children:
            continue
        best = None
        for k in cands:
            q = (beacons[k]['x'], beacons[k]['y'])
            if link_ok(p, q, lim):
                r = rssi(dist(p, q))
                if best is None or r > best[1]:
                    best = (k, r)
        if best is not None:
            return (p[0], p[1], best[0], dist(start, p))
    return None


# ------------------------------------------------------------------ physical trail graph
def shortest_path(adj, src, dst):
    """Dijkstra on an undirected weighted graph {node: {nbr: length}}. Returns (length, [nodes])
    or (None, None)."""
    import heapq
    if src == dst:
        return 0.0, [src]
    best = {src: 0.0}
    prev = {}
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == dst:
            path = [u]
            while path[-1] in prev:
                path.append(prev[path[-1]])
            return d, path[::-1]
        if d > best.get(u, 1e18):
            continue
        for v, w in adj.get(u, {}).items():
            nd = d + w
            if nd < best.get(v, 1e18):
                best[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return None, None


# ------------------------------------------------------------------ Wi-Fi CSI sensing
def seg_dist(p, a, b):
    """Distance from point p to the segment a-b."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def csi_link_ok(a, b):
    """A usable Wi-Fi CSI link: a few metres apart at least, within CSI_LINK_R, line of sight."""
    d = dist(a, b)
    return 1.5 <= d <= C.CSI_LINK_R and los(a, b)


def csi_link_score(src, rx, people, rng=None):
    """Channel State Information change on the Wi-Fi link src -> rx over one window (0 = empty
    channel, 1 = strongly disturbed). In a gallery the multipath makes the whole cross-section
    near the link sensitive: a person within ~CSI_ZONE of the link disturbs it, in proportion to
    how much the body moves (walking >> breathing >> still). people: [{'x','y','motion'}]."""
    s = 0.0
    for v in people:
        d = seg_dist((v['x'], v['y']), src, rx)
        if d > 3.0 * C.CSI_ZONE:
            continue
        s = max(s, C.CSI_MOTION.get(v.get('motion', 'still'), 0.3) * math.exp(-(d / C.CSI_ZONE) ** 2))
    if rng is not None:
        s += rng.gauss(0.0, C.CSI_NOISE)
    return max(0.0, min(1.0, s))


def csi_classify(scores):
    """Stand-in for the AI presence classifier (trained on empty / occupied CSI windows): each
    link is weak evidence, several disturbed links agree -> presence probability."""
    q = 1.0
    for s in scores:
        q *= 1.0 - 0.9 * max(0.0, min(1.0, (s - 0.1) / 0.9))
    return 1.0 - q


def fuse_confidence(c_cam, c_csi):
    """Camera and CSI are independent evidence of the same person: combined confidence."""
    return min(0.99, 1.0 - (1.0 - c_cam) * (1.0 - c_csi))

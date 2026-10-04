"""Core tests (no ROS needed):  python3 -m pytest src/living_map/test  or  python3 test_core.py"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from living_map import config as C  # noqa: E402
from living_map import packet as P  # noqa: E402
from living_map.geometry import link_ok, los, plan_relocation, route_to_entrance  # noqa: E402


def _ros():
    """Node modules import rclpy: without ROS here, use the dry-run stand-in."""
    try:
        import rclpy  # noqa: F401
    except ImportError:
        from living_map import fake_ros
        fake_ros.install()


def test_packet_is_32_bytes_and_roundtrips():
    """The record stores what the report asks: event identifier, event type, position,
    timestamp, detection confidence, source / Writer identifier (+ where to go)."""
    p = {'id': 7, 'type': C.EV_VICTIM, 'parent': 6, 'flags': C.FL_NO_GROUND, 'src': C.SRC_ID['flix_b'],
         'x': 48.48, 'y': -9.79, 'z': -0.35, 'heading': -90.0, 'a': 32, 'b': C.CAM_THERMAL,
         'conf': 0.87, 'next_in': 0, 'bearing_in': 135.5, 'dist_in': 1.8, 'next_out': 5,
         't_ms': 150700, 'eid': 42}
    raw = P.encode(p)
    assert len(raw) == 32
    d = P.decode(raw)
    assert d['id'] == 7 and d['type'] == C.EV_VICTIM and d['parent'] == 6
    assert d['src'] == C.SRC_ID['flix_b'] and d['source'] == 'Flix B' and d['flags'] == C.FL_NO_GROUND
    assert abs(d['conf'] - 0.87) < 1e-9 and d['eid'] == 42
    assert abs(d['x'] - 48.48) < 1e-4 and abs(d['y'] + 9.79) < 1e-4 and d['t_ms'] == 150700
    assert d['z'] == -0.35 and d['a'] == 32 and d['b'] == C.CAM_THERMAL and d['next_out'] == 5
    assert abs(d['bearing_in'] - 135.5) < 0.01 and abs(d['dist_in'] - 1.8) < 0.05
    assert d['utc'] == C.MISSION_EPOCH_UTC + 150 and d['utc_ms'] == 700      # absolute time
    assert 'confidence 87 %' in P.value_text(d) and 'thermal' in P.value_text(d)
    # CSI presence: region radius + links, a CSI-network event id (128..255)
    q = P.decode(P.encode({'id': 55, 'type': C.EV_PRESENCE, 'src': 2, 'x': 70.3, 'y': -17.1,
                           'a': 8, 'b': 3, 'conf': 0.83, 'eid': 130, 't_ms': 232000}))
    assert q['type'] == C.EV_PRESENCE and q['eid'] == 130 and q['source'] == 'Writer B'
    assert 'presence within 8 m (3 links), confidence 83 %' in P.value_text(q)


def test_csi_channel_model_and_classifier():
    """Wi-Fi CSI: a person near a link disturbs it (moving >> breathing >> still); several
    disturbed links -> presence; an empty channel -> no presence."""
    from living_map.geometry import csi_classify, csi_link_ok, csi_link_score, fuse_confidence
    a, b = (50.0, 16.0), (50.0, 29.0)                     # beacon -> CSI node along a branch
    near = [{'x': 50.8, 'y': 23.5, 'motion': 'moving'}]
    assert csi_link_score(a, b, near) > 0.85
    assert csi_link_score(a, b, [dict(near[0], motion='still')]) < 0.3
    assert csi_link_score(a, b, [{'x': 50.0, 'y': 45.0, 'motion': 'moving'}]) < 0.01
    assert csi_link_ok(a, b) and not csi_link_ok((50.0, 16.0), (90.0, 0.0))   # rock / range
    assert csi_classify([0.6, 0.6]) >= C.CSI_THRESHOLD > csi_classify([0.6])
    assert csi_classify([0.02] * 8) < 0.05
    # fusion of independent evidence (camera 84 % + CSI 70 %) -> one hypothesis, 95 %
    assert abs(fuse_confidence(0.84, 0.70) - 0.952) < 1e-3 and fuse_confidence(0.99, 0.99) == 0.99


def test_frontier_utility():
    """Utility = gain + relevance - distance - energy + accessibility (report 2.2)."""
    import math
    from types import SimpleNamespace
    _ros()
    from living_map.writer_node import Writer
    w = SimpleNamespace(lkp=(70.0, -20.0), marks=[], battery=1.0, gas_reported=[], lessons={})
    w.exit_bias = lambda n, k: Writer.exit_bias(w, n, k)
    n = {'x': 70.0, 'y': 0.0}
    ex = {'st': 'open', 'd': 12.0, 'w': 1.0}
    south = Writer.exit_utility(w, n, '-90', ex, 0.0, 70.0)
    north = Writer.exit_utility(w, n, '90', ex, 0.0, 70.0)
    assert south['U'] > north['U'] and south['rel'] > north['rel']          # toward the miners
    far = Writer.exit_utility(w, n, '-90', ex, 60.0, 70.0)
    assert far['U'] < south['U']                                         # distance costs
    # a person seen only by a Flix in that direction raises the relevance
    w.marks = [{'kind': 'victim', 'x': 70.0, 'y': 25.0, 'at_victim': False}]
    assert Writer.exit_utility(w, n, '90', ex, 0.0, 70.0)['rel'] > north['rel']
    # a hazard in that direction lowers the accessibility; not enough battery -> not feasible
    w.gas_reported = [(70.0, 8.0)]
    assert Writer.exit_utility(w, n, '90', ex, 0.0, 70.0)['acc'] < 1.0
    w.battery = 0.01
    assert Writer.exit_utility(w, n, '90', ex, 0.0, 70.0) is None
    assert math.isclose(C.ENERGY_PER_M * C.WRITER_SPEED * C.GROUND_ENDURANCE, 1.0)


def test_executor_order_confidence_then_distance():
    _ros()
    from living_map.command_post import conf_class
    c = [(-conf_class(0.97), 120.0, 'V2'), (-conf_class(0.96), 40.0, 'V4'),
         (-conf_class(0.84), 10.0, 'V3')]
    assert [k for _, _, k in sorted(c)] == ['V4', 'V2', 'V3']     # same 5 % class: nearest first


def test_gps_roundtrip_centimetre():
    for x, y in ((0, 0), (50, -20), (-10, 15)):
        lat, lon = P.local_to_gps(x, y, C.GEO_LAT0, C.GEO_LON0, C.GEO_AZ_DEG)
        x2, y2 = P.gps_to_local(lat, lon, C.GEO_LAT0, C.GEO_LON0, C.GEO_AZ_DEG)
        assert abs(x - x2) < 0.01 and abs(y - y2) < 0.01


def test_briefing_under_espnow_limit():
    path = [(i, float(i), 0.0) for i in range(1, 9)]
    raw = P.encode_briefing(P.MSG_BRIEF, 8, path, (49.8, -14.0), [(4, C.EV_HAZARD, 16.5, -0.9)],
                            (0.0, -1.8), 0.0, 0.96, 6.0)
    assert len(raw) <= 250
    b = P.decode_briefing(raw)
    assert b['target'] == 8 and len(b['path']) == 8 and b['verify'][0][0] == 4
    assert abs(b['conf'] - 0.96) < 1e-9 and b['radius'] == 6


def test_rock_blocks_line_of_sight_around_corners():
    assert los((1, 0), (40, 0))              # straight along line B
    assert not los((1, 0), (10, 8))          # into rock
    assert not link_ok((34, 0), (70, -9))    # around the X2 corner into the south branch
    assert link_ok((68, 0), (70, -8.8))      # from the junction down the south branch
    assert not link_ok((1, 0), (45, 0))      # beyond the 30 m ESP-NOW range


def test_self_healing_plan_restores_chain():
    g = C.GUARD_SPACING
    b = {1: {'x': 0.3, 'y': 0.1, 'parent': 0, 'alive': True},
         2: {'x': g, 'y': 0.0, 'parent': 1, 'alive': True},
         4: {'x': 2 * g, 'y': 0.0, 'parent': 2, 'alive': False},     # destroyed
         5: {'x': 3 * g, 'y': 0.0, 'parent': 4, 'alive': True},
         6: {'x': 3 * g + 14, 'y': 0.0, 'parent': 5, 'alive': True}}
    plan = plan_relocation(b, 5, 4)
    assert plan is not None
    x, y, new_parent, move = plan
    assert new_parent == 2 and 3.0 < move < 12.0, plan
    b[5].update(x=x, y=y, parent=new_parent)
    assert route_to_entrance(b, 6) == [6, 5, 2, 1]


def test_gallery_lock_geometry():
    """Scan-to-map gallery measurement on a synthetic straight gallery (4 m wide)."""
    import math
    import numpy as np
    from living_map.estimation import corridor_axis
    th, off = 0.1, 0.4                       # robot heading and offset from the centre line
    pts = []
    for s in np.arange(-6, 6, 0.1):
        for wy in (2.0, -2.0):
            wx, wyr = s, wy - off            # wall point relative to the robot (gallery frame)
            c, si = math.cos(-th), math.sin(-th)
            pts.append((c * wx - si * wyr, si * wx + c * wyr))
    a_n, sp, sn, ratio = corridor_axis(np.array(pts))
    assert ratio < 0.05
    assert abs(((a_n + th) % math.pi) - math.pi / 2) < 1e-3   # wall normal = gallery normal
    assert abs((sp + sn) / 2 * (1 if math.cos(a_n + th - math.pi / 2) > 0 else -1) + off) < 0.02


def test_radio_tolerates_wider_real_galleries():
    from living_map.geometry import snap_free
    assert los((20, 2.8), (35, -2.6))        # endpoints just outside the 4 m model corridor
    assert snap_free((20, 2.8))[1] == 2.0
    assert not los((20, 6.0), (35, 0))       # 4 m into rock stays blocked


def test_short_radio_links_need_no_line_of_sight():
    from living_map.geometry import link_ok, los, route_ok
    from living_map import config as C
    assert not los((50.0, 6.0), (54.0, 0.0))
    assert link_ok((50.0, 6.0), (54.0, 0.0))    # 7.2 m round a junction corner: still a link
    assert not link_ok((20.0, 7.0), (35.0, 0.0))    # 16 m with rock in between: no link
    assert C.RF_NLOS_RANGE < C.GUARD_SPACING
    # route_ok checks the radio link of every hop, not only the parent pointers
    view = {1: {'x': 0.0, 'y': 0.0, 'parent': 0, 'alive': True},
            2: {'x': 17.0, 'y': 0.0, 'parent': 1, 'alive': True},
            3: {'x': 17.0, 'y': 40.0, 'parent': 2, 'alive': True},     # 40 m away: link broken
            4: {'x': 17.0, 'y': 45.0, 'parent': 3, 'alive': True}}
    assert route_ok(view, 2) and not route_ok(view, 3) and not route_ok(view, 4)


def test_mission_memory_lessons():
    """The server turns stored missions (from every ONA) into Bayesian lessons."""
    _ros()
    from living_map.command_post import MissionMemory
    os.environ['LM_MEMORY'] = 'off'
    m = MissionMemory()
    m.missions = [{'onas': ['B'], 'beacon_lost': [{'x': 21.0, 'y': 0.2}], 'writer_lost': [],
                   'gas_fp': 1, 'gas_real': 1, 'victims': [{'x': 70.0, 'y': -19.0}],
                   'exec_speed': 0.62},
                  {'onas': ['A'], 'beacon_lost': [{'x': 22.5, 'y': -0.4}], 'writer_lost': [],
                   'gas_fp': 1, 'gas_real': 0, 'victims': [{'x': 71.0, 'y': -19.5}],
                   'exec_speed': 0.58}]
    L = m.lessons()
    assert L['missions'] == 2 and L['onas'] == ['A', 'B']
    assert L['rockfall'] and abs(L['rockfall'][0]['x'] - 21.75) < 0.1        # zone learned
    assert abs(L['gas_fp'] - (1 + 2) / (1 + 2 + 9 + 1)) < 1e-3 and L['gas_confirm']
    assert L['victim_prior'] and abs(L['exec_speed'] - 0.6) < 1e-6


def test_dfs_exit_keys():
    _ros()
    from living_map.writer_node import back_key, key_of
    import math
    assert key_of(0.0) == '0' and key_of(math.pi / 2) == '90' and key_of(-math.pi / 2) == '-90'
    assert key_of(math.pi) == '180' and key_of(-math.pi) == '180' and back_key('90') == '-90'
    assert key_of(math.radians(87)) == '90'                 # heading noise snaps to the gallery


def test_chase_camera_stays_in_the_gallery():
    import math
    _ros()
    from living_map.director_node import chase_point
    from living_map.geometry import inside_free
    # Writer at the gas-branch dead end, turning around: camera must not go through rock
    for yaw in [k * math.pi / 8 for k in range(16)]:
        cx, cy = chase_point(50.0, 27.0, yaw, 1.8)
        assert inside_free(cx, cy), (yaw, cx, cy)
    cx, cy = chase_point(30.0, 0.0, 0.0, 1.8)      # straight gallery: full distance behind
    assert abs(cx - 28.2) < 0.15 and abs(cy) < 1e-6
    # across the joint of two gallery segments (rockfall shot): still the full distance
    for x in (17.5, 21.0, 38.0):
        cx, cy = chase_point(x, 0.0, math.pi, 4.5)
        assert abs(cx - (x + 4.5)) < 0.15, (x, cx)


if __name__ == '__main__':
    for k, f in list(globals().items()):
        if k.startswith('test_'):
            f()
            print('ok', k)

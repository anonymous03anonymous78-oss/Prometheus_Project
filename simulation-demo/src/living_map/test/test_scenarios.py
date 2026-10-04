"""End-to-end mission checks for all 5 scenarios (runs the real nodes on the dry-run stand-in),
aligned with the technical report: exploration, spatial memory, CSI + camera victim detection
and fusion, Executor order, failure cases.
    python3 src/living_map/test/test_scenarios.py            (LM_PERTURB=1 for the robustness run)

The mission memory is switched off (LM_MEMORY=off) so every run starts from the same state.
"""
import json
import os
import subprocess
import sys
import tempfile

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def run(scenario):
    rep = os.path.join(os.environ.get('LM_REPORT_DIR') or tempfile.mkdtemp(), f'report_{scenario}.json')
    env = dict(os.environ, LM_MEMORY=os.environ.get('LM_MEMORY', 'off'))
    subprocess.run([sys.executable, '-m', 'living_map.dry_run', '--scenario', scenario, '--fast',
                    '--no-view', '--quiet', '--report', rep], cwd=PKG, check=True,
                   capture_output=True, timeout=2400, env=env)
    with open(rep) as f:
        return json.load(f)


def check(scenario):
    sys.path.insert(0, PKG)
    from living_map import config as C

    def conf_class(c):                 # = command_post.conf_class (5 % classes)
        return round(c * 20.0) / 20.0
    r = run(scenario)
    cp, k = r['cp'], r['flow_kinds']
    flows = r['flows']
    wd = scenario in ('writer_destroyed', 'all')
    kn = scenario in ('beacon_knockout', 'all')
    fp = scenario in ('false_positive', 'all')
    # 1. portals found by the two Flix, without a list: A, B clear, C flooded, D collapsed
    ents = {e['id']: e['state'] for e in cp['entrances']}
    assert ents == {'A': 'clear', 'B': 'clear', 'C': 'flooded', 'D': 'collapsed'}, ents
    assert cp['assign'] == {'writer_a': 'A', 'writer_b': 'B'}, cp['assign']
    # 2. Writers: frontier exploration (modified DFS + utility function), junction beacons =
    #    mapping events, a junction found by one Writer used by the other
    drops = [f for f in flows if f['kind'] == 'drop']
    for w in ('writer_a', 'writer_b'):
        assert any(f['by'] == w and f['btype'] == 'entrance' for f in drops), w
        assert any(f['by'] == w and f['btype'] == 'junction' for f in drops), w
        assert sum(1 for f in drops if f['by'] == w) <= C.BEACON_STOCK, 'beacon stock exceeded'
    shared = [f for f in flows if f['kind'] == 'junction' and f.get('claim') is not None
              and f['by'] == 'writer_a' and 51 <= f['id'] <= 99]
    assert shared, 'Writer A never took an exit of a junction mapped by Writer B'
    assert any(r['poses'][w].get('utility') for w in C.WRITERS), 'no frontier utility evaluated'
    # 3. Wi-Fi CSI presence sensing: ESP32-S3 nodes dropped by the slow Writers, a PRESENCE
    #    from an aggregator's classifier, camera + CSI fused into one victim hypothesis
    csi = [f for f in flows if f['kind'] == 'csi_drop']
    assert {f['by'] for f in csi} >= ({'writer_a'} if wd else {'writer_a', 'writer_b'}), csi
    assert all(sum(1 for f in csi if f['by'] == w) <= C.CSI_STOCK for w in C.WRITERS)
    assert k.get('presence', 0) >= 1, 'no CSI presence detection'
    vic = [e for e in cp['events'] if e['type'] == 'VICTIM']
    assert any(e['cam'] > 0 and e['csi'] > 0 for e in vic), 'no camera + CSI fusion'
    assert all(0.0 < e['conf'] <= 0.99 for e in vic)
    # 4. flooded portal C: Flix B flies over the water, finds the miner in the air pocket with its
    #    camera and writes him into ONA-C's own fibered first beacon: not reachable by ground
    c = [e for e in vic if e['no_ground']]
    assert len(c) == 1 and c[0]['sources'][0].startswith('Flix B') and c[0]['ona'] == 'C', c
    assert any(f['id'] == C.ONA_BEACON_ID for f in drops) and k.get('ona_beacon', 0) == 1
    # 5. the three miners ground robots can reach: identified on site by the Executor, visited
    #    most confident first, then the nearest
    mine = [e for e in vic if not e['no_ground']]
    assert len(mine) == 3, [(e['key'], e['mx'], e['my']) for e in mine]
    assert all(e['status'] == 'CONFIRMED' for e in mine), [(e['key'], e['status']) for e in mine]
    for b in cp['cand_log']:
        cl = [conf_class(cf) for _, cf, _ in b['cands']]
        assert b['cands'][0][0] == b['chosen'] and cl == sorted(cl, reverse=True), b
    assert all(cp['flix'][f].startswith('landed') for f in ('flix_a', 'flix_b')), cp['flix']
    # 6. Executor briefed while the Writers were still exploring
    fb = cp['first_briefing']
    assert fb and not all(s.startswith(('DFS complete', 'beacon stock', 'NO CONTACT'))
                          for s in fb['writers'].values()), fb
    assert cp['phase'] == 'mission_complete', cp['phase']
    assert r['poses']['executor']['state'] in ('first_aid', 'awaiting_next'), r['poses']['executor']
    # the end is visible: mission summary (2D banner + terminal) with every miner accounted for
    sm = cp.get('summary')
    assert sm and len(sm['confirmed']) == 3 and len(sm['rescue_team']) == 1 \
        and not sm['next_mission'] and 'stored' in sm, sm
    # short stops at the victims: first aid ~FIRST_AID_S, the next briefing is received during it
    ex = [(row[0], row[6]) for row in r['trace'] if row[1] == 'executor']
    runs = []                    # (state, start, end) of consecutive trace samples (every 2 s)
    for t, st in ex:
        if runs and runs[-1][0] == st:
            runs[-1][2] = t
        else:
            runs.append([st, t, t])
    aid = [e - s0 for st, s0, e in runs if st == 'first_aid']
    assert aid and max(aid) <= C.FIRST_AID_S + 2.5, aid
    direct = sum(1 for a, b in zip(runs, runs[1:]) if a[0] == 'first_aid' and b[0] == 'en_route')
    if scenario == 'nominal':
        assert direct >= 1, [(st, s0, e) for st, s0, e in runs]
    waits = [round(e - s0) for st, s0, e in runs if st == 'awaiting_next']
    # 7. spatial memory + data path: records (with source + event id) stored in the beacons;
    #    fibre to the ONA stations, GPS translation; first beacon of A, B (Writers) and C (crew)
    assert any(b.get('events') for b in r['beacons']['beacons'])
    assert k.get('fiber_up', 0) > 0 and k.get('translate', 0) > 0
    assert {b.get('ona') for b in r['beacons']['beacons'] if b.get('parent') == 0} >= {'A', 'B', 'C'}
    for w in C.WRITERS:          # back on the charger of an ONA station (unless lost)
        p = r['poses'][w]
        assert p['state'] == ('destroyed' if (wd and w == 'writer_b') else 'parked'), (w, p['state'])
        assert p.get('charging') == (p['state'] == 'parked'), w
    # 8. camera story ends with the cutaway; GUI requests in a valid order
    assert [c[1] for c in r['camera']][-1] == 'cutaway', r['camera']
    gui = [tuple(g) for g in r['gui']]
    for i, (srv, req) in enumerate(gui):
        if srv == '/gui/follow/offset':
            assert gui[i - 1][0] == '/gui/follow' and gui[i - 1][1] != 'data: ""', gui[i - 1]
        if srv == '/gui/move_to/pose':
            assert gui[i - 1] == ('/gui/follow', 'data: ""'), gui[i - 1]
    # normal run: < 5 m; stress runs (LM_PERTURB: double slip, lag, terrain yaw, lidar smoke;
    # LM_ROUGH: rock bumps on every wall): < 6 m
    lim = 6.0 if '1' in (os.environ.get('LM_PERTURB'), os.environ.get('LM_ROUGH')) else 5.0
    assert max(r['max_err'].values()) < lim, r['max_err']
    # 9. failure cases
    assert ('NO CONTACT' in cp['writers']['writer_b']['status']) == wd, cp['writers']['writer_b']
    if wd:                       # silent Writer: missed heartbeats -> lost -> claims released
        assert k.get('writer_lost', 0) == 1 and k.get('release', 0) >= 1
    assert (k.get('beacon_dead', 0) == 1) == kn
    if kn:
        assert 'rockfall' in [c[1] for c in r['camera']], r['camera']      # camera cut to it
        assert k.get('relinked', 0) >= 1
        assert any(not b.get('alive', True) for b in cp['beacons']), 'BEACON_LOST not received'
        assert k.get('flush', 0) >= 1, 'no store-and-forward happened'
        heal = [f for f in flows if f['kind'] == 'heal_plan']
        if heal and heal[0]['move'] > 0:
            orphan = f"data: \"beacon_{heal[0]['at']}\""
            assert ('/gui/follow', orphan) in gui, 'camera did not follow the relocating beacon'
    gas = [e for e in cp['events'] if e['type'] == 'HAZARD']
    assert any(e['status'] == 'FALSE POSITIVE' for e in gas) == fp
    assert any(e['status'] in ('REPORTED', 'CONFIRMED') for e in gas)
    stock = {w: C.BEACON_STOCK - sum(1 for f in drops if f['by'] == w) for w in C.WRITERS}
    return (f"t_end={r['t_end']:.0f}s beacons={len(r['beacons']['beacons'])} stock_left={stock} "
            f"csi={len(csi)} presence={k.get('presence', 0)} aid_then_go={direct} waits={waits} "
            f"conf={ {e['key']: e['conf'] for e in vic} } "
            f"max_err={ {n: round(v, 2) for n, v in r['max_err'].items()} }")


if __name__ == '__main__':
    bad = 0
    which = sys.argv[1:] or ['nominal', 'writer_destroyed', 'beacon_knockout', 'false_positive', 'all']
    for s in which:
        try:
            print(f'PASS {s:17s} {check(s)}', flush=True)
        except AssertionError as e:
            bad += 1
            print(f'FAIL {s:17s} {e}', flush=True)
    sys.exit(bad)

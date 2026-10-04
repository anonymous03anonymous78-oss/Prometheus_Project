"""Single source of truth for The Living Map simulation (v3.2 - aligned with the technical report).

World frame = Gazebo world frame = SITE frame (metres, x east, y north, z up).
x < 0 is the OUTSIDE zone (GPS available). The hillside with the mine portals is the
plane x = 0. The site origin (0, 0) is surveyed with GPS by the Outside Network Areas;
every robot takes a GPS + compass fix at the staging area and then runs GPS-denied on
its own EKF inside the mine.

The mine is assembled from real DARPA SubT tunnel tiles (20 m x 20 m) from Gazebo Fuel.
Only straight tiles, 4-way intersections and ramps are used.

NOTHING in this file about the mine layout is given to the robots: the Writers explore it
with a depth-first search on their own lidar, and Flix discovers the portals itself. The
layout below is used only to build the world, to model sensors/radio physics and for the
"error vs truth" display.
"""
import math

WORLD_NAME = 'living_map'

# ------------------------------------------------------------------ geo anchor
GEO_LAT0 = 34.3264          # demo point in the Gafsa mining basin - change freely
GEO_LON0 = 8.4011
GEO_ALT0 = 320.0            # m above sea level at the site origin (portal B sill)
GEO_AZ_DEG = 70.0           # compass azimuth of world +x
MISSION_EPOCH_UTC = 1791187200   # 2026-10-05 08:00:00 UTC: mission clock t = 0 (beacon "when")

# ------------------------------------------------------------------ SubT tiles
FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
PI2 = math.pi / 2
# (name, fuel model, cx, cy, cz, yaw)   cz = floor height of the tile's low end
TILES = [
    # line B
    ('b1', 'Tunnel Tile 5', 10, 0, 0, -PI2),
    ('b2', 'Tunnel Tile 5', 30, 0, 0, -PI2),
    ('x1', 'Tunnel Tile 1', 50, 0, 0, 0.0),
    ('g1', 'Tunnel Tile 5', 50, 20, 0, 0.0),       # gas side-branch (north of X1)
    ('x2', 'Tunnel Tile 1', 70, 0, 0, 0.0),
    ('s1', 'Tunnel Tile 5', 70, -20, 0, 0.0),      # south branch: miner 1
    ('n1', 'Tunnel Tile 5', 70, 20, 0, 0.0),       # link to line A
    ('m1', 'Tunnel Tile 5', 90, 0, 0, -PI2),
    ('x3', 'Tunnel Tile 1', 110, 0, 0, 0.0),
    ('e1', 'Tunnel Tile 5', 130, 0, 0, -PI2),      # east gallery: miner 3
    # line A
    ('a1', 'Tunnel Tile 5', 10, 40, 0, -PI2),
    ('a2', 'Tunnel Tile 5', 30, 40, 0, -PI2),
    ('a3', 'Tunnel Tile 5', 50, 40, 0, -PI2),
    ('x4', 'Tunnel Tile 1', 70, 40, 0, 0.0),
    # line C (flooded sump -> air pocket with miner 4)
    ('c1', 'Tunnel Tile 6', 10, -60, -5, PI2),     # ramp DOWN from the portal (0 -> -5)
    ('c2', 'Tunnel Tile 5', 30, -60, -5, -PI2),    # flooded flat section
    ('c3', 'Tunnel Tile 6', 50, -60, -5, -PI2),    # ramp UP (-5 -> 0)
    ('c4', 'Tunnel Tile 5', 70, -60, 0, -PI2),     # dry air pocket
]
BLOCKERS = [(50, -11, 0), (50, 31, 0), (70, -31, 0), (110, 11, 0), (110, -11, 0),
            (141, 0, 0), (70, 51, 0), (81, 40, 0), (81, -60, 0)]
HALF_W = 2.0                 # corridor half-width used by the radio/LOS model (conservative)

# ------------------------------------------------------------------ portals (ground truth)
# Only the world builder and the sensor models know these. Flix finds them on its own.
ENTRANCES = {
    'A': {'x': 0.0, 'y': 40.0, 'state': 'clear'},
    'B': {'x': 0.0, 'y': 0.0, 'state': 'clear'},
    'C': {'x': 0.0, 'y': -60.0, 'state': 'flooded'},
    'D': {'x': 0.0, 'y': -30.0, 'state': 'collapsed'},     # old adit, roof fall 2.5 m inside
}
ADIT_D = (0.0, 2.5)          # collapsed adit D: open from x = 0 to the rubble face at x = 2.5


def _tile_rects():
    out = []
    for name, model, cx, cy, cz, yaw in TILES:
        along_x = abs(math.sin(yaw)) > 0.5
        if model == 'Tunnel Tile 1':
            out.append((cx - 10, cx + 10, cy - HALF_W, cy + HALF_W))
            out.append((cx - HALF_W, cx + HALF_W, cy - 10, cy + 10))
        elif along_x:
            out.append((cx - 10, cx + 10, cy - HALF_W, cy + HALF_W))
        else:
            out.append((cx - HALF_W, cx + HALF_W, cy - 10, cy + 10))
    d = ENTRANCES['D']
    out.append((ADIT_D[0], ADIT_D[1], d['y'] - 1.7, d['y'] + 1.7))
    return out


TUNNEL_RECTS = _tile_rects()
OUTSIDE_RECT = (-70.0, 0.0, -90.0, 70.0)
HILL_Y = (-76.0, 62.0)       # extent of the hillside face (Flix sweeps it)

# miners' last known position (last cap-lamp tag ping relayed by the mine's tag system)
LAST_KNOWN = (71.8, -20.0)

# C line vertical profile (floor height as a function of x, on y = -60)
WATER_Z = -2.5               # water surface of the flooded sump (leaves room to fly over it)
WATER_X = (-4.0 * WATER_Z, 40.0 + 4.0 * (WATER_Z + 5.0))     # flooded stretch of line C (x)


def c_floor_z(x):
    if x <= 0:
        return 0.0
    if x <= 20:
        return -x / 4.0
    if x <= 40:
        return -5.0
    if x <= 60:
        return -5.0 + (x - 40) / 4.0
    return 0.0


# ------------------------------------------------------------------ robots
# Ground writers (Robotika X2) and fast writers (Flix) work in pairs, one pair per clear
# portal. Names are also the Gazebo model names and the topic prefixes.
WRITERS = ('writer_a', 'writer_b')
FLIXES = ('flix_a', 'flix_b')
PAIR = {'writer_a': 'flix_a', 'writer_b': 'flix_b', 'flix_a': 'writer_a', 'flix_b': 'writer_b'}
GROUND = WRITERS + ('executor',)
LABEL = {'writer_a': 'Writer A', 'writer_b': 'Writer B', 'executor': 'Executor',
         'flix_a': 'Flix A', 'flix_b': 'Flix B'}

SPAWN = {
    'writer_a': (-16.5, 2.0, 0.07, 0.0),
    'writer_b': (-14.0, 2.0, 0.07, 0.0),
    'executor': (-14.0, -2.5, 0.07, 0.0),
    'flix_a': (-10.0, 14.0, 0.02, 0.0),
    'flix_b': (-10.0, -8.0, 0.02, 0.0),
}
WRITER_SPAWN = SPAWN['writer_b']           # (kept for older tools)
EXECUTOR_SPAWN = SPAWN['executor']
FLIX_SPAWN = SPAWN['flix_b']

# Beacon id ranges (uint8): each Writer numbers its own beacons. The first beacon of an entrance
# no Writer enters (flooded portal C) is placed by its Outside Network Area station crew.
BEACON_IDS = {'writer_a': (1, 49), 'writer_b': (51, 99)}
ONA_BEACON_ID = 100          # ONA-C's own fibered first beacon at the flooded portal C
BEACON_STOCK = 10            # ESP32 relay beacons carried by each Writer

# robot subject ids in packets
ID_ROBOT = {'writer_a': 0xFE, 'writer_b': 0xF9, 'executor': 0xFD, 'flix_a': 0xFC,
            'flix_b': 0xF8}
ROBOT_OF_ID = {v: k for k, v in ID_ROBOT.items()}
ID_WRITER = ID_ROBOT['writer_b']
ID_EXECUTOR = ID_ROBOT['executor']
ID_FLIX = ID_ROBOT['flix_b']
PARENT_ONA = 0

# source / Writer identifier carried in every record (flags byte, high nibble)
SRC_ID = {'beacon': 0, 'writer_a': 1, 'writer_b': 2, 'flix_a': 3, 'flix_b': 4, 'executor': 5,
          'ona': 6}
SRC_OF_ID = {v: k for k, v in SRC_ID.items()}
SRC_NAME = {0: 'beacon network', 1: 'Writer A', 2: 'Writer B', 3: 'Flix A', 4: 'Flix B',
            5: 'Executor', 6: 'ONA station'}

WRITER_SPEED = 1.0
EXECUTOR_SPEED = 0.9
EXEC_CLOSE_R = 4.0                   # final approach: full speed until this close to the person,
EXEC_CLOSE_SPEED = 0.45              # then slow down to stop at the stand-off distance
FLIX_SPEED_OUT = 3.0                 # outside
FLIX_SPEED_IN = 1.5                  # inside the mine
FLIX_ALT = 1.6
FLIX_ALT_WATER = 0.7                 # over the flooded sump: height above the water surface
MISSION_START_DELAY = 8.0
DROP_BEHIND = 0.45

# ------------------------------------------------------------------ Writer DFS (lidar topology)
DFS_OPEN = 4.0               # a side gallery is "open" when the lidar sees >= 4 m free into it
DFS_SEEN_CLOSED = 10.4       # ... and "seen to its end" when its end wall is closer than this
DFS_LIDAR_RANGE = 12.0
DFS_DEAD_END = 1.8           # dead end: rock this close ahead and no side opening
DFS_KNOWN_NODE_R = 4.0       # an estimate within 4 m of a known junction = arrival there
DFS_NODE_SPACING = 6.0       # no new junction detection within 6 m of the last one
GUARD_DEFER = 6.0            # a due spacing beacon waits up to 6 m for a junction (one beacon less)
HEARTBEAT_S = 15.0           # Writer status packet period
CV_DWELL = 1.0               # s the Writer pauses on a person: several camera frames -> confidence
# where a Writer parks after leaving: the charging pad of its Outside Network Area station
EXIT_PARK = {'A': (-6.5, 46.5), 'B': (-5.5, 6.0)}

# Frontier utility (Writer exploration): U = w_gain*gain + w_rel*relevance - w_dist*distance
#   - w_energy*energy + w_acc*accessibility, every term normalised to about 0..1
UTILITY_W = {'gain': 1.0, 'relevance': 1.5, 'distance': 0.8, 'energy': 0.6, 'access': 0.5}

# Robot batteries (ground robots: runtime on one charge; recharged at their ONA station)
GROUND_ENDURANCE = 3600.0    # s of driving
GROUND_IDLE_DRAIN = 0.25     # idle draw as a fraction of the driving draw
CHARGE_FULL_S = 1200.0       # s for a full charge on an ONA station charger
ENERGY_PER_M = 1.0 / (WRITER_SPEED * GROUND_ENDURANCE)     # battery fraction per metre driven

# ------------------------------------------------------------------ Flix (fast writer)
# Portal discovery: sweep along the hillside face, side ToF looking at the rock face.
FLIX_SWEEP_X = -3.0
FLIX_SWEEP = {'flix_a': (4.5, HILL_Y[1] - 4.0), 'flix_b': (2.5, HILL_Y[0] + 4.0)}
FLIX_SWEEP_SPEED = 2.0
FLIX_OPENING_MIN_W = 1.5     # metres of "no return" along the face = an opening
FLIX_INSPECT_DEPTH = 6.0
FLIX_INSPECT_S = 2.5
FLIX_RF_MARGIN = 5.0                 # stays within RF_RANGE - margin of a routed beacon
FLIX_THERMAL_R = 25.0                # thermal camera human detection range (LOS)
FLIX_THERMAL_FOV = math.radians(70)  # half field of view
FLIX_LEAD = 14.0                     # flies up to this far ahead of its Writer
FLIX_TOF_DOWN_Z = 0.02               # downward ToF mounted 2 cm below the body origin
FLIX_INSPECT_END = 72.0              # flooded line C: fly in up to here (air pocket beyond the sump)
ROOF_CLEAR = 4.85                    # gallery roof underside above the floor (line C profile)

# ------------------------------------------------------------------ hazards and victims
GAS_LEAK = {'x': 50.0, 'y': 19.0, 'peak': 2.5, 'sigma': 2.0}     # % CH4
GAS_ALARM = 1.0
FP_GLITCH = {'x': 70.0, 'y': -7.5, 'r': 2.0, 'value': 1.3}        # false_positive scenario

# Miners (ground truth). 'motion' only drives the Wi-Fi CSI channel model: a person who moves
# disturbs the channel most, a still (unconscious) person very little.
VICTIMS = [
    {'id': 'V1', 'x': 71.8, 'y': -20.0, 'z': 0.004, 'yaw': math.pi, 'where': 'south branch',
     'motion': 'moving'},
    {'id': 'V2', 'x': 50.8, 'y': 23.5, 'z': 0.004, 'yaw': -PI2, 'where': 'gas branch',
     'motion': 'breathing'},
    {'id': 'V3', 'x': 133.5, 'y': 0.9, 'z': 0.004, 'yaw': math.pi, 'where': 'east gallery',
     'motion': 'still'},
    {'id': 'V4', 'x': 64.0, 'y': -58.2, 'z': 0.004, 'yaw': math.pi, 'where': 'air pocket, line C',
     'motion': 'moving'},
]
VICTIM = VICTIMS[0]
VICTIM2 = VICTIMS[3]
VICTIM_DETECT_R = 4.0                # RealSense person detection (ground robots)
VICTIM_STANDOFF = 1.9

# Computer vision: person-detector confidence vs range (frames fused over the CV dwell)
CV_GROUND = (0.96, 0.04)             # RGB-D camera: conf = a - b * range (m)
CV_FLIX = (0.92, 0.012)              # Flix thermal + RGB camera
CONF_MIN_EXEC = 0.5                  # a victim hypothesis below this is not worth an Executor trip
EXEC_SIGHT_R = 25.0                  # Executor can go the last metres from a beacon in line of sight

# ------------------------------------------------------------------ Wi-Fi CSI presence sensing
# Dedicated ESP32-S3 CSI nodes dropped by the slow Writers (independent of the ESP-NOW chain):
# Wi-Fi sources = every relay beacon + the Writers; receivers = CSI nodes (and the Flix ESP32-S3
# while it hovers). One beacon per cluster aggregates the CSI and runs the AI classifier.
CSI_STOCK = 8                        # CSI nodes carried by each slow Writer
CSI_IDS = {'writer_a': (110, 129), 'writer_b': (130, 149)}
CSI_LINK_R = 20.0                    # usable Wi-Fi CSI link length in a gallery (line of sight)
CSI_ZONE = 2.5                       # m: a person this close to a link changes its channel (tunnel multipath)
CSI_MOTION = {'moving': 1.0, 'breathing': 0.7, 'still': 0.3}
CSI_NOISE = 0.06                     # per-link score noise (empty channel)
CSI_WINDOW = 2.0                     # s per classification window
CSI_THRESHOLD = 0.6                  # presence probability needed ...
CSI_CONFIRM = 2                      # ... in this many consecutive windows
CSI_REGION_R = 6.0                   # m: smallest region reported (Phase 1: region, not position)
CSI_DENSE_R = 22.0                   # near the last known position: denser CSI nodes ...
CSI_DENSE_SPACING = 8.0              # ... one every 8 m of gallery
CSI_AGG_R = 20.0                     # a node streams its CSI to a beacon within this range

WRITER_COLLAPSE_Y = -21.5            # writer_destroyed: roof collapse in the south branch
COLLAPSE_VICTIM = 'writer_b'
KNOCKOUT_MIN_X = -6.0                # beacon_knockout: once the Executor is at the portal
FIRST_AID_S = 5.0                    # Executor at a victim: SCSR oxygen unit + radio delivered
                                     # (the next briefing is received during it: leaves right after)

# ------------------------------------------------------------------ radio
RF_P1M = -45.0
RF_N = 2.0
RF_RANGE = 30.0                      # ESP-NOW effective range in tunnel (30-50 m indoor)
RF_NOISE_DB = 0.3
RF_NLOS_RANGE = 8.0                  # short links survive without line of sight
GUARD_SPACING = 18.0                 # 0.6 x range: any single loss bridgeable by one relocation
GUARD_SPACING_RISK = 11.0            # learned rockfall zones: denser chain (lesson from memory)
WEAK_DROP_MIN = 5.0
WEAK_DROP_MAX = 2
WRITER_SILENCE = 3 * HEARTBEAT_S    # CP: 3 missed heartbeats (no packet at all) -> presumed lost
WRITER_SILENCE_RISK = 2 * HEARTBEAT_S   # ... in a learned roof-collapse zone
HEAL_MARGIN = 1.0
HEARTBEAT_TIMEOUT = 2.0
RELOCATE_SPEED = 0.6
HOP_DELAY = 0.30                     # slowed for visibility (real ESP-NOW ~3 ms)
FIBER_DELAY = 0.10
SAT_DELAY = 0.60
EVENT_REFRESH = 10.0
TRAIL_TAU = 300.0
STALE_FRESHNESS = 0.2

EXEC_ARRIVE_R = 0.8
EXEC_ANTENNA_OFFSET = 0.3
EXEC_VERIFY_R = 1.5

# ------------------------------------------------------------------ outside network areas
# One ONA station per portal: communication computer, directional (parabolic) antenna to the
# relay satellite, solar panels + high-capacity battery (local energy: it also recharges the
# robots between missions), optical-fibre cable to the first beacon of its portal. All stations
# and the Command Post are linked through the relay satellite to the mission server.
ONAS = {
    'A': {'pos': (-9.0, 50.0), 'portal': (0.0, 40.0)},
    'B': {'pos': (-8.0, 9.0), 'portal': (0.0, 0.0)},
    'C': {'pos': (-9.0, -50.0), 'portal': (0.0, -60.0)},
}
ONA_POS = ONAS['B']['pos']
ONA_ENERGY = {'battery_kwh': 15.0, 'soc0': 0.8, 'solar_kwp': 2.4, 'base_w': 180.0,
              'charger_w': {'ground': 400.0, 'flix': 15.0}}
ONA_C_BEACON = (0.8, -60.0)          # ONA-C's own fibered first beacon at the flooded portal
CP_POS = (-45.0, -32.0)
SERVER_POS = (-49.0, -27.0)          # mission server rack next to the Command Post
SAT_POS = (-75.0, -95.0, 100.0)      # relay satellite (symbolic distance; geostationary, ~45 deg
                                     # elevation toward the south as seen from Tunisia)
ENTRANCE_ONA = {'A': 'A', 'B': 'B', 'C': 'C'}

# ------------------------------------------------------------------ mission memory (server AI)
MEMORY_ZONE = 10.0                   # lessons are kept per 10 m x 10 m zone
MEMORY_RISK_R = 15.0                 # a learned risk applies within this radius
GAS_FP_PRIOR = (1.0, 9.0)            # Beta prior of the CH4 sensor false-positive rate
GAS_FP_CONFIRM = 0.15                # above this posterior mean: Writers re-sample before reporting

# ------------------------------------------------------------------ sensors (noise models)
SENS = {
    'enc_slip_v': 0.03,        # dry-run only: skid-steer longitudinal slip (fraction)
    'enc_slip_w': 0.15,        # dry-run only: skid-steer turning slip (fraction)
    'gyro_sigma': 0.009,       # rad/s white noise (SubT X1 IMU spec)
    'gyro_bias': 0.004,        # rad/s constant bias (dry run draws +/-)
    'vo_sigma_v': 0.03,        # visual odometry: m/s noise
    'vo_scale': 0.01,          # visual odometry scale error (stereo VIO, metric)
    'vo_sigma_w': 0.02,
    'flow_sigma': 0.05,        # optical flow velocity noise (m/s at 1 m)
    'tof_sigma': 0.01,         # ToF laser noise (m)
    'cv_sigma': 0.02,          # person-detector confidence noise
}

# ------------------------------------------------------------------ packet event types / flags
# The two event types of the report are HAZARD (hazardous area: gas) and the victim encounter,
# seen by computer vision (VICTIM) or by Wi-Fi CSI (PRESENCE); the others build the map.
EV_PLACEMENT, EV_VICTIM, EV_HAZARD, EV_ENTRANCE = 0, 1, 2, 3
EV_PRESENCE = 4              # Wi-Fi CSI Level-1 human presence in a region (AI classifier)
EV_EXPLORATION_COMPLETE = 5
EV_VICTIM_CONFIRMED, EV_FALSE_POSITIVE = 6, 7
EV_ROUTE_HEALED, EV_BEACON_LOST = 8, 9
EV_JUNCTION = 10             # junction beacon = mapping event (DFS node + its exit table)
EV_WRITER_EXIT = 11          # Writer left the mine (stock empty / DFS complete)
EV_STATUS = 12               # Writer heartbeat: position, beacons left, explored, battery
EV_CSI_NODE = 13             # an ESP32-S3 CSI node joined the sensing infrastructure
EV_NAMES = {0: 'PLACEMENT', 1: 'VICTIM', 2: 'HAZARD', 3: 'ENTRANCE', 4: 'PRESENCE',
            5: 'EXPLORATION_COMPLETE', 6: 'VICTIM_CONFIRMED', 7: 'FALSE_POSITIVE',
            8: 'ROUTE_HEALED', 9: 'BEACON_LOST', 10: 'JUNCTION', 11: 'WRITER_EXIT',
            12: 'STATUS', 13: 'CSI_NODE'}
# flags byte: low nibble = record flags, high nibble = source / Writer identifier (SRC_ID)
FL_STORED_FWD = 0x01         # held by a beacon and forwarded after the chain healed
FL_RELOCATED = 0x02          # the beacon moved itself (self-healing)
FL_NO_GROUND = 0x04          # event beyond water / rubble: not reachable by a ground robot
FL_FUSED = 0x08              # camera + CSI evidence combined in this record
CAM_RGBD, CAM_THERMAL = 1, 2     # VICTIM valueB: camera that saw the person
CAM_NAME = {1: 'RGB-D camera', 2: 'thermal camera'}

# ------------------------------------------------------------------ scenarios
SCENARIOS = {
    'nominal':          dict(writer_destroyed=False, beacon_knockout=False, false_positive=False),
    'writer_destroyed': dict(writer_destroyed=True,  beacon_knockout=False, false_positive=False),
    'beacon_knockout':  dict(writer_destroyed=False, beacon_knockout=True,  false_positive=False),
    'false_positive':   dict(writer_destroyed=False, beacon_knockout=False, false_positive=True),
    'all':              dict(writer_destroyed=True,  beacon_knockout=True,  false_positive=True),
}
SCENARIO_TEXT = {
    'nominal': 'Two Flix sweep the hillside and find 4 openings (A, B clear; C flooded; D '
               'collapsed). Flix B flies over the water of C and finds the miner in the air '
               'pocket. Writer A and Writer B explore the unknown mine (frontiers + modified DFS '
               'with a utility function), share junctions through their beacons and drop Wi-Fi CSI '
               'nodes; camera and CSI evidence are fused into victim hypotheses; the Executor is '
               'briefed as soon as one is known and visits them, most confident first.',
    'writer_destroyed': 'Roof collapse hits Writer B in the south branch: it falls silent. After 3 '
                        'missed heartbeats the server presumes it lost and releases its claimed '
                        'branches; its finds stay in the beacons and Writer A takes over.',
    'beacon_knockout': 'Rockfall destroys a relay beacon while the Executor enters; the orphaned '
                       'neighbour re-links (or drives itself back) and flushes held messages.',
    'false_positive': 'A gas-sensor glitch on the Executor route creates a spurious gas event; the '
                      'Executor re-measures on the way -> FALSE POSITIVE (the server learns it).',
    'all': 'All failure cases in one run: false positive + beacon knockout + Writer destroyed.',
}

WEB_PORT = 8765


def scenario_flags(name):
    return SCENARIOS.get(name, SCENARIOS['nominal'])


def as_dict():
    return {
        'world': WORLD_NAME, 'tunnels': TUNNEL_RECTS, 'outside': OUTSIDE_RECT,
        'tiles': [(t[0], t[1], t[2], t[3], t[4], t[5]) for t in TILES], 'blockers': BLOCKERS,
        'entrances': ENTRANCES, 'last_known': LAST_KNOWN, 'spawn': SPAWN,
        'writer_spawn': SPAWN['writer_b'], 'executor_spawn': SPAWN['executor'],
        'flix_spawn': SPAWN['flix_b'], 'ona_c_beacon': ONA_C_BEACON,
        'onas': ONAS, 'ona': ONA_POS, 'cp': CP_POS, 'server': SERVER_POS, 'sat': SAT_POS,
        'water_z': WATER_Z, 'water_x': WATER_X, 'roof_clear': ROOF_CLEAR, 'gas': GAS_LEAK, 'gas_alarm': GAS_ALARM,
        'fp': FP_GLITCH, 'victims': VICTIMS, 'rf_range': RF_RANGE, 'guard': GUARD_SPACING,
        'hop_delay': HOP_DELAY, 'sat_delay': SAT_DELAY,
        'tau': TRAIL_TAU, 'ev_names': EV_NAMES, 'labels': LABEL, 'writers': WRITERS,
        'flixes': FLIXES, 'stock': BEACON_STOCK, 'csi_stock': CSI_STOCK,
        'csi_link_r': CSI_LINK_R, 'csi_region_r': CSI_REGION_R, 'src_name': SRC_NAME,
        'utility_w': UTILITY_W, 'cam_name': CAM_NAME,
        'geo': {'lat0': GEO_LAT0, 'lon0': GEO_LON0, 'alt0': GEO_ALT0, 'az': GEO_AZ_DEG,
                'epoch': MISSION_EPOCH_UTC},
        'scenario_text': SCENARIO_TEXT,
    }

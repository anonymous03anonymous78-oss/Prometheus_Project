"""Beacon packet (32 bytes), briefing packet and frame translation.

Beacon packet layout, little-endian, packed (identical to the ESP32 C struct
`struct __attribute__((packed)) BeaconMsg`, see README). Every record carries what the
technical report asks a beacon to store: event identifier, event type, position, timestamp,
detection confidence and source / Writer identifier.

    offset size field       meaning
    ---------------------- WHO / WHAT ----------------------------------------------
    0      1    beaconID    subject (beacon id, CSI node id, or a robot id 0xF8..0xFE)
    1      1    eventType   PLACEMENT / VICTIM / HAZARD / PRESENCE / JUNCTION ... (config.EV_*)
    2      1    parentID    radio uplink neighbour at send time (0 = fiber to an ONA)
    3      1    flags       bits 0-3: stored-fwd, relocated, not-reachable-by-ground, fused
                            bits 4-7: SOURCE / Writer identifier (config.SRC_ID)
    ---------------------- WHERE IT IS ---------------------------------------------
    4      4    localX      float32, m, site frame (Writer SLAM frame tied to the entrance beacon)
    8      4    localY      float32, m
    12     2    localZ      int16, cm (altitude for the GPS translation)
    14     2    heading     int16, centi-degrees, of the robot that wrote it
    ---------------------- WHAT WAS FOUND -------------------------------------------
    16     1    valueA      HAZARD: CH4 in 0.1 % | VICTIM: range to the person, dm |
                            PRESENCE: region radius, m | STATUS: beacons left
    17     1    valueB      HAZARD: O2 in 0.1 % - 150 | VICTIM: camera (1 RGB-D, 2 thermal) |
                            PRESENCE: CSI links that saw the person | STATUS: explored / 2 m
    18     1    conf        DETECTION CONFIDENCE 0..100 % (camera detector / CSI classifier;
                            STATUS: battery %)
    ---------------------- WHERE TO GO ---------------------------------------------
    19     1    nextIn      next beacon TOWARD the event this beacon points to (0 = here)
    20     2    bearingIn   int16 centi-degrees from this beacon to nextIn / to the event
    22     2    distIn      uint16 dm, distance along the trail to the event
    24     1    nextOut     next beacon TOWARD THE EXIT (escape route; 0 = portal)
    ---------------------- WHEN / WHICH -------------------------------------------
    25     4    utc         uint32, UTC seconds (absolute time it was written)
    29     2    utcMs       uint16, milliseconds
    31     1    eventID     EVENT IDENTIFIER, numbered by its source: 1..127 the robot's own
                            records, 128..255 the records of its CSI network (aggregator beacons).
                            (source, eventID) identifies an event; re-broadcasts keep it.
                            ---- 32 bytes (ESP-NOW limit 250)

Message aging: receivers compute age = now_utc - (utc + utcMs/1000).
"""
import math
import struct

from . import config as C

BEACON_FMT = '<BBBBffhhBBBBhHBIHB'
BEACON_SIZE = struct.calcsize(BEACON_FMT)
assert BEACON_SIZE == 32, BEACON_SIZE

BRIEF_HDR = '<BBBBBfff'      # msgType, targetID, nPath, nVerify, conf %, offX, offY, dYaw
BRIEF_PATH = '<Bff'          # id, x, y
BRIEF_VICTIM = '<ffB'        # victim hypothesis x, y, search radius (m)
BRIEF_VERIFY = '<BBff'       # beacon id, event type, x, y
MSG_BRIEF, MSG_BRIEF_UPDATE = 0xB1, 0xB2

FIELDS = [('beaconID', 0, 1), ('eventType', 1, 2), ('parentID', 2, 3), ('flags', 3, 4),
          ('localX', 4, 8), ('localY', 8, 12), ('localZ', 12, 14), ('heading', 14, 16),
          ('valueA', 16, 17), ('valueB', 17, 18), ('conf', 18, 19), ('nextIn', 19, 20),
          ('bearingIn', 20, 22), ('distIn', 22, 24), ('nextOut', 24, 25), ('utc', 25, 29),
          ('utcMs', 29, 31), ('eventID', 31, 32)]
GROUPS = {'who/what': ('beaconID', 'eventType', 'parentID', 'flags'),
          'where it is': ('localX', 'localY', 'localZ', 'heading'),
          'what was found': ('valueA', 'valueB', 'conf'),
          'where to go': ('nextIn', 'bearingIn', 'distIn', 'nextOut'),
          'when / which': ('utc', 'utcMs', 'eventID')}


def _i16(v, lo=-32768, hi=32767):
    return max(lo, min(hi, int(round(v))))


def _u8(v):
    return max(0, min(255, int(round(v))))


def mission_to_utc(t_ms):
    """Mission clock (ms since t = 0) -> (utc seconds, ms)."""
    t_ms = int(t_ms)
    return C.MISSION_EPOCH_UTC + t_ms // 1000, t_ms % 1000


def utc_to_mission_ms(utc, ms):
    return (int(utc) - C.MISSION_EPOCH_UTC) * 1000 + int(ms)


def src_of(name):
    """Source identifier of a robot name ('writer_a' ...), 0 = beacon network."""
    return C.SRC_ID.get(name, 0)


def encode(p):
    """p: dict with id, type, x, y and optional parent, flags (low nibble), src (source id),
    z, heading (deg), a, b, conf (0..1), next_in, bearing_in (deg), dist_in (m), next_out,
    t_ms (mission ms) or utc/utc_ms, eid (event identifier)."""
    if 'utc' in p:
        utc, ms = int(p['utc']), int(p.get('utc_ms', 0))
    else:
        utc, ms = mission_to_utc(p.get('t_ms', 0))
    flags = (int(p.get('flags', 0)) & 0x0F) | ((int(p.get('src', 0)) & 0x0F) << 4)
    return struct.pack(BEACON_FMT, int(p['id']) & 0xFF, int(p['type']) & 0xFF,
                       int(p.get('parent', 0)) & 0xFF, flags, float(p['x']), float(p['y']),
                       _i16(100.0 * p.get('z', 0.0)), _i16(100.0 * _wrap_deg(p.get('heading', 0.0))),
                       _u8(p.get('a', 0)), _u8(p.get('b', 0)), _u8(100.0 * p.get('conf', 0.0)),
                       int(p.get('next_in', 0)) & 0xFF,
                       _i16(100.0 * _wrap_deg(p.get('bearing_in', 0.0))),
                       max(0, min(65535, int(round(10.0 * p.get('dist_in', 0.0))))),
                       int(p.get('next_out', 0)) & 0xFF, utc, ms, int(p.get('eid', 0)) & 0xFF)


def _wrap_deg(a):
    a = float(a)
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def decode(b):
    (i, t, par, fl, x, y, z, h, va, vb, cf, nin, bin_, din, nout, utc, ms,
     eid) = struct.unpack(BEACON_FMT, b)
    return {'id': i, 'type': t, 'parent': par, 'flags': fl & 0x0F, 'src': fl >> 4,
            'source': C.SRC_NAME.get(fl >> 4, '?'), 'x': x, 'y': y, 'z': z / 100.0,
            'heading': h / 100.0, 'a': va, 'b': vb, 'conf': cf / 100.0, 'next_in': nin,
            'bearing_in': bin_ / 100.0, 'dist_in': din / 10.0, 'next_out': nout, 'utc': utc,
            'utc_ms': ms, 'eid': eid, 't_ms': utc_to_mission_ms(utc, ms)}


def fields_hex(b):
    """Split the 32 bytes into labelled groups for the packet inspector."""
    h = b.hex()
    return [(n, h[2 * a:2 * z]) for n, a, z in FIELDS]


def value_text(p):
    """Human-readable 'what was found' of a decoded packet."""
    t = p['type']
    cf = f"{round(100 * p.get('conf', 0.0))} %"
    if t == C.EV_HAZARD:
        return f"CH4 {p['a'] / 10.0:.1f} %, O2 {(p['b'] + 150) / 10.0:.1f} %"
    if t == C.EV_VICTIM:
        return (f"person seen by the {C.CAM_NAME.get(p['b'], 'camera')} at {p['a'] / 10.0:.1f} m, "
                f"confidence {cf}")
    if t == C.EV_VICTIM_CONFIRMED:
        return f"person identified by the Executor camera at {p['a'] / 10.0:.1f} m, confidence {cf}"
    if t == C.EV_PRESENCE:
        return (f"Wi-Fi CSI: human presence within {p['a']} m ({p['b']} links), "
                f"confidence {cf}")
    if t == C.EV_CSI_NODE:
        return f"CSI node N{p['a']} streams to this beacon ({p['b']} node(s) in the cluster)"
    if t == C.EV_STATUS:
        return f"{p['a']} beacons left, ~{2 * p['b']} m explored, battery {cf}"
    if t == C.EV_FALSE_POSITIVE:
        return 'not confirmed on site'
    return ''


def encode_briefing(msg_type, target, path, victim, verify, off, dyaw, conf=0.0, radius=0.0):
    out = struct.pack(BRIEF_HDR, msg_type, target, len(path), len(verify), _u8(100.0 * conf),
                      off[0], off[1], dyaw)
    for bid, x, y in path:
        out += struct.pack(BRIEF_PATH, bid, x, y)
    out += struct.pack(BRIEF_VICTIM, victim[0], victim[1], _u8(radius))
    for bid, et, x, y in verify:
        out += struct.pack(BRIEF_VERIFY, bid, et, x, y)
    return out


def decode_briefing(b):
    hs = struct.calcsize(BRIEF_HDR)
    mt, tgt, n, nv, cf, ox, oy, dy = struct.unpack(BRIEF_HDR, b[:hs])
    o = hs
    path = []
    ps = struct.calcsize(BRIEF_PATH)
    for _ in range(n):
        bid, x, y = struct.unpack(BRIEF_PATH, b[o:o + ps])
        path.append((bid, x, y))
        o += ps
    vs = struct.calcsize(BRIEF_VICTIM)
    vx, vy, vr = struct.unpack(BRIEF_VICTIM, b[o:o + vs])
    o += vs
    verify = []
    fs = struct.calcsize(BRIEF_VERIFY)
    for _ in range(nv):
        bid, et, x, y = struct.unpack(BRIEF_VERIFY, b[o:o + fs])
        verify.append((bid, et, x, y))
        o += fs
    return {'msg_type': mt, 'target': tgt, 'path': path, 'victim': (vx, vy), 'radius': vr,
            'verify': verify, 'offset': (ox, oy), 'dyaw': dy, 'conf': cf / 100.0}


# ---------------------------------------------------------------- geo math
R_EARTH = 6378137.0


def local_to_gps(x, y, lat0, lon0, az_deg):
    """Local frame (x forward along azimuth az, y to the left) -> lat/lon.
    Flat-earth tangent plane: error < 1 cm over 100 m."""
    az = math.radians(az_deg)
    east = x * math.sin(az) - y * math.cos(az)
    north = x * math.cos(az) + y * math.sin(az)
    lat = lat0 + math.degrees(north / R_EARTH)
    lon = lon0 + math.degrees(east / (R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


def gps_to_local(lat, lon, lat0, lon0, az_deg):
    az = math.radians(az_deg)
    north = math.radians(lat - lat0) * R_EARTH
    east = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    x = east * math.sin(az) + north * math.cos(az)
    y = -east * math.cos(az) + north * math.sin(az)
    return x, y


def world_to_gps(wx, wy):
    """Simulated GPS receiver (only valid in the OUTSIDE zone)."""
    return local_to_gps(wx, wy, C.GEO_LAT0, C.GEO_LON0, C.GEO_AZ_DEG)

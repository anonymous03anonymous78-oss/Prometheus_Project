#!/usr/bin/env python3
"""Generates the v3.2 Gazebo Fortress world (worlds/mine.sdf) and every local model/asset
from living_map/config.py.  Needs python3 + numpy.

    python3 tools/gen_world.py

World content
  * mine galleries: real DARPA SubT tunnel tiles from Gazebo Fuel (downloaded once by setup.sh)
  * hillside with four openings: portals A, B (clear), C (flooded), D (collapsed adit with a
    roof fall), outside terrain, staging area, Command Post + mission server, two drone pads
  * three Outside Network Area stations (ONA-A/B/C): communication computer (control console,
    osrf/gazebo_models), NASA "Tall Dish" parabolic antenna, hub cabinet, solar panel array +
    high-capacity battery cabinet, robot charger, optical fibre to the first beacon of its portal;
    an LTE mast (osrf/gazebo_models radio_tower); the relay satellite (NASA TDRS model) overhead
  * flooded sump on line C (water, seen by the Flix downward ToF), gas leak (particle cloud),
    four trapped miners (Rescue Randy from Fuel), tunnel lamps
  * robots: Writer A / Writer B = Robotika X2 (+ beacon magazine and Wi-Fi CSI node stack),
            Executor = Explorer X1 (SubT meshes, Apache-2.0), Flix A / Flix B quadcopters
            (okalachev/flix mesh)
    SubT sensors are replaced by the ones this project uses; motion plugins are Fortress
    systems (DiffDrive + WheelSlip for the ground robots, VelocityControl for Flix).

Gazebo Fortress (ign-gazebo 6, sdformat 12) syntax: ignition-gazebo-*-system plugins with
ignition::gazebo::systems::* names.
"""
import math
import os
import random
import struct
import sys
import xml.etree.ElementTree as ET
import zlib

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)
from living_map import config as C  # noqa: E402

MODELS = os.path.join(ROOT, 'models')
REF = os.path.join(ROOT, 'tools', 'ref')
SEED = 11

ROCK = '0.36 0.30 0.25 1'
ROCK_DARK = '0.22 0.19 0.16 1'
TIMBER = '0.45 0.30 0.15 1'
STEEL = '0.55 0.56 0.58 1'


# ================================================================== small helpers
def mat(rgba, emissive=None, spec='0.1 0.1 0.1 1'):
    e = f'<emissive>{emissive}</emissive>' if emissive else ''
    return (f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>'
            f'<specular>{spec}</specular>{e}</material>')


def box_visual(name, pose, size, rgba, emissive=None, flags=None):
    fl = f'<visibility_flags>{flags}</visibility_flags>' if flags is not None else ''
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size></box>'
            f'</geometry>{fl}{mat(rgba, emissive)}<cast_shadows>false</cast_shadows></visual>')


def cyl_visual(name, pose, r, length, rgba, emissive=None):
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><cylinder><radius>{r}</radius>'
            f'<length>{length}</length></cylinder></geometry>{mat(rgba, emissive)}'
            f'<cast_shadows>false</cast_shadows></visual>')


def box_link(name, pose, size, rgba, collision=True):
    col = (f'<collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>'
           if collision else '')
    return (f'<link name="{name}"><pose>{pose}</pose>{col}'
            f'{box_visual("v", "0 0 0 0 0 0", size, rgba)}</link>')


def plugin(fname, cls, body=''):
    return (f'<plugin filename="ignition-gazebo-{fname}-system" '
            f'name="ignition::gazebo::systems::{cls}">{body}</plugin>')


def fuel(model):
    return C.FUEL + model


def include(name, uri, pose, static=None):
    s = '' if static is None else f'<static>{"true" if static else "false"}</static>'
    return f'<include><name>{name}</name><uri>{uri}</uri><pose>{pose}</pose>{s}</include>'


def write(path, text, mode='w'):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, mode) as f:
        f.write(text)


def model_config(name, desc):
    return (f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
            f'<sdf version="1.9">model.sdf</sdf><description>{desc}</description></model>\n')


# ================================================================== textures (PNG, no PIL)
def png(path, img):
    """img: HxWx3 or HxWx4 uint8 -> PNG file."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w, ch = img.shape
    raw = b''.join(b'\x00' + img[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = struct.pack('>I', len(data)) + tag + data
        return c + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    color = 6 if ch == 4 else 2
    data = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, color, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))
    write(path, data, 'wb')


def fractal(n, beta, seed):
    """Tileable fractal noise in [0, 1] (spectral synthesis)."""
    rng = np.random.default_rng(seed)
    f = np.fft.fftfreq(n)
    k = np.sqrt(f[:, None] ** 2 + f[None, :] ** 2)
    k[0, 0] = 1.0
    spec = np.fft.fft2(rng.normal(size=(n, n))) / k ** beta
    spec[0, 0] = 0.0
    a = np.real(np.fft.ifft2(spec))
    return (a - a.min()) / (a.max() - a.min())


def rock_texture(path, base, seed, cracks=True):
    n = 512
    a = fractal(n, 1.1, seed)
    b = fractal(n, 1.9, seed + 1)
    v = 0.55 * a + 0.45 * b
    if cracks:
        c = fractal(n, 1.6, seed + 2)
        v = v - 0.35 * np.exp(-((c - 0.5) / 0.012) ** 2)      # thin dark veins
    v = np.clip(v, 0, 1)
    rgb = np.stack([np.clip(base[i] * (0.55 + 0.9 * v), 0, 1) for i in range(3)], axis=2)
    png(path, (rgb * 255).astype(np.uint8))


def gas_texture(path):
    n = 64
    y, x = np.mgrid[0:n, 0:n] / (n - 1) - 0.5
    r = np.sqrt(x * x + y * y) * 2
    alpha = np.clip(1 - r, 0, 1) ** 2 * 85        # thin smoke
    img = np.zeros((n, n, 4))
    img[..., 0], img[..., 1], img[..., 2], img[..., 3] = 255, 255, 255, alpha
    png(path, img.astype(np.uint8))


def gas_colors(path):
    """Colour ramp over the particle lifetime: yellow-green -> transparent grey."""
    n = 64
    t = np.linspace(0, 1, n)
    img = np.zeros((4, n, 4))
    img[..., 0] = 200 - 60 * t
    img[..., 1] = 210 - 50 * t
    img[..., 2] = 80 + 60 * t
    img[..., 3] = 255
    png(path, img.astype(np.uint8))


# ================================================================== OBJ meshes
class Mesh:
    def __init__(self):
        self.v, self.vt, self.f = [], [], []

    def vert(self, p, uv):
        self.v.append(p)
        self.vt.append(uv)
        return len(self.v)

    def tri(self, a, b, c, want):
        """Add triangle with winding chosen so its normal points along `want`."""
        pa, pb, pc = (np.array(self.v[i - 1]) for i in (a, b, c))
        n = np.cross(pb - pa, pc - pa)
        if np.dot(n, want) < 0:
            b, c = c, b
        self.f.append((a, b, c))

    def quad(self, a, b, c, d, want):
        self.tri(a, b, c, want)
        self.tri(a, c, d, want)

    def save(self, path, mtl_name, material):
        # smooth vertex normals
        V = np.array(self.v)
        N = np.zeros_like(V)
        for a, b, c in self.f:
            n = np.cross(V[b - 1] - V[a - 1], V[c - 1] - V[a - 1])
            for i in (a, b, c):
                N[i - 1] += n
        N /= np.maximum(np.linalg.norm(N, axis=1)[:, None], 1e-9)
        out = [f'mtllib {mtl_name}.mtl', f'o {mtl_name}']
        out += [f'v {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}' for p in V]
        out += [f'vt {u:.4f} {w:.4f}' for u, w in self.vt]
        out += [f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}' for n in N]
        out.append(f'usemtl {material}')
        out += [f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}' for a, b, c in self.f]
        write(path, '\n'.join(out) + '\n')


def mtl(path, name, texture, ka='0.6 0.6 0.6'):
    write(path, f'newmtl {name}\nKa {ka}\nKd 1.0 1.0 1.0\nKs 0.05 0.05 0.05\nNs 5\n'
                f'map_Kd {texture}\n')


# ------------------------------------------------------------------ hillside with 3 portals
HILL = {'x1': 152.0, 'y0': -76.0, 'y1': 62.0, 'slope': 32.0, 'h': 17.0}
PORTAL = {'w': 3.4, 'side': 2.6}        # opening width, straight side height (arch on top)


def portal_top(y):
    """Height of the portal opening at lateral position y (None if no portal there)."""
    hw = PORTAL['w'] / 2
    for e in C.ENTRANCES.values():
        d = abs(y - e['y'])
        if d < hw:
            return PORTAL['side'] + math.sqrt(hw * hw - d * d)
    return None


def hill_height(x, y, noise):
    """Top surface: plateau over the mine, slopes down to the plain on N, E and S sides."""
    H = HILL['h']
    dx = max(0.0, x - HILL['x1'])
    dy = max(0.0, HILL['y0'] - y, y - HILL['y1'])
    d = math.hypot(dx, dy)
    s = 1.0 if d <= 0 else max(0.0, 1.0 - d / HILL['slope'])
    s = s * s * (3 - 2 * s)
    return max(0.0, (H + noise) * s)


def build_hillside(path_dir):
    rng = np.random.default_rng(SEED)
    X0, X1 = 0.0, HILL['x1'] + HILL['slope']
    Y0, Y1 = HILL['y0'] - HILL['slope'], HILL['y1'] + HILL['slope']
    step = 2.0
    nx, ny = int((X1 - X0) / step) + 1, int((Y1 - Y0) / step) + 1
    nz = fractal(256, 1.8, SEED + 5)

    def noise(x, y):
        i = int((x - X0) / (X1 - X0) * 255) % 256
        j = int((y - Y0) / (Y1 - Y0) * 255) % 256
        return (nz[i, j] - 0.5) * 6.0

    m = Mesh()
    top = {}
    for i in range(nx):
        for j in range(ny):
            x, y = X0 + i * step, Y0 + j * step
            z = hill_height(x, y, noise(x, y))
            top[i, j] = m.vert((x, y, z), (x / 6.0, y / 6.0))
    for i in range(nx - 1):
        for j in range(ny - 1):
            m.quad(top[i, j], top[i + 1, j], top[i + 1, j + 1], top[i, j + 1], (0, 0, 1))

    # cliff face (x ~ 0) with the portal openings; columns of 0.5 m, rows of ~1 m
    # columns every 0.5 m, every 0.1 m across the openings, and exactly on the opening edges
    ys = set(np.round(np.arange(Y0, Y1 + 1e-6, 0.5), 3))
    for e in C.ENTRANCES.values():
        hw = PORTAL['w'] / 2
        ys -= {y for y in ys if abs(y - e['y']) <= hw + 0.3}
        ys |= {round(e['y'] + k * 0.1, 3) for k in range(-int(hw * 10) + 1, int(hw * 10))}
        ys |= {e['y'] - hw, e['y'] - hw + 0.001, e['y'] + hw - 0.001, e['y'] + hw}
    ys = sorted(ys)
    rough = fractal(256, 1.5, SEED + 7)

    def face_x(y, z, edge):
        """Rough rock face; flat (x = -0.25) along the portal openings so the reveals meet it."""
        if edge:
            return -0.25
        amp = 1.0
        hw, side = PORTAL['w'] / 2, PORTAL['side']
        for e in C.ENTRANCES.values():
            dy = abs(y - e['y'])
            if z <= side:                                  # beside the straight jambs
                dist = max(0.0, dy - hw)
            else:                                          # around the arch
                dist = max(0.0, math.hypot(dy, z - side) - hw)
            amp = min(amp, dist / 1.5)
        i = int((y - Y0) / (Y1 - Y0) * 255) % 256
        k = int(z * 12) % 256
        return -0.25 - 0.5 * amp * rough[i, k]

    def top_at(y):
        j = (y - Y0) / step
        j0 = min(int(j), ny - 2)
        t = j - j0
        return (1 - t) * m.v[top[0, j0] - 1][2] + t * m.v[top[0, j0 + 1] - 1][2]

    cols = []
    for y in ys:
        zt = top_at(y)
        pb = portal_top(y)
        zb = 0.0 if pb is None else pb
        if zt <= zb + 0.05:
            cols.append(None)
            continue
        nrow = max(1, int(math.ceil((zt - zb) / 1.0)))
        ids = []
        for r in range(nrow + 1):
            z = zb + (zt - zb) * r / nrow
            edge = r == nrow or (pb is not None and r == 0)
            ids.append(m.vert((face_x(y, z, edge) if r < nrow else 0.0, y, z), (y / 4.0, z / 4.0)))
        cols.append(ids)
    for a, b in zip(cols[:-1], cols[1:]):
        if a is None or b is None:
            continue
        n = min(len(a), len(b))
        for r in range(n - 1):
            m.quad(a[r], b[r], b[r + 1], a[r + 1], (-1, 0, 0))
        # rows differ in count: fan the remaining vertices to the other column's top
        for r in range(n - 1, len(a) - 1):
            m.tri(a[r], b[-1], a[r + 1], (-1, 0, 0))
        for r in range(n - 1, len(b) - 1):
            m.tri(a[-1], b[r], b[r + 1], (-1, 0, 0))

    # portal reveals: rock collar from the face (x=-0.25) 1.2 m into the gallery mouth
    for e in C.ENTRANCES.values():
        hw = PORTAL['w'] / 2
        pts = []
        for k in range(0, 25):
            a = math.pi * k / 24                       # arch from right side to left side
            pts.append((e['y'] + hw * math.cos(a), PORTAL['side'] + hw * math.sin(a)))
        outline = [(e['y'] + hw, 0.0)] + pts + [(e['y'] - hw, 0.0)]
        prev = None
        for (y, z) in outline:
            p0 = m.vert((-0.25, y, z), (y / 4.0, z / 4.0))
            p1 = m.vert((1.2, y, z), (y / 4.0 + 0.3, z / 4.0))
            if prev is not None:
                (q0, q1, yq, zq) = prev
                cy, cz = (y + yq) / 2, (z + zq) / 2
                want = (0.0, e['y'] - cy, PORTAL['side'] * 0.5 - cz)
                m.quad(q0, p0, p1, q1, want)
            prev = (p0, p1, y, z)
    m.save(os.path.join(path_dir, 'meshes', 'hillside.obj'), 'hillside', 'hill_rock')
    mtl(os.path.join(path_dir, 'meshes', 'hillside.mtl'), 'hill_rock', '../materials/textures/hill_rock.png')
    rock_texture(os.path.join(path_dir, 'materials', 'textures', 'hill_rock.png'),
                 (0.52, 0.44, 0.36), SEED + 11)


# ------------------------------------------------------------------ terrain (visual)
def build_terrain(path_dir):
    """Plain around the hill (z=0 outside) and a lower 'mine level' under the hill that shows
    the galleries once the hill and tunnel shells are removed for the cutaway."""
    m = Mesh()
    X0, X1, Y0, Y1, step = -130.0, 230.0, -150.0, 140.0, 2.0
    nx, ny = int((X1 - X0) / step) + 1, int((Y1 - Y0) / step) + 1
    under = (1.0, HILL['x1'] + HILL['slope'] - 4, HILL['y0'] - HILL['slope'] + 4,
             HILL['y1'] + HILL['slope'] - 4)

    def z_at(x, y):
        if under[0] <= x <= under[1] and under[2] <= y <= under[3]:
            return -0.4
        return 0.0

    ids = {}
    for i in range(nx):
        for j in range(ny):
            x, y = X0 + i * step, Y0 + j * step
            ids[i, j] = m.vert((x, y, z_at(x, y)), (x / 5.0, y / 5.0))
    slot = (0.2, 62.0, -63.0, -57.0)                   # line C stays open (water below)
    for i in range(nx - 1):
        for j in range(ny - 1):
            x, y = X0 + i * step, Y0 + j * step
            if slot[0] < x + step and x < slot[1] and slot[2] < y + step and y < slot[3]:
                continue
            m.quad(ids[i, j], ids[i + 1, j], ids[i + 1, j + 1], ids[i, j + 1], (0, 0, 1))
    m.save(os.path.join(path_dir, 'meshes', 'terrain.obj'), 'terrain', 'dirt')
    mtl(os.path.join(path_dir, 'meshes', 'terrain.mtl'), 'dirt', '../materials/textures/dirt.png')
    rock_texture(os.path.join(path_dir, 'materials', 'textures', 'dirt.png'),
                 (0.62, 0.53, 0.40), SEED + 21, cracks=False)


# ------------------------------------------------------------------ rocks (spawned debris)
def build_rock(path_dir):
    """Low-poly irregular rock (unit size ~0.4 m)."""
    rng = np.random.default_rng(SEED + 3)
    m = Mesh()
    nu, nv = 10, 7
    ids = {}
    for i in range(nv + 1):
        th = math.pi * i / nv
        for j in range(nu):
            ph = 2 * math.pi * j / nu
            r = 0.2 * (0.8 + 0.35 * rng.random())
            p = (r * math.sin(th) * math.cos(ph) * 1.2, r * math.sin(th) * math.sin(ph),
                 r * math.cos(th) * 0.75)
            ids[i, j] = m.vert(p, (j / nu, i / nv))
    for i in range(nv):
        for j in range(nu):
            a, b = ids[i, j], ids[i, (j + 1) % nu]
            c, d = ids[i + 1, (j + 1) % nu], ids[i + 1, j]
            cen = np.mean([m.v[k - 1] for k in (a, b, c, d)], axis=0)
            m.quad(a, b, c, d, tuple(cen))
    m.save(os.path.join(path_dir, 'meshes', 'rock.obj'), 'rock', 'rock')
    mtl(os.path.join(path_dir, 'meshes', 'rock.mtl'), 'rock', '../materials/textures/rock.png')
    rock_texture(os.path.join(path_dir, 'materials', 'textures', 'rock.png'),
                 (0.40, 0.34, 0.28), SEED + 31)


# ================================================================== robots
def _rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def compose(a, b):
    """SDF pose strings: returns pose of b expressed through a (a * b)."""
    A = [float(v) for v in a.split()]
    B = [float(v) for v in b.split()]
    Ra, Rb = _rot(*A[3:]), _rot(*B[3:])
    t = np.array(A[:3]) + Ra @ np.array(B[:3])
    R = Ra @ Rb
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    r = math.atan2(R[2, 1], R[2, 2])
    y = math.atan2(R[1, 0], R[0, 0])
    return f'{t[0]:.4f} {t[1]:.4f} {t[2]:.4f} {r:.4f} {p:.4f} {y:.4f}'


def _strip_frames(el):
    for e in el.iter():
        if 'frame' in e.attrib:
            del e.attrib['frame']
    for axis in el.iter('axis'):
        for u in axis.findall('use_parent_model_frame'):
            axis.remove(u)
    for vis in el.iter('visual'):               # SubT X1 source has a misplaced <material>
        g = vis.find('geometry')
        if g is not None:
            for m in g.findall('material'):
                g.remove(m)
                if vis.find('material') is None:
                    vis.append(m)


def _bitmask(link, mask='0x01'):
    for col in link.findall('collision'):
        surf = col.find('surface')
        if surf is None:
            surf = ET.SubElement(col, 'surface')
        con = surf.find('contact')
        if con is None:
            con = ET.SubElement(surf, 'contact')
        bm = con.find('collide_bitmask')
        if bm is None:
            bm = ET.SubElement(con, 'collide_bitmask')
        bm.text = mask


def _xml(text):
    return ET.fromstring(text)


LIDAR_MASK = 1          # robot lidars / ToF see visuals with bit 0 set (default: all bits)
DECOR_FLAGS = 2         # non-physical visuals (water surface, gallery footprint): camera only


def gpu_lidar(name, topic, pose, samples, amin, amax, rmin, rmax, rate, noise=0.01):
    return (f'<sensor name="{name}" type="gpu_lidar"><pose>{pose}</pose><topic>{topic}</topic>'
            f'<update_rate>{rate}</update_rate><always_on>1</always_on><visualize>false</visualize>'
            f'<lidar><scan><horizontal><samples>{samples}</samples><resolution>1</resolution>'
            f'<min_angle>{amin}</min_angle><max_angle>{amax}</max_angle></horizontal></scan>'
            f'<range><min>{rmin}</min><max>{rmax}</max><resolution>0.01</resolution></range>'
            f'<noise><type>gaussian</type><mean>0</mean><stddev>{noise}</stddev></noise>'
            f'<visibility_mask>{LIDAR_MASK}</visibility_mask></lidar></sensor>')


def imu(name, topic, rate, gyro_sd, bias_mean, bias_sd):
    ax = ''.join(
        f'<{a}><noise type="gaussian"><mean>0</mean><stddev>{gyro_sd}</stddev>'
        f'<bias_mean>{bias_mean}</bias_mean><bias_stddev>{bias_sd}</bias_stddev>'
        f'<dynamic_bias_stddev>0.00002</dynamic_bias_stddev>'
        f'<dynamic_bias_correlation_time>400.0</dynamic_bias_correlation_time>'
        f'</noise></{a}>' for a in 'xyz')
    acc = ''.join(f'<{a}><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></{a}>'
                  for a in 'xyz')
    return (f'<sensor name="{name}" type="imu"><topic>{topic}</topic><update_rate>{rate}</update_rate>'
            f'<always_on>1</always_on><imu><enable_orientation>0</enable_orientation>'
            f'<angular_velocity>{ax}</angular_velocity><linear_acceleration>{acc}'
            f'</linear_acceleration></imu></sensor>')


def odom_truth(name, dims=2):
    """Ground truth pose (world frame) - used only for the sensor models and the 2D window."""
    return plugin('odometry-publisher', 'OdometryPublisher',
                  f'<odom_frame>world</odom_frame><robot_base_frame>{name}</robot_base_frame>'
                  f'<odom_topic>/model/{name}/ground_truth</odom_topic>'
                  f'<odom_publish_frequency>30</odom_publish_frequency>'
                  f'<dimensions>{dims}</dimensions>')


# per-robot adaptation of the SubT models: which head/spot lights to keep, where our sensors
# go, the payload that tells the two robots apart, and the SubT drive parameters
SUBT = {
    'writer': dict(
        ref='robotika_x2_sensor_config_1.sdf', pkg='lm_x2', z=0.0635 + 0.02,
        lights=('left_light_source', 'right_light_source'),
        drop_visuals=('laser_visual',),
        lidar_pose='0 0 0.30 0 0 0',
        extra=(cyl_visual('lidar_mast', '0 0 0.25 0 0 0', 0.012, 0.1, '0.1 0.1 0.1 1')
               + cyl_visual('lidar_head', '0 0 0.30 0 0 0', 0.035, 0.06, '0.08 0.08 0.08 1')
               + box_visual('beacon_magazine', '-0.12 0 0.215 0 0 0', '0.18 0.20 0.06',
                            '0.25 0.25 0.27 1')
               + cyl_visual('beacon_stack', '-0.12 0 0.235 0 0 0', 0.07, 0.03,
                            '0.1 0.8 0.3 1', '0.05 0.4 0.15 1')
               + box_visual('csi_stack', '0.02 -0.10 0.205 0 0 0', '0.08 0.06 0.05',
                            '0.1 0.75 0.85 1', '0.05 0.35 0.45 1')
               + box_visual('gas_sensor', '0.16 0.10 0.2 0 0 0', '0.05 0.04 0.03',
                            '0.95 0.75 0.1 1')),
        left=('front_left_wheel_joint', 'rear_left_wheel_joint'),
        right=('front_right_wheel_joint', 'rear_right_wheel_joint'),
        sep=0.33559 * 1.23, radius=0.098, vmax=2, acc=6,
        wheels=('front_left_wheel', 'rear_left_wheel', 'front_right_wheel', 'rear_right_wheel'),
        slip_lat=0.116, normal=45.20448),
    'executor': dict(
        ref='explorer_x1_sensor_config_1.sdf', pkg='lm_x1', z=0.1321 + 0.03,
        lights=('left_headlight_body_light_source_light', 'right_headlight_body_light_source_light'),
        drop_visuals=(),
        lidar_pose='0.2812 0 0.494 0 0 0',
        extra=(box_visual('medkit', '-0.18 0 0.36 0 0 0', '0.22 0.26 0.12', '0.95 0.95 0.95 1')
               + box_visual('medkit_cross_a', '-0.18 0 0.421 0 0 0', '0.05 0.16 0.003',
                            '0.85 0.1 0.1 1', '0.4 0 0 1')
               + box_visual('medkit_cross_b', '-0.18 0 0.421 0 0 0', '0.16 0.05 0.003',
                            '0.85 0.1 0.1 1', '0.4 0 0 1')
               + box_visual('lightbar', '-0.38 0 0.33 0 0 0', '0.06 0.34 0.04',
                            '0.3 0.6 1 1', '0.2 0.45 1 1')
               + box_visual('rf_array', '0.05 0 0.35 0 0 0', '0.08 0.5 0.02', '0.15 0.15 0.15 1')),
        left=('front_left_wheel_joint', 'rear_left_wheel_joint'),
        right=('front_right_wheel_joint', 'rear_right_wheel_joint'),
        sep=0.45649 * 1.5, radius=0.1651, vmax=1, acc=3,
        wheels=('front_left_wheel_link', 'rear_left_wheel_link', 'front_right_wheel_link',
                'rear_right_wheel_link'),
        slip_lat=0.172, normal=138.767),
}


ID_COLOR = {'writer_a': '0.15 0.45 0.95 1', 'writer_b': '0.98 0.55 0.1 1',
            'flix_a': '0.15 0.45 0.95 1', 'flix_b': '0.98 0.55 0.1 1'}


def subt_robot(name, spawn):
    cfg = dict(SUBT['writer' if name.startswith('writer') else name])
    if name in ID_COLOR:
        c = ID_COLOR[name]
        cfg['extra'] = cfg['extra'] + (box_visual('id_plate', '-0.12 0 0.255 0 0 0', '0.20 0.22 0.012', c, c)
                                       + cyl_visual('id_beacon', '0.10 0 0.27 0 0 0', 0.025, 0.05, c, c))
    src = ET.parse(os.path.join(REF, cfg['ref'])).getroot().find('model')
    _strip_frames(src)
    parts = []
    for el in list(src):
        if el.tag == 'link':
            for s in el.findall('sensor'):
                el.remove(s)
            for li in el.findall('light'):
                if li.get('name') not in cfg['lights']:
                    el.remove(li)
                    continue
                li.find('cast_shadows').text = '0'
                li.find('attenuation/range').text = '15'
                ET.SubElement(li, 'visualize').text = 'false'
            for v in el.findall('visual'):
                if v.get('name') in cfg['drop_visuals']:
                    el.remove(v)
            for u in el.iter('uri'):
                if u.text and u.text.strip().startswith('meshes/'):
                    u.text = f'model://{cfg["pkg"]}/' + u.text.strip()
            _bitmask(el)
            if el.get('name') == 'base_link':
                el.append(_xml(gpu_lidar('lidar', f'/{name}/scan', cfg['lidar_pose'], 271,
                                         -2.356, 2.356, 0.1, 12.0, 5)))
                el.append(_xml(imu('imu', f'/{name}/imu', 50, C.SENS['gyro_sigma'], 0.00075, 0.005)))
                for v in ET.fromstring(f'<x>{cfg["extra"]}</x>'):
                    el.append(v)
            parts.append(ET.tostring(el, encoding='unicode'))
        elif el.tag == 'joint':
            parts.append(ET.tostring(el, encoding='unicode'))
    dd = plugin('diff-drive', 'DiffDrive',
                ''.join(f'<left_joint>{j}</left_joint>' for j in cfg['left'])
                + ''.join(f'<right_joint>{j}</right_joint>' for j in cfg['right'])
                + f'<wheel_separation>{cfg["sep"]:.5f}</wheel_separation>'
                  f'<wheel_radius>{cfg["radius"]}</wheel_radius>'
                  f'<odom_publish_frequency>30</odom_publish_frequency>'
                  f'<min_velocity>-{cfg["vmax"]}</min_velocity><max_velocity>{cfg["vmax"]}</max_velocity>'
                  f'<min_acceleration>-{cfg["acc"]}</min_acceleration>'
                  f'<max_acceleration>{cfg["acc"]}</max_acceleration>')
    ws = plugin('wheel-slip', 'WheelSlip', ''.join(
        f'<wheel link_name="{w}"><slip_compliance_lateral>{cfg["slip_lat"]}</slip_compliance_lateral>'
        f'<slip_compliance_longitudinal>0</slip_compliance_longitudinal>'
        f'<wheel_normal_force>{cfg["normal"]}</wheel_normal_force>'
        f'<wheel_radius>{cfg["radius"]}</wheel_radius></wheel>' for w in cfg['wheels']))
    x, y, _, yaw = spawn
    return (f'<model name="{name}"><pose>{x} {y} {cfg["z"]:.4f} 0 0 {yaw}</pose>'
            + ''.join(parts) + dd + ws + odom_truth(name) + '</model>')


def flix(name):
    """Flix quadcopter (okalachev/flix): 65 g, 95 mm frame, ESP32-S3 (ESP-NOW + Wi-Fi CSI).
    Sensors of the fast writer: IMU (real), downward ToF (1-beam lidar), 3 side ToF rangers,
    thermal + RGB camera (payload visual; person detection is a sensor model).
    Scripted motion: VelocityControl."""
    x, y, z, yaw = C.SPAWN[name]
    idc = ID_COLOR[name]
    props = ''
    for i, (px, py, col) in enumerate(((-0.04243, 0.04243, '0.8 0.3 0.3 0.5'),
                                       (-0.04243, -0.04243, '0.8 0.3 0.3 0.5'),
                                       (0.04243, -0.04243, '1 1 1 0.5'),
                                       (0.04243, 0.04243, '1 1 1 0.5'))):
        props += (f'<visual name="prop{i}"><pose>{px} {py} 0.0142 0 0 0</pose><geometry><cylinder>'
                  f'<radius>0.0275</radius><length>0.001</length></cylinder></geometry>'
                  f'{mat(col)}<transparency>0.5</transparency><cast_shadows>false</cast_shadows></visual>')
    return f'''
    <model name="{name}">
      <pose>{x} {y} {z + 0.02} 0 0 {yaw}</pose>
      <link name="body">
        <inertial><mass>0.065</mass><inertia><ixx>3.55e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>4.23e-5</iyy><iyz>0</iyz><izz>7.47e-5</izz></inertia></inertial>
        <collision name="collision"><geometry><box><size>0.095 0.095 0.0276</size></box></geometry>
          <surface><contact><collide_bitmask>0x01</collide_bitmask></contact></surface></collision>
        <visual name="frame"><geometry><mesh><uri>model://lm_flix/meshes/flix.stl</uri></mesh></geometry>
          {mat('0.5 0.5 0.6 1')}</visual>
        {props}
        {box_visual('thermal_cam', '0.045 0 0.0 0 0.35 0', '0.012 0.02 0.014', '0.1 0.1 0.1 1')}
        {cyl_visual('thermal_lens', '0.052 0 -0.002 0 1.92 0', 0.005, 0.004, '0.3 0.2 0.5 1')}
        {box_visual('tof_down', '0.02 0 -0.016 0 0 0', '0.012 0.012 0.004', '0.05 0.05 0.05 1')}
        {cyl_visual('status_led', '-0.03 0 0.012 0 0 0', 0.006, 0.006, idc, idc)}
        {box_visual('id_band', '0 0 0.016 0 0 0', '0.05 0.05 0.003', idc, idc)}
        {box_visual('esp32s3', '-0.012 0 -0.016 0 0 0', '0.025 0.018 0.004', '0.1 0.75 0.85 1')}
        {imu('imu', f'/{name}/imu', 200, 0.00174533, 0.0005, 0.002)}
        {gpu_lidar('tof_down_sensor', f'/{name}/tof_down', f'0 0 -{C.FLIX_TOF_DOWN_Z} 0 1.5708 0', 1, 0, 0, 0.05, 4.0, 10, 0.01)}
        {gpu_lidar('tof_side_sensor', f'/{name}/tof', '0 0 0.025 0 0 0', 3, -1.5708, 1.5708, 0.05, 4.0, 10, 0.01)}
        <light name="flix_light" type="spot"><pose>0.05 0 -0.005 0 0 0</pose>
          <diffuse>1 0.97 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
          <attenuation><range>14</range><constant>0.2</constant><linear>0.04</linear>
          <quadratic>0.006</quadratic></attenuation><direction>1 0 -0.25</direction>
          <spot><inner_angle>0.6</inner_angle><outer_angle>1.0</outer_angle><falloff>1</falloff></spot>
          <cast_shadows>false</cast_shadows><visualize>false</visualize></light>
      </link>
      {plugin('velocity-control', 'VelocityControl', '<initial_linear>0 0 0</initial_linear>')}
      {odom_truth(name, 3)}
    </model>'''


# ================================================================== scenery
def pbr_rock(color=ROCK_DARK, tex='model://lm_rock/materials/textures/rock.png'):
    return (f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular><pbr><metal><albedo_map>{tex}</albedo_map>'
            f'<roughness>0.95</roughness><metalness>0.0</metalness></metal></pbr></material>')


def tiles():
    out = []
    for name, model, cx, cy, cz, yaw in C.TILES:
        out.append(include(f'tile_{name}', fuel(model), f'{cx} {cy} {cz} 0 0 {yaw:.6f}', True))
    return ''.join(out)


def _tile_center_near(x, y):
    return min(((t[2], t[3]) for t in C.TILES), key=lambda c: math.hypot(c[0] - x, c[1] - y))


def blockers():
    """Rock plugs closing the open tile ends (one per BLOCKERS entry)."""
    out = []
    for i, (x, y, z) in enumerate(C.BLOCKERS):
        cx, cy = _tile_center_near(x, y)
        along_x = abs(x - cx) > abs(y - cy)
        size = '2.0 12 12' if along_x else '12 2.0 12'
        out.append(f'<model name="blocker_{i}"><static>true</static><pose>{x} {y} {z + 4.0} 0 0 0</pose>'
                   f'<link name="l"><collision name="c"><geometry><box><size>{size}</size></box>'
                   f'</geometry></collision><visual name="v"><geometry><box><size>{size}</size>'
                   f'</box></geometry>{pbr_rock()}</visual></link></model>')
    return ''.join(out)


def _free(x, y):
    return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in C.TUNNEL_RECTS)


def basic_galleries():
    """Offline fallback (no Fuel): rock-walled galleries with the same free space as the tiles,
    floors (incl. the line C ramps) and roofs."""
    cell = 1.0
    xs = np.arange(-1.0, 152.0, cell)
    ys = np.arange(-80.0, 70.0, cell)
    free = {(i, j): x < 0 or _free(x + 0.5, y + 0.5) for i, x in enumerate(xs) for j, y in enumerate(ys)}
    # wall faces between a free cell and a rock cell: (axis, line coordinate, side) -> starts
    # side = +1: rock on the + side of the line
    segs = {}
    for (i, j), f in free.items():
        x, y = float(xs[i]), float(ys[j])
        if not f or x < 0:                         # x < 0 is outside: the cliff face closes it
            continue
        if not free.get((i, j + 1), False):
            segs.setdefault(('x', round(y + cell, 3), 1), []).append(x)
        if not free.get((i, j - 1), False):
            segs.setdefault(('x', round(y, 3), -1), []).append(x)
        if not free.get((i + 1, j), False):
            segs.setdefault(('y', round(x + cell, 3), 1), []).append(y)
        if not free.get((i - 1, j), False):
            segs.setdefault(('y', round(x, 3), -1), []).append(y)
    links = []
    T = 0.6

    def wall(n, x0, x1, y0, y1, zc, h):
        links.append(f'<link name="w{n}"><pose>{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} {zc:.2f} 0 0 0</pose>'
                     f'<collision name="c"><geometry><box><size>{abs(x1 - x0):.3f} {abs(y1 - y0):.3f} {h}'
                     f'</size></box></geometry></collision><visual name="v"><geometry><box><size>'
                     f'{abs(x1 - x0):.3f} {abs(y1 - y0):.3f} {h}</size></box></geometry>{pbr_rock()}'
                     f'</visual></link>')
    n = 0
    for (axis, k, side), starts in sorted(segs.items()):
        starts = sorted(starts)
        runs, run = [], [starts[0], starts[0] + cell]
        for s0 in starts[1:]:
            if abs(s0 - run[1]) < 1e-6:
                run[1] = s0 + cell
            else:
                runs.append(run)
                run = [s0, s0 + cell]
        runs.append(run)
        for a, b in runs:
            n += 1
            # extend runs by the wall thickness so corners close
            a, b = a - T, b + T
            line_c = (axis == 'x' and k < -50) or (axis == 'y' and b < -50)
            zc, h = (-0.75, 12.5) if line_c else (2.4, 5.2)
            lo, hi = (k, k + side * T) if side > 0 else (k + side * T, k)
            if axis == 'x':
                wall(n, a, b, lo, hi, zc, h)
            else:
                wall(n, lo, hi, a, b, zc, h)
    walls = f'<model name="galleries_walls"><static>true</static>{"".join(links)}</model>'
    # floors and roofs (line C follows its vertical profile)
    fl = []
    rf = []
    for m, (x0, x1, y0, y1) in enumerate(C.TUNNEL_RECTS):
        if y1 < -50:
            continue
        cx, cy, sx, sy = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0 + 0.6, y1 - y0 + 0.6
        fl.append(f'<link name="f{m}"><pose>{cx} {cy} -0.1 0 0 0</pose><collision name="c"><geometry>'
                  f'<box><size>{sx} {sy} 0.2</size></box></geometry></collision><visual name="v">'
                  f'<geometry><box><size>{sx} {sy} 0.2</size></box></geometry>'
                  f'{pbr_rock("0.30 0.26 0.22 1")}</visual></link>')
        rf.append(f'<link name="r{m}"><pose>{cx} {cy} 4.95 0 0 0</pose><visual name="v"><geometry>'
                  f'<box><size>{sx} {sy} 0.3</size></box></geometry>{pbr_rock()}</visual></link>')
    slope = math.atan2(5.0, 20.0)
    L = math.hypot(20.0, 5.0)
    for m, (cx, cz, pitch, length) in enumerate(((10, -2.5, slope, L), (30, -5.0, 0.0, 20.0),
                                                 (50, -2.5, -slope, L), (70, 0.0, 0.0, 20.0))):
        fl.append(f'<link name="c{m}"><pose>{cx} -60 {cz - 0.1} 0 {pitch:.4f} 0</pose><collision name="c">'
                  f'<geometry><box><size>{length + 0.2} 4.6 0.2</size></box></geometry></collision>'
                  f'<visual name="v"><geometry><box><size>{length + 0.2} 4.6 0.2</size></box></geometry>'
                  f'{pbr_rock("0.30 0.26 0.22 1")}</visual></link>')
        rf.append(f'<link name="cr{m}"><pose>{cx} -60 {cz + 5.0} 0 {pitch:.4f} 0</pose><visual name="v">'
                  f'<geometry><box><size>{length + 0.2} 4.6 0.3</size></box></geometry>{pbr_rock()}'
                  f'</visual></link>')
    floors = f'<model name="galleries_floor"><static>true</static>{"".join(fl)}</model>'
    roof = f'<model name="galleries_roof"><static>true</static>{"".join(rf)}</model>'
    return walls + floors + roof


def terrain_and_hill():
    ground_col = ('<model name="outside_ground"><static>true</static><link name="l">'
                  '<pose>-65 -5 -0.5 0 0 0</pose><collision name="c"><geometry><box>'
                  '<size>130 290 1.0</size></box></geometry></collision></link></model>')
    terrain = ('<model name="terrain"><static>true</static><link name="l"><visual name="v">'
               '<geometry><mesh><uri>model://lm_terrain/meshes/terrain.obj</uri></mesh></geometry>'
               '<cast_shadows>false</cast_shadows></visual></link></model>')
    hill = ('<model name="hillside"><static>true</static><link name="l"><visual name="v">'
            '<geometry><mesh><uri>model://lm_hillside/meshes/hillside.obj</uri></mesh></geometry>'
            '</visual></link></model>')
    # gallery footprint on the mine level: only seen in the cutaway (hidden under tile floors)
    fp = []
    for m, (x0, x1, y0, y1) in enumerate(C.TUNNEL_RECTS):
        if y1 < -50:
            continue
        fp.append(box_visual(f'fp{m}', f'{(x0 + x1) / 2} {(y0 + y1) / 2} -0.33 0 0 0',
                             f'{x1 - x0} {y1 - y0} 0.02', '0.58 0.50 0.40 1', flags=DECOR_FLAGS))
    foot = f'<model name="gallery_footprint"><static>true</static><link name="l">{"".join(fp)}</link></model>'
    return ground_col + terrain + hill + foot


def portal_frames():
    """Timber portal sets at the four openings + signs (Flix reads the sign). Portal D: the cap
    beam has come down with the roof fall."""
    L = []
    for k, e in C.ENTRANCES.items():
        y = e['y']
        for sd in (-1, 1):
            L.append(box_link(f'post_{k}{"ab"[sd > 0]}', f'-0.4 {y + sd * 1.55} 1.35 0 0 0',
                              '0.28 0.28 2.7', TIMBER, collision=False))
        if e['state'] == 'collapsed':
            L.append(box_link(f'cap_{k}', f'-0.9 {y + 0.4} 0.55 0.35 0.9 0.5', '0.3 3.6 0.3', TIMBER,
                              collision=False))
        else:
            L.append(box_link(f'cap_{k}', f'-0.4 {y} 2.8 0 0 0', '0.3 3.6 0.3', TIMBER, collision=False))
        sign = {'clear': '0.95 0.8 0.1 1', 'flooded': '0.1 0.35 0.9 1',
                'collapsed': '0.85 0.15 0.1 1'}[e['state']]
        L.append(box_link(f'sign_{k}', f'-0.62 {y + 2.3} 1.6 0 0 0', '0.04 0.8 0.5', sign, collision=False))
        L.append(box_link(f'label_{k}', f'-0.65 {y + 2.3} 1.6 0 0 0', '0.02 0.3 0.3', '0.1 0.1 0.1 1',
                          collision=False))
    return f'<model name="portal_sets"><static>true</static>{"".join(L)}</model>'


def collapsed_adit(fuel_ok):
    """Portal D: a short adit closed by a roof fall 2.5 m inside (rock pile to the roof)."""
    d = C.ENTRANCES['D']
    y = d['y']
    x0, x1 = C.ADIT_D
    rng = random.Random(9)
    L = []
    if fuel_ok:                      # the offline world builds walls from TUNNEL_RECTS already
        for sd in (-1, 1):
            L.append(box_link(f'adit_wall{sd}', f'{(x0 + x1 + 2) / 2} {y + sd * 2.0} 2.4 0 0 0',
                              f'{x1 - x0 + 2.2} 0.6 5.2', ROCK_DARK))
        L.append(box_link('adit_roof', f'{(x0 + x1 + 2) / 2} {y} 4.95 0 0 0', f'{x1 - x0 + 2.2} 4.6 0.3',
                          ROCK_DARK, collision=False))
        L.append(box_link('adit_floor', f'{(x0 + x1 + 2) / 2} {y} -0.1 0 0 0', f'{x1 - x0 + 2.2} 4.6 0.2',
                          '0.30 0.26 0.22 1'))
    L.append(box_link('adit_plug', f'{x1 + 1.2} {y} 2.4 0 0 0', '2.2 4.0 5.0', ROCK_DARK))
    rub = ''
    for i in range(48):
        sc = rng.uniform(1.8, 3.2)
        zz = rng.uniform(0.1, 3.6) if i < 34 else rng.uniform(0.05, 0.5)     # + a debris slope
        xx = x1 + rng.uniform(-0.4, 0.5) if i < 34 else x1 - rng.uniform(0.3, 1.2)
        rub += (f'<visual name="r{i}"><pose>{xx:.2f} {y + rng.uniform(-1.6, 1.6):.2f} '
                f'{zz:.2f} {rng.uniform(0, 3):.2f} {rng.uniform(0, 3):.2f} '
                f'{rng.uniform(0, 3):.2f}</pose><geometry><mesh><uri>model://lm_rock/meshes/rock.obj</uri>'
                f'<scale>{sc:.2f} {sc:.2f} {sc:.2f}</scale></mesh></geometry></visual>')
    L.append(f'<link name="rubble">{rub}<collision name="c"><pose>{x1} {y} 1.6 0 0 0</pose><geometry>'
             f'<box><size>1.0 3.6 3.2</size></box></geometry></collision></link>')
    return f'<model name="adit_d"><static>true</static>{"".join(L)}</model>'


def sat_view(px, py, pz):
    """Azimuth / elevation of the relay satellite seen from (px, py, pz)."""
    sx, sy, sz = C.SAT_POS
    dx, dy, dz = sx - px, sy - py, sz - pz
    return math.atan2(dy, dx), math.atan2(dz, math.hypot(dx, dy))


DISH_PIVOT = 2.628                  # reflector pivot height of the converted NASA dish
DISH_OFF = (-2.4, -1.2)             # dish position relative to its ONA


def dish_xyz(ox, oy):
    return ox + DISH_OFF[0], oy + DISH_OFF[1], DISH_PIVOT


def dish_visuals(name, x, y, scale=1.0):
    """NASA 'Tall Dish' (mount + reflector) with the reflector pointed at the satellite."""
    az, el = sat_view(x, y, DISH_PIVOT * scale)
    return (f'<link name="{name}_mount"><pose>{x} {y} 0 0 0 {az:.4f}</pose><visual name="v"><geometry><mesh>'
            f'<uri>model://lm_dish/meshes/dish_mount.obj</uri><scale>{scale} {scale} {scale}</scale>'
            f'</mesh></geometry></visual></link>'
            f'<link name="{name}_reflector"><pose>{x} {y} {DISH_PIVOT * scale:.3f} 0 {math.pi / 2 - el:.4f} '
            f'{az:.4f}</pose><visual name="v"><geometry><mesh><uri>model://lm_dish/meshes/'
            f'dish_reflector.obj</uri><scale>{scale} {scale} {scale}</scale></mesh></geometry></visual></link>')


def ona_site(letter):
    """One Outside Network Area: control console PC, hub cabinet with status light, fiber reel,
    NASA dish pointed at the relay satellite."""
    ox, oy = C.ONAS[letter]['pos']
    px, py = C.ONAS[letter]['portal']
    face = math.atan2(py - oy, px - ox)
    L = []
    L.append(f'<link name="console"><pose>{ox - 0.6} {oy + 1.3} 0 0 0 {face + math.pi / 2:.4f}</pose>'
             f'<visual name="v"><geometry><mesh><uri>model://control_console/meshes/console.obj</uri></mesh>'
             f'</geometry></visual><collision name="c"><pose>0 0 1.3 0 0 0</pose><geometry><box>'
             f'<size>1.78 1.0 2.6</size></box></geometry></collision></link>')
    L.append(box_link('hub', f'{ox + 0.9} {oy} 0.7 0 0 0', '0.8 1.2 1.4', '0.85 0.85 0.8 1'))
    L.append(box_link('hub_door', f'{ox + 0.49} {oy} 0.75 0 0 0', '0.02 1.0 1.1', '0.25 0.45 0.7 1',
                      collision=False))
    L.append(f'<link name="led"><pose>{ox + 0.9} {oy} 1.5 0 0 0</pose>'
             f'{cyl_visual("v", "0 0 0 0 0 0", 0.08, 0.12, "0.2 0.8 1 1", "0.2 0.8 1 1")}</link>')
    L.append(f'<link name="reel"><pose>{ox + 0.5} {oy - 1.4} 0.35 1.5708 0 0</pose>'
             f'{cyl_visual("v", "0 0 0 0 0 0", 0.35, 0.3, "1 0.45 0 1")}</link>')
    dx, dy, _ = dish_xyz(ox, oy)
    L.append(dish_visuals('dish', dx, dy))
    L.append(box_link('sign', f'{ox - 0.6} {oy - 0.6} 1.0 0 0 0', '0.05 0.9 0.6',
                      {'A': '0.15 0.45 0.95 1', 'B': '0.98 0.55 0.1 1', 'C': '0.1 0.7 0.75 1'}[letter],
                      collision=False))
    # solar panel array (tilted toward the south) + high-capacity battery cabinet: the station's
    # own energy, also used to recharge the robots between missions
    for i in range(3):
        px_, py_ = ox - 3.6, oy + 2.2 + 1.15 * i
        L.append(f'<link name="solar{i}"><pose>{px_} {py_} 1.0 0 0.55 -1.5708</pose>'
                 f'{box_visual("panel", "0 0 0 0 0 0", "1.0 1.7 0.04", "0.08 0.12 0.3 1", "0.02 0.03 0.08 1")}'
                 f'{box_visual("frame", "0 0 -0.03 0 0 0", "1.04 1.74 0.02", STEEL)}</link>')
        L.append(box_link(f'solar_leg{i}', f'{px_} {py_ + 0.1} 0.45 0 0 0', '0.06 0.06 0.9', STEEL,
                          collision=False))
    L.append(box_link('battery', f'{ox + 0.9} {oy + 1.4} 0.55 0 0 0', '0.7 0.9 1.1', '0.92 0.92 0.88 1'))
    L.append(box_link('battery_label', f'{ox + 0.54} {oy + 1.4} 0.8 0 0 0', '0.02 0.5 0.2',
                      '0.1 0.7 0.2 1', collision=False))
    if letter in C.EXIT_PARK:
        cx, cy = C.EXIT_PARK[letter]
        L.append(f'<link name="charger"><pose>{cx} {cy} 0.004 0 0 0</pose>'
                 f'{box_visual("pad", "0 0 0 0 0 0", "1.6 1.6 0.008", "0.2 0.2 0.22 1")}'
                 f'{box_visual("bolt", "0 0 0.005 0 0 0.5", "0.9 0.12 0.004", "1 0.8 0.1 1", "0.5 0.4 0 1")}'
                 f'</link>')
    return f'<model name="ona_{letter.lower()}"><static>true</static>{"".join(L)}</model>'


def outside_infra():
    cx, cy = C.CP_POS
    sx, sy = C.SERVER_POS
    L = []

    def cable(name, pts, col='1 0.45 0 1'):
        for i, ((x0, y0), (x1, y1)) in enumerate(zip(pts[:-1], pts[1:])):
            ln, a = math.hypot(x1 - x0, y1 - y0), math.atan2(y1 - y0, x1 - x0)
            L.append(f'<link name="{name}{i}"><pose>{(x0 + x1) / 2} {(y0 + y1) / 2} 0.03 0 0 {a}</pose>'
                     f'{box_visual("v", "0 0 0 0 0 0", f"{ln} 0.04 0.04", col, "0.5 0.2 0 1")}</link>')
    # optical fibre: each station to the first beacon of its portal (C: its own beacon at the
    # flooded portal, placed by the station crew)
    for k in ('A', 'B'):
        ox, oy = C.ONAS[k]['pos']
        py = C.ONAS[k]['portal'][1]
        cable(f'fiber_{k.lower()}', [(0.3, py + 0.6), (-4.0, py + 1.5), (ox + 0.5, oy - 1.0)])
    gx, gy = C.ONA_C_BEACON
    ox, oy = C.ONAS['C']['pos']
    cable('fiber_c', [(gx - 0.2, gy + 0.6), (-4.0, gy + 4.0), (ox + 0.5, oy - 1.0)])
    # Command Post: container office, its own dish, mast
    L.append(box_link('cp_container', f'{cx} {cy} 1.3 0 0 0.3', '6.1 2.45 2.6', '0.75 0.2 0.15 1'))
    L.append(box_link('cp_roof', f'{cx} {cy} 2.65 0 0 0.3', '6.3 2.6 0.1', '0.5 0.5 0.5 1', collision=False))
    L.append(box_link('cp_window', f'{cx + 0.4} {cy - 1.23} 1.5 0 0 0.3', '2.0 0.02 0.8', '0.4 0.6 0.8 1',
                      collision=False))
    L.append(dish_visuals('cp_dish', cx + 4.5, cy - 2.0))
    # mission server rack (shared mission memory of every ONA)
    L.append(box_link('server_rack', f'{sx} {sy} 1.0 0 0 0.3', '1.2 1.0 2.0', '0.12 0.12 0.14 1'))
    for i in range(6):
        L.append(f'<link name="server_led{i}"><pose>{sx + 0.62 * math.cos(0.3) - 0.2} '
                 f'{sy + 0.62 * math.sin(0.3) - 0.3 + 0.12 * i} {0.5 + 0.22 * i} 0 0 0.3</pose>'
                 f'{box_visual("v", "0 0 0 0 0 0", "0.02 0.08 0.03", "0.2 1 0.4 1", "0.2 1 0.4 1")}</link>')
    # rescue vehicles at the staging area
    for i, (vx, vy, col) in enumerate(((-30.0, 12.0, '0.9 0.9 0.9 1'), (-31.0, 5.0, '0.85 0.2 0.15 1'))):
        L.append(box_link(f'truck{i}_body', f'{vx} {vy} 1.4 0 0 0', '6.0 2.4 2.0', col))
        L.append(box_link(f'truck{i}_cab', f'{vx + 3.6} {vy} 1.2 0 0 0', '1.6 2.3 1.6', col))
        L.append(box_link(f'truck{i}_glass', f'{vx + 4.41} {vy} 1.55 0 0 0', '0.02 2.0 0.6',
                          '0.3 0.45 0.6 1', collision=False))
        for wx in (-2.0, 3.4):
            for sd in (-1, 1):
                L.append(f'<link name="truck{i}_w{wx}{sd}"><pose>{vx + wx} {vy + sd * 1.2} 0.45 1.5708 0 0</pose>'
                         f'{cyl_visual("v", "0 0 0 0 0 0", 0.45, 0.3, "0.08 0.08 0.08 1")}</link>')
    # staging pad for the ground robots and the two drone pads
    L.append(f'<link name="staging"><pose>-15 0 0.005 0 0 0</pose>'
             f'{box_visual("v", "0 0 0 0 0 0", "7 9 0.01", "0.35 0.35 0.37 1")}</link>')
    for f in C.FLIXES:
        fx, fy = C.SPAWN[f][:2]
        c = ID_COLOR[f]
        L.append(f'<link name="pad_{f}"><pose>{fx} {fy} 0.006 0 0 0</pose>'
                 f'{cyl_visual("v", "0 0 0 0 0 0", 0.6, 0.01, "0.15 0.15 0.18 1")}'
                 f'{cyl_visual("ring", "0 0 0.004 0 0 0", 0.62, 0.004, c, c)}'
                 f'{box_visual("h1", "0 0.18 0.006 0 0 0", "0.08 0.04 0.002", "1 1 1 1")}'
                 f'{box_visual("h2", "0 -0.18 0.006 0 0 0", "0.08 0.04 0.002", "1 1 1 1")}'
                 f'{box_visual("h3", "0 0 0.006 0 0 0", "0.3 0.035 0.002", "1 1 1 1")}</link>')
    infra = f'<model name="outside_infrastructure"><static>true</static>{"".join(L)}</model>'
    onas = ''.join(ona_site(k) for k in C.ONAS)
    # LTE backup mast (cell tower) between the ONAs and the Command Post
    tower = ('<model name="lte_mast"><static>true</static><pose>-38 18 0 0 0 0.4</pose><link name="l">'
             '<visual name="v"><geometry><mesh><uri>model://radio_tower/meshes/radio_tower.obj</uri>'
             '<scale>0.45 0.45 0.45</scale></mesh></geometry></visual></link></model>')
    sx, sy, sz = C.SAT_POS
    az, _ = sat_view(-20.0, 0.0, 0.0)
    sat = (f'<model name="relay_satellite"><static>true</static><pose>{sx} {sy} {sz} 0 0 {az:.3f}</pose>'
           f'<link name="l"><visual name="v"><geometry><mesh><uri>model://lm_tdrs/meshes/tdrs.obj</uri>'
           f'</mesh></geometry><cast_shadows>false</cast_shadows></visual></link></model>')
    return infra + onas + tower + sat


def water():
    """Flooded sump on line C. Seen by the cameras AND by the Flix downward ToF (it flies low
    over the surface); no collision."""
    return (f'<model name="flooded_sump"><static>true</static><link name="l">'
             f'<pose>{sum(C.WATER_X) / 2:.2f} -60 {(C.WATER_Z - 5.3) / 2:.2f} 0 0 0</pose><visual name="v">'
             f'<geometry><box><size>{C.WATER_X[1] - C.WATER_X[0] + 2.0:.2f} 6 {C.WATER_Z + 5.3:.2f}</size>'
             f'</box></geometry>'
             f'<visibility_flags>{DECOR_FLAGS | LIDAR_MASK}</visibility_flags>'
             f'<material><ambient>0.05 0.2 0.3 0.5</ambient><diffuse>0.08 0.3 0.4 0.5</diffuse>'
             f'<specular>0.6 0.6 0.6 1</specular></material><transparency>0.55</transparency>'
             f'<cast_shadows>false</cast_shadows></visual></link></model>')


def victims(fuel_ok):
    out = []
    for v in C.VICTIMS:
        k = v['id'][1:]
        if fuel_ok:
            out.append(include(f'victim_{k}', fuel('Rescue Randy Sitting'),
                               f'{v["x"]} {v["y"]} {v["z"]} 0 0 {v["yaw"]}', True))
        else:
            body = [('torso', '0 0.1 0.13 0 0 0', '0.36 0.55 0.24', '1 0.45 0.05 1'),
                    ('legs', '0 -0.55 0.1 0 0 0', '0.32 0.8 0.18', '0.15 0.2 0.35 1')]
            links = ''.join(box_link(n, p, sz, c) for n, p, sz, c in body)
            links += (f'<link name="head"><pose>0 0.52 0.13 0 0 0</pose>'
                      f'{cyl_visual("helmet", "0 0.03 0.06 0 0 0", 0.13, 0.1, "1 0.9 0 1", "0.3 0.25 0 1")}'
                      f'<visual name="v"><geometry><sphere><radius>0.12</radius></sphere></geometry>'
                      f'{mat("0.85 0.65 0.5 1")}</visual></link>')
            out.append(f'<model name="victim_{k}"><static>true</static>'
                       f'<pose>{v["x"]} {v["y"]} 0 0 0 {v["yaw"] - 1.5708}</pose>{links}</model>')
    # rubble pinning miner 1 (south branch) and miner 3 (east gallery)
    rng = random.Random(5)
    for v in (C.VICTIMS[0], C.VICTIMS[2]):
        rub = ''
        for i in range(7):
            a = rng.uniform(0, 2 * math.pi)
            r = rng.uniform(0.3, 1.1)
            sc = rng.uniform(0.8, 1.6)
            rub += (f'<visual name="r{i}"><pose>{v["x"] + r * math.cos(a) - 0.4:.2f} {v["y"] + r * math.sin(a):.2f} '
                    f'{0.08 * sc:.2f} {rng.uniform(0, 3):.2f} {rng.uniform(0, 3):.2f} {a:.2f}</pose><geometry><mesh>'
                    f'<uri>model://lm_rock/meshes/rock.obj</uri><scale>{sc:.2f} {sc:.2f} {sc:.2f}</scale></mesh>'
                    f'</geometry></visual>')
        out.append(f'<model name="rubble_{v["id"][1:]}"><static>true</static><link name="l">{rub}</link></model>')
    return ''.join(out)


def gas():
    """Methane leak: thin smoke layered under the roof (CH4 is lighter than air). The emitter
    is pitched so particles rise; it stays well above both robot lidar planes. Fortress ignores
    the SDF particle_scatter_ratio, so the robots' lidar processing also filters dust/smoke."""
    g = C.GAS_LEAK
    return (f'<model name="gas_leak"><static>true</static><pose>{g["x"]} {g["y"]} 1.35 0 -1.5708 0</pose>'
            f'<link name="l">'
            f'<particle_emitter name="methane" type="box"><emitting>true</emitting>'
            f'<size>0.3 5.0 3.0</size><particle_size>0.6 0.6 0.6</particle_size><lifetime>14</lifetime>'
            f'<rate>6</rate><min_velocity>0.02</min_velocity><max_velocity>0.06</max_velocity>'
            f'<scale_rate>0.15</scale_rate><particle_scatter_ratio>0.0</particle_scatter_ratio>'
            f'<material><diffuse>0.8 0.85 0.4</diffuse><specular>0.1 0.1 0.1</specular>'
            f'<pbr><metal><albedo_map>model://lm_gas/materials/textures/gas.png</albedo_map></metal></pbr>'
            f'</material><color_range_image>model://lm_gas/materials/textures/gas_colors.png'
            f'</color_range_image></particle_emitter></link></model>')


def lamps():
    pts = [(8, 0), (30, 0), (58, 0), (90, 0), (122, 0), (50, 14), (70, -12), (70, -24), (70, 22),
           (15, 40), (45, 40), (70, -60), (1.5, -30)]
    out = []
    for i, (x, y) in enumerate(pts):
        out.append(f'<light type="point" name="lamp{i}"><pose>{x} {y} 3.2 0 0 0</pose>'
                   f'<diffuse>1 0.72 0.42 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
                   f'<attenuation><range>14</range><constant>0.4</constant><linear>0.05</linear>'
                   f'<quadratic>0.008</quadratic></attenuation><cast_shadows>false</cast_shadows>'
                   f'<visualize>false</visualize></light>')
    for i, x in enumerate((20, 42)):             # dim blue-green light in the flooded sump
        out.append(f'<light type="point" name="sump{i}"><pose>{x} -60 {C.c_floor_z(x) + 1.2:.2f} 0 0 0</pose>'
                   f'<diffuse>0.3 0.7 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
                   f'<attenuation><range>11</range><constant>0.6</constant><linear>0.08</linear>'
                   f'<quadratic>0.02</quadratic></attenuation><cast_shadows>false</cast_shadows>'
                   f'<visualize>false</visualize></light>')
    return ''.join(out)


# ================================================================== GUI + world
def gui_plugin(fname, name, props, body=''):
    p = ''.join(f'<property key="{k}" type="{t}">{v}</property>' for k, t, v in props)
    return f'<plugin filename="{fname}" name="{name}"><ignition-gui>{p}</ignition-gui>{body}</plugin>'


FLOAT = [('resizable', 'bool', 'false'), ('width', 'double', '5'), ('height', 'double', '5'),
         ('state', 'string', 'floating'), ('showTitleBar', 'bool', 'false')]

# 3D view: Fortress's GzScene3D (not MinimalScene + CameraTracking): it lets the world set the
# camera-follow gain. CameraTracking's gain is fixed at 0.01 per rendered frame, which makes a
# followed robot drift away and the camera crawl after it in small steps.
FOLLOW_P_GAIN = 0.08
GUI = ('<gui fullscreen="0">'
       + gui_plugin('GzScene3D', '3D View',
                    [('showTitleBar', 'bool', 'false'), ('state', 'string', 'docked')],
                    '<engine>ogre2</engine><scene>scene</scene><ambient_light>0.4 0.4 0.4</ambient_light>'
                    '<background_color>0.62 0.72 0.82</background_color>'
                    '<camera_pose>-42 -24 16 0 0.28 0.42</camera_pose>'
                    f'<camera_follow><p_gain>{FOLLOW_P_GAIN}</p_gain><world_frame>false</world_frame>'
                    '</camera_follow>')
       + '<plugin filename="WorldControl" name="World control"><ignition-gui><title>World control</title>'
         '<property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property>'
         '<property type="double" key="height">72</property><property type="double" key="width">121</property>'
         '<property type="double" key="z">1</property><property type="string" key="state">floating</property>'
         '<anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors>'
         '</ignition-gui><play_pause>true</play_pause><step>true</step><start_paused>false</start_paused>'
         '<use_event>true</use_event></plugin>'
       + '<plugin filename="WorldStats" name="World stats"><ignition-gui><title>World stats</title>'
         '<property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property>'
         '<property type="double" key="height">110</property><property type="double" key="width">290</property>'
         '<property type="double" key="z">1</property><property type="string" key="state">floating</property>'
         '<anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors>'
         '</ignition-gui><sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor>'
         '<iterations>true</iterations></plugin>'
       + '</gui>')


def world(fuel_ok):
    mine = (tiles() + blockers()) if fuel_ok else basic_galleries()
    kind = 'SubT Fuel tunnel tiles' if fuel_ok else 'offline fallback galleries (no Fuel)'
    return f'''<?xml version="1.0" ?>
<!-- GENERATED by tools/gen_world.py from living_map/config.py ({kind}) - edit the generator -->
<sdf version="1.9">
  <world name="{C.WORLD_NAME}">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    {plugin('physics', 'Physics')}
    {plugin('user-commands', 'UserCommands')}
    {plugin('scene-broadcaster', 'SceneBroadcaster')}
    {plugin('sensors', 'Sensors', '<render_engine>ogre2</render_engine>')}
    {plugin('imu', 'Imu')}
    {plugin('particle-emitter2', 'ParticleEmitter2')}
    {GUI}
    <scene>
      <ambient>0.35 0.35 0.35 1</ambient>
      <background>0.62 0.72 0.82 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 60 0 0 0</pose>
      <diffuse>0.95 0.92 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>0.45 0.25 -0.85</direction>
      <visualize>false</visualize>
    </light>
    {lamps()}
    {terrain_and_hill()}
    {mine}
    {portal_frames()}
    {collapsed_adit(fuel_ok)}
    {outside_infra()}
    {water()}
    {victims(fuel_ok)}
    {gas()}
    {subt_robot('writer_a', C.SPAWN['writer_a'])}
    {subt_robot('writer_b', C.SPAWN['writer_b'])}
    {subt_robot('executor', C.SPAWN['executor'])}
    {flix('flix_a')}
    {flix('flix_b')}
  </world>
</sdf>
'''


# ================================================================== spawnable models
# same roles / colours as the 2D window: relay = neutral, events = status colours
BEACON_COLORS = {'entrance': '0.95 0.95 0.9 1', 'trail': '0.76 0.76 0.72 1',
                 'victim': '0.82 0.23 0.23 1', 'gas': '0.98 0.7 0.1 1',
                 'junction': '0.55 0.35 0.95 1'}


def beacon_model(kind):
    """ESP32 relay beacon puck (dropped by the Writer). collide_bitmask 0x02: robots drive
    over it; walls, floor and debris still collide."""
    c = BEACON_COLORS[kind]
    extra = ''
    if kind == 'entrance':
        extra = cyl_visual('spool', '-0.12 0 0.02 0 0 0', 0.05, 0.06, '1 0.45 0', '0.5 0.2 0 1')
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="beacon_{kind}">
    <link name="body">
      <pose>0 0 0.035 0 0 0</pose>
      <inertial><mass>0.25</mass><inertia><ixx>0.0006</ixx><ixy>0</ixy><ixz>0</ixz>
        <iyy>0.0006</iyy><iyz>0</iyz><izz>0.001</izz></inertia></inertial>
      <collision name="c">
        <geometry><cylinder><radius>0.09</radius><length>0.07</length></cylinder></geometry>
        <surface><contact><collide_bitmask>0x02</collide_bitmask></contact></surface>
      </collision>
      {cyl_visual('puck', '0 0 0 0 0 0', 0.09, 0.07, '0.12 0.12 0.14 1')}
      {cyl_visual('ring', '0 0 0.02 0 0 0', 0.094, 0.014, c, c)}
      <visual name="led"><pose>0 0 0.05 0 0 0</pose><geometry><sphere><radius>0.03</radius></sphere>
        </geometry>{mat(c, c)}</visual>
      {cyl_visual('antenna', '0.05 0 0.11 0 0 0', 0.006, 0.16, '0.05 0.05 0.05 1')}
      {extra}
    </link>
  </model>
</sdf>
'''


def csi_node_model():
    """ESP32-S3 Wi-Fi CSI sensing node dropped by a slow Writer (CSI receiver + battery, cyan
    LED, small antenna). collide_bitmask 0x02: robots drive over it."""
    c = '0.1 0.8 0.9 1'
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="csi_node">
    <link name="body">
      <pose>0 0 0.02 0 0 0</pose>
      <inertial><mass>0.06</mass><inertia><ixx>1e-5</ixx><ixy>0</ixy><ixz>0</ixz>
        <iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz></inertia></inertial>
      <collision name="c"><geometry><box><size>0.07 0.05 0.04</size></box></geometry>
        <surface><contact><collide_bitmask>0x02</collide_bitmask></contact></surface></collision>
      {box_visual('case', '0 0 0 0 0 0', '0.07 0.05 0.04', '0.12 0.12 0.14 1')}
      {box_visual('band', '0 0 0.021 0 0 0', '0.072 0.052 0.004', c, c)}
      <visual name="led"><pose>0.02 0 0.026 0 0 0</pose><geometry><sphere><radius>0.008</radius></sphere>
        </geometry>{mat(c, c)}</visual>
      {cyl_visual('antenna', '-0.025 0 0.07 0 0 0', 0.004, 0.1, '0.05 0.05 0.05 1')}
    </link>
  </model>
</sdf>
'''


def beam_model(name, legs, rgba):
    """Satellite link flash: thin glowing cylinders dish -> satellite (-> Command Post dish).
    Spawned for ~1 s by gz_actions when a packet goes up or down."""
    vis = ''
    for i, (a, b) in enumerate(legs):
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        L = math.sqrt(dx * dx + dy * dy + dz * dz)
        pitch = math.acos(max(-1.0, min(1.0, dz / L)))
        yaw = math.atan2(dy, dx)
        vis += (f'<visual name="leg{i}"><pose>{(a[0] + b[0]) / 2:.3f} {(a[1] + b[1]) / 2:.3f} '
                f'{(a[2] + b[2]) / 2:.3f} 0 {pitch:.4f} {yaw:.4f}</pose><geometry><cylinder>'
                f'<radius>0.12</radius><length>{L:.3f}</length></cylinder></geometry>{mat(rgba, rgba)}'
                f'<transparency>0.35</transparency><cast_shadows>false</cast_shadows></visual>')
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}"><static>true</static><link name="l">{vis}</link></model>
</sdf>
'''


def pulse_model(name, r, rgba):
    """Expanding Wi-Fi CSI pulse (presence detected), drawn as three rings spawned one after the
    other."""
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}"><static>true</static><link name="l">
    <visual name="v"><geometry><cylinder><radius>{r}</radius><length>0.03</length></cylinder></geometry>
      {mat(rgba, rgba)}<transparency>0.55</transparency><cast_shadows>false</cast_shadows></visual>
    <visual name="core"><geometry><sphere><radius>{min(0.25, r / 6):.2f}</radius></sphere></geometry>
      {mat(rgba, rgba)}<cast_shadows>false</cast_shadows></visual>
  </link></model>
</sdf>
'''


def rock_model(name, scale, mass, bitmask):
    s = scale
    col = f'<surface><contact><collide_bitmask>{bitmask}</collide_bitmask></contact></surface>' if bitmask else ''
    i = mass * (0.4 * s) ** 2 / 6
    return f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <link name="l">
      <inertial><mass>{mass}</mass><inertia><ixx>{i:.4f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{i:.4f}</iyy>
        <iyz>0</iyz><izz>{i:.4f}</izz></inertia></inertial>
      <collision name="c"><geometry><box><size>{0.42 * s:.3f} {0.34 * s:.3f} {0.26 * s:.3f}</size></box>
        </geometry>{col}</collision>
      <visual name="v"><geometry><mesh><uri>model://lm_rock/meshes/rock.obj</uri>
        <scale>{s} {s} {s}</scale></mesh></geometry></visual>
    </link>
  </model>
</sdf>
'''


def main():
    # local assets
    build_hillside(os.path.join(MODELS, 'lm_hillside'))
    build_terrain(os.path.join(MODELS, 'lm_terrain'))
    build_rock(os.path.join(MODELS, 'lm_rock'))
    gas_texture(os.path.join(MODELS, 'lm_gas', 'materials', 'textures', 'gas.png'))
    gas_colors(os.path.join(MODELS, 'lm_gas', 'materials', 'textures', 'gas_colors.png'))
    for d, desc in (('lm_hillside', 'Hillside with the three mine portals (generated)'),
                    ('lm_terrain', 'Outside terrain (generated)'),
                    ('lm_rock', 'Low-poly rock (generated)'),
                    ('lm_gas', 'Methane particle textures (generated)'),
                    ('lm_x2', 'Robotika X2 meshes - DARPA SubT, Apache-2.0'),
                    ('lm_x1', 'Explorer X1 meshes - DARPA SubT, Apache-2.0'),
                    ('lm_flix', 'Flix quadcopter frame - github.com/okalachev/flix'),
                    ('lm_dish', 'NASA 3D Resources "Tall Dish" (github.com/nasa/NASA-3D-Resources), '
                                'converted glb -> obj, Z-up, scaled'),
                    ('lm_tdrs', 'NASA 3D Resources "Tracking and Data Relay Satellite (TDRS) B" '
                                '(github.com/nasa/NASA-3D-Resources), converted glb -> obj')):
        write(os.path.join(MODELS, d, 'model.config'), model_config(d, desc))
    write(os.path.join(ROOT, 'worlds', 'mine.sdf'), world(True))
    write(os.path.join(ROOT, 'worlds', 'mine_basic.sdf'), world(False))
    for k in BEACON_COLORS:
        write(os.path.join(MODELS, f'beacon_{k}.sdf'), beacon_model(k))
    write(os.path.join(MODELS, 'csi_node.sdf'), csi_node_model())
    # satellite link flashes: ONA dish -> satellite -> Command Post dish
    sat = C.SAT_POS
    cx, cy = C.CP_POS
    cp_dish = (cx + 4.5, cy - 2.0, DISH_PIVOT + 0.6)
    for k in C.ONAS:
        d = dish_xyz(*C.ONAS[k]['pos'])
        d = (d[0], d[1], d[2] + 0.6)
        write(os.path.join(MODELS, f'beam_{k}.sdf'), beam_model(f'beam_{k}', [(d, sat), (sat, cp_dish)],
                                                             '0.3 0.85 1 1'))
    for i, r in enumerate((1.5, 3.5, 6.0)):
        write(os.path.join(MODELS, f'pulse_csi_{i}.sdf'), pulse_model(f'pulse_csi_{i}', r, '0.1 0.8 0.9 1'))
    # roof-collapse rocks hit the Writer; rockfall debris on a relay beacon (0x02) never hits
    # robots and stays below the lidar planes so it cannot wall off the gallery
    write(os.path.join(MODELS, 'rock.sdf'), rock_model('rock', 1.0, 12, None))
    write(os.path.join(MODELS, 'rock_small.sdf'), rock_model('rock_small', 0.55, 5, '0x02'))
    print('wrote worlds/mine.sdf, worlds/mine_basic.sdf and models/*')


if __name__ == '__main__':
    main()

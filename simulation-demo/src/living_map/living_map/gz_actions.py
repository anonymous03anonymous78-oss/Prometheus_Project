"""Executes Gazebo world edits (spawn / remove / set_pose) requested by the
mission nodes, through the Fortress `ign service` CLI (UserCommands system).
Runs the calls in a worker thread so ROS callbacks never block.

Effects: 'beam' flashes the satellite link of an Outside Network Area (dish -> relay satellite
-> Command Post dish) for ~1 s; 'pulse' draws an expanding Wi-Fi CSI pulse (three cyan rings)
where an aggregator beacon's classifier reports human presence."""
import math
import os
import queue
import shutil
import subprocess
import threading
import time

from . import config as C
from .common import LMNode, spin_node


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('living_map')
    except Exception:
        return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def pose_txt(m):
    qx, qy, qz, qw = quat(m.get('roll', 0.0), m.get('pitch', 0.0), m.get('yaw', 0.0))
    return (f"position: {{x: {m['x']:.4f}, y: {m['y']:.4f}, z: {m.get('z', 0.0):.4f}}}, "
            f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}")


class GzActions(LMNode):
    def __init__(self):
        super().__init__('gz_actions')
        self.declare_parameter('dry_run', False)
        self.dry = bool(self.get_parameter('dry_run').value)
        self.models = os.path.join(share_dir(), 'models')
        self.ign = shutil.which('ign')
        if not self.dry and self.ign is None:
            self.get_logger().error("'ign' CLI not found - Gazebo world edits disabled")
        self.q = queue.Queue()
        self.n_fx = 0
        self.pending_pose = {}
        self.lock = threading.Lock()
        self.sub('/lm/gz/cmd', self.on_cmd)
        threading.Thread(target=self.worker, daemon=True).start()

    def on_cmd(self, m):
        if m['op'] in ('beam', 'pulse'):
            if not self.dry and self.ign is not None:
                threading.Thread(target=self.effect, args=(m,), daemon=True).start()
            return
        if m['op'] == 'set_pose':
            with self.lock:
                fresh = m['name'] not in self.pending_pose
                self.pending_pose[m['name']] = m
            if fresh:
                self.q.put(('set_pose', m['name']))
        else:
            self.q.put((m['op'], m))

    def call(self, service, reqtype, req):
        if self.dry or self.ign is None:
            return True
        cmd = [self.ign, 'service', '-s', f'/world/{C.WORLD_NAME}/{service}',
               '--reqtype', reqtype, '--reptype', 'ignition.msgs.Boolean',
               '--timeout', '3000', '--req', req]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
            ok = r.returncode == 0 and 'true' in r.stdout
            if not ok:
                self.get_logger().warn(f'{service} failed: {r.stdout.strip()} {r.stderr.strip()}')
            return ok
        except Exception as e:
            self.get_logger().warn(f'{service} error: {e}')
            return False

    def spawn(self, model, name, x, y, z, yaw=0.0):
        path = os.path.join(self.models, model + '.sdf')
        self.call('create', 'ignition.msgs.EntityFactory',
                  f'sdf_filename: "{path}", name: "{name}", allow_renaming: false, '
                  f'pose: {{{pose_txt({"x": x, "y": y, "z": z, "yaw": yaw})}}}')

    def remove(self, name):
        self.call('remove', 'ignition.msgs.Entity', f'name: "{name}", type: MODEL')

    def effect(self, m):
        self.n_fx += 1
        n = self.n_fx
        if m['op'] == 'beam':
            name = f'fx_beam_{m["ona"]}_{n}'
            self.spawn(f'beam_{m["ona"]}', name, 0.0, 0.0, 0.0)
            time.sleep(1.0)
            self.remove(name)
            return
        kind = 'csi'
        names = []
        for i in range(3):
            nm = f'fx_{kind}_{n}_{i}'
            self.spawn(f'pulse_{kind}_{i}', nm, m['x'], m['y'], m.get('z', 0.3))
            names.append(nm)
            time.sleep(0.35)
        if m.get('to'):
            tx, ty, tz = m['to']
            nm = f'fx_{kind}_{n}_rx'
            self.spawn(f'pulse_{kind}_0', nm, tx, ty, tz)
            names.append(nm)
        time.sleep(0.6)
        for nm in names:
            self.remove(nm)

    def worker(self):
        while True:
            op, m = self.q.get()
            if op == 'set_pose':
                with self.lock:
                    m = self.pending_pose.pop(m, None)
                if m is None:
                    continue
                self.call('set_pose', 'ignition.msgs.Pose', f'name: "{m["name"]}", ' + pose_txt(m))
            elif op == 'spawn':
                path = os.path.join(self.models, m['model'] + '.sdf')
                self.call('create', 'ignition.msgs.EntityFactory',
                          f'sdf_filename: "{path}", name: "{m["name"]}", '
                          f'allow_renaming: false, pose: {{{pose_txt(m)}}}')
            elif op == 'remove':
                self.call('remove', 'ignition.msgs.Entity', f'name: "{m["name"]}", type: MODEL')


def main(args=None):
    spin_node(GzActions, args)


if __name__ == '__main__':
    main()

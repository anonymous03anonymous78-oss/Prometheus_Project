"""Shared helpers for all Living Map nodes."""
import json
import math

from rclpy.node import Node
from std_msgs.msg import String

from . import config as C

QOS_DEPTH = 100


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomTracker:
    """Converts an odometry stream into a pose in a chosen frame.

    Works whether the odometry is relative to the start (DiffDrive) or already
    in world coordinates (OdometryPublisher): the first message is taken as the
    reference and mapped onto `origin` (x, y, z, yaw).
    """

    def __init__(self, origin=(0.0, 0.0, 0.0, 0.0)):
        self.origin = origin
        self.ref = None
        self.x = origin[0]
        self.y = origin[1]
        self.z = origin[2]
        self.yaw = origin[3]
        self.ok = False

    def update(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.ref is None:
            self.ref = (p.x, p.y, p.z, yaw)
        rx, ry, rz, ryaw = self.ref
        dx, dy = p.x - rx, p.y - ry
        rot = self.origin[3] - ryaw
        c, s = math.cos(rot), math.sin(rot)
        self.x = self.origin[0] + c * dx - s * dy
        self.y = self.origin[1] + s * dx + c * dy
        self.z = self.origin[2] + (p.z - rz)
        self.yaw = wrap(self.origin[3] + (yaw - ryaw))
        self.ok = True


class LMNode(Node):
    """Base node: JSON-over-String pub/sub, sim-time scheduler, scenario flags."""

    def __init__(self, name, publishes=()):
        super().__init__(name)
        self.declare_parameter('scenario', 'nominal')
        self.declare_parameter('robot', '')
        self.declare_parameter('ona', '')
        self.scenario = str(self.get_parameter('scenario').value)
        if self.scenario not in C.SCENARIOS:
            self.get_logger().warn(f'unknown scenario {self.scenario!r}, using nominal')
            self.scenario = 'nominal'
        self.flags = C.scenario_flags(self.scenario)
        self._pubs = {}
        for t in tuple(publishes) + ('/lm/flow',):
            self._declare_pub(t)
        self._sched = []
        self._seq = 0
        self.create_timer(0.05, self._tick_sched)

    # ---- time
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def now_ms(self):
        return int(self.now() * 1000.0)

    # ---- pub/sub
    def _declare_pub(self, topic):
        if topic not in self._pubs:
            self._pubs[topic] = self.create_publisher(String, topic, QOS_DEPTH)
        return self._pubs[topic]

    def pub(self, topic, obj):
        p = self._pubs.get(topic) or self._declare_pub(topic)
        m = String()
        m.data = json.dumps(obj)
        p.publish(m)

    def sub(self, topic, cb):
        def _cb(msg):
            try:
                cb(json.loads(msg.data))
            except Exception as e:  # keep the node alive whatever happens
                self.get_logger().error(f'{topic}: {type(e).__name__}: {e}')
        return self.create_subscription(String, topic, _cb, QOS_DEPTH)

    def flow(self, kind, **kw):
        kw['kind'] = kind
        kw['t'] = round(self.now(), 3)
        self.pub('/lm/flow', kw)

    # ---- scheduler (sim time)
    def after(self, delay, fn):
        self._seq += 1
        self._sched.append((self.now() + delay, self._seq, fn))

    def _tick_sched(self):
        if not self._sched:
            return
        t = self.now()
        due = [s for s in self._sched if s[0] <= t]
        if not due:
            return
        self._sched = [s for s in self._sched if s[0] > t]
        for s in sorted(due, key=lambda s: (s[0], s[1])):
            try:
                s[2]()
            except Exception as e:
                self.get_logger().error(f'scheduled task failed: {type(e).__name__}: {e}')

    def utc(self):
        """Absolute UTC time of the mission clock (seconds)."""
        return C.MISSION_EPOCH_UTC + self.now()

    def started(self):
        """True once the mission start delay has elapsed in sim time."""
        return self.now() >= C.MISSION_START_DELAY


def spin_node(cls, args=None):
    import rclpy
    rclpy.init(args=args)
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

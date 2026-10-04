"""Minimal stand-in for rclpy + message types, used by dry_run.py.

It implements only the subset of the rclpy API the Living Map nodes use, on a
single simulated clock, so the complete mission logic can run (and be tested)
without ROS 2 or Gazebo installed. On a machine WITH ROS this module is only
used when you explicitly run the dry run.
"""
import sys
import types
from collections import deque

STATE = {'t_ns': 0, 'params': {}, 'quiet': False}


class Bus:
    def __init__(self):
        self.subs = {}
        self.queue = deque()
        self.timers = []

    def publish(self, topic, msg):
        for cb in self.subs.get(topic, ()):
            self.queue.append((cb, msg))

    def drain(self, limit=200000):
        n = 0
        while self.queue and n < limit:
            cb, msg = self.queue.popleft()
            cb(msg)
            n += 1


BUS = Bus()


# ------------------------------------------------------------------ messages
class _Msg:
    def __repr__(self):
        return f'{type(self).__name__}({self.__dict__})'


class Vector3(_Msg):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Quaternion(_Msg):
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.w = 1.0


class Twist(_Msg):
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class String(_Msg):
    def __init__(self, data=''):
        self.data = data


class Bool(_Msg):
    def __init__(self, data=False):
        self.data = data


class _Pose(_Msg):
    def __init__(self):
        self.position = Vector3()
        self.orientation = Quaternion()


class _PoseCov(_Msg):
    def __init__(self):
        self.pose = _Pose()


class _TwistCov(_Msg):
    def __init__(self):
        self.twist = Twist()


class Odometry(_Msg):
    def __init__(self):
        self.pose = _PoseCov()
        self.twist = _TwistCov()


class Imu(_Msg):
    def __init__(self):
        self.orientation = Quaternion()
        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()


class LaserScan(_Msg):
    def __init__(self):
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges = []


# ------------------------------------------------------------------ rclpy API
class _Time:
    def __init__(self, ns):
        self.nanoseconds = ns


class _Clock:
    def now(self):
        return _Time(STATE['t_ns'])


class _Logger:
    def __init__(self, name):
        self.name = name

    def _p(self, lvl, msg):
        if STATE['quiet'] and lvl == 'INFO':
            return
        print(f"[{STATE['t_ns'] / 1e9:7.2f}s] [{lvl}] [{self.name}] {msg}", flush=True)

    def info(self, m):
        self._p('INFO', m)

    def warn(self, m):
        self._p('WARN', m)

    warning = warn

    def error(self, m):
        self._p('ERROR', m)


class _Param:
    def __init__(self, v):
        self.value = v


class _Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        BUS.publish(self.topic, msg)


class _Timer:
    def __init__(self, period, cb):
        self.period = period
        self.cb = cb
        self.next = STATE['t_ns'] / 1e9 + period


class Node:
    def __init__(self, name):
        # like a ROS launch file giving each instance its own node name
        pr = STATE['params']
        if pr.get('robot'):
            name = pr['robot']
        elif pr.get('ona'):
            name = f"ona_{pr['ona'].lower()}"
        self._name = name
        self._params = {}
        self._logger = _Logger(name)

    def get_name(self):
        return self._name

    # rclpy.node.Node exposes these as properties; mirror them so tests catch
    # accidental attribute collisions (e.g. `self.executor = ...` breaks real rclpy).
    @property
    def executor(self):
        return None

    @executor.setter
    def executor(self, value):
        if value is not None and not hasattr(value, 'add_node'):
            raise AttributeError("'executor' is a reserved rclpy Node property")

    publishers = property(lambda self: [])
    subscriptions = property(lambda self: [])
    timers = property(lambda self: [])
    clients = property(lambda self: [])
    services = property(lambda self: [])
    guards = property(lambda self: [])
    waitables = property(lambda self: [])
    handle = property(lambda self: None)
    context = property(lambda self: None)

    def declare_parameter(self, name, default=None):
        self._params[name] = STATE['params'].get(name, default)

    def get_parameter(self, name):
        return _Param(self._params.get(name))

    def create_publisher(self, _type, topic, _qos):
        return _Publisher(topic)

    def create_subscription(self, _type, topic, cb, _qos):
        BUS.subs.setdefault(topic, []).append(cb)
        return cb

    def create_timer(self, period, cb):
        t = _Timer(period, cb)
        BUS.timers.append(t)
        return t

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


def fire_timers():
    t = STATE['t_ns'] / 1e9
    for tm in list(BUS.timers):
        if t + 1e-9 >= tm.next:
            tm.next += tm.period
            if tm.next < t:
                tm.next = t + tm.period
            tm.cb()


def install(params=None, quiet=False):
    STATE['params'] = dict(params or {})
    STATE['quiet'] = quiet

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    rclpy = mod('rclpy', init=lambda args=None: None, shutdown=lambda: None,
                spin=lambda node: None, ok=lambda: True)
    rclpy.node = mod('rclpy.node', Node=Node)
    std = mod('std_msgs')
    std.msg = mod('std_msgs.msg', String=String, Bool=Bool)
    geo = mod('geometry_msgs')
    geo.msg = mod('geometry_msgs.msg', Twist=Twist, Vector3=Vector3)
    nav = mod('nav_msgs')
    nav.msg = mod('nav_msgs.msg', Odometry=Odometry)
    sen = mod('sensor_msgs')
    sen.msg = mod('sensor_msgs.msg', LaserScan=LaserScan, Imu=Imu)

"""2D dataflow window: a tiny HTTP + Server-Sent-Events server that streams
every Living Map topic to a browser page (web/index.html). Standard library only.

It also records the run to ~/living_map_runs/<scenario>_<date>.jsonl.gz (all mission
messages; robot poses at 1 Hz) so a Gazebo run can be replayed / debugged afterwards."""
import gzip
import json
import os
import time
import queue
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from std_msgs.msg import String

from . import config as C
from .common import LMNode, spin_node

TOPICS = (['/lm/flow', '/lm/beacons', '/lm/cp/state', '/lm/ona/state', '/lm/scenario', '/lm/camera']
          + [f'/lm/pose/{n}' for n in C.GROUND + C.FLIXES]
          + [f'/lm/sensors/{n}' for n in C.GROUND])


def web_dir():
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')
    if os.path.isfile(os.path.join(here, 'index.html')):
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('living_map'), 'web')
    except Exception:
        return here


class Hub:
    def __init__(self):
        self.clients = set()
        self.lock = threading.Lock()
        self.latest = {}
        self.flows = deque(maxlen=400)

    def publish(self, topic, raw):
        line = '{"topic":%s,"data":%s}' % (json.dumps(topic), raw)
        with self.lock:
            if topic == '/lm/flow':
                self.flows.append(line)
            elif topic == '/lm/ona/state':
                # three ONAs publish on one topic: keep the latest of each for new viewers
                k = raw.find('"ona": "')
                self.latest[topic + (raw[k + 8] if k >= 0 else '')] = line
            else:
                self.latest[topic] = line
            for q in list(self.clients):
                try:
                    q.put_nowait(line)
                except queue.Full:
                    pass

    def attach(self):
        q = queue.Queue(maxsize=5000)
        with self.lock:
            for line in self.latest.values():
                q.put_nowait(line)
            for line in list(self.flows)[-120:]:
                q.put_nowait(line.replace('"topic":"/lm/flow"', '"topic":"/lm/flow_replay"', 1))
            self.clients.add(q)
        return q

    def detach(self, q):
        with self.lock:
            self.clients.discard(q)


def make_handler(hub, scenario):
    wd = web_dir()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path in ('/', '/index.html'):
                with open(os.path.join(wd, 'index.html'), 'rb') as f:
                    self._send(200, 'text/html; charset=utf-8', f.read())
            elif path == '/config':
                cfg = C.as_dict()
                cfg['scenario'] = scenario
                self._send(200, 'application/json', json.dumps(cfg).encode())
            elif path == '/events':
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                q = hub.attach()
                try:
                    while True:
                        try:
                            line = q.get(timeout=10)
                            self.wfile.write(b'data: ' + line.encode() + b'\n\n')
                        except queue.Empty:
                            self.wfile.write(b': ping\n\n')
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    hub.detach(q)
            else:
                self._send(404, 'text/plain', b'not found')

    return H


class DataflowView(LMNode):
    def __init__(self):
        super().__init__('dataflow_view')
        self.declare_parameter('port', C.WEB_PORT)
        port = int(self.get_parameter('port').value)
        self.hub = Hub()
        for t in TOPICS:
            self.create_subscription(String, t, lambda msg, t=t: self.on_msg(t, msg.data), 200)
        self.rec = None
        self._rec_last = {}
        try:
            d = os.path.expanduser('~/living_map_runs')
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, f'{self.scenario}_{time.strftime("%Y%m%d_%H%M%S")}.jsonl.gz')
            self.rec = gzip.open(path, 'wt')
            self.get_logger().info(f'recording this run to {path}')
        except Exception as e:
            self.get_logger().warn(f'run recording disabled: {e}')
        self.server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(self.hub, self.scenario))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'2D dataflow window at http://localhost:{port}')


    def on_msg(self, topic, raw):
        self.hub.publish(topic, raw)
        if self.rec is None:
            return
        now = self.now()
        if topic.startswith(('/lm/pose/', '/lm/sensors/')) or topic in ('/lm/beacons', '/lm/camera',
                                                                         '/lm/ona/state'):
            key = topic + (raw[raw.find('"ona": "') + 8] if topic == '/lm/ona/state' else '')
            if now - self._rec_last.get(key, -1e9) < 1.0:
                return
            self._rec_last[key] = now
        try:
            self.rec.write('{"t":%.2f,"topic":%s,"data":%s}\n' % (now, json.dumps(topic), raw))
            if now - self._rec_last.get('_flush', 0.0) > 5.0:
                self._rec_last['_flush'] = now
                self.rec.flush()
        except Exception:
            self.rec = None


def main(args=None):
    spin_node(DataflowView, args)


if __name__ == '__main__':
    main()

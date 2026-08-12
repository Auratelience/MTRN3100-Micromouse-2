"""Stdlib HTTP server streaming simulation frames to a browser canvas.

Server-Sent Events rather than WebSockets because SSE needs no handshake, no
framing and no dependency -- the whole transport is "write `data: {...}` and
flush". The sim runs on its own daemon thread; request handlers only ever read
snapshots taken under a lock.
"""

import json
import pathlib
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from ..constants import AXLE_LEN, MAZE_CELL_SIZE
from ..lidar import L
from ..plant import LIDAR_MOUNTS

_INDEX = pathlib.Path(__file__).with_name("index.html")

# Frames pushed to the browser per second. The sim runs at its own loop rate
# underneath; this only controls how often the display is told about it.
STREAM_HZ = 60


class SimThread(threading.Thread):
    """Advances the runner in wall-clock time with a speed multiplier."""

    def __init__(self, runner, lock, speed=1.0):
        super().__init__(daemon=True)
        self.runner = runner
        self.lock = lock
        self.speed = speed
        self.paused = False
        self.generation = 0
        self._stop = threading.Event()
        self._step_once = 0

    def run(self):
        last = time.monotonic()
        carry = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            elapsed = now - last
            last = now

            with self.lock:
                paused = self.paused
                speed = self.speed
                pending = self._step_once
                self._step_once = 0

            if paused:
                carry = 0.0
                if pending:
                    with self.lock:
                        for _ in range(pending):
                            self.runner.step()
                time.sleep(0.005)
                continue

            carry += elapsed * speed * self.runner.loop_hz
            steps = int(carry)
            carry -= steps
            # Cap the catch-up so a stalled thread cannot then run away and
            # freeze the display trying to make up minutes of simulation.
            steps = min(steps, int(self.runner.loop_hz))

            if steps:
                with self.lock:
                    for _ in range(steps):
                        self.runner.step()

            time.sleep(0.002)

    def stop(self):
        self._stop.set()

    # --- controls -------------------------------------------------------

    def pause(self):
        with self.lock:
            self.paused = True

    def resume(self):
        with self.lock:
            self.paused = False

    def step_once(self, n=1):
        with self.lock:
            self.paused = True
            self._step_once += n

    def reset(self):
        with self.lock:
            self.runner.reset()
            self.generation += 1

    def set_speed(self, speed):
        with self.lock:
            self.speed = max(0.01, min(100.0, float(speed)))


class VizServer:
    def __init__(self, runner, host="127.0.0.1", port=8420, speed=1.0):
        self.runner = runner
        self.lock = threading.Lock()
        self.sim = SimThread(runner, self.lock, speed)

        server = ThreadingHTTPServer((host, port), _make_handler(self))
        server.daemon_threads = True
        self.httpd = server
        self.host, self.port = server.server_address[0], server.server_address[1]
        self._thread = None

    @property
    def url(self):
        return f"http://{self.host}:{self.port}"

    def start(self):
        self.sim.start()
        self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        self.sim.stop()
        self.httpd.shutdown()
        self.httpd.server_close()
        if self._thread:
            self._thread.join(timeout=5)

    # --- payloads -------------------------------------------------------

    def world_payload(self):
        w = self.runner.world
        with self.lock:
            geometry = self.runner.geometry()
        return {
            "width": w.width,
            "height": w.height,
            "mm_per_pixel": w.mm_per_pixel,
            "origin_px": list(w.origin_px),
            "x_axis": w.x_axis,
            "bounds_mm": list(w.bounds_mm()),
            "rle": w.to_rle(),
            "geometry": geometry,
            "cell_size": MAZE_CELL_SIZE,
            "axle_len": AXLE_LEN,
            "lidar_max": L.MAX_DIST,
            "mounts": {
                k: [v.x, v.y, v.theta] for k, v in LIDAR_MOUNTS.items()
            },
            "scenario": self.runner.scenario.name,
            "description": self.runner.scenario.description,
        }

    def map_payload(self):
        with self.lock:
            return {"rle": self.runner.mapper.to_rle()}

    def frame_payload(self, cursor):
        """Snapshot for one streamed frame, with trail points appended since
        `cursor`. Returns (payload, new_cursor)."""
        with self.lock:
            r = self.runner
            gen = self.sim.generation
            paused = self.sim.paused
            speed = self.sim.speed
            n = len(r.true_trail)
            start = 0 if cursor > n else cursor
            true_delta = r.true_trail[start:n]
            est_delta = r.est_trail[start:n]
            frame = r.last_frame
            payload = {
                "gen": gen,
                "reset": start == 0,
                "t": r.t,
                "paused": paused,
                "speed": speed,
                "true": [r.true_pose.x, r.true_pose.y, r.true_pose.theta],
                "est": [r.est_pose.x, r.est_pose.y, r.est_pose.theta],
                "trail_true": [c for p in true_delta for c in p],
                "trail_est": [c for p in est_delta for c in p],
                "planner_idx": frame.planner_idx if frame else 0,
                "done": r.done(),
                "readings": frame.readings if frame else {},
                "error": r.position_error(),
            }
        return payload, n

    def control(self, body):
        action = body.get("action")
        if action == "pause":
            self.sim.pause()
        elif action == "resume":
            self.sim.resume()
        elif action == "step":
            self.sim.step_once(int(body.get("steps", 1)))
        elif action == "reset":
            self.sim.reset()
        if "speed" in body:
            self.sim.set_speed(body["speed"])
        return {"ok": True}


def _make_handler(viz):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            """Silence the default per-request stderr logging."""

        def _send_json(self, obj, status=200):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]

            if path == "/":
                body = _INDEX.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == "/world":
                self._send_json(viz.world_payload())
                return

            if path == "/map":
                self._send_json(viz.map_payload())
                return

            if path == "/stream":
                self._stream()
                return

            self._send_json({"error": "not found"}, status=404)

        def do_POST(self):
            if self.path.split("?", 1)[0] != "/control":
                self._send_json({"error": "not found"}, status=404)
                return
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b"{}"
            try:
                body = json.loads(raw or b"{}")
            except json.JSONDecodeError:
                self._send_json({"error": "bad json"}, status=400)
                return
            self._send_json(viz.control(body))

        def _stream(self):
            # No Content-Length and no chunked encoding, so the body is
            # delimited by connection close -- which HTTP/1.1 only permits
            # with Connection: close. EventSource reconnects on its own if
            # the stream ever does drop.
            self.close_connection = True
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()

            cursor = 0
            period = 1.0 / STREAM_HZ
            try:
                while True:
                    payload, cursor = viz.frame_payload(cursor)
                    self.wfile.write(b"data: " + json.dumps(payload).encode() + b"\n\n")
                    self.wfile.flush()
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError, OSError):
                # client navigated away
                return

    return Handler

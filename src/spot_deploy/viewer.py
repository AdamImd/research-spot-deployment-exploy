"""Loopback-only, read-only browser viewer for demo and deployment run telemetry."""

import argparse
import json
import math
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from pydantic import ValidationError

from .contracts import ContractError, JOINTS, State
from .kinematics import schematic
from .visual_assets import VisualAssets, load_visual_model

WEB = Path(__file__).parent / "web"
MAX_FRAMES = 24000
MAX_LINE = 256_000


def read_json(path):
    try:
        if path.stat().st_size > 2_000_000:
            return {}
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def finite_vector(value, size):
    return (
        isinstance(value, list)
        and len(value) == size
        and all(type(x) in (int, float) and math.isfinite(x) for x in value)
    )


class RunFeed:
    def __init__(self, directory, urdf=None, estop_status=None, visual_urdf=None, mesh_root=None):
        self.directory = Path(directory)
        if not (self.directory / "run.json").is_file():
            raise ContractError("viewer needs a deployment run directory with run.json")
        self.meta = read_json(self.directory / "run.json")
        policy = read_json(self.directory / "policy.json")
        self.policy = {k: policy.get(k) for k in (
            "name", "policy_hz", "state_source", "action_joint_names", "velocity_command",
            "arm_mode", "mode", "torso_target", "body_height", "note",
        )} if policy else None
        self.frames = deque(maxlen=MAX_FRAMES)
        self.offset = 0
        self.pending_bytes = 0
        self.line = bytearray()
        self.discard_line = False
        self.next_id = 1
        self.skipped = 0
        self.target = self.command = self.health = None
        self.lock = threading.Lock()
        self.estop_status = estop_status
        automatic = self.directory / "robot.urdf"
        model_path = urdf or (automatic if automatic.is_file() else None)
        if visual_urdf and not model_path:
            raise ContractError("a visual template requires a measured URDF")
        self.model, self.assets = (load_visual_model(model_path, visual_urdf, mesh_root)
                                  if model_path else (schematic(), VisualAssets()))
        self.last_event_wall = None
        self.file_identity = None

    def append(self, event):
        kind = event.get("event")
        if kind == "health":
            self.health = {
                k: event.get(k) for k in ("time", "estop_ready", "battery_percent", "fault_count")
            }
            return
        if kind not in ("state", "policy", "shadow_sample", "replay_sample", "command"):
            return
        state = State.model_validate(event.get("state"))
        when = state.robot_time_s
        if kind in ("policy", "shadow_sample", "replay_sample"):
            if not finite_vector(event.get("targets"), 19) or not finite_vector(
                event.get("raw_actions"), 12
            ):
                raise ContractError("invalid viewer prediction")
            self.target = dict(
                positions=event["targets"],
                raw=event["raw_actions"],
                at=when,
                inference_s=event.get("inference_s"),
            )
        if kind == "command":
            if not finite_vector(event.get("positions"), 19):
                raise ContractError("invalid viewer command")
            self.command = dict(positions=event["positions"], at=when, key=event.get("key"))
        self.frames.append(
            dict(
                id=self.next_id,
                t=when,
                event=kind,
                recorded_at=event.get("time"),
                state=state.model_dump(),
                target=self.target,
                command=self.command,
                health=self.health,
            )
        )
        self.next_id += 1
        self.last_event_wall = event.get("time")

    def poll(self):
        path = self.directory / "events.jsonl"
        try:
            stat = path.stat()
            identity = (stat.st_dev, stat.st_ino)
            if (
                self.file_identity and identity != self.file_identity
            ) or stat.st_size < self.offset:
                raise ContractError("run events were replaced or truncated; reopen the viewer")
            self.file_identity = identity
            with path.open("rb") as stream:
                stream.seek(self.offset)
                chunk = stream.read(2_000_000)
                self.offset = stream.tell()
                self.pending_bytes = max(0, stat.st_size - self.offset)
            # Keep partial writes until a newline; bound a malformed line before parsing.
            for part in chunk.splitlines(keepends=True):
                if not self.discard_line:
                    self.line.extend(part)
                    if len(self.line) > MAX_LINE:
                        self.line.clear()
                        self.discard_line = True
                        self.skipped += 1
                if part.endswith(b"\n"):
                    if not self.discard_line:
                        try:
                            self.append(json.loads(self.line))
                        except (ValueError, TypeError, AttributeError, ValidationError):
                            self.skipped += 1
                    self.line.clear()
                    self.discard_line = False
        except FileNotFoundError:
            pass

    def response(self, after=0, limit=1000):
        with self.lock:
            self.poll()
            frames = [x for x in self.frames if x["id"] > after][:limit]
            complete = read_json(self.directory / "COMPLETE.json")
            snap = read_json(self.directory / "robot-snapshot.json")
            hardware = read_json(self.estop_status) if self.estop_status else None
            status = read_json(self.directory / "shadow-status.json")
            # Only expose known status fields, never arbitrary content from a supplied file.
            if hardware:
                hardware = {
                    k: hardware.get(k)
                    for k in (
                        "mode",
                        "state",
                        "updated_unix_s",
                        "device_id",
                        "reason",
                        "robot_confirmed",
                        "input_type",
                        "stop_button",
                        "rearm_button",
                        "input_state",
                    )
                }
            return dict(
                source=self.directory.name,
                mode=self.meta.get("mode", "unknown"),
                demo=False,
                complete=complete.get("status"),
                frames=frames,
                latest_id=self.next_id - 1,
                first_id=self.frames[0]["id"] if self.frames else None,
                skipped=self.skipped,
                pending_bytes=self.pending_bytes,
                server_unix_s=time.time(),
                hardware=hardware,
                server_monotonic_s=time.monotonic(),
                same_host=self.meta.get("host") == socket.gethostname(),
                policy=self.policy,
                rates={k: status.get(k) for k in
                       ("state_hz", "policy_hz", "samples", "updated_unix_s", "motion_commands")}
                if status else None,
                snapshot={
                    k: snap.get(k)
                    for k in (
                        "serial",
                        "firmware",
                        "battery_percent",
                        "estop_ready",
                        "robot_model_sha256",
                    )
                },
                more=bool(self.pending_bytes or
                          (frames and frames[-1]["id"] < self.next_id - 1)),
            )


class DemoFeed:
    def __init__(self):
        self.model = schematic()
        self.assets = VisualAssets()
        self.frames = []
        for i in range(400):
            t = i / 20
            q = [0.0, 0.75, -1.5] * 4 + [0.0, -2.6, 2.7, 0.0, -0.1, 0.0, 0.0]
            q = [x + (0.035 * math.sin(t * 1.5 + j) if j < 12 else 0) for j, x in enumerate(q)]
            target = [
                x + (0.05 * math.sin(t * 1.5 + j + 0.5) if j < 12 else 0) for j, x in enumerate(q)
            ]
            self.frames.append(
                dict(
                    id=i + 1,
                    t=t,
                    event="synthetic",
                    recorded_at=None,
                    state=dict(
                        robot_time_s=t,
                        received_monotonic_s=t,
                        positions=q,
                        velocities=[
                            0.0525 * math.cos(t * 1.5 + j) if j < 12 else 0 for j in range(19)
                        ],
                        loads=[math.sin(t + j) * 4 for j in range(19)],
                        odom_quaternion_wxyz=[
                            math.cos(0.01 * math.sin(t)),
                            math.sin(0.01 * math.sin(t)),
                            0,
                            0,
                        ],
                        linear_velocity_odom=[0, 0, 0],
                        angular_velocity_odom=[0.02 * math.cos(t), 0, 0],
                        last_command_key=0,
                        last_command_received_robot_s=0,
                    ),
                    target=dict(
                        positions=target,
                        raw=[math.sin(t + j) * 0.15 for j in range(12)],
                        at=t,
                        inference_s=0.0012,
                    ),
                    command=None,
                    health=None,
                )
            )

    def response(self, after=0, limit=1000):
        frames = [x for x in self.frames if x["id"] > after][:limit]
        return dict(
            source="Synthetic standing preview",
            mode="demo",
            demo=True,
            complete="demo",
            frames=frames,
            latest_id=len(self.frames),
            first_id=1,
            skipped=0,
            server_unix_s=time.time(),
            hardware=None,
            snapshot={},
            more=bool(frames and frames[-1]["id"] < len(self.frames)),
        )


def make_server(feed, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            allowed = {
                f"127.0.0.1:{self.server.server_port}",
                f"localhost:{self.server.server_port}",
            }
            if self.headers.get("Host") not in allowed:
                self.send_error(403)
                return
            origin = self.headers.get("Origin")
            if origin and origin not in {f"http://{host}" for host in allowed}:
                self.send_error(403)
                return
            url = urlparse(self.path)
            try:
                if url.path == "/api/model":
                    body, mime = (
                        json.dumps(dict(joint_names=JOINTS, **feed.model)).encode(),
                        "application/json",
                    )
                elif url.path == "/api/frames":
                    query = parse_qs(url.query)
                    after = int(query.get("after", ["0"])[0])
                    if after < 0:
                        raise ValueError()
                    body, mime = (
                        json.dumps(feed.response(after), allow_nan=False).encode(),
                        "application/json",
                    )
                elif url.path in feed.assets.data:
                    body, mime = feed.assets.data[url.path]
                elif url.path in ("/", "/app.js", "/style.css", "/robot_scene.js",
                                  "/kinematics.js", "/vendor/three/three.module.min.js",
                                  "/vendor/three/three.core.min.js"):
                    name, mime = {
                        "/": ("index.html", "text/html"),
                        "/app.js": ("app.js", "text/javascript"),
                        "/style.css": ("style.css", "text/css"),
                        "/robot_scene.js": ("robot_scene.js", "text/javascript"),
                        "/kinematics.js": ("kinematics.js", "text/javascript"),
                        "/vendor/three/three.module.min.js":
                            ("vendor/three/three.module.min.js", "text/javascript"),
                        "/vendor/three/three.core.min.js":
                            ("vendor/three/three.core.min.js", "text/javascript"),
                    }[url.path]
                    body = (WEB / name).read_bytes()
                else:
                    self.send_error(404)
                    return
            except (ValueError, OSError):
                self.send_error(400, "Telemetry unavailable or invalid request")
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "public, max-age=31536000, immutable"
                             if url.path.startswith("/assets/") else "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'",
            )
            self.end_headers()
            self.wfile.write(body)

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--demo", action="store_true")
    source.add_argument("--run", type=Path, help="Replay or tail an existing deployment run")
    parser.add_argument("--urdf", type=Path, help="Optional URDF joint and visual model")
    parser.add_argument("--visual-urdf", type=Path,
                        help="Local visual template; measured joint frames must match the URDF")
    parser.add_argument("--mesh-root", type=Path,
                        help="Allowed local asset root; default is each URDF's directory")
    parser.add_argument(
        "--estop-status", type=Path, help="Display this local bridge status (no control)"
    )
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args(argv)
    if not 1024 <= args.port <= 65535:
        parser.error("port must be between 1024 and 65535")
    try:
        feed = DemoFeed() if args.demo else RunFeed(
            args.run, args.urdf, args.estop_status, args.visual_urdf, args.mesh_root)
        if args.demo and args.urdf:
            feed.model, feed.assets = load_visual_model(args.urdf, args.visual_urdf, args.mesh_root)
        elif args.demo and args.visual_urdf:
            raise ContractError("--visual-urdf requires --urdf in demo mode")
        server = make_server(feed, args.port)
        print(f"Spot viewer: http://127.0.0.1:{args.port} (read only)", flush=True)
        try:
            server.serve_forever(poll_interval=0.2)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
        return 0
    except (OSError, ValueError) as exc:
        from .cli import safe_error

        print(safe_error(exc))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

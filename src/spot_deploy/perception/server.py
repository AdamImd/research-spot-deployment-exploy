"""Loopback selection UI. Mutations only affect local perception data."""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlparse

import numpy as np
from PIL import Image

from spot_deploy.contracts import RobotConfig
from spot_deploy.records import source_identity
from .cameras import SpotCameras
from .geometry import Frame
from .pipeline import Pipeline
from .segmentation import Segmenter

WEB = Path(__file__).parent/'web'
VENDOR = Path(__file__).parents[1]/'web/vendor/three'


class ReplayCameras:
    def __init__(self, path):
        self.paths = sorted(Path(path).glob('frame-*'))
        if not self.paths:
            raise ValueError('replay needs recorded frame-* folders')
        self.index = 0

    def capture(self, output=None):
        if self.index >= len(self.paths):
            raise ValueError('end of replay; no repeated frames supplied as live observations')
        frames = {p.stem: Frame.load(p) for p in self.paths[self.index].glob('*.npz')}
        self.index += 1
        if not frames:
            raise ValueError('recorded frame contains no camera arrays')
        if output:
            Path(output).mkdir(parents=True, exist_ok=False)
            for v, f in frames.items():
                f.save(Path(output)/(v+'.npz'))
        return frames

    def close(self):
        pass


def make_handler(pipeline):
    lock = threading.RLock()
    token = secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, body, kind='application/json', status=200):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            if isinstance(body, str):
                body = body.encode()
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def allowed(self):
            # Prevent DNS rebinding; no arbitrary filesystem routes or CORS.
            expected = f'127.0.0.1:{self.server.server_port}'
            return self.headers.get('Host') in (expected, f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.allowed():
                return self.send({'error': 'loopback Host required'}, status=403)
            route = urlparse(self.path).path
            if route == '/':
                return self.send((WEB/'index.html').read_text().replace('__TOKEN__', token),
                                 'text/html; charset=utf-8')
            if route in ('/app.js', '/style.css'):
                return self.send((WEB/route[1:]).read_bytes(),
                                 'text/javascript' if route.endswith('.js') else 'text/css')
            if route in ('/three/three.module.min.js', '/three/three.core.min.js'):
                return self.send((VENDOR/Path(route).name).read_bytes(), 'text/javascript')
            with lock:
                if route == '/api/state':
                    return self.send(pipeline.summary())
                if route == '/api/cloud':
                    # Display capped deterministically; full cloud remains in NPZ/PLY.
                    stride = max(1, int(np.ceil(len(pipeline.cloud)/30000)))
                    return self.send(dict(xyz=pipeline.cloud[::stride].tolist(),
                                          rgb=pipeline.colors[::stride].tolist()))
                if route.startswith('/image/'):
                    view = route.removeprefix('/image/')
                    if view not in pipeline.frames:
                        return self.send({'error': 'view unavailable'}, status=404)
                    rgb = pipeline.frames[view].rgb.copy()
                    pending = pipeline.pending
                    mask = pending['mask'] if pending and pending['view'] == view else pipeline.masks.get(view)
                    if mask is not None:
                        rgb[mask] = (rgb[mask]*.6+np.array([0, 220, 170])*.4).astype(np.uint8)
                    out = io.BytesIO()
                    Image.fromarray(rgb).save(out, format='PNG')
                    return self.send(out.getvalue(), 'image/png')
            return self.send({'error': 'not found'}, status=404)

        def do_POST(self):
            if not self.allowed() or not secrets.compare_digest(
                    self.headers.get('X-Session-Token', ''), token):
                return self.send({'error': 'session token required'}, status=403)
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 100000:
                    raise ValueError('invalid request length')
                body = json.loads(self.rfile.read(length))
                route = urlparse(self.path).path
                with lock:
                    if route == '/api/preview':
                        result = pipeline.preview(body['view'], body['prompt'], body['sequence'])
                    elif route == '/api/accept':
                        result = pipeline.accept()
                    elif route == '/api/step':
                        result = pipeline.step()
                    else:
                        return self.send({'error': 'not found'}, status=404)
                return self.send(result)
            except (ValueError, KeyError, TypeError) as exc:
                return self.send({'error': str(exc)}, status=400)
            except Exception as exc:
                # Never return SDK error strings (may contain connection/auth details).
                pipeline.event('error', exception=type(exc).__name__)
                return self.send({'error': type(exc).__name__+'; inspect local session log'}, status=500)

    return Handler


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument('--robot', type=Path, help='local RobotConfig; read-only camera access')
    source.add_argument('--replay', type=Path, help='recorded session; never contacts Spot')
    p.add_argument('--output', type=Path, required=True, help='new private recording directory')
    p.add_argument('--checkpoint', type=Path, help='official SAM2.1 hiera tiny checkpoint')
    p.add_argument('--device', default='cuda:0')
    p.add_argument('--port', type=int, default=8766)
    p.add_argument('--duration', type=float, default=1800, help='server lifetime seconds')
    args = p.parse_args(argv)
    if not 1 <= args.duration <= 7200 or not 1024 <= args.port <= 65535:
        p.error('duration must be 1..7200 s and port 1024..65535')
    if args.output.exists():
        p.error('output must not already exist')
    segmenter = Segmenter(args.checkpoint, args.device)
    cameras = (SpotCameras(RobotConfig.model_validate_json(args.robot.read_text())) if args.robot
               else ReplayCameras(args.replay))
    pipeline = Pipeline(cameras, segmenter, args.output)
    # Include exact source state and module hashes, even during local development.
    sources = {str(f.relative_to(Path(__file__).parents[1])):
               hashlib.sha256(f.read_bytes()).hexdigest()
               for f in Path(__file__).parent.rglob('*') if f.is_file() and '__pycache__' not in str(f)}
    pipeline.event('provenance', source=source_identity(), module_hashes=sources,
                   replay=bool(args.replay), session_started_unix_s=time.time())
    server = None
    try:
        try:
            pipeline.initial_capture()
        except Exception as exc:
            pipeline.event('initial_capture_failed', exception=type(exc).__name__)
            print('Initial capture rejected; use Capture / track once to request a new bundle.',
                  flush=True)
        server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(pipeline))
        timer = threading.Timer(args.duration, server.shutdown)
        timer.daemon = True
        timer.start()
        print(f'Chair selection: http://127.0.0.1:{args.port}/', flush=True)
        print(f'Recording: {args.output}; camera-only; no robot commands', flush=True)
        server.serve_forever()
    finally:
        if server:
            server.server_close()
        cameras.close()
        pipeline.event('session_closed')


if __name__ == '__main__':
    main()

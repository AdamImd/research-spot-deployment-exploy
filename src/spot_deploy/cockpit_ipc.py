"""Optional owner-only, nonblocking IPC: CUT latch and latest joystick state.

No rearm verb, motion command, network listener, or blocking subscriber writes.
"""

import json
import os
import socket
import stat
import time
from pathlib import Path


class CockpitIPC:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        info = self.path.parent.stat()
        if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) & 0o077:
            raise ValueError("IPC directory must be owner-only")
        # Existing sockets are not silently removed: one bridge owner per path.
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.bind(str(self.path))
        os.chmod(self.path, 0o600)
        self.socket.setblocking(False)
        self.pending = []
        self.sequence = 0

    def poll(self):
        self.pending = []
        cut = False
        for _ in range(16):
            try:
                data, address = self.socket.recvfrom(512)
            except BlockingIOError:
                break
            if data in (b"CUT", b"STATE"):
                cut |= data == b"CUT"
                self.pending.append((address, data))
        return cut

    def publish(self, link, status, state, confirmed):
        self.sequence += 1
        payload = {
            **status.base,
            "state": state,
            "robot_confirmed": confirmed,
            "updated_monotonic_s": time.monotonic(),
            "sequence": self.sequence,
            "axes": getattr(link, "axes", {}),
            "buttons": getattr(link, "buttons", {}),
        }
        raw = json.dumps(payload).encode()
        for address, request in self.pending:
            if not address:
                continue
            try:
                self.socket.sendto(raw, address)
            except (BlockingIOError, FileNotFoundError, ConnectionRefusedError):
                pass
        self.pending = []

    def close(self):
        self.socket.close()
        self.path.unlink(missing_ok=True)

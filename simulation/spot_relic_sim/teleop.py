"""Loopback-only simulation commands with a monotonic freshness deadline."""
import json
import socket
import time
import numpy as np
from .comms import Communications, FRESHNESS

PORT = 18765
LIMITS = np.array([.8, .5, 1.0], dtype=np.float32)


def decode(data, now):
    try:
        packet = json.loads(data)
        if packet.get("version") != 1 or not isinstance(packet["seq"], int):
            return None
        packet["monotonic"] = float(packet["monotonic"])
        age = now - packet["monotonic"]
        velocity = np.asarray(packet["velocity"], dtype=np.float32)
        if not 0 <= age < FRESHNESS or velocity.shape != (3,) or not np.isfinite(velocity).all():
            return None
        if type(packet.get("deadman")) is not bool or type(packet.get("quit")) is not bool:
            return None
        packet["velocity"] = np.clip(velocity, -LIMITS, LIMITS) if packet["deadman"] else np.zeros(3)
        return packet
    except (ValueError, TypeError, KeyError, OverflowError, AttributeError):
        return None


def decode_telemetry(data, now, previous=None):
    """Do not let delayed, reordered telemetry move the display backward."""
    try:
        packet = json.loads(data)
        stamp = float(packet["monotonic"])
        if packet.get("version") != 1 or not 0 <= now - stamp < .5:
            return None
        if previous is not None and stamp <= previous["monotonic"]:
            return None
        for key, size in (("measured", 3), ("command", 3), ("action", 12)):
            value = np.asarray(packet[key], dtype=float)
            if value.shape != (size,) or not np.isfinite(value).all():
                return None
            packet[key] = value.tolist()
        if not np.isfinite(float(packet["height"])) or packet["mode"] not in ("four", "fl", "fr"):
            return None
        packet["monotonic"] = stamp
        packet["height"] = float(packet["height"])
        return packet
    except (ValueError, TypeError, KeyError, OverflowError, AttributeError):
        return None


class Receiver:
    def __init__(self, port=PORT, communications=None):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", port))
        self.socket.setblocking(False)
        self.port = self.socket.getsockname()[1]
        self.communications = communications or Communications()
        self.commands = self.communications.links["command"]
        self.telemetry_link = self.communications.links["telemetry"]
        self.started = time.monotonic()
        self.requested = None
        self.requested_velocity = np.zeros(3, dtype=np.float32)
        self.status = {}

    def poll(self, simulation_time=None):
        now = time.monotonic()
        t = now - self.started if simulation_time is None else simulation_time
        self.flush_telemetry(t)
        while True:
            try:
                data, address = self.socket.recvfrom(8192)
            except BlockingIOError:
                break
            packet = decode(data, now)
            if address[0] != "127.0.0.1" or packet is None:
                continue
            # Escape is local session management, not a simulated robot command.
            if packet["quit"]:
                return np.zeros(3, dtype=np.float32), True
            if self.requested is None or packet["seq"] > self.requested["seq"]:
                self.requested = packet
            self.commands.send(packet, t, sent_at=t - (now - packet["monotonic"]), sequence=packet["seq"])
        packet = self.commands.poll(t)
        stale = packet is None or now - packet.payload["monotonic"] >= FRESHNESS
        self.requested_velocity = (self.requested["velocity"] if self.requested is not None and
            now - self.requested["monotonic"] < FRESHNESS else np.zeros(3, dtype=np.float32))
        self.status = dict(command_age_s=self.commands.age(t), command_stale=stale,
                           command_sequence=None if packet is None else packet.sequence)
        return np.zeros(3, dtype=np.float32) if stale else packet.payload["velocity"], False

    def telemetry(self, state, action, command, mode, simulation_time=0., status="running", reason=None,
                  communications_status=None):
        payload = dict(version=1, monotonic=time.monotonic(), mode=mode,
                       simulation_time=simulation_time, status=status, reason=reason,
                       height=float(state["position"][2]), gravity=state["gravity"].tolist(),
                       measured=np.r_[state["linear_velocity"][:2], state["angular_velocity"][2]].tolist(),
                       action=action.tolist(), command=command.velocity.tolist(),
                       communications=self.communications.config.metadata() | (communications_status or {}))
        self.telemetry_link.send(payload, simulation_time)
        self.flush_telemetry(simulation_time)

    def flush_telemetry(self, t):
        packet = self.telemetry_link.poll(t, max_age=.5)
        if packet is not None and packet.sequence != getattr(self, "last_telemetry", None):
            self.socket.sendto(json.dumps(packet.payload, allow_nan=False).encode(), ("127.0.0.1", self.port + 1))
            self.last_telemetry = packet.sequence

    def close(self):
        self.socket.close()

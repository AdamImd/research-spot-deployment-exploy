"""Fail-closed challenged serial bridge; this process owns all E-stop check-ins."""

import fcntl
import secrets
import signal
import socket
import threading
import time
from pathlib import Path

from .contracts import ContractError
from .estop_interlock import boot_id
from .estop_protocol import MAX_FRAME, PhysicalLatch, parse, request
from .records import atomic_json


class SerialLink:
    def __init__(self, profile):
        import serial

        self.profile = profile
        # Opening a DevKit USB port can reset it. Startup remains STOP and must be rearmed.
        self.port = serial.Serial(
            profile.serial_device,
            profile.baud,
            timeout=profile.response_timeout_s,
            write_timeout=profile.response_timeout_s,
            exclusive=True,
        )
        # Bound the USB auto-reset boot interval before an endpoint is registered.
        time.sleep(1.5)
        self.port.reset_input_buffer()

    def exchange(self, nonce):
        self.port.write(request(nonce))
        deadline = time.monotonic() + self.profile.response_timeout_s
        frame = bytearray()
        while time.monotonic() < deadline and len(frame) <= MAX_FRAME:
            self.port.timeout = max(0.001, deadline - time.monotonic())
            byte = self.port.read(1)
            if not byte:
                break
            frame.extend(byte)
            if byte == b"\n":
                break
        return bytes(frame)

    def close(self):
        self.port.close()


class StatusWriter:
    def __init__(self, path, profile, profile_hash, robot_hash=None, session=None):
        self.path = Path(path)
        # A second bridge cannot overwrite the first bridge's status.
        self.lock = self.path.with_suffix(self.path.suffix + ".lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise ContractError("E-stop status already has an owner") from None
        self.base = dict(
            schema_version=1,
            host=socket.gethostname(),
            host_boot_id=boot_id(),
            mode="robot" if robot_hash else "bench",
            session=session or secrets.token_hex(16),
            device_id=profile.device_id,
            input_type=getattr(profile, "input_type", "esp32"),
            stop_button=getattr(profile, "stop_button", None),
            rearm_button=getattr(profile, "rearm_button", None),
            profile_sha256=profile_hash,
            robot_binding_sha256=robot_hash,
        )
        self._condition = threading.Condition()
        self._pending = None
        self._writing = self._closing = self._failed = False
        self._writer = None

    def start_async(self):
        """Latest-only disk publication; timestamps still mark the original check-in."""
        if self._writer is None:
            self._writer = threading.Thread(target=self._write_latest, daemon=True,
                                            name="estop-status")
            self._writer.start()

    def check(self):
        if self._failed:
            raise ContractError("E-stop status writer failed")

    def _write_latest(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._pending is not None or self._closing)
                    if self._pending is None:
                        return
                    value, self._pending = self._pending, None
                    self._writing = True
                atomic_json(self.path, value)
                with self._condition:
                    self._writing = False
                    self._condition.notify_all()
        except BaseException:
            with self._condition:
                self._failed = True
                self._condition.notify_all()

    def flush(self):
        with self._condition:
            ready = self._condition.wait_for(
                lambda: self._failed or (self._pending is None and not self._writing), timeout=2
            )
            self.check()
            if not ready:
                raise ContractError("E-stop status flush timed out")

    def write(self, state, reason, endpoint=None, packet=None, confirmed=False, input_state=None):
        self.check()
        value = dict(
                self.base,
                state=state,
                reason=reason,
                updated_monotonic_s=time.monotonic(),
                updated_unix_s=time.time(),
                endpoint_id=endpoint.endpoint_id if endpoint else None,
                robot_confirmed=confirmed,
                device_boot=packet.boot if packet else None,
                sequence=packet.sequence if packet else None,
                input_state=input_state,
            )
        if self._writer is None:
            atomic_json(self.path, value)
        else:
            with self._condition:
                if self._closing:
                    raise ContractError("E-stop status writer closed")
                self._pending = value
                self._condition.notify_all()

    def close(self):
        if self._writer is not None:
            with self._condition:
                self._closing = True
                self._condition.notify_all()
            self._writer.join(timeout=2)
            if self._writer.is_alive():
                # Retain ownership until process exit; never allow two disk writers.
                raise ContractError("E-stop status writer did not stop")
        self.lock.close()


def run_bridge(profile, link, status, record, endpoint=None, stop=None, duration=None):
    stop = stop or threading.Event()
    latch = PhysicalLatch()
    start, prior_state, packet = time.monotonic(), None, None
    primary_error = None
    status.write("stopped", "Physical stop and rearm required", endpoint)
    try:
        while not stop.is_set() and (duration is None or time.monotonic() - start < duration):
            began = time.monotonic()
            nonce = secrets.token_hex(8)
            frame = link.exchange(nonce)
            packet = parse(frame, nonce, profile.device_id)
            if time.monotonic() - began > profile.response_timeout_s:
                raise ContractError("E-stop serial response deadline exceeded")
            allowed = latch.accept(packet)
            if stop.is_set():
                break
            if endpoint:
                endpoint.check_in(allowed)
            # A late RPC must not publish ARMED, even if the server processed it.
            if time.monotonic() - began > profile.response_timeout_s + profile.rpc_timeout_s:
                raise ContractError("E-stop cycle deadline exceeded")
            state = "armed" if allowed else "stopped"
            status.write(
                state,
                "Physical rearm accepted" if allowed else "Physical stop or rearm required",
                endpoint,
                packet,
                confirmed=endpoint is not None,
            )
            if state != prior_state:
                record.event(
                    "estop_transition",
                    state=state,
                    boot=packet.boot,
                    sequence=packet.sequence,
                    robot_confirmed=endpoint is not None,
                )
                prior_state = state
            stop.wait(max(0, profile.poll_interval_s - (time.monotonic() - began)))
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        # Best effort explicit STOP; the robot's endpoint watchdog remains independent.
        confirmed = False
        cleanup_error = None
        try:
            if endpoint:
                endpoint.stop()
                confirmed = True
        except BaseException as exc:
            cleanup_error = exc
        finally:
            try:
                status.write(
                    "fault" if primary_error or cleanup_error else "stopped",
                    "Bridge inactive; restart and physical rearm required",
                    endpoint,
                    packet,
                    confirmed,
                )
            except BaseException as exc:
                cleanup_error = cleanup_error or exc
        if cleanup_error and primary_error is None:
            raise cleanup_error
    return {"status": "stopped", "robot_connected": endpoint is not None}


def signal_stop(event):
    old = {}
    for signum in (signal.SIGTERM, signal.SIGINT):
        old[signum] = signal.signal(signum, lambda *_: event.set())
    return old

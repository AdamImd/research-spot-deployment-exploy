"""Linux joystick stop input. No SDK imports, motion commands, or automatic rearm."""

import array
import fcntl
import os
import select
import stat
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from pydantic import Field, model_validator

from .contracts import ContractError, StrictModel

EVENT = struct.Struct("=IhBB")
BUTTON, AXIS, INIT = 1, 2, 128
BTN_TRIGGER = 0x120


class JoystickProfile(StrictModel):
    schema_version: Literal[1]
    input_type: Literal["linux_joystick"]
    joystick_device: str = Field(pattern=r"^/dev/input/by-id/[^/]+-joystick$")
    expected_name: str = Field(min_length=1, max_length=127)
    vendor_id: str = Field(pattern=r"^[0-9a-f]{4}$")
    product_id: str = Field(pattern=r"^[0-9a-f]{4}$")
    stop_button: Literal[0, 1]
    rearm_button: int = Field(ge=1, le=31)
    release_hold_s: float = Field(ge=0.5, le=2)
    poll_interval_s: float = Field(ge=0.01, le=0.05)
    max_input_gap_s: float = Field(ge=0.1, le=0.25)
    status_interval_s: float = Field(ge=0.02, le=0.1)
    rpc_timeout_s: float = Field(ge=0.02, le=0.1)
    endpoint_name: str = Field(pattern=r"^[A-Za-z0-9_-]{1,40}$")
    endpoint_role: str = Field(pattern=r"^(PDB_rooted|spot_joystick_[A-Za-z0-9_-]{1,32})$")
    endpoint_timeout_s: float = Field(ge=0.2, le=1)
    cut_power_timeout_s: float = Field(ge=0.2, le=2)
    stop_level: Literal["cut", "settle_then_cut"]

    @property
    def device_id(self):
        return f"{self.vendor_id}:{self.product_id}:{self.expected_name}"

    @model_validator(mode="after")
    def timing(self):
        if self.stop_button == self.rearm_button:
            raise ValueError("stop and rearm must be separate buttons")
        if not self.poll_interval_s + self.rpc_timeout_s < self.max_input_gap_s:
            raise ValueError("poll and RPC budgets must fit the input freshness bound")
        if not self.max_input_gap_s < self.endpoint_timeout_s <= self.cut_power_timeout_s:
            raise ValueError("input freshness must precede the robot watchdogs")
        if self.joystick_device.endswith("-event-joystick"):
            raise ValueError("use the joydev by-id path, not the evdev path")
        return self


@dataclass(frozen=True)
class ButtonEvent:
    button: int
    pressed: bool
    initial: bool = False


class JoystickLink:
    """Nonblocking joydev input; button numbers match Linux/SDL joystick indices.

    Idle devices emit no events. A fresh ioctl and poll error check establish
    kernel-device availability each cycle, not a hardware challenge/heartbeat.
    Queue overflow produces INIT events; any resynchronization after startup faults.
    """

    def __init__(self, profile):
        self.profile, self.fd = profile, None
        self.ready = False
        self.axes = {}
        self.buttons, self.initial_buttons, self.initial_axes = {}, set(), set()
        try:
            self.path = Path(profile.joystick_device)
            self.node = self.path.resolve(strict=True)
            if self.node.parent != Path("/dev/input") or not self.node.name.startswith("js"):
                raise ContractError("expected a Linux joystick character device")
            self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
            info = os.fstat(self.fd)
            if not stat.S_ISCHR(info.st_mode):
                raise ContractError("joystick path is not a character device")
            self.identity = (info.st_dev, info.st_ino, info.st_rdev)
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                raise ContractError("joystick already has a stop-input owner") from None
            name = bytearray(128)
            fcntl.ioctl(self.fd, 0x80806A13, name)  # JSIOCGNAME(128)
            name = name.split(b"\0")[0].decode("utf-8")
            count, axes = bytearray(1), bytearray(1)
            fcntl.ioctl(self.fd, 0x80016A12, count)  # JSIOCGBUTTONS
            fcntl.ioctl(self.fd, 0x80016A11, axes)  # JSIOCGAXES
            mapping = array.array("H", [0] * 512)
            fcntl.ioctl(self.fd, 0x84006A34, mapping)  # JSIOCGBTNMAP
            sysfs = Path("/sys/class/input") / self.node.name / "device"
            vendor = (sysfs / "id/vendor").read_text().strip()
            product = (sysfs / "id/product").read_text().strip()
            if (name, vendor, product) != (
                profile.expected_name, profile.vendor_id, profile.product_id
            ):
                raise ContractError("joystick USB identity differs from the profile")
            if (count[0] <= max(profile.stop_button, profile.rearm_button)
                    or mapping[profile.stop_button] != BTN_TRIGGER + profile.stop_button):
                raise ContractError("joystick STOP mapping must match BTN_TRIGGER/BTN_THUMB; rearm must exist")
            self.metadata = dict(name=name, vendor_id=vendor, product_id=product,
                                 device=str(self.path), button_count=count[0], axis_count=axes[0],
                                 button_codes=list(mapping[:count[0]]))
            self.poller = select.poll()
            self.poller.register(self.fd, select.POLLIN | select.POLLERR | select.POLLHUP)
        except BaseException:
            self.close()
            raise

    def _decode(self, data):
        if not data or len(data) % EVENT.size:
            raise ContractError("joystick disconnected or returned a partial event")
        buttons = []
        for _, value, kind, number in EVENT.iter_unpack(data):
            initial, base = bool(kind & INIT), kind & ~INIT
            if base not in (BUTTON, AXIS) or (initial and self.ready):
                raise ContractError("joystick event loss/resynchronization; restart required")
            if base == AXIS:
                if number >= self.metadata["axis_count"]:
                    raise ContractError("invalid joystick axis index")
                self.axes[number] = value / 32767
                if initial:
                    self.initial_axes.add(number)
                continue
            if number >= self.metadata["button_count"] or value not in (0, 1):
                raise ContractError("invalid joystick button event")
            if initial:
                self.initial_buttons.add(number)
            self.buttons[number] = bool(value)
            buttons.append(ButtonEvent(number, bool(value), initial))
        return buttons

    def poll(self):
        info = self.path.stat()
        if (info.st_dev, info.st_ino, info.st_rdev) != self.identity:
            raise ContractError("joystick removed or replaced; restart required")
        if any(flags & (select.POLLERR | select.POLLHUP | select.POLLNVAL)
               for _, flags in self.poller.poll(0)):
            raise ContractError("joystick disconnected")
        fcntl.ioctl(self.fd, 0x80046A01, bytearray(4))  # JSIOCGVERSION also fails on removal
        result = []
        for _ in range(9):
            try:
                data = os.read(self.fd, 64 * EVENT.size)
            except BlockingIOError:
                break
            result.extend(self._decode(data))
        else:
            raise ContractError("joystick input backlog; restart required")
        if (len(self.initial_buttons) != self.metadata["button_count"]
                or len(self.initial_axes) != self.metadata["axis_count"]):
            raise ContractError("joystick did not supply complete initial state")
        self.ready = True
        return result

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class JoystickLatch:
    def __init__(self, profile):
        self.profile = profile
        self.buttons = {}
        self.armed, self.observed_stop = False, False
        self.released_since = None

    def accept(self, events, now):
        stop_in_batch = any(e.button == self.profile.stop_button and e.pressed for e in events)
        for event in events:
            previous = self.buttons.get(event.button)
            self.buttons[event.button] = event.pressed
            if event.button == self.profile.stop_button:
                if event.pressed:
                    self.armed, self.observed_stop, self.released_since = False, True, None
                elif previous is not False:
                    self.released_since = now
            elif (event.button == self.profile.rearm_button and event.pressed
                  and previous is False and not event.initial and not stop_in_batch
                  and self.observed_stop and self.buttons.get(self.profile.stop_button) is False
                  and self.released_since is not None
                  and now - self.released_since >= self.profile.release_hold_s):
                self.armed = True
        if stop_in_batch or self.buttons.get(self.profile.stop_button) is not False:
            self.armed = False
        return self.armed

    def state(self):
        return dict(stop_pressed=self.buttons.get(self.profile.stop_button),
                    rearm_pressed=self.buttons.get(self.profile.rearm_button),
                    observed_stop=self.observed_stop)


def run_joystick_bridge(profile, link, status, record, endpoint=None, stop=None, duration=None):
    stop = stop or threading.Event()
    latch = JoystickLatch(profile)
    start, last_poll, prior_state = time.monotonic(), None, None
    last_status, last_metrics = None, start
    timing = dict(cycles=0, max_poll_gap_s=0., max_input_s=0., max_rpc_s=0., max_status_s=0.)
    primary_error = None
    ipc = None
    try:
        status.start_async()
        ipc_path = os.environ.get("SPOT_JOYSTICK_IPC")
        if ipc_path:
            from .cockpit_ipc import CockpitIPC
            ipc = CockpitIPC(ipc_path)
        record.event("joystick_device", **link.metadata)
        while not stop.is_set() and (duration is None or time.monotonic() - start < duration):
            began = time.monotonic()
            status.check()
            if last_poll is not None:
                gap = began - last_poll
                timing["max_poll_gap_s"] = max(timing["max_poll_gap_s"], gap)
                if gap > profile.max_input_gap_s:
                    raise ContractError(f"joystick polling stalled ({gap:.3f} s); restart required")
            events = link.poll()
            allowed = latch.accept(events, began)
            if ipc and ipc.poll():
                # CUT wins over a rearm in the same input batch. Require a NEW
                # physical release/press sequence; IPC cannot synthesize rearm.
                latch.armed = False
                latch.observed_stop = True
                latch.released_since = began if latch.buttons.get(profile.stop_button) is False else None
                allowed = False
            input_s = time.monotonic() - began
            timing["max_input_s"] = max(timing["max_input_s"], input_s)
            if input_s > profile.max_input_gap_s:
                raise ContractError("joystick input is stale")
            last_poll = began
            for event in events:
                record.event("joystick_button", button=event.button, pressed=event.pressed,
                             initial=event.initial)
            if stop.is_set():
                break
            rpc_start = time.monotonic()
            if endpoint:
                endpoint.check_in(allowed)
            timing["max_rpc_s"] = max(timing["max_rpc_s"], time.monotonic() - rpc_start)
            if time.monotonic() - began > profile.max_input_gap_s:
                raise ContractError("joystick E-stop cycle exceeded freshness bound")
            state = "armed" if allowed else "stopped"
            if ipc:
                ipc.publish(link, status, state, endpoint is not None)
            reason = ("Physical rearm accepted" if allowed else
                      f"Button {profile.stop_button + 1} (index {profile.stop_button}) latches CUT; "
                      f"release, then press rearm index "
                      f"{profile.rearm_button} to rearm")
            # Keep joystick polling/check-ins at 50 Hz; avoid 50 filesystem rewrites/s.
            # Physical transitions still update the operator status immediately.
            if (state != prior_state or events or last_status is None
                    or time.monotonic() - last_status >= profile.status_interval_s):
                status_start = time.monotonic()
                status.write(state, reason, endpoint, confirmed=endpoint is not None,
                             input_state=latch.state())
                last_status = status_start
                timing["max_status_s"] = max(timing["max_status_s"],
                                              time.monotonic() - status_start)
            if state != prior_state:
                record.event("estop_transition", state=state, robot_confirmed=endpoint is not None)
                prior_state = state
            timing["cycles"] += 1
            if time.monotonic() - last_metrics >= 5:
                record.event("joystick_timing", **timing)
                last_metrics = time.monotonic()
            stop.wait(max(0, profile.poll_interval_s - (time.monotonic() - began)))
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        confirmed, cleanup_error = False, None
        try:
            if endpoint:
                endpoint.stop()
                confirmed = True
        except BaseException as exc:
            cleanup_error = exc
        finally:
            try:
                status.write("fault" if primary_error or cleanup_error else "stopped",
                             "Bridge inactive; restart and physical rearm required", endpoint,
                             confirmed=confirmed, input_state=latch.state())
                status.flush()
            except BaseException as exc:
                cleanup_error = cleanup_error or exc
            try:
                record.event("joystick_timing", **timing, final=True)
            except BaseException as exc:
                cleanup_error = cleanup_error or exc
        if ipc:
            ipc.close()
        if cleanup_error and primary_error is None:
            raise cleanup_error
    return dict(status="stopped", robot_connected=endpoint is not None, motion_commands=0,
                timing=timing)

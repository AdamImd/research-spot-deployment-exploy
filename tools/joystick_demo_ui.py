#!/usr/bin/python3
"""Local GTK desktop UI; launching this program never starts a physical run."""

import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


class Session:
    """One subprocess, one pipe writer. Closing writer requests normal finish, even on UI crash."""

    def __init__(self):
        self.process = None
        self.writer = self.mode_writer = None
        self.messages = queue.Queue()

    def start(self, command, staged=False):
        if self.process is not None:
            raise RuntimeError("session still running or cleaning up")
        rd, wr = os.pipe()
        mr, mw = os.pipe() if staged else (None, None)
        reads = (rd, mr) if staged else (rd,)
        try:
            self.process = subprocess.Popen(
                command(*reads),
                cwd=ROOT,
                pass_fds=reads,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except BaseException:
            os.close(wr)
            if staged:
                os.close(mw)
            raise
        finally:
            for read_fd in reads:
                os.close(read_fd)
        self.mode_writer = mw
        self.writer = wr
        proc = self.process

        def collect():
            with proc.stdout:
                for line in proc.stdout:
                    self.messages.put(line.rstrip()[:1200])
            self.messages.put(proc.wait())

        threading.Thread(target=collect, daemon=True).start()

    def request_mode(self, token):
        if self.mode_writer is None or self.writer is None:
            raise RuntimeError("session has ended")
        os.write(self.mode_writer, (token + "\n").encode("ascii"))

    def reap(self):
        self.stop()
        if self.mode_writer is not None:
            os.close(self.mode_writer)
            self.mode_writer = None
        self.process = None

    def stop(self):
        if self.writer is not None:
            os.close(self.writer)
            self.writer = None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--bundle", type=Path, help="Explicit reviewed physical joystick bundle; otherwise USB-only"
    )
    p.add_argument("--profile", type=Path, default=ROOT / "examples/joystick-relic-arm.json")
    p.add_argument("--sim", action="store_true", help="Select MuJoCo at startup")
    p.add_argument(
        "--fleet-url", default="http://10.0.0.20:8790", help="Fleet read-only preview service"
    )
    p.add_argument("--fleet-repo", type=Path, default=Path.home() / "repos/flir-apriltag")
    p.add_argument(
        "--live-monitor",
        action="store_true",
        help="Open the read-only Spot connection and camera streams on launch",
    )
    args = p.parse_args()
    if args.live_monitor and (not args.bundle or args.sim):
        p.error("--live-monitor needs a physical --bundle and cannot be combined with --sim")
    from room_camera_panel import CameraPanel, NAMES
    import gi

    gi.require_version("Gtk", "3.0")
    from gi.repository import Gtk, GLib

    bundle = args.bundle.resolve() if args.bundle else None
    profile = json.loads(
        ((bundle / "policy/manifest.json") if bundle else args.profile).read_text()
    )
    if bundle:
        profile = profile["teleop"]
    family = profile.get("study_family")
    python = str(ROOT / ".venv/bin/python")
    session, connection, camera_service = Session(), Session(), Session()
    connected = False
    phase = token = pending = None
    camera_on = False
    win = Gtk.Window(
        title="ReLIC · simulated joystick demo" if args.sim else "ReLIC · joystick demo recorder"
    )
    win.set_default_size(590, 360)
    panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    panel.set_border_width(20)
    layout = Gtk.Box(spacing=8)
    layout.pack_start(panel, False, False, 0)
    win.add(layout)
    cameras = CameraPanel(Gtk, args.fleet_url)
    layout.pack_start(cameras.grid, True, True, 0)

    def label(text):
        w = Gtk.Label(label=text, xalign=0, selectable=True, wrap=True)
        w.set_max_width_chars(65)
        panel.pack_start(w, False, False, 0)
        return w

    label("ReLIC   /   Walking demo")
    label("Stowed arm · Forward / lateral / yaw · Release enable for zero velocity")
    label(
        f"Center stick, release then hold printed {profile['enable_button'] + 1} to drive. "
        f"Printed {profile['finish_button'] + 1} finishes. While enabled, trigger marks the recording. Hat left/right selects a camera; up/down shows all."
    )
    if family:
        label(
            f"Registered {family} repeat · Physical mode follows the frozen trace while enable is held. "
            "USB-only mode records manual stick input."
        )
    mode = Gtk.ComboBoxText()
    mode.append("usb", "USB only — record commands without Spot")
    mode.append("sim", "MuJoCo — drive simulated Spot")
    if bundle:
        mode.append("live", "Physical Spot — record states, actions and commands")
    mode.set_active_id("live" if args.live_monitor else "sim" if args.sim else "usb")
    panel.pack_start(mode, False, False, 0)
    fields = Gtk.Box(spacing=6)
    panel.pack_start(fields, False, False, 0)
    operator = Gtk.Entry(placeholder_text="Robot operator")
    safety = Gtk.Entry(placeholder_text="Tablet E-stop operator")
    for w in [operator, safety]:
        fields.pack_start(w, True, True, 0)
    physical_info = label(
        "Stand up powers Spot and waits for native standing. Enable ReLIC hands control to the joystick. Release sits Spot, powers off and returns the lease. The tablet is the emergency stop."
    )

    connections = Gtk.Box(spacing=6)
    panel.pack_start(connections, False, False, 0)
    connect = Gtk.Button(label="Connect")
    streams = Gtk.Button(label="Start camera streams")
    for widget in [connect, streams]:
        connections.pack_start(widget, True, True, 0)
    connection_label = label("Disconnected")
    camera_label = label("Camera streams off")
    selector = Gtk.ComboBoxText()
    for selection in ["All cameras", *NAMES]:
        selector.append_text(selection)
    selector.set_active(0)
    selector.connect("changed", lambda box: cameras.select(box.get_active() - 1))
    panel.pack_start(selector, False, False, 0)

    def monitor_changed():
        selector.set_visible(camera_on)
        cameras.grid.set_visible(camera_on)
        cameras.reset((out if mode.get_active_id() == "sim" else "live") if camera_on else None)

    def controls():
        kind = mode.get_active_id()
        physical = kind == "live"
        usb = kind == "usb"
        sim = kind == "sim"
        running = session.process is not None
        staged = not usb and not (physical and family)
        linked = (running and phase is not None) if sim else connected
        fields.set_visible(physical)
        fields.set_sensitive(not running)
        physical_info.set_visible(physical and not family)
        connect.set_visible(not usb)
        streams.set_visible(True)
        connection_label.set_visible(not usb)
        camera_label.set_visible(True)
        connect.set_label("Disconnect" if (running if sim else connection.process) else "Connect")
        connect.set_sensitive(
            (not running or (sim and phase == "seated" and pending is None)) and not closing
        )
        streams.set_label(
            "Stop camera streams"
            if camera_on
            else ("Start room cameras" if usb else "Start camera streams")
        )
        streams.set_sensitive(
            not closing
            and (camera_on or camera_service.process is None)
            and (not sim or (linked and session.writer is not None))
        )
        mode.set_sensitive(
            not running
            and connection.process is None
            and camera_service.process is None
            and not camera_on
        )
        start.set_label("Stand up" if staged else "Start recording")
        start.set_sensitive(
            (
                (sim and phase == "seated" and pending is None and session.writer is not None)
                or (not sim and not running and (usb or linked))
            )
            and not closing
        )
        toggle.set_visible(staged)
        toggle.set_label("Disable ReLIC · native stand" if phase == "relic" else "Enable ReLIC")
        toggle.set_sensitive(
            running
            and phase in ("native", "relic")
            and pending is None
            and session.writer is not None
        )
        stop.set_label("Release · sit down" if staged else "Stop and save")
        stop.set_sensitive(
            running and session.writer is not None and not (sim and phase == "seated")
        )
        monitor_changed()

    mode.connect("changed", lambda *_: controls())
    limits = profile["axes"]
    label(
        f"Stick limits   ±{limits[0]['maximum']:g} m/s forward     "
        f"±{limits[1]['maximum']:g} m/s lateral     ±{limits[2]['maximum']:g} rad/s yaw"
    )
    status = label("Idle")
    axes = label("Forward —     Lateral —     Yaw —")
    where = label("No recording yet")
    markers = label("Markers   0")
    last_hat_view = None
    buttons = Gtk.Box(spacing=8)
    panel.pack_end(buttons, False, False, 0)
    start = Gtk.Button(label="Start recording")
    stop = Gtk.Button(label="Stop and save")
    toggle = Gtk.Button(label="Enable ReLIC")
    for w in [start, toggle, stop]:
        buttons.pack_start(w, True, True, 0)
    stop.set_sensitive(False)
    out = None
    closing = False
    live = False
    stage = None
    diagnostic = ""

    def pressed(*_):
        nonlocal out, live, stage, diagnostic, last_hat_view, token, phase, pending
        if session.process:
            if mode.get_active_id() == "sim" and phase == "seated":
                change_mode()
            return
        live = mode.get_active_id() == "live"
        dev = profile["device"]["joystick_device"]
        if not Path(dev).exists():
            status.set_text("USB joystick missing: " + dev)
            return
        names = [w.get_text().strip() for w in [operator, safety]]
        if live:
            if not all(names) or names[0].casefold() == names[1].casefold():
                status.set_text("Enter two distinct operator names.")
                return
            d = Gtk.MessageDialog(
                transient_for=win,
                modal=True,
                message_type=Gtk.MessageType.QUESTION,
                buttons=Gtk.ButtonsType.NONE,
                text="Start a registered physical trial?"
                if family
                else "Power on and stand Spot now?",
            )
            d.format_secondary_text(
                (
                    f"Replay registered {family} commands with held enable. "
                    if family
                    else "Spot will wait in native stand until you choose Enable ReLIC. "
                )
                + f"{names[0]} operating; {names[1]} holding the functioning tablet E-stop. "
                f"Confirm rig, floor path and stowed arm clear. Native stand, then joystick control for up to "
                f"{profile['duration_s']:g} s. Recording and shutdown are automatic. No retries."
            )
            d.add_button("Cancel", Gtk.ResponseType.CANCEL)
            d.add_button("Confirm and start Spot", Gtk.ResponseType.OK)
            d.set_default_response(Gtk.ResponseType.CANCEL)
            answer = d.run()
            d.destroy()
            if answer != Gtk.ResponseType.OK:
                return
        token = phase = pending = None
        out = None
        last_hat_view = None
        markers.set_text("Markers   0")
        monitor_changed()
        diagnostic = ""
        stage = None
        dest = (
            ROOT
            / "runs"
            / (
                ("joystick-sim-" if mode.get_active_id() == "sim" else "joystick-stick-")
                + time.strftime("%Y%m%d-%H%M%S")
                + "-"
                + uuid.uuid4().hex[:8]
            )
        )

        def command(fd, mode_fd=None):
            mode_args = ["--mode-fd", str(mode_fd)] if mode_fd is not None else []
            if live:
                return [
                    python,
                    "-u",
                    "tools/run_joystick_bundle.py",
                    "--bundle",
                    str(bundle),
                    *mode_args,
                    "--demo-finish-fd",
                    str(fd),
                    "--operator",
                    names[0],
                    "--safety-operator",
                    names[1],
                    "--execute",
                ]
            if mode.get_active_id() == "sim":
                return [
                    python,
                    "-u",
                    "tools/sim_joystick_demo.py",
                    *mode_args,
                    *(["--manifest", str(bundle / "policy/manifest.json")] if bundle else []),
                    "--duration",
                    str(min(60.0, profile["duration_s"])),
                    "--profile",
                    str(args.profile.resolve()),
                    "--finish-fd",
                    str(fd),
                    "--output",
                    str(dest),
                ]
            return [
                python,
                "-u",
                "tools/joystick_demo_preview.py",
                "--profile",
                str(args.profile.resolve()),
                "--finish-fd",
                str(fd),
                "--output",
                str(dest),
                *(["--bundle", str(bundle)] if bundle else []),
            ]

        try:
            session.start(command, staged=mode.get_active_id() == "sim" or (live and not family))
        except OSError as e:
            status.set_text(str(e))
            return
        status.set_text("Checking readiness…" if live else "Starting session…")
        where.set_text("")
        axes.set_text("")
        controls()

    def change_mode(*_):
        nonlocal pending
        if token is None or pending is not None:
            return
        try:
            session.request_mode(token)
            pending = token
            status.set_text("Waiting for controller acknowledgement…")
        except (OSError, RuntimeError) as exc:
            status.set_text(str(exc))
        controls()

    def connect_pressed(*_):
        if mode.get_active_id() == "sim":
            if session.process:
                finish()
            else:
                pressed()
            return
        if connection.process:
            connection.stop()
        else:
            connection.start(
                lambda fd: [
                    python,
                    "-u",
                    "tools/teleop_services.py",
                    "connect",
                    "--bundle",
                    str(bundle),
                    "--finish-fd",
                    str(fd),
                ]
            )
            connection_label.set_text("Connecting to Spot…")
        controls()

    def set_cameras(enabled):
        nonlocal camera_on
        if mode.get_active_id() == "sim" and out:
            flag = out / "camera-enabled"
            if enabled:
                flag.touch()
            else:
                flag.unlink(missing_ok=True)
        elif enabled:
            camera_service.start(
                lambda fd: [
                    python,
                    "-u",
                    "tools/teleop_services.py",
                    "cameras",
                    "--fleet-url",
                    args.fleet_url,
                    "--fleet-repo",
                    str(args.fleet_repo),
                    "--finish-fd",
                    str(fd),
                ]
            )
        else:
            camera_service.stop()
        camera_on = enabled
        camera_label.set_text("Waiting for camera frames…" if enabled else "Camera streams off")
        controls()

    def finish(*_):
        session.stop()
        status.set_text("Release requested · waiting for shutdown and recording")
        controls()

    def close(*_):
        nonlocal closing
        closing = True
        camera_service.stop()
        connection.stop()
        session.stop()
        controls()
        return any(x.process is not None for x in (camera_service, connection, session))

    def velocity(v):
        axes.set_text(
            f"Forward {v[0]:+.2f} m/s      Lateral {v[1]:+.2f} m/s      Yaw {v[2]:+.2f} rad/s"
        )

    def show_input(m):
        nonlocal last_hat_view
        markers.set_text(f"Markers   {m['markers']}")
        if m["view"] != last_hat_view:
            selector.set_active(m["view"] + 1)
            last_hat_view = m["view"]

    def update():
        nonlocal out, stage, diagnostic, connected, camera_on, phase, token, pending
        for service, widget, is_connection in [
            (connection, connection_label, True),
            (camera_service, camera_label, False),
        ]:
            while not service.messages.empty():
                msg = service.messages.get()
                if isinstance(msg, int):
                    service.reap()
                    if is_connection:
                        connected = False
                    else:
                        camera_on = False
                    if msg == 0:
                        widget.set_text("Disconnected" if is_connection else "Camera streams off")
                    else:
                        widget.set_text("Failed · " + widget.get_text())
                else:
                    widget.set_text(msg)
                    if is_connection:
                        connected = msg.startswith("Connected ·")
        while not session.messages.empty():
            msg = session.messages.get()
            if isinstance(msg, int):
                session.reap()
                if msg and out and not live and (out / "result.json").exists():
                    result = json.loads((out / "result.json").read_text())
                    diagnostic = result.get("error") or result.get("renderer_error") or diagnostic
                phase = token = pending = None
                if mode.get_active_id() == "sim":
                    camera_on = False
                    camera_label.set_text("Camera streams off")
                    connection_label.set_text("Disconnected")
                status.set_text(
                    ("Completed" if msg == 0 else "Stopped / failed")
                    + (
                        ": " + (out / "STATUS").read_text().strip()
                        if out and (out / "STATUS").is_file()
                        else ""
                    )
                    + (" · " + diagnostic if msg != 0 and diagnostic else "")
                )
            elif msg.startswith("Recording: "):
                out = Path(msg[11:])
                where.set_text(str(out))
                monitor_changed()
            elif msg.startswith("Monitor: "):
                show_input(json.loads(msg[9:]))
            elif msg.startswith("Velocity: "):
                velocity(json.loads(msg[10:]))
            else:
                if msg.startswith(
                    (
                        "Not started",
                        "Preflight failed",
                        "Trial stopped",
                        "Robot fault",
                        "ValueError",
                        "ContractError",
                        "Traceback",
                    )
                ):
                    diagnostic = msg
                status.set_text(msg)
        if session.process and out and live:
            sf = out / "STATUS"
            if sf.is_file():
                st = sf.read_text().strip()
                if st != stage and st:
                    stage = st
                    status.set_text(st)
            eventfile = out / "trial/events.jsonl"
            if eventfile.is_file():
                with eventfile.open("rb") as f:
                    f.seek(max(0, os.fstat(f.fileno()).st_size - 32000))
                    for line in reversed(f.read().splitlines()):
                        if b'"event": "joystick"' not in line:
                            continue
                        try:
                            data = json.loads(line)
                            velocity(data["velocity_command"])
                            if "monitor" in data:
                                show_input(data["monitor"])
                            break
                        except (ValueError, KeyError):
                            continue
        if not session.process and status.get_text().startswith("Idle"):
            present = Path(profile["device"]["joystick_device"]).exists()
            status.set_text(
                "Idle · joystick connected" if present else "Idle · joystick disconnected"
            )
        if session.process and out:
            state_file = (out / "trial" if live else out) / "operator-state.json"
            if state_file.is_file():
                state = json.loads(state_file.read_text())
                if state["token"] != token:
                    phase, token, pending = state["phase"], state["token"], None
                    status.set_text(
                        {
                            "seated": "Seated",
                            "native": "Standing · Spot native control",
                            "relic": "Standing · ReLIC joystick control",
                            "returning": "Returning to native stand…",
                            "released": "Released · seated",
                            "enabling": "Enabling ReLIC…",
                            "releasing": "Sitting and releasing…",
                        }.get(phase, phase)
                    )
                    if not live:
                        connection_label.set_text("Connected to MuJoCo · native poses simulated")
        if closing and not any(x.process for x in (camera_service, connection, session)):
            win.destroy()
            return False
        controls()
        cameras.refresh()
        if camera_on and cameras.source is not None:
            if cameras.source == "live" and cameras.deadlines:
                camera_label.set_text(f"{len(cameras.deadlines)} fresh camera views")
            elif cameras.source == "live" and camera_label.get_text().endswith(
                "fresh camera views"
            ):
                camera_label.set_text("Waiting for fresh camera frames…")
            elif cameras.source != "live" and cameras.tokens:
                camera_label.set_text("Simulated room camera streams")
        return True

    connect.connect("clicked", connect_pressed)
    streams.connect("clicked", lambda *_: set_cameras(not camera_on))
    toggle.connect("clicked", change_mode)
    start.connect("clicked", pressed)
    stop.connect("clicked", finish)
    win.connect("delete-event", close)
    win.connect("destroy", lambda *_: Gtk.main_quit())
    if not Path(profile["device"]["joystick_device"]).exists():
        status.set_text("Idle · USB joystick not connected")
    GLib.timeout_add(200, update)
    win.show_all()
    controls()
    if args.live_monitor:
        connect_pressed()
        set_cameras(True)
    Gtk.main()
    cameras.close()


if __name__ == "__main__":
    main()

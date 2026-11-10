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
        self.writer = None
        self.messages = queue.Queue()

    def start(self, command):
        if self.process is not None:
            raise RuntimeError("session still running or cleaning up")
        rd, wr = os.pipe()
        try:
            self.process = subprocess.Popen(
                command(rd),
                cwd=ROOT,
                pass_fds=(rd,),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except BaseException:
            os.close(wr)
            raise
        finally:
            os.close(rd)
        self.writer = wr
        proc = self.process

        def collect():
            with proc.stdout:
                for line in proc.stdout:
                    self.messages.put(line.rstrip()[:1200])
            self.messages.put(proc.wait())

        threading.Thread(target=collect, daemon=True).start()

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
    args = p.parse_args()
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
    session = Session()
    win = Gtk.Window(title="ReLIC · joystick demo recorder")
    win.set_default_size(590, 360)
    panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
    panel.set_border_width(20)
    win.add(panel)

    def label(text):
        w = Gtk.Label(label=text, xalign=0, selectable=True, wrap=True)
        panel.pack_start(w, False, False, 0)
        return w

    label("ReLIC   /   Walking demo")
    label("Stowed arm · Forward / lateral / yaw · Release enable for zero velocity")
    label(
        f"Center stick, release then hold printed {profile['enable_button'] + 1} to drive. "
        f"Printed {profile['finish_button'] + 1} finishes. Trigger does not deploy the arm."
    )
    if family:
        label(
            f"Registered {family} repeat · Physical mode follows the frozen trace while enable is held. "
            "USB-only mode records manual stick input."
        )
    mode = Gtk.ComboBoxText()
    mode.append_text("USB only — record commands without Spot")
    if bundle:
        mode.append_text("Physical Spot — record states, actions and commands")
    mode.set_active(0)
    panel.pack_start(mode, False, False, 0)
    fields = Gtk.Box(spacing=6)
    panel.pack_start(fields, False, False, 0)
    operator = Gtk.Entry(placeholder_text="Robot operator")
    safety = Gtk.Entry(placeholder_text="Tablet E-stop operator")
    for w in [operator, safety]:
        fields.pack_start(w, True, True, 0)
    physical_info = label(
        "Physical Start performs preflight, native stand and ReLIC motion. Stop requests normal shutdown; use the tablet for emergency stop."
    )

    def show_mode(*_):
        physical = mode.get_active() == 1
        fields.set_visible(physical)
        physical_info.set_visible(physical)

    mode.connect("changed", show_mode)
    limits = profile["axes"]
    label(
        f"Stick limits   ±{limits[0]['maximum']:g} m/s forward     "
        f"±{limits[1]['maximum']:g} m/s lateral     ±{limits[2]['maximum']:g} rad/s yaw"
    )
    status = label("Idle")
    axes = label("Forward —     Lateral —     Yaw —")
    where = label("No recording yet")
    buttons = Gtk.Box(spacing=8)
    panel.pack_end(buttons, False, False, 0)
    start = Gtk.Button(label="Start recording")
    stop = Gtk.Button(label="Stop and save")
    for w in [start, stop]:
        buttons.pack_start(w, True, True, 0)
    stop.set_sensitive(False)
    out = None
    closing = False
    live = False
    stage = None
    diagnostic = ""

    def pressed(*_):
        nonlocal out, live, stage, diagnostic
        if session.process:
            return
        live = mode.get_active() == 1
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
                text="Start one physical ReLIC trial now?",
            )
            d.format_secondary_text(
                (
                    f"Replay registered {family} commands with held enable. "
                    if family
                    else "Drive with the stick. "
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
        out = None
        diagnostic = ""
        stage = None
        dest = (
            ROOT
            / "runs"
            / ("joystick-stick-" + time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
        )

        def command(fd):
            if live:
                return [
                    python,
                    "-u",
                    "tools/run_joystick_bundle.py",
                    "--bundle",
                    str(bundle),
                    "--demo-finish-fd",
                    str(fd),
                    "--operator",
                    names[0],
                    "--safety-operator",
                    names[1],
                    "--execute",
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
            session.start(command)
        except OSError as e:
            status.set_text(str(e))
            return
        status.set_text("Checking readiness…" if live else "Recording USB commands…")
        where.set_text("")
        axes.set_text("")
        for w in [start, mode, fields]:
            w.set_sensitive(False)
        stop.set_sensitive(True)

    def finish(*_):
        session.stop()
        stop.set_sensitive(False)
        status.set_text("Stop requested · waiting for cleanup and files")

    def close(*_):
        nonlocal closing
        if session.process:
            closing = True
            finish()
            return True
        return False

    def velocity(v):
        axes.set_text(
            f"Forward {v[0]:+.2f} m/s      Lateral {v[1]:+.2f} m/s      Yaw {v[2]:+.2f} rad/s"
        )

    def update():
        nonlocal out, stage, diagnostic
        while not session.messages.empty():
            msg = session.messages.get()
            if isinstance(msg, int):
                session.stop()
                session.process = None
                status.set_text(
                    ("Completed" if msg == 0 else "Stopped / failed")
                    + (
                        ": " + (out / "STATUS").read_text().strip()
                        if out and (out / "STATUS").is_file()
                        else ""
                    )
                    + (" · " + diagnostic if msg != 0 and diagnostic else "")
                )
                for w in [start, mode, fields]:
                    w.set_sensitive(True)
                stop.set_sensitive(False)
                if closing:
                    win.destroy()
                    return False
            elif msg.startswith("Recording: "):
                out = Path(msg[11:])
                where.set_text(str(out))
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
                            velocity(json.loads(line)["velocity_command"])
                            break
                        except (ValueError, KeyError):
                            continue
        return True

    start.connect("clicked", pressed)
    stop.connect("clicked", finish)
    win.connect("delete-event", close)
    win.connect("destroy", lambda *_: Gtk.main_quit())
    if not Path(profile["device"]["joystick_device"]).exists():
        status.set_text("Idle · USB joystick not connected")
    GLib.timeout_add(200, update)
    win.show_all()
    show_mode()
    Gtk.main()


if __name__ == "__main__":
    main()

"""GTK camera monitor. Physical frames come exclusively from the fleet GET API."""

from concurrent.futures import ThreadPoolExecutor
import json
import math
import time
from urllib.request import urlopen

from room_array import NAMES

AGE_LIMIT = 1.0


def live_frame(base, name):
    started = time.monotonic()
    with urlopen(base.rstrip("/") + "/frame-snapshot/" + name, timeout=0.7) as r:
        meta = json.loads(r.headers["X-Fleet-Frame"])
        ages = [float(meta[k]) for k in ("capture_age_s", "preview_age_s")]
        if not all(math.isfinite(age) and age >= 0 for age in ages):
            raise ValueError("invalid age")
        age = max(ages)
        deadline = started + AGE_LIMIT - age
        if meta["camera"] != name or meta["live"] is not True or deadline <= time.monotonic():
            raise ValueError("stale frame")
        jpeg = r.read(4 * 1024 * 1024 + 1)
        if len(jpeg) > 4 * 1024 * 1024 or not jpeg.startswith(b"\xff\xd8"):
            raise ValueError("invalid preview")
    return (
        (meta["receiver_session"], meta["frame_key"]),
        deadline,
        jpeg,
        meta["rotation_deg"],
        meta["roi"],
    )


class CameraPanel:
    def __init__(self, Gtk, base):
        from gi.repository import GdkPixbuf

        self.Gtk, self.pix, self.base = Gtk, GdkPixbuf, base
        self.grid = Gtk.Grid(column_spacing=10, row_spacing=5)
        self.pool = ThreadPoolExecutor(max_workers=len(NAMES))
        self.pending = {}
        self.labels = {}
        self.images = {}
        self.tiles = {}
        self.tokens = {}
        self.deadlines = {}
        self.focus = -1
        self.source = None
        self.last_request = 0.0
        for i, name in enumerate(NAMES):
            box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
            self.tiles[name] = box
            self.images[name] = Gtk.Image()
            self.labels[name] = Gtk.Label(label=name)
            box.pack_start(self.images[name], True, True, 0)
            box.pack_end(self.labels[name], False, False, 0)
            self.grid.attach(box, i % 3, i // 3, 1, 1)

    def select(self, index):
        self.focus = index
        for i, name in enumerate(NAMES):
            self.tiles[name].set_visible(index in (-1, i))
        self.tokens.clear()

    def reset(self, source):
        if source == self.source:
            return
        self.source = source
        for f in self.pending.values():
            f.cancel()
        self.pending.clear()
        self.tokens.clear()
        self.deadlines.clear()
        for n in NAMES:
            self.clear(n, "waiting")

    def clear(self, n, why):
        self.images[n].clear()
        self.labels[n].set_text(n + " · " + why)
        self.deadlines.pop(n, None)

    def image(self, n, data, rotation):
        loader = self.pix.PixbufLoader.new_with_type("jpeg")
        width = 260 if self.focus == -1 else 650
        loader.connect("size-prepared", lambda obj, w, h: obj.set_size(width, round(width * h / w)))
        loader.write(data)
        loader.close()
        pb = loader.get_pixbuf()
        if rotation:
            pb = pb.rotate_simple(
                {
                    90: self.pix.PixbufRotation.CLOCKWISE,
                    180: self.pix.PixbufRotation.UPSIDEDOWN,
                    270: self.pix.PixbufRotation.COUNTERCLOCKWISE,
                }[rotation]
            )
        self.images[n].set_from_pixbuf(pb)

    def refresh(self):
        if self.source is None:
            return
        now = time.monotonic()
        names = NAMES if self.focus == -1 else (NAMES[self.focus],)
        if self.source != "live":
            try:
                meta = json.loads((self.source / "views/frame.json").read_text())
                stale = now - meta["sampled_at"] > AGE_LIMIT
                for n in names:
                    if self.tokens.get(n) != meta["tick"]:
                        self.image(n, (self.source / "views" / (n + ".jpg")).read_bytes(), 0)
                        self.tokens[n] = meta["tick"]
                    self.labels[n].set_text(
                        f"{n} · sim {meta['seconds']:.1f} s" + (" · last frame" if stale else "")
                    )
            except (OSError, ValueError):
                pass
            return
        for n in names:
            future = self.pending.get(n)
            if future and future.done():
                del self.pending[n]
                try:
                    token, expires, jpeg, rotation, roi = future.result()
                    if token != self.tokens.get(n):
                        if expires <= now:
                            raise ValueError("expired")
                        self.image(n, jpeg, rotation)
                        self.tokens[n] = token
                        self.deadlines[n] = expires
                        self.labels[n].set_text(f"{n} · live · ROI {roi}")
                except Exception:
                    self.clear(n, "unavailable")
            if n in self.deadlines and self.deadlines[n] <= now:
                self.clear(n, "stale")
            if n not in self.pending and now - self.last_request >= 0.24:
                self.pending[n] = self.pool.submit(live_frame, self.base, n)
        if now - self.last_request >= 0.24:
            self.last_request = now

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)

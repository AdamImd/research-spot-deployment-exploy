"""Bounded, atomic run records with source, dependency and input provenance."""

import importlib.metadata
import json
import platform
import queue
import socket
import subprocess
import threading
from datetime import datetime, timezone
from pathlib import Path

from .contracts import ContractError, canonical_hash, sha256


def utcnow():
    return datetime.now(timezone.utc).isoformat()


def root():
    return Path(__file__).resolve().parents[2]


def source_identity():
    base = root()
    paths = sorted((base / "src").rglob("*.py"))
    paths += sorted((base / "src" / "spot_deploy" / "web").rglob("*"))
    firmware = base / "firmware" / "esp32_estop"
    paths += sorted((firmware / "src").glob("*")) + sorted((firmware / "include").glob("*"))
    paths += [firmware / "platformio.ini", base / "pyproject.toml", base / "uv.lock"]
    paths += [base / "requirements-exploy-export.txt"]
    hashes = {str(p.relative_to(base)): sha256(p) for p in paths if p.is_file()}

    def git(*args):
        return subprocess.run(
            ["git", "-C", str(base), *args], capture_output=True, text=True, timeout=5, check=True
        ).stdout.strip()

    try:
        commit, dirty = git("rev-parse", "HEAD"), bool(git("status", "--porcelain"))
    except (subprocess.SubprocessError, OSError):
        commit, dirty = None, True
    return {
        "commit": commit,
        "dirty": dirty,
        "source_sha256": canonical_hash(hashes),
        "files": hashes,
    }


def atomic_json(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n")
    temp.replace(path)


def runtime_packages():
    """Record source-only/isolated benchmark environments as well as installed CLIs."""
    result = {}
    for name in ("spot-deployment", "numpy", "onnxruntime", "onnxruntime-gpu",
                 "bosdyn-api", "bosdyn-client", "pydantic"):
        try:
            result[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result[name] = None
    return result


class RunRecord:
    def __init__(self, directory: Path, mode: str, inputs=()):
        directory.mkdir(parents=True, exist_ok=False)
        self.directory = directory
        self.source = source_identity()
        self.meta = {
            "schema_version": 1,
            "mode": mode,
            "started_at": utcnow(),
            "host": socket.gethostname(),
            "python": platform.python_version(),
            "source": self.source,
            "inputs": {
                str(Path(p).resolve()): sha256(Path(p)) for p in inputs if p and Path(p).is_file()
            },
            "packages": runtime_packages(),
            "retry_policy": "none",
            "resource": "local CPU",
            "paid_budget": 0,
        }
        atomic_json(directory / "run.json", self.meta)
        self._events = (directory / "events.jsonl").open("x")
        self._queue = queue.Queue(maxsize=2048)
        self._writer_error = None
        self._finished = False
        self._writer = threading.Thread(target=self._write_events, name="run-record", daemon=True)
        self._writer.start()

    def _write_events(self):
        try:
            while True:
                value = self._queue.get()
                if value is None:
                    break
                self._events.write(value)
                self._events.flush()
        except BaseException:
            self._writer_error = True
        finally:
            self._events.close()

    def event(self, kind, **fields):
        if self._writer_error or self._finished:
            raise ContractError("run recorder unavailable")
        try:
            self._queue.put_nowait(
                json.dumps({"time": utcnow(), "event": kind, **fields}, allow_nan=False) + "\n"
            )
        except queue.Full:
            raise ContractError("run recorder overflow") from None

    def finish(self, status, result):
        if self._finished:
            return
        self._finished = True
        try:
            self._queue.put(None, timeout=2)
        except queue.Full:
            self._writer_error = True
        self._writer.join(2)
        if self._writer.is_alive() or self._writer_error:
            # Preserve partial logs, but never hash a file that can still be changing.
            atomic_json(
                self.directory / "FAILED.json",
                {
                    "status": "failed",
                    "error": "Recorder shutdown failed; no valid completion record",
                },
            )
            raise ContractError("run recorder shutdown failed")
        atomic_json(self.directory / "result.json", result)
        artifacts = {
            str(p.relative_to(self.directory)): sha256(p)
            for p in sorted(self.directory.rglob("*"))
            if p.is_file() and p != self.directory / "COMPLETE.json"
        }
        atomic_json(
            self.directory / "COMPLETE.json",
            {
                "schema_version": 1,
                "finished_at": utcnow(),
                "status": status,
                "artifacts": artifacts,
            },
        )


def verify_run(directory, *, status="passed"):
    base = Path(directory).resolve()
    complete = json.loads((base / "COMPLETE.json").read_text())
    if complete["status"] != status:
        return False
    for name, digest in complete["artifacts"].items():
        path = (base / name).resolve()
        if not path.is_relative_to(base) or not path.is_file() or sha256(path) != digest:
            return False
    return True

"""Read-only Spot connection and separately owned room-camera session."""

import argparse
import errno
import json
from pathlib import Path
import select
import subprocess
from urllib.error import URLError
from urllib.request import urlopen
import uuid

from room_array import PATHS


def connection(bundle, fd):
    from spot_deploy.contracts import RobotConfig, load
    from spot_deploy.network import wired_route
    from spot_deploy.sdk_read import ReadOnlySpot

    config = load(bundle / "robot.json", RobotConfig)
    wired_route(config)
    reader = ReadOnlySpot(config)
    try:
        reader.connect(streaming=False)
        snapshot = reader.snapshot()
        if not all(snapshot[k] for k in ("identity_matches", "joint_layout_matches", "has_arm")):
            raise ValueError("Spot identity, arm or joint layout mismatch")
        while True:
            health = reader.health()
            print(
                "Connected · tablet stop "
                + ("ready" if health["estop_ready"] else "engaged or unavailable"),
                flush=True,
            )
            if select.select([fd], [], [], 1)[0]:
                return
    finally:
        reader.close()


def fleet(url, repo, fd):
    proc = None
    try:
        with urlopen(url.rstrip("/") + "/status", timeout=1.5) as response:
            status = json.load(response)
        if status.get("schema") != 1 or "cameras" not in status or status.get("state") != "running":
            raise ValueError("Endpoint is not a running room-camera service")
        print("Attached to existing camera service", flush=True)
    except URLError as exc:
        if getattr(exc.reason, "errno", None) != errno.ECONNREFUSED:
            raise
        config = json.loads((repo / PATHS[3]).read_text())
        if url.rstrip("/") != f"http://{config['bind']}:{config['port']}":
            raise ValueError("Camera launcher must match the configured fleet address")
        out = repo / "data/fleet-tracking" / ("teleop-" + uuid.uuid4().hex)
        log = out.with_suffix(".log")
        log.parent.mkdir(parents=True, exist_ok=True)
        if select.select([fd], [], [], 0)[0]:
            return
        with log.open("x") as stream:
            proc = subprocess.Popen(
                [
                    str(repo / "scripts/fleet-track"),
                    "run",
                    "--cameras",
                    "all",
                    "--config",
                    PATHS[3],
                    "--calibration",
                    PATHS[0],
                    "--out",
                    str(out),
                    "--seconds",
                    "1200",
                ],
                cwd=repo,
                stdout=stream,
                stderr=subprocess.STDOUT,
            )
        print(f"Starting camera streams · log {log}", flush=True)
    try:
        while not select.select([fd], [], [], 0.25)[0]:
            if proc is not None and proc.poll() is not None:
                if proc.returncode:
                    raise RuntimeError(f"Camera service exited {proc.returncode}; see {log}")
                return
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            proc.wait()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("service", choices=["connect", "cameras"])
    parser.add_argument("--finish-fd", required=True, type=int)
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--fleet-url")
    parser.add_argument("--fleet-repo", type=Path)
    args = parser.parse_args()
    if args.service == "connect":
        connection(args.bundle, args.finish_fd)
    else:
        fleet(args.fleet_url, args.fleet_repo.resolve(), args.finish_fd)

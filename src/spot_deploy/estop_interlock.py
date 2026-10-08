"""Additional runtime gate; Spot's independent E-stop service remains stop authority."""

import json
import socket
import time
from pathlib import Path

from .contracts import ContractError, canonical_hash


def boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def robot_binding(config):
    return canonical_hash(config.model_dump(mode="json"))


class HardwareInterlock:
    def __init__(self, config):
        self.requirement = config.hardware_estop
        self.robot_hash = robot_binding(config)
        self.host = socket.gethostname()
        self.boot = boot_id()
        self.session = None
        self.endpoint_id = None

    def check(self, health=None):
        if self.requirement is None:
            return
        try:
            path = Path(self.requirement.status_file)
            with path.open("rb") as stream:
                raw = stream.read(8193)
            if len(raw) > 8192:
                raise ValueError()
            status = json.loads(raw)
            timestamp = status["updated_monotonic_s"]
            if type(timestamp) not in (int, float):
                raise ValueError()
            age = time.monotonic() - timestamp
            if not 0 <= age <= self.requirement.max_status_age_s:
                raise ValueError()
            if (
                status["schema_version"] != 1
                or status["mode"] != "robot"
                or status["state"] != "armed"
                or status["robot_confirmed"] is not True
                or status["host"] != self.host
                or status["host_boot_id"] != self.boot
                or status["profile_sha256"] != self.requirement.profile_sha256
                or status["robot_binding_sha256"] != self.robot_hash
                or not status["session"]
                or not status["endpoint_id"]
            ):
                raise ValueError()
            if self.session is not None and (
                status["session"] != self.session or status["endpoint_id"] != self.endpoint_id
            ):
                raise ValueError()
            if health is not None:
                endpoints = health.get("estop_endpoints", [])
                matches = [e for e in endpoints if e["id"] == status["endpoint_id"]]
                if len(matches) != 1 or not matches[0]["allowed"]:
                    raise ValueError()
            self.session, self.endpoint_id = status["session"], status["endpoint_id"]
        except (OSError, ValueError, TypeError, KeyError):
            raise ContractError(
                "hardware E-stop is stopped, stale, unconfirmed or mismatched"
            ) from None

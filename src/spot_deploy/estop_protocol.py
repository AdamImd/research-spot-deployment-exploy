"""USB E-stop wire contract, independent of serial, SDK and UI dependencies."""

import re
import zlib
from dataclasses import dataclass
from typing import Literal

from pydantic import Field, model_validator

from .contracts import ContractError, StrictModel

MAX_FRAME = 160
FIRMWARE_WATCHDOG_S = 0.25


class EstopProfile(StrictModel):
    schema_version: Literal[1]
    serial_device: str = Field(pattern=r"^/dev/serial/by-id/[^/]+$")
    device_id: str = Field(pattern=r"^[0-9a-f]{12}$")
    baud: Literal[115200]
    endpoint_name: str = Field(pattern=r"^[A-Za-z0-9_-]{1,40}$")
    endpoint_role: str = Field(pattern=r"^spot_esp32_[A-Za-z0-9_-]{1,32}$")
    poll_interval_s: float = Field(ge=0.02, le=0.08)
    response_timeout_s: float = Field(ge=0.01, le=0.08)
    rpc_timeout_s: float = Field(ge=0.02, le=0.1)
    endpoint_timeout_s: float = Field(ge=0.2, le=1)
    cut_power_timeout_s: float = Field(ge=0.2, le=2)
    stop_level: Literal["cut", "settle_then_cut"]

    @model_validator(mode="after")
    def timing(self):
        cycle = self.poll_interval_s + self.response_timeout_s + self.rpc_timeout_s
        if cycle >= min(self.endpoint_timeout_s, FIRMWARE_WATCHDOG_S):
            raise ValueError("poll, response and RPC budgets must fit both watchdogs")
        if self.cut_power_timeout_s < self.endpoint_timeout_s:
            raise ValueError("hard cut timeout cannot precede endpoint timeout")
        return self


@dataclass(frozen=True)
class Packet:
    device_id: str
    boot: str
    sequence: int
    uptime_ms: int
    nonce: str
    stopped: bool
    armed: bool


def request(nonce):
    if not re.fullmatch(r"[0-9a-f]{16}", nonce):
        raise ContractError("invalid E-stop challenge")
    return f"Q,{nonce}\n".encode("ascii")


def parse(data: bytes, nonce: str, device_id: str) -> Packet:
    if len(data) > MAX_FRAME or not data.endswith(b"\n"):
        raise ContractError("incomplete or oversized E-stop frame")
    try:
        payload, crc = data[:-1].rsplit(b",", 1)
        if not re.fullmatch(rb"[0-9a-f]{8}", crc):
            raise ValueError()
        if zlib.crc32(payload) != int(crc, 16):
            raise ValueError()
        fields = payload.decode("ascii").split(",")
        version, device, boot, sequence, uptime, echoed, stopped, armed = fields
        if (
            version != "E1"
            or device != device_id
            or not re.fullmatch(r"[0-9a-f]{12}", device)
            or not re.fullmatch(r"[0-9a-f]{8}", boot)
            or echoed != nonce
            or not re.fullmatch(r"[0-9a-f]{16}", echoed)
            or not re.fullmatch(r"[0-9]{1,10}", sequence)
            or not re.fullmatch(r"[0-9]{1,10}", uptime)
            or int(sequence) > 0xFFFFFFFF
            or int(uptime) > 0xFFFFFFFF
            or (stopped, armed) not in (("1", "0"), ("0", "1"))
        ):
            raise ValueError()
        return Packet(
            device, boot, int(sequence), int(uptime), echoed, stopped == "1", armed == "1"
        )
    except (ValueError, UnicodeError):
        raise ContractError("invalid E-stop identity, challenge, flags or checksum") from None


class PhysicalLatch:
    """Require observed STOP before allowing; reboot/replay is a terminal fault.

    Only fresh challenged firmware responses can change this state. After an error,
    the owning bridge exits; reconstructing this object alone does not arm it.
    """

    def __init__(self):
        self.previous = None
        self.observed_stop = False

    def accept(self, packet):
        old = self.previous
        if old and (
            packet.boot != old.boot
            or packet.sequence <= old.sequence
            or packet.uptime_ms < old.uptime_ms
        ):
            raise ContractError("E-stop reboot or non-increasing sequence/uptime")
        self.previous = packet
        if packet.stopped:
            self.observed_stop = True
        return self.observed_stop and packet.armed and not packet.stopped

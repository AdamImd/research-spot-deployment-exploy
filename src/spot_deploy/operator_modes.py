"""Local operator mode requests. The worker alone acknowledges state changes."""

import os
import stat
import uuid

from .contracts import ContractError
from .records import atomic_json


class OperatorModes:
    def __init__(self, fd, directory, record):
        if not stat.S_ISFIFO(os.fstat(fd).st_mode):
            raise ContractError("operator control requires an inherited pipe")
        os.set_blocking(fd, False)
        self.fd = fd
        self.path = directory / "operator-state.json"
        self.record = record
        self.token = None

    def publish(self, phase, reason=None):
        self.token = uuid.uuid4().hex
        state = dict(phase=phase, token=self.token)
        if reason:
            state["reason"] = reason
        atomic_json(self.path, state)
        self.record.event("operator_state", **state)

    def requested(self):
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return False
        if not data:
            raise ContractError("operator mode connection closed")
        # Each small write is atomic on a local pipe. Old clicks cannot arm
        # another segment after returning through the same displayed state.
        return self.token.encode() in data.splitlines()

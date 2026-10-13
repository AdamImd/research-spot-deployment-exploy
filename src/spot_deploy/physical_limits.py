"""Continuous physical-limit persistence; never handles transport or data validity."""

import numpy as np

from .contracts import ContractError


class PhysicalLimits:
    def __init__(self, persistence_s=0.0, event=None):
        self.persistence_s = persistence_s
        self.event = event
        self.enabled = True
        self.pending = {}
        self.last_at = None

    def check(self, violations, now, message, *, key=None, values=None):
        if not np.isfinite(now) or (self.last_at is not None and now < self.last_at):
            raise ContractError("physical-limit clock is nonfinite or moved backwards")
        self.last_at = now
        mask = np.atleast_1d(violations).astype(bool)
        if not self.pending and not mask.any():
            return
        group = key or message
        duration = self.persistence_s if self.enabled else 0.0
        for index, violated in enumerate(mask):
            identity = (group, index)
            since = self.pending.get(identity)
            if not violated and since is None:
                continue
            detail = dict(
                limit=group, index=index, reason=message, monotonic_s=now, persistence_s=duration
            )
            if values is not None:
                detail["value"] = float(np.atleast_1d(values)[index])
            if not violated:
                if since is not None:
                    del self.pending[identity]
                    self._event(
                        "physical_limit_recovered", **detail, since_s=since, elapsed_s=now - since
                    )
                continue
            if since is None:
                since = now
                self.pending[identity] = since
                self._event("physical_limit_started", **detail, since_s=since, elapsed_s=0.0)
            # Compare absolute deadlines to preserve the strict >500ms boundary.
            if duration == 0.0 or now > since + duration:
                self._event("physical_limit_stop", **detail, since_s=since, elapsed_s=now - since)
                raise ContractError(
                    message
                    if duration == 0.0
                    else f"{message} persisted {now - since:.6f}s > {duration:.6f}s"
                )

    def _event(self, name, **data):
        if self.event is not None:
            self.event(name, **data)

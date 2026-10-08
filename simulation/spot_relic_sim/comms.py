"""Seeded, simulation-time datagram impairment; these are synthetic test presets."""
from copy import deepcopy
from dataclasses import asdict, dataclass
import heapq
import numpy as np

FRESHNESS = .250
STREAMS = ("command", "state", "target", "telemetry")


@dataclass(frozen=True)
class Profile:
    delay_s: float = 0.
    jitter_s: float = 0.
    loss: float = 0.
    outage_start_s: float = 5.
    outage_period_s: float = 0.
    outage_duration_s: float = 0.

    def outage(self, t):
        return (self.outage_period_s > 0 and t >= self.outage_start_s and
                (t - self.outage_start_s) % self.outage_period_s < self.outage_duration_s)


PROFILES = {
    "none": Profile(),
    "wifi_average": Profile(delay_s=.010, jitter_s=.005, loss=.01),
    "wifi_worst_case": Profile(delay_s=.080, jitter_s=.040, loss=.10,
                               outage_period_s=10., outage_duration_s=.500),
}


@dataclass(frozen=True)
class CommsConfig:
    profile: str = "none"
    scope: str = "offboard"
    seed: int = 101

    def __post_init__(self):
        if self.profile not in PROFILES or self.scope not in ("offboard", "teleop"):
            raise ValueError("Unknown communications profile or scope")
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError("Communications seed must be an integer in [0, 2**32)")

    def metadata(self):
        return asdict(self) | dict(preset=asdict(PROFILES[self.profile]), synthetic=True,
            clock="simulation_seconds", delivery_tick_s=.020, freshness_s=FRESHNESS,
            rng="numpy.PCG64; SeedSequence([seed, stream_index])",
            impaired_streams=list(STREAMS if self.scope == "offboard" else ("command", "telemetry")))


def add_comms_arguments(parser):
    parser.add_argument("--comms", choices=list(PROFILES), default="none",
                        help="Synthetic Wi-Fi latency/loss preset (default: none)")
    parser.add_argument("--comms-scope", choices=["offboard", "teleop"], default="offboard",
                        help="Offboard: also delay RL states/targets; teleop: policy stays onboard")
    parser.add_argument("--comms-seed", type=int, default=101,
                        help="Independent reproducible packet impairment seed")


def config_from_args(args):
    return CommsConfig(args.comms, args.comms_scope, args.comms_seed)


@dataclass
class Packet:
    sequence: int
    sent_at: float
    payload: object


class Link:
    """Latest-packet-wins UDP model, with sender-age expiry and bounded queues."""
    def __init__(self, profile, seed=101, stream=0, capacity=256):
        self.profile = profile
        self.rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed, stream])))
        self.capacity = capacity
        self.queue = []
        self.latest = None
        self.last_sequence = -1
        self.counter = 0
        self.counts = dict(sent=0, random_loss=0, outage_loss=0, overflow=0,
                           delivered=0, reordered=0, expired=0)

    def send(self, payload, now, *, sent_at=None, sequence=None):
        self.counter += 1
        self.counts["sent"] += 1
        if self.profile.outage(now):
            self.counts["outage_loss"] += 1
            return
        if self.profile.loss and self.rng.random() < self.profile.loss:
            self.counts["random_loss"] += 1
            return
        delay = self.profile.delay_s
        if self.profile.jitter_s:
            delay += self.rng.uniform(-self.profile.jitter_s, self.profile.jitter_s)
        if len(self.queue) >= self.capacity:
            self.counts["overflow"] += 1
            return
        packet = Packet(self.counter if sequence is None else sequence,
                        now if sent_at is None else sent_at, deepcopy(payload))
        heapq.heappush(self.queue, (now + max(0., delay), self.counter, packet))

    def poll(self, now, max_age=FRESHNESS):
        while self.queue and self.queue[0][0] <= now + 1e-12:
            arrival, _, packet = heapq.heappop(self.queue)
            if self.profile.outage(arrival):
                self.counts["outage_loss"] += 1
            elif packet.sequence <= self.last_sequence:
                self.counts["reordered"] += 1
            elif max_age is not None and now - packet.sent_at >= max_age:
                self.counts["expired"] += 1
            else:
                self.latest = packet
                self.last_sequence = packet.sequence
                self.counts["delivered"] += 1
        if self.latest is None or (max_age is not None and now - self.latest.sent_at >= max_age):
            return None
        return self.latest

    def age(self, now):
        return None if self.latest is None else max(0., now - self.latest.sent_at)

    def summary(self):
        return self.counts | dict(pending=len(self.queue))


class Communications:
    def __init__(self, config=None):
        self.config = config or CommsConfig()
        self.links = {}
        for i, name in enumerate(STREAMS):
            local = self.config.scope == "teleop" and name in ("state", "target")
            self.links[name] = Link(PROFILES["none" if local else self.config.profile], self.config.seed, i)

    def summary(self):
        return dict(configuration=self.config.metadata(),
                    streams={name: link.summary() for name, link in self.links.items()})


class ControllerLink:
    """Remote policy at 50 Hz; robot-local PD stays at 200 Hz in each backend."""
    def __init__(self, contract, policy, communications):
        self.contract, self.policy = contract, policy
        self.states = communications.links["state"]
        self.targets = communications.links["target"]
        self.action = np.zeros(12, dtype=np.float32)
        self.target = contract.q0.copy()
        self.observation = None

    def step(self, state, command, t):
        self.states.send(state, t)
        received_state = self.states.poll(t)
        if received_state is not None:
            self.observation = self.contract.observation(received_state.payload, command, self.action)
            self.action = self.policy(self.observation)
            self.target = self.contract.targets(self.action, command)
            self.targets.send(self.target, t)
        received_target = self.targets.poll(t)
        applied = self.contract.q0.copy() if received_target is None else received_target.payload
        failure = None
        if t >= FRESHNESS:
            if received_state is None:
                failure = "comms_state_timeout"
            elif received_target is None:
                failure = "comms_target_timeout"
        status = dict(state_age_s=self.states.age(t), target_age_s=self.targets.age(t),
                      policy_updated=received_state is not None,
                      state_sequence=None if received_state is None else received_state.sequence,
                      target_sequence=None if received_target is None else received_target.sequence)
        return applied, failure, status

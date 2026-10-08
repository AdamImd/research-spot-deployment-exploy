# Architecture and contracts

## Capability boundaries

The CLI resolves and validates local inputs before constructing any SDK client.
`policy`, `contracts`, `safety`, `readiness`, `records` and `runtime.replay` require
no robot access. `sdk_read.ReadOnlySpot` exposes authentication, time sync, identity,
health and state streaming only. `sdk_control` is imported by the CLI only after
the standing gate passes. SDK registration itself is not an authorization boundary:
the tested boundary is that read-only CLI paths never call control APIs.

No mode discovers a robot address, acquires a lease to inspect state, remotely
resets the ESP32 latch, takes another operator's lease, or clears a robot fault. Robot credentials
come only from `BOSDYN_CLIENT_USERNAME` and `BOSDYN_CLIENT_PASSWORD`. They are never
accepted as CLI arguments or configuration fields. SDK error strings and Pydantic
input values are omitted from error records to avoid accidental secret disclosure.

## Policy manifest v1

`schemas/Manifest.json` is authoritative. A manifest binds an ONNX model, the exact
training configuration, training commit, robot URDF hash and payload-configuration
hash. Paths stay under the manifest directory and symlinks cannot escape it.
The only supported task is standing on `spot-arm-stowed` morphology.

The model has one fixed float32 input `[1, N]` and output `[1, 12]`, named explicitly
by the manifest. Recurrent states, dynamic batches and custom adapters are rejected.
CPU execution uses one intra-op and one inter-op thread. No GPU is needed.

Observation terms are declared in network order. Each uses `(value - offset) * scale`;
the concatenated frame is normalized by `(frame - mean) / std`, optionally clipped, and stacked
oldest-first. History initialization is explicitly zero or repeated first frame.
Body velocities and unit gravity use the inverse of the body-to-odom rotation.
Quaternion convention is **w, x, y, z**; accepted rounding drift is normalized.
Joint positions are relative to the
manifest's defaults; observation and action joint permutations are independent.
The previous-action term uses network output order, with raw/clipped behavior
explicitly declared. The optional velocity command is always zero in this package.

Actions are optionally clipped to the declared training bounds, scaled and offset in policy
joint order, then mapped by name into SDK order. All arm/gripper targets come from
the fixed stow contract. Gains and feedforward are explicit 19-element vectors.
No stow pose, gain or physical envelope is borrowed from the reference handstand
runtime. Policy gain changes require renewed qualification.

Clip fields are always required: observation `clip: null` and action
`clip_min: null, clip_max: null` explicitly disable clipping. Missing fields and
one-sided bounds are rejected. No clipping is needed for parity with the pinned
reference; a safety limit must not silently change the network's trained semantics.
Non-finite conversions and float32 observation overflow are rejected before ONNX.
See [joint-order and runtime audit](UPSTREAM_PARITY.md) for the exact permutation,
45-input reference contract and intentional runtime differences.

Canonical names: leg triples `fl`, `fr`, `hl`, `hr`, each `hx`, `hy`, `kn`, followed
by `arm0_sh0`, `arm0_sh1`, `arm0_el0`, `arm0_el1`, `arm0_wr0`, `arm0_wr1`, `arm0_f1x`.
Tests compare these directly with SDK 5.1.1's `JointIndex` enum. Full-state dotted
names are normalized to underscores; streamed joint arrays use the official order.

## Timing and termination

Manifest v3 (`relic-exploy`) uses Exploy's native environment-exported policy-step
graph. Named state tensors replace the single manually assembled `obs` input;
the graph emits observations, raw actions and all 19 position targets. The host
only converts SDK frames and joint names. Export provenance, graph metadata,
source checkpoints, gains and input/output shapes are checked at load time.
The same `ReLICRollout` owns initial zero history, acknowledgements and scheduling
for both v2 and v3. See [export and validation](EXPLOY.md). Exploy's C++ controller
and automatic memory update are not part of this version.

Manifest v2 selects the explicit `relic84` adapter. It preserves the released
checkpoint, 84D contract and gains, with a bound direct or four-foot preparation
mode and stowed-arm/height settings. Direct mode requires preparation duration zero
and uses the initial measured pose for its handover check. `relic_rollout` is an SDK-free deterministic core used
in simulation and by `relic_control`; it does not use the generic v1 blend described
below. It initializes history to zero at policy handover and
uses acknowledged previous raw actions. Per-command gravity feedforward is bounded
and logged separately from position setpoints. See [the complete state sequence,
watchdogs and simulation limitations](RELIC_ROLLOUT.md).

Envelopes may explicitly declare `scope: simulation`; `require_live` rejects them.
Legacy v1 envelopes retain their existing hardware scope, and fixture policies
remain rejected independently. No evidence gate is automatically promoted.

The separate `spot_deploy.relic_shadow` module is an observation-only diagnostic
for the released 84-input ReLIC checkpoint. Its named joint mapping, 50 Hz policy
schedule, root-COM velocity conversion, held arm input and raw-action history are
documented in [RELIC_SHADOW.md](RELIC_SHADOW.md). It consumes the approximately
333 Hz state stream, performs CPU inference and emits viewer predictions. It does
not use the standing manifest/envelope or command adapter and cannot promote a
policy into live control. Its input freshness and finite-value checks protect
diagnostic validity; stopped motors and a stopped E-stop remain observable.
When a ReLIC prediction tick sees the same still-fresh mailbox sample, it records
a skipped tick and waits for the next scheduled tick without running inference or
advancing previous-action history. Both 100 ms age checks still apply, and actual
duplicate/reordered received messages remain stream errors. This behavior is
specific to the read-only ReLIC runner; control scheduling and guards are unchanged.

A single-slot state mailbox rejects duplicate/out-of-order acquisition timestamps
and propagates receiver failures. Reusing a sample between faster command ticks is
allowed only within the freshness bound. Robot clock age and local receive age are
both checked. Policy and command loops have independent monotonic schedules; missed
ticks are not followed by catch-up bursts. State callbacks never advance inference.

The host enforces measured position, velocity, load, body attitude/speed, arm stow,
tracking error, predicted PD-plus-feedforward load, target position, slew, inference,
policy age, state age, stream gaps and full-state health limits. Fresh commands have
robot-time expiry capped by the sample's permitted age. Extrapolation is disabled.
Gains are sent in every command. SDK's shared velocity threshold uses the smallest
per-joint bound; the host also checks each joint individually.

The supervised control state sequence is acquire → native stand → stream state →
hold measured standing legs → activate joint control → verify active feedback →
blend to policy → bounded standing → cancel stream → native safe power-off → return
lease. The arm must already be stowed. The blend and all shutdown behavior require
simulator evidence; holding leg targets is not assumed inherently stable.

An independent monitor retains the lease throughout native power-on and standing,
then checks E-stop/full health and command feedback during policy execution.
Command acknowledgements are matched by key, with both latency and progress bounds
strictly below command TTL. Inference
stall does not renew policy freshness. The logger uses a bounded queue and faults
on overflow; slow disk writes cannot block the command generator. Command/state
gRPC calls are cancellable and worker joins are bounded. A stuck native inference
thread is daemonized and cannot keep producing commands after the stop event.

The SDK 5.1.1 high-level streaming helpers omit deadline/cancellation access, so the
adapter deliberately uses their generated stubs. Tests construct the actual SDK
messages. Private-stub compatibility must be revalidated before an SDK upgrade.

Stopping a stream is not treated as proof of standing or power-off. On a fault,
the client cancels command production and attempts bounded native safe power-off.
It does not clear faults or cut motor power blindly. Failed shutdown is explicitly
reported to the independent E-stop operator. This software is not a safety-rated
controller and network loss cannot guarantee completion of a sit command.

## Viewer and physical stop bridge

`spot-estop --input joystick` supports RPM's identified Logitech USB joystick
without Pygame or window-focus dependence. STOP index 1 is validated as BTN_THUMB;
STOP is latched in the host process, with separate physical button 4 (index 3) rearm after
release. Input loss, overflow, replacement and stalls terminate the bridge after
a bounded STOP attempt. Kernel availability is not a physical-device heartbeat.
The same status binding and live-control interlock apply. A joystick PDB_rooted
profile may initialize an empty E-stop configuration with motors off; an existing
root endpoint is preserved by using a dedicated secondary joystick role. ESP32
setup still requires the independent root endpoint. See [joystick operation](JOYSTICK_ESTOP.md).

`spot-view` serves bundled HTML/CSS/JS and filtered telemetry on IPv4 loopback.
It imports no SDK adapter and exposes only GET routes. Host/Origin validation,
no CORS, a self-only CSP and a fixed asset whitelist prevent remote pages from
turning it into a file server. An incremental, bounded log tail tolerates partial
writes, rejects malformed state, and fails if a log is replaced/truncated.
The frames response reports unread log bytes and requests continued fast polling
while a bounded read still leaves file backlog, even if the current frame buffer
has been consumed. Startup freshness is assessed after this backlog is drained;
an older page of a valid log is not evidence of a stale robot stream.
Targets, raw actions and sent commands retain their separate provenance. Overlays
older than 250 ms in the sample timeline are omitted; following stale live state
also suppresses overlays. A retained pose is a historical observation.

The WebGL renderer computes every link's FK from the supplied/snapshotted URDF and
measured quaternion. Locally vendored Three.js renders visual OBJ/STL meshes,
supported primitives, diffuse MTL/URDF materials and PNG textures. The server
resolves assets only inside explicit roots, bounds parsing, and serves immutable
derived geometry under content-hash URLs. It exposes no arbitrary file routes or
remote asset loads. `--visual-urdf` requires identical measured joint transforms
and adds only matching visuals and fixed links; the hardware URDF hash stays
separate from visual-template provenance. Missing visuals remain visibly flagged.
Without a URDF the approximate schematic remains available. Neither geometry nor
viewer freshness thresholds are used in robot control.

`spot-estop` is a separate process with sole ownership of serial and E-stop
check-ins. Every serial response binds a random challenge, board identity, boot
ID, increasing sequence, uptime, physical latch flags and CRC32. Startup requires
observed STOP before physical rearm can allow. Link faults/reboots terminate the
bridge and require a new invocation and physical rearm. The reference firmware
latches STOP on open circuit, startup or 250 ms without valid host queries.

`sdk_estop` preserves existing endpoint specifications and adds a unique role.
Setup/registration verify motors off and robot identity. It never uses
`force_simple_setup`, default `EstopKeepAlive`, automatic reconnect or deregistration
on shutdown. Bounded manual check-ins only send NONE after fresh physical evidence.
The chosen STOP level and both robot watchdog timeouts are explicit profile fields.
This endpoint can stop the robot independently of the RL process and GUI; its
watchdog is still subject to Spot's E-stop semantics and must be physically tested.

When `RobotConfig.hardware_estop` is present, standing requires a ninth qualification
gate and checks bridge status before acquiring the lease, in the health monitor,
and before every streamed command. Status binds profile hash, robot config hash,
host/boot, session and the acknowledged endpoint ID. It must be fresh, armed and
robot-connected; bench output cannot satisfy it. The full-state monitor independently
checks that exact endpoint in Spot's status. A session change terminates the trial.
The local status file is an additional gate, not an authenticated safety channel.

## Evidence and reproducibility

Eight reviewed evidence gates (nine with hardware E-stop configured) bind to the manifest, envelope, robot config and
source/dependency hash. Every record names a reviewer, artifact SHA-256 and validity
interval. Missing, expired, future, duplicated, unknown or mismatched evidence
cannot promote a live run. This is local provenance validation, not a cryptographic
signature system or protection against someone intentionally bypassing the CLI.

Source hashes cover Python source, bundled web assets, firmware source/header and
PlatformIO configuration, `pyproject.toml` and `uv.lock`; every run also
records Git commit/dirty state and actual package versions. Live control additionally
requires a clean committed checkout. Logs are written under ignored `runs/` and do
not dirty the checkout. `local/` holds operator configuration and candidate artifacts.
Changing source or any bound configuration invalidates earlier evidence.

The checked-in fixture has no robot model or physical validity. Its provenance is
deterministic synthetic data, generator seed 0, no training, and a zero-output ONNX
graph. `tools/make_fixture.py` regenerates fixtures and JSON schemas. Fixture tests
prove software contracts only; neither replay nor shadow establishes stability.

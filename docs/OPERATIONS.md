# Operator runbook

Start with [deployment preparation](DEPLOYMENT.md) to assemble the local candidate,
review template and command arguments. The Exploy graph uses manifest v3; the
original actor uses v2. Both share the standing gates described here.

The selected authority is the [manufacturer tablet](TABLET_ESTOP.md). Use
`estop_authority: tablet` and `hardware_estop: null`; the local joystick/ESP32
bridge stays inactive. Robot stop-state monitoring and the independent operator
remain required. The following local bridge options are for an explicit future
change of authority, not parallel startup with the tablet profile.

For the RPM Logitech joystick, follow [JOYSTICK_ESTOP.md](JOYSTICK_ESTOP.md):
index 1 (button 2) is CUT, index 3 (button 4) is separate rearm. Identify/bench do not connect to Spot;
configuration/registration require motors off. This input does not bypass the
standing-controller gates below.

For measured-state/policy visualization, see [VIEWER.md](VIEWER.md). For the
optional USB ESP32 physical stop input, complete [ESP32_ESTOP.md](ESP32_ESTOP.md)
and bind `hardware_estop` in the robot profile before collecting live evidence.
The GUI cannot arm or reset it. The bridge must run independently throughout a
trial; it starts stopped and preserves the manufacturer E-stop endpoint.

## Control-host preparation

Operators select and verify their actual control host, wired interface and route.
No network configuration is changed automatically. Use the locked environment,
read STATUS.md, and run the offline suite before preparing a local robot profile.

## Local robot configuration

Create `local/robot.json` from `schemas/RobotConfig.json`, supplying these fields:

| Field | Required value |
| --- | --- |
| `schema_version` | `1` |
| `endpoint` | Operator-confirmed robot IPv4 address or unambiguous hostname |
| `expected_serial` | Actual robot serial, verified against the repaired robot |
| `expected_firmware` | Exact observed `5.0.1` or `5.1.x` version; SDK remains pinned at 5.1.1 |
| `expected_host` | Operator-verified control hostname |
| `wired_interface` | Verified wired interface carrying the robot route |
| `rpc_timeout_s` | Reviewed bounded RPC timeout, greater than zero and at most 10 seconds |
| `joint_control_feature` | `joint_level_control`; verify its enabled response on the robot |

There is no usable default endpoint or credential example. Supply the two documented
credential environment variables through your existing secret-handling workflow.
Older profiles containing `joint_control` are normalized to `joint_level_control`;
the former was an incorrect local feature code, not a missing robot license.
Never put passwords in commands, JSON, Git, logs, screenshots or evidence files.
Verify current payload registration and credentials with the robot operator.

## Read-only checks

For the released ReLIC checkpoint, use its explicit v2 manifest and the
[ReLIC import/rollout procedure](RELIC_ROLLOUT.md). The generic 45/48-value manifest
does not describe ReLIC. The v2 physical runtime uses the manifest's explicit direct
startup or supported preparation and a checked handover; its simulation-scoped example envelope is deliberately rejected
by the live gate. A complete adapter is not itself hardware qualification.

With the operator having supplied the endpoint and credential environment:

```bash
.venv/bin/spot-deploy preflight --robot local/robot.json --output runs/preflight-001
```

This reads identity/firmware, SDK compatibility, full-state joint names, arm stow,
URDF, payload mass/COM/inertia/transforms, joint-control entitlement, faults, battery,
E-stop status and the selected route. It does not register an E-stop, acquire a
control lease, switch motor power or issue a movement. Failed checks return nonzero.
It cannot establish post-repair mechanical condition or a rig's load rating.
Preflight uses unary state access, so a missing streaming service does not prevent
it from reporting license and health checks. For a bounded, read-only directory,
license and state-stream diagnostic, use:

```bash
.venv/bin/python tools/probe_robot_compatibility.py --robot local/robot.json --duration 2 --output runs/compatibility-001
```

This diagnostic does not invoke the command stream. See the
[firmware review](FIRMWARE_COMPATIBILITY.md) for protocol evidence and limitations.

Import a candidate with its original training configuration and produce a manifest
matching `schemas/ReLICManifest.json` for ReLIC, or `schemas/Manifest.json` for the
generic adapter. Supply a fully reviewed envelope matching
`schemas/Envelope.json`. Set the actual URDF and payload hashes from preflight;
simulator qualification must also validate the corresponding physical configuration.
Do not relabel the fixture or handstand checkpoint as a standing candidate.
Verify both joint-order lists, exact observation width/order/frames, defaults,
scales, clipping, previous-action semantics, effective gains and policy timestep
against the actual training/export contract. [The reference audit](UPSTREAM_PARITY.md)
shows the separate 45-input reference convention; ReLIC's explicit contract has
84 values. The general 48-input zero fixture is not an import template for either.

```bash
.venv/bin/spot-deploy inspect-policy --manifest local/candidate/manifest.json --output runs/candidate-inspect-001
.venv/bin/spot-deploy shadow --robot local/robot.json --manifest local/candidate/manifest.json --envelope local/envelope.json --duration 30 --output runs/shadow-001
```

Shadow remains read-only. Record the desktop workload during timing collection.
It records state ages, gaps, policy execution and loop intervals. Compare golden
training observations/actions on the same states. Command-network latency remains
unmeasured until controlled hardware commissioning; shadow does not claim it.

## Evidence and the live gate

Run `readiness` with the three configuration inputs to obtain `binding_sha256`.
Prepare `local/evidence.json` using `schemas/EvidenceIndex.json`. Each of the eight
named gates needs a reviewed record with a real artifact hash, matching binding,
review time and expiry. Evidence paths stay inside the index directory. Keep records
and artifacts together for handoff. Do not mark inspection or qualification passed
from a chat statement, fixture result, photo alone, or absent process.

```bash
.venv/bin/spot-deploy readiness --robot local/robot.json --manifest local/candidate/manifest.json --envelope local/envelope.json --evidence local/evidence.json --output runs/readiness-001
```

The final live sequence requires all evidence to pass, a clean committed checkout,
fresh matching preflight, a real candidate, and two distinct named operators. The
robot starts motors-off with its arm already stowed. The robot operator controls
the trial; a separate person holds and can actuate an independently functioning
E-stop. Inspect the existing approved rig, attachment, clearance and test area for
the actual arm-attached configuration before each session.

Only after those gates, the operator constructs `spot-deploy stand` with the same
manifest/envelope/robot/evidence paths, a new `--output`, both `--operator` and
`--safety-operator`, an explicit `--execute`, and `--duration` longer than the
registered transition and no longer than the registered maximum (absolute cap 60 s).
This command powers on and moves the robot. No such command was run during package
preparation. Do not launch it remotely without the two operators in position.

The first live milestone is standing only. Zero velocity command is enforced.
Before live promotion, register the duration, drift/tracking limits, initial state,
transition and stop behavior in the qualification record; the package supplies no
hardware numeric defaults. Walking and arm manipulation need a separate plan.

## Stop and recovery

Ctrl-C, failed freshness/limit checks, inference stalls, lease loss, E-stop state,
robot faults, logging failure, command acknowledgement loss, unexpected stream exit
and the duration cap all stop policy command production. The client attempts native
safe power-off and lease return. If it cannot confirm shutdown, the independent
E-stop operator follows the lab's reviewed emergency procedure. Do not infer that
a robot is safe merely because the terminal exited.

Preserve the run directory, earliest failure and cleanup status. No automatic
restart, fault clearing, lease takeover, reconnect-to-control or resumption is
implemented. Before a new trial, verify physical state and prior process identity,
diagnose the cause, revise evidence if any inputs change, and create a new run record.

# Standalone joint-API hold/crouch diagnostic

Prepared for the next supervised session; **not physically executed**. This is a
deterministic reference trajectory, not a ReLIC rollout. The purpose is to measure
joint-command transport, timing, tracking and PD behavior before a policy handover.
The tested stock baseline is recorded in `records/stock-rehearsal-20261009/`.

Update: the first physical crouch aborted on attitude at approximately 4.75 s;
automatic shutdown was unconfirmed, followed by tablet/operator shutdown and
read-only confirmation. See `records/joint-api-crouch-20261009/PHYSICAL_RESULT.md`.
The user explicitly requested a smaller repeat. `configs/joint-api-crouch-small.json`
reduces the hip/knee offsets to +1/−2 degrees with all other settings unchanged.
Select it with `--settings configs/joint-api-crouch-small.json`.

The explicitly selected diagnostic flag `--allow-known-payload-info` accepts only
system `payload.fault` code 9 at INFO severity 1. Original counts/details remain
recorded; the warning is never cleared. Any other system, behavior or service fault
still blocks. ReLIC's fault rules are unchanged. This exception was requested in
the discussion of the informational COM warning and the smaller repeat; it does
not relax body, joint, timing or tablet-stop checks.

## Small environment and entry point

Run from this checkout. The standalone CLI uses shared low-level SDK/limit modules
under `src/`, but requires no installed deployment package, ONNX, PyTorch, policy
weights or simulator. Its PEP 723 environment and transitive lock are next to the
script. UV creates the isolated environment automatically:

```bash
uv run --script --locked tools/joint_api_crouch.py plan \
  --output runs/crouch-plan-NEW
```

`plan` is offline and outputs the relative joint trajectory. All output directories
must be new. Copy the checkout, not just the script, to use it on another machine.
The pinned direct dependencies are SDK client 5.1.1, NumPy 2.2.6 and Pydantic 2.12.3.

## Draft parameters for review on return

The ten-second **active joint-control** sequence is 2 s hold, 3 s lower, 1 s crouch,
3 s return and 1 s hold. Each lowering/return segment uses a quintic curve with
zero first and second derivatives at the endpoints. At the crouch endpoint all
four hip-pitch targets increase by 5 degrees and knee targets decrease by 10
degrees. Hip roll and all seven arm/gripper coordinates remain at the initial
measured native standing pose. This is a small joint-space flexion, not an IK
guarantee of a particular body height or stationary foot placement.

The command stream is 200 Hz. Desired joint velocity sent to Spot is zero to match
the planned ReLIC PD convention; analytic reference velocity is recorded separately.
Leg Kp/Kd are 60/1.5; arm gains match the existing candidate configuration. Review
the complete values in `configs/joint-api-crouch.json` before executing.

After native stand, one second of fresh measurements must contain at least 100
distinct states and remain below 0.1 rad/s joint speed. The last measured pose
becomes the reference. Median native joint loads become a **constant feedforward**
baseline, rather than asking a zero position error to support the robot with zero
torque. This is measured-load initialization, not a validated inverse-dynamics
gravity model, and it is not updated while crouching. Native load sensor estimates
and the joint API's load convention still require physical validation.

The script keeps the reviewed manufacturer torque, position, speed, tracking,
body and timing limits. It prechecks the complete reference path against joint
position bounds. The prior captured native-standing pose passes this geometric
check, with the smallest margin about 0.00661 rad; a fresh pose can differ.
There is no clipping or automatic relaxation to force a run to pass.

## Read-only inspection, then explicit physical execution

Credentials use the existing `BOSDYN_CLIENT_USERNAME` and
`BOSDYN_CLIENT_PASSWORD` environment variables. The selected profile must have
tablet authority and no local hardware E-stop bridge. `binding.json` contains
only the reviewed `robot_model_sha256` and `payload_config_sha256` values.

```bash
uv run --script --locked tools/joint_api_crouch.py inspect \
  --robot local/spot-deployment-manufacturer-20261009-001/robot.json \
  --envelope local/spot-deployment-manufacturer-20261009-001/envelope.json \
  --binding local/joint-api-crouch/binding.json \
  --output runs/crouch-inspection-NEW
```

At the next session, after the requested parameter review, fresh preflight and
Adam/Minghao are in position with the inspected rig and tablet, the same command
with `execute` instead of `inspect`, plus `--execute --operator Adam
--safety-operator Minghao`, performs physical motion. Starting from motors off, it
acquires an available lease, powers on, stands natively, measures the baseline,
establishes the command stream, activates joint control and runs the ten-second
reference. Failure or completion stops production, requests native safe power-off,
checks explicit motor-OFF state and returns the lease. No automatic retry or takeover.

**The tablet is the E-stop.** The script only reads stop state; it never registers,
checks in, allows, resets or replaces an E-stop endpoint. Faults, stop state,
stale telemetry/health, missing ACKs, timing/limit violations and buffer exhaustion
abort the diagnostic. No robot faults are cleared. Motors-off must be confirmed;
an unconfirmed shutdown is reported for the tablet operator to handle.

The native shutdown measurement allows 10 seconds because the prior stock routine
took 3.11 seconds. Its measured duration is also compared with the existing 2-second
RL budget; that comparison does not qualify the ReLIC shutdown path. The independent
tablet remains available during bounded SDK calls.

## Lightweight measurements

The script preallocates space for 6,000 commands, roughly 10 MB, and writes compressed
`samples.npz` only after control cleanup. No compression/file writes occur in the
200 Hz command generator. Initial metadata/baseline files are written before joint
activation. A process/host crash can lose the in-memory samples; use the existing
audited rollout logger when durable continuous safety evidence is required.

`run.json` records settings, input hashes, source, joint order and timing columns.
`baseline.json` contains the captured native load/pose, height and `load/Kp` equivalent
angle offset. `events.json`, `result.json` and hash-bearing `COMPLETE.json` record
health, activation, status and shutdown. `samples.npz` contains:

- Target and measured joint angle, measured velocity/load, target-minus-measured
  position error and constant feedforward for each joint.
- P = Kp × position error, D = −Kd × measured velocity, and estimated total
  P + D + feedforward. This estimate is not a measured actuator torque.
- Reference derivative (not commanded velocity), body quaternion and velocities.
- Command keys, state and receipt timestamps, command expiry, sampled ACK key/time,
  reference-production time and scheduling lateness. Summary includes command-gap,
  observed ACK-delay and state-age percentiles plus joint-error/load maxima.

ACK timing is host-observed cumulative receipt of the most recently acknowledged
key, including local scheduling, not isolated network latency or proof of tracking.
Measurements sample the latest state per command (nominally 200 Hz); the separate
receiver statistics describe the approximately 333 Hz incoming state stream.
Skipped keys are not by themselves packet-loss evidence.

```python
import numpy as np
with np.load("runs/crouch-NEW/samples.npz") as data:
    error = data["position_error"]  # N × 19, SDK joint order, radians
    torque_estimate = data["estimated_total_torque"]
    measured_load = data["measured_load"]
```

This diagnostic is a prerequisite measurement, not a claim that ReLIC is qualified.
The existing policy evidence gate remains unchanged. Review actual results and
physical behavior before changing any controller, amplitude, gains or operating limit.

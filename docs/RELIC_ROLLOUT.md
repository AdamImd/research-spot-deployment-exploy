# Guarded ReLIC rollout infrastructure

This adds a physical-runtime adapter for the unchanged released checkpoint, with
one deterministic preparation/policy core shared by the simulator. It does not
qualify the policy for physical control or replace the existing evidence gates.
The generic manifest-v1 path and read-only `relic_shadow` remain available.
The acceptance record (historical archive; see [public validation](VALIDATION.md)) contains
the completed Isaac, baseline MuJoCo and contact-corrected MuJoCo trials.
Use the [final-check sheet](RELIC_FINAL_CHECKS.md) to prepare the physical session.

Offline GPU testing is explicit: `simulate_relic.py --policy-device cuda
--physics-device cuda:0` selects both CUDA policy and native Isaac GPU physics.
These defaults remain CPU, including in the live CLI. The
[timing protocol](RELIC_GPU_TIMING.md) separates provider-verified inference,
200 Hz host command production and synchronized simulator throughput. Optional
discarded inference priming does not advance action history or add a pose fade.

The [direct-start diagnostic](RELIC_DIRECT_STARTUP.md) additionally supports
`preparation.kind: direct` with zero duration and `envelope.transition_s: 0`.
It skips interpolation after activation and checks pose drift relative to the
initial standing pose. Supported hold before activation, zero-first history,
handover velocity/torque gates and all other guards remain. Existing five-second
manifests retain their behavior; the direct simulation fixture is not hardware evidence.

## Contract and state sequence

`ReLICManifest` schema v2 binds the original ONNX SHA-256, source training
configuration bundle, physical model/payload hashes, preparation URDF hash, fixed
stowed arm and explicit startup settings. It enforces the released gains, unmodified
action scaling, zero policy feedforward and exact observation/action mapping.
The 84-value observation retains root-COM velocity correction, all 19 joints,
four-foot/zero-velocity commands and the previous raw policy action. It is not a
45/48-value manifest with padding. Inference is 50 Hz; commands are 200 Hz.

Sequence: existing gates and fresh preflight → acquire lease → native stand →
read initial ground-relative height/four contacts → start state/command streams
with supported measured-pose hold → acknowledge activation → selected startup
(direct, or the retained quintic preparation) → gated ReLIC handover → bounded standing → cancel → attempt and
confirm native safe power-off → return lease. The initial height request is latched
after native stand, before preparation, rather than from a resting preflight pose.
The duration cap of 60 seconds includes preparation.

Preparation computes static support from the bound URDF inertials/kinematics and
four foot contacts. It balances the floating-base gravity wrench, then sends
the remaining joint gravity torque as SDK feedforward, along with the desired
pose and unchanged gains. This is algebraically equivalent to the earlier
simulation's target offset before saturation. It never applies a root constraint,
changes gravity or depends on visual meshes. Four contacts are checked initially;
the static support calculation during preparation assumes they persist. Physical
qualification must validate that assumption and the actual payload/model.

Handover requires measured pose and velocity bounds plus an explicit per-joint
torque-discontinuity limit. ReLIC starts with **zero previous actions**, after
preparation; there is no preactivation prediction warm-up or generic target blend.
History changes only after state telemetry acknowledges receipt of a command
carrying that action. An acknowledgement is not proof of mechanical tracking.
Missing acknowledgement holds the current action within its original freshness
budget; it cannot renew policy age or generate an unexecuted history sequence.

The pure core has no SDK imports or clock reads. The live wrapper maintains lease,
health, E-stop and activation checks independently. The command producer evaluates
the policy at its fixed schedule; a stalled computation cannot renew command expiry,
and an independent producer watchdog cancels it. There are no catch-up bursts,
automatic retries, fault clearing or reconnect-to-control behavior. Per-command
feedforward is finite/bounded, logged and encoded explicitly. Other envelope checks
and the independent physical-stop interlock remain in force.

## Offline preparation

```bash
uv sync --locked --all-groups
.venv/bin/python tools/import_relic.py \
  --simulation-source simulation \
  --capture runs/relic-settled-pose-capture-002 \
  --settings fixtures/relic-direct-simulation/settings.json \
  --output local/relic-rollout-candidate-NEW
.venv/bin/spot-deploy inspect-policy \
  --manifest local/relic-rollout-candidate-NEW/manifest.json \
  --output runs/relic-inspect-NEW
```

The settings above are an explicitly **simulation-only starting point for review**.
They implement the requested direct startup; the earlier
`fixtures/relic-simulation/settings.json` retains the five-second comparison.
The importer creates no operating envelope or passed evidence. Its manifest records
the historical capture's arm/model/payload; a new physical configuration requires
rebinding and renewed qualification. `replay --height-record` accepts a recorded
height/contact object when the manifest requests an initial height. `shadow` latches
fresh height through the read-only adapter. No offline command imports robot control.

## Registered shared-core simulation acceptance

Before launching, freeze deployment and simulator source commits and record both,
candidate/capture/envelope hashes and actual package versions. Use the existing
standing capture, zero initial simulator velocities, nominal friction/flat floor,
seed 101, unchanged checkpoint and actuators. Run one MuJoCo and one native Isaac
cell, 60 seconds total: 5 seconds preparation plus 55 seconds ReLIC. Stop on any
core guard, height <0.25 m, tilt >0.9 rad, nonfinite state, parity assertion or
900 seconds wall time. No retries or parameter changes. A guard/fall is a retained
scientific failure, not a passed standing qualification.

```bash
.venv/bin/python tools/simulate_relic.py --backend mujoco \
  --simulation-source FROZEN_SIM --capture CAPTURE --manifest CANDIDATE/manifest.json \
  --envelope fixtures/relic-simulation/envelope.json --duration 60 --output NEW_CELL
```

For native Isaac, use the simulation project's `tools/python_isaac.sh` with the same
tool/arguments and `--backend isaaclab`, and set `PYTHONPATH` to frozen deployment
`src` before import. Preserve its existing CPU-physics/GPU-0 Kit ownership locks.
The native environment's Python/NumPy/Pydantic versions differ from deployment;
record them. Every policy evaluation is compared against the simulator's separate
84D contract and original ONNX runner. Tolerances: 1e-5 observation units, 1e-4 raw
actions, 3e-5 radians in target mapping. These are numerical parity limits, not
physical tracking bounds. Retain all 200 Hz commands/state, 50 Hz observations/raw
actions, handover torque changes, acknowledgements, failures and timing.

Apply the existing final 10 s stationary criteria without relaxation. The capture
stands in for the proprietary stock-stand behavior; simulated acknowledgements are
ideal. Actual native stand, motor shutdown, E-stop propagation and command-network
timing require hardware checks. Simulated effort saturation is still not a claim
about physical actuator limits. `scope: simulation` envelopes are rejected by the
live gate even with a candidate policy and human evidence.

### Registered contact-drift confirmation

The independent `transient-noslip-001` comparison passed 60 seconds with MuJoCo's
Noslip postprocessor at ten iterations, while the zero-iteration baseline drifted.
Confirm exactly that one simulator option with this shared runtime using
`--mujoco-noslip-iterations 10`. Run one MuJoCo cell with the same candidate,
capture, simulation envelope, five-second preparation and 60-second total cap.
All other settings, artifact/parity checks, fall bounds, 900-second wall cap and
no-retry rule remain unchanged. Compare with the retained default MuJoCo and
Isaac cells from `relic-core-acceptance-001`; do not rerun those baselines.
This does not change policy weights, physical gains or the default simulation
model. A pass identifies contact-model sensitivity and supports runtime testing;
it does not by itself validate physical contact or actuator behavior.

Completed: `relic-core-noslip-001` passes the final stationary window using the
actual shared core. Baseline and corrected results remain separate. The default
stays at zero iterations; reproduction of the contact-corrected case requires
the explicit flag. The observed startup lowering and torque discontinuity remain
part of physical transition review.

## Final physical checks still required

Use the existing [operator runbook](OPERATIONS.md): fresh Ethernet preflight,
inspected rig, two named roles, tested independent stop path, exact arm/payload,
reviewed physical envelope, simulator/transition evidence and bounded first-trial
command-acknowledgement commissioning. Reuse `spot-deploy stand` with the v2 manifest
only after those checks pass. The tool does not turn a diagnostic simulation pass
or the presence of a built rig into hardware approval.

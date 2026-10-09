# Manufacturer-limit integration — October 9, 2026

## Question and decision

Apply Adam's selection of manufacturer-based torque limits while retaining the
released policy and direct-start contract. Check whether measured-position knee
limits and coupled arm constraints are enforced, and whether they change the
archived standing behavior in a fresh simulation regression.

Operator source statements are retained in [operator-review.json](operator-review.json).
The prior lower-cap draft is preserved separately. Other checks were reported
tested and acceptable by Adam; no new physical measurements are claimed here.

## Implementation and protocol

- New runtime `actuator_limits.py` contains the 101-point knee table and motor
  constraints. All knee entries match the pinned manufacturer source and bundled
  ReLIC copy; see [source hashes](sources.json).
- The selected hardware envelope and separately scoped torque-regression envelope
  are under `configs/spot-manufacturer-*.json`. Their paths and hashes are recorded
  in the validation run. Handover configuration and policy artifacts are unchanged.
- `Guard` checks measured, feedforward and total requested torque; knee lookup uses
  measured position and rejects extrapolation. Individual load caps remain active.
- An explicit SDK speed backstop fixes the lower-arm-bound/global-threshold conflict
  without changing host per-joint speed checks. Omitted fields preserve old behavior.
- Tests cover table entries/interpolation, out-of-range input, coupled loads that
  pass individual caps, measured/requested/feedforward enforcement, tighter
  application caps, SDK/host speed separation, and invalid configurations.
- Fresh 60-second MuJoCo and Isaac regressions: seed 101, public standing capture,
  zero initial velocity/history, 50 Hz policy, 200 Hz commands, direct startup,
  zero base velocity, fixed arm, unchanged weights/gains, ideal ACKs, CPU physics.
  MuJoCo uses noslip iterations 10. Other monitoring bounds are the original
  simulation bounds to isolate the changed torque guard, not the hardware envelope.
- Run one simulator at a time, maximum 900 seconds each, no retry or parameter
  changes after failures. Record source commit/dirty state, config hashes, commands,
  package versions, raw results and completion markers. No robot clients or control
  requests; no paid resources.

## Results

Targeted regression: 101 tests passed; Ruff passed before the pinned campaign.
Pinned campaign commit: `875068519d48aa6e64ce985a946d9996f98566fc`, clean tree.
**345 tests passed**, Ruff passed, and the offline demo passed. Both simulations
completed 60 seconds with a passing stable final window and 3,001 policy samples.
Maximum target parity error was 2.384e-7 rad in each. See [validation.json](validation.json)
for numerical results, environment and raw artifact hashes. Raw logs and completion
records are in `runs/manufacturer-limits-validation-20261009-001` on the control host.
Isaac emitted USD visual-reference warnings; physics/parity criteria passed.

The selected hardware envelope was packaged offline into
`local/spot-deployment-manufacturer-20261009-001`. Policy, runtime, robot profile,
hardware envelope and fresh preflight checks passed; the final evidence index is
not populated. The operator's completed-test confirmation is stored alongside the
bundle, without inventing per-test files or expiry dates.

A fresh read-only preflight at **12:11:17 America/Chicago** passed identity, model,
payload, arm stow, entitlement, fault and tablet stop-state checks. Battery was
100%; **motors were already on**. No control lease was acquired or motion command
sent. See [sanitized snapshot](preflight.json); raw identity records remain local.
The separate control-entry check requires motors initially off and would reject
that snapshot for activation. Motor state was not changed by this work.

Decision: manufacturer torque selection is implemented and passes the registered
offline regression. Retain the distinction between the torque-isolation simulation
and the tighter full hardware envelope. Attach the team's completed-test artifacts
to the final binding and perform the normal fresh preflight/operator entry sequence
before activation. No physical policy rollout was attempted.

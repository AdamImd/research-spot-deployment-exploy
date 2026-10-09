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
Full-suite and simulator results will be recorded from the campaign's completion
records. The selected limit configuration is distinct from approval of a physical
rollout; numerical checks do not turn operator-reported tests into new measurements.

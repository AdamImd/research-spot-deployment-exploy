# Hardware operating-envelope review, October 9, 2026

## Question and comparison

Which operating limits can we propose from firmware-matched manufacturer data,
the captured robot model and recorded standing demand, and which proposed limits
would already reject the archived startup? Hypothesis: the leg PD target offsets
can fit bounded standing demand while arm/model and timing issues remain distinct.
This is retrospective engineering analysis, not a new qualification experiment.

## Inputs and protocol

- User inputs and the numerical proposal are in [proposal.json](proposal.json).
  All proposals remain unreviewed. See [the review](../../docs/HARDWARE_LIMIT_REVIEW.md)
  for sources, rationale and outstanding decisions.
- Historical campaign `exploy-validation-002`, MuJoCo and Isaac Lab: source commit
  `0a023ef7d49fd4c6bd6e4fc5e582541605bd58ab`, simulation commit
  `74cb498719164d4c6e698a1f056db5ca348f60d2`, seed 101. Both are 60 s,
  direct start from the same historical standing capture, zero initial velocities
  and raw-action history, 50 Hz policy, 200 Hz commands, ideal ACKs. Native stock
  standing was not simulated. See [original validation](../../docs/VALIDATION.md).
- Live model came from the existing `readonly-rpc-probe-20261009-001` motors-off
  capture. No new robot reads, lease, motor, E-stop or command requests were made.
- Tool: [review_hardware_limits.py](../../tools/review_hardware_limits.py).
  It verifies every artifact hash in both completion records, checks native-to-SDK
  joint order, recomputes all logged PD torques, and includes the supported hold
  before row zero in target slew and handover torque. It checks both new and
  preceding target tracking, requested/applied simulated loads and hold feedforward.
- Report windows are 0–10 s inclusive and 0–60 s inclusive. Sample spacing is
  checked against 200 Hz. This cannot detect unlogged physics-substep extrema.
  Measured simulator q/dq use native order; command targets/torques use SDK order.
- [summary.json](summary.json) records source hashes, analysis commit/script hash,
  Python/NumPy/package versions, seed, model limits, per-joint statistics and
  exact violations. The archived graph is hash-matched to the included policy.
  The code base at review start was `0ef7df7`; runtime/weights were not changed.

## Reproduction

Use the locked UV environment. Supply the two original completed run directories
and captured URDF from the maintainer archive:

```bash
uv run --no-sync python tools/review_hardware_limits.py \
  --run "$MUJOCO_RUN" --run "$ISAAC_RUN" --urdf "$CAPTURED_URDF" \
  --trial-duration 10 \
  --proposal records/hardware-limit-review-20261009/proposal.json \
  --output runs/hardware-limit-audit-001.json
```

Those shell variables denote input paths, not credentials. The exact local paths
are retained in the ignored active bundle's review note. Raw records are not in
the public checkout. The committed hashes enable an archive holder to reproduce
this analysis; a public clone alone can run new simulations, not reconstruct the
original private capture. The tool refuses altered artifacts or an existing output.

## Results and decision

The first-ten-second and full-trace comparisons both reject the proposal: arm
position boundaries in both simulators, plus gripper speed/load in Isaac. All
sampled leg proposal bounds pass. Both original rollouts completed and met their
final-standing criterion using the original broad simulation envelope.

Tables and interpretation are in the review document. No new plots were necessary
for this limit comparison. Native order, first-step inclusion, altered-artifact
rejection and the proposal's rejection as an executable Envelope have targeted
tests. The full offline suite passed: **332 tests**, Ruff and the Exploy demo, at clean
commit `4f2a3ea3bb649f5e14fdf89aa34e387c995939d1`. See
[validation.json](validation.json) for stage results and artifact hashes. The
validation wrapper could not use an unavailable bare `python` for its final
metadata writer; the executor collected completion from recorded zero exit codes
without repeating the tests. Analysis and numerical bounds were not changed.

**Decision:** keep this as an unapproved review artifact. Do not create an approved
hardware envelope or evidence entries. A shared SDK velocity threshold cannot
currently implement these heterogeneous joint limits as intended. Arm-boundary
semantics and the Isaac gripper transient need resolution, with command transport,
tablet stop response and physical limits still awaiting their existing reviews.

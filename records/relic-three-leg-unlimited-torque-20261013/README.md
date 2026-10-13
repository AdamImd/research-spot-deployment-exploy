# Unlimited actuator torque: paired native transition diagnostic

## Question and protocol

Does the unfiltered pretrained ReLIC transition recover if simulated actuator
clipping is entirely removed? Compare one 47-second MuJoCo run against the
[previous saturated run](../relic-three-leg-native-recovery-20261013/README.md).
Hypothesis: the native transition remains recoverable with ideal PD actuation;
transient forces and speeds may change when saturation is removed.

Dynamics source: clean `239a203cf53a67ddf5daa36fd19e97b95a04a7f3`. Seed 101;
same latest recorded standing pose, root placement 0.44790276786489125 m,
zero initial velocities/actions, independently retained torso-height request,
unchanged actor, gains, native ACK-owned history and two-second return blend.
No entry blend or target-rate filter. Sequence: 5 s four feet, 6 s lift, 3 s hold,
12 s IK square, 6 s lower, 2 s return blend, 12 s final standing plus 1 s margin.
Policy 50 Hz, control/recording 200 Hz, no-slip iterations 10, one physics substep.

The only dynamics intervention was `--no-actuator-torque-limits`, alongside
`--no-guards`. All hip/arm effort and knee position/speed torque clipping were
bypassed. Contact physics, model constraints and finite/integrity checks remained.
CPU 2, existing locked UV environment, GPU 0 for EGL replay rendering only, no
paid compute or robot access. Exact commands, resources, timeouts and stop rules
are in the three contract files. The dynamics ran exactly once. Runtime package
versions and input/model/source hashes are retained in the ignored raw run.

## Observed results

Both runs completed the full lift, three-leg hold, IK square and return, without
a fall proxy, and passed the existing final 10-second standing checks.

| Measurement | Saturated actuators | Unlimited actuators |
| --- | ---: | ---: |
| FR HX peak applied torque | 45.00 N m | 49.92 N m |
| FR knee peak requested torque | 77.18 N m | 75.85 N m |
| FR HX peak joint speed | 13.82 rad/s | 14.38 rad/s |
| FR knee peak joint speed | 15.79 rad/s | 19.66 rad/s |
| Maximum absolute roll | 3.069 deg | 2.966 deg |
| Maximum absolute pitch | 4.350 deg | 4.257 deg |
| Maximum body angular speed | 1.537 rad/s | 1.411 rad/s |
| Square tracking error p95 | 1.901 cm | 1.902 cm |
| Final window maximum tilt | 0.886 deg | 0.836 deg |
| Final window height span | 0.046 mm | 0.172 mm |

Requested versus applied torque matched within recording precision throughout;
no actuator clipping was detected. FL was unloaded throughout hold/IK; each of
FR, HL and HR had normal force above the diagnostic 5 N threshold throughout.
Foot forces and joint torque are independent of the geometric clearance checks.
IK error is evaluated over the square phase; larger lift/lower transients remain
visible in the full recovery graph. A finite fall would have failed scientific
acceptance even if execution reached 47 seconds.

The 75 N m requested-demand cap would reject the 75.85 N m FR knee request at
5.115 s; requests exceed 75 N m at 5.105, 5.110 and 5.115 s (15 ms total). The retained measured hip torque and speed bounds
would also reject this ideal-actuator trajectory. Guard event reasons are
recorded only on their first occurrence; they are not exhaustive event counts.
This is not a successful rehearsal under the proposed hardware envelope.

## Measurement precision and retained failures

A first video attempt failed on a plot-title concatenation bug after the complete
dynamics run. Renderer commit `1ecc886` fixed it; both plot modes passed their
regression checks. The source recording was unchanged. The recovered video is
100 fps, 4701 frames (47.01 s container duration for samples from 0 through 47 s).

The initial analysis assertion compared float32 telemetry-derived PD demand with
MuJoCo's float64 applied torque at an absolute 1e-5 N m tolerance. 5141 joint/time
pairs crossed that threshold; the largest was SH1, 1.4304873e-5 N m. Every error
was below the state-quantization bound:

`0.5 * abs(spacing(float32(q))) * kp + 0.5 * abs(spacing(float32(dq))) * kd`.

Analysis commit `fef3b0c` reports raw differences separately and detects clipping
above this derived bound plus the original 1e-5 N m tolerance. The maximum excess
above the precision bound was -1.52e-14 N m. A numeric regression confirms that
an actual 0.01 N m discrepancy is still detected. Request at row k is aligned
with preceding applied torque at row k+1. Original failed outputs were preserved.
There was no dynamics rerun or change to physical parameters to obtain a pass.

## Requested hardware change and decision

Adam explicitly selected **75 N m requested leg PD torque only**, retaining
measured manufacturer limits. The accompanying `hardware-envelope.patch.json`
changes only `leg_requested_torque_max` and its required operator reason.
Predicted total PD-plus-feedforward demand is checked before firmware saturation.
Measured `load_max`, manufacturer knee table, measured knee table domain,
arm/coupled-motor and feedforward-only checks remain enforced. No firmware setting
or gain changed. The local envelope is validated but remains an unbound draft;
no physical trial was authorized or run.

Interpretation: actuator saturation is not necessary for recovery in this one
MuJoCo trajectory. Removing it increases some joint speeds but slightly reduces
peak body pitch/angular speed; it does not eliminate the native transition jump.
This single paired diagnostic cannot establish real actuator or harness behavior.
The retained physical guards still require review before any exact trial approval.

## Artifacts, validation and reproduction

- Dynamics and original failure: `runs/relic-three-leg-unlimited-torque-20261013-001/`.
- Complete video/plots: `runs/relic-three-leg-unlimited-torque-media-20261013-001/video/`.
- Aligned torque/contact/IK analysis: `runs/relic-three-leg-unlimited-torque-analysis-20261013-001/analysis/`.
- Local hardware draft: `local/relic-three-leg-requested75-preparation-20261013-001/`.
- Public sanitized metrics: `summary.json`; checksums: `artifacts.json`.
- Source validation: 490 tests and Exploy demo passed; renderer repair added two
  passing checks; measurement repair added one passing numeric check; Ruff passed.

Reproduce using the pinned dynamics commit and `dynamics-contract.json`, with the
private capture/candidate files at the named local paths (they are not public
robot data). Use the pinned renderer and analysis commits with their separate
contracts for saved-data processing; choose fresh output directories. Use the
existing UV lock with the simulation dependency group. The operator's private
capture is required for an exact paired replay; public code alone cannot
reconstruct that starting pose.

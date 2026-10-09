# Supervised ReLIC harness trial, October 9

The single operator-requested physical repeat completed ten seconds of ReLIC
standing control. Adam operated Spot and Minghao held the manufacturer tablet
E-stop. The session used the operator-confirmed slack fall-arrest rig with all
four feet loaded. Adam explicitly reported that the harness pushes the arm and
requested that this motion be observed rather than stop the leg controller.

The reviewed hardware envelope selected `arm_motion_guard=observe` with that
reason. All arm torque and coupled-motor checks, held arm targets and gains,
leg/body guards, faults, timing and tablet stop monitoring remained enforced.
Policy weights and graph were unchanged, startup was direct without a fade,
previous actions started at zero and advanced on acknowledgements, and the
initial requested body height was captured from Spot at 0.52257 m. All four
initial foot contacts were reported. Policy rate was 50 Hz, commands 200 Hz,
command expiry 30 ms, zero base velocity, and native shutdown allowance 10 s.

## Validation and execution

The clean runtime commit was `85e1d1fa0adde6ed5c59b0effaa4a6a64b3fcf9b`.
The first offline campaign at `2202d5b` failed because the standalone crouch
guard did not initialize the new fields. That failure is preserved in
`runs/relic-harness-validation-20261009-001`. The constructor was fixed to retain
all nineteen-joint enforcement and reject the ReLIC-only harness exception.

Fresh campaign `runs/relic-harness-validation-20261009-002` then passed:

- 414 tests, Ruff, demo and 197/197 independent hypothetical startup guard cases.
- A 60-second MuJoCo rollout with a stable final window.
- A 60-second CPU production/serialization benchmark with no missed releases:
  production p99/max 1.034/4.551 ms, deadline response p99/max 1.106/4.608 ms.
  Its state and acknowledgement clocks were idealized; it was not robot transport.

Fresh motors-off preflight and deployment-check passed. The candidate review
artifacts explicitly accepted one supervised commissioning attempt with recorded
operator exceptions and existing rig/tablet observations. They did not relabel
partial physical qualification as completed tests. Native standing preceded the
single recorded policy attempt; no retry occurred.

## Observed results

| Measurement | Result |
| --- | ---: |
| Policy-active duration | 10.0 s |
| Policy predictions / history acknowledgements | 500 / 500 |
| Joint commands | 2,003 |
| Maximum inference time | 0.575 ms |
| Command-gap p99 / maximum | 5.541 / 7.005 ms |
| Observed acknowledgement p99 / maximum | 9.618 / 11.288 ms |
| Maximum absolute roll / pitch | 0.964 / 2.439 degrees |
| Final two seconds: maximum absolute roll / pitch | 0.951 / 0.549 degrees |
| Final two seconds: maximum leg speed | 0.06474 rad/s |
| Native shutdown including lease return | 5.020 s |
| Compact state/action recording size | 1,565,422 bytes |

No first-failure or guard-failure event occurred. Twelve arm-motion observation
events recorded changing violation flags, including shoulder-pitch crossing its
configured position boundary (arm-local index 1). Arm targets remained identical
throughout; the stow-distance bound was not crossed. This directly demonstrates
that the requested motion exception permitted that boundary crossing while
the leg policy continued. It does not prove the causal joint of the preceding
failed trial, whose rejected sample was not recorded.

The `shutdown_result` event explicitly confirmed motor state OFF, lease return,
no cleanup errors, and completion within the approved ten-second allowance.
Separate read-only post-preflight passed with motors OFF and no active faults.

Measured joint state, targets and feedforward permit reconstruction of position
offsets, P and D terms and estimated torque. Per-leg-joint maxima are saved in
`result-summary.json`; estimated PD torque is not an independent actuator-torque
measurement. Body translation was not recorded in command states, so this summary
does not claim a measured drift bound. This is a successful short standing
commissioning run; longer operation, walking and full hardware qualification were
not tested.

## Artifacts and reproduction

Private artifacts stay ignored to exclude robot identities and network details:

- Raw: `runs/relic-standing-harness-20261009-001`.
- Compact: `runs/relic-standing-harness-compact-20261009-001/rollout.jsonl.gz`.
- Postcheck: `runs/relic-standing-harness-post-preflight-20261009-001`.
- Supervisor: `runs/relic-standing-harness-supervisor-20261009-001`.
- Frozen candidate: `local/relic-standing-harness-20261009-002`.
- Explicit operator request: `local/relic-harness-review-20261009-001/operator-request.json`.

Raw run metadata pins source, environment and input/model/weight hashes. Raw,
postcheck and compact completion/artifact hashes were verified. The compact
record preserves all actions/commands and unique command-state samples; the raw
event log additionally retains the arm-observation and shutdown events.

Recompute this sanitized result with the private artifacts present, from the repo:

```bash
uv run --no-sync python records/relic-standing-harness-20261009/summarize.py
```

Physical execution is not part of summary reproduction. Any new physical trial
needs fresh matching review/preflight and explicit operator authorization.

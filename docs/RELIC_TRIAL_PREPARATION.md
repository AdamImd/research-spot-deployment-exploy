# ReLIC preparation after the joint-API diagnostic

The next intended physical test is four-foot, zero-velocity ReLIC standing with
stowed arm, direct startup, zero initial previous actions, 50 Hz policy and
200 Hz joint commands. Preparation does not power motors or send joint commands.
The tablet remains the stop authority. Adam and Minghao are the recorded operators.

For the operator-reported harness pushing the arm, see [arm harness mode](ARM_HARNESS.md).
This is an explicit per-envelope observation mode, while default arm guards remain
enforced. It requires a new source/configuration binding before the requested repeat.

The smaller diagnostic established partial-run wired timing and useful PD/load
agreement; it did not exercise ReLIC inference or complete ten seconds. See
`records/joint-api-crouch-20261009/SMALL_PHYSICAL_RESULT.md`.

## Recorded standing startup audit

```bash
uv run --no-sync python tools/audit_relic_startup.py \
  --source-run runs/joint-api-crouch-small-20261009-001 \
  --manifest local/spot-deployment-manufacturer-20261009-001/policy/manifest.json \
  --envelope local/spot-deployment-manufacturer-20261009-001/envelope.json \
  --output runs/relic-startup-audit-NEW
```

This verifies the source recording hashes, then treats each captured native
standing sample as an independent startup with zero previous actions. It records
the actual policy output, signed PD torque, static-support torque, feedforward
removal, handover torque step and the real core's first guard failure. Historical
timestamps and ideal next-tick receipt are explicit. It does not roll out the
policy against unresponsive recorded state or claim dynamic stability.

The guard audit uses the selected hardware envelope unchanged. The handover-step
limit embedded in the current candidate is still 200 N m per joint; this is a
distinct review item from the manufacturer absolute torque limits. Passing that
configured step guard is not itself proof of a suitably bounded transition.

## Timing and shutdown

Run the full offline suite, a bounded 60 s MuJoCo regression and the existing
`benchmark_relic_gpu.py --device cpu` protocol with actual ReLIC core, asynchronous
logging and SDK message serialization. The benchmark uses simulation-scoped limits
and ideal state/ACK clocks: it measures host work, not physical transport or a
new hardware operating envelope. Record concurrent dashboard load.

Shutdown now requires explicit SDK motor STATE_OFF, rather than simply a false
is_powered_on result. The adapter records cancellation, native safe-power-off,
lease-return durations and confirmation separately, including failures. A stream
cancellation exception must not skip the power-off and lease-return attempts.
The ReLIC event log retains this report even when its primary controller failed.
These are observability/confirmation changes; no shorter physical stop is claimed.

The current hardware shutdown budget remains 2 s. A measured successful diagnostic
shutdown took 6.02 s and an earlier attempt was unconfirmed after 10 s. Preparation
must preserve this unresolved distinction between prompt command termination and
completion of native safe-power-off. Do not silently raise the budget or qualify
the failed shutdown by the later tablet stop.

## Fresh candidate and final review

After code validation, create a new local bundle with the existing candidate graph,
robot profile and hardware envelope; never rewrite an old frozen bundle. Keep the
new evidence index unverified until the applicable artifacts and review are bound.
The run-specific preparation report records its exact paths and commands.

Before physical activation, resolve startup torque-step and posture-envelope
review, shutdown timing/behavior, and the candidate's hash-bound evidence. A fresh
motors-off preflight must be collected immediately before the operators' trial.
The actual inference/ACK-history/recording plus wired command path still needs its
supervised combined validation. The dashboard is observation-only and cannot arm
or authorize the launch. No repeated test or physical motion is part of preparation.

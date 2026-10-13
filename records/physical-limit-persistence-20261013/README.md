# Continuous physical-limit persistence: implementation and validation

## Operator request and semantics

Adam requested stops only for violations lasting **more than 500 ms**, and selected
all physical operating limits including attitude, position and tracking. The new
explicit envelope option is `physical_limit_persistence_s: 0.5` plus a reason.
Default zero retains immediate stops. Each enforced limit/component has its own
continuous timer; recovery resets it, separate impulses never accumulate, and
exactly 500 ms is allowed. Targets, gains, weights, 50/200 Hz rates and native ACK
history are unchanged. The requested leg torque cap remains 75 N m.

[Scope and immediate checks](../../docs/PHYSICAL_LIMIT_PERSISTENCE.md) specify the
startup boundary, manufacturer knee-table endpoint monitoring during grace,
walking path limits and all logging. Observation-only settings remain unchanged.
Tablet, faults, data validity, transport/timing, expiry and startup qualification
remain immediate. No physical trial was authorized or performed.

## Research question and protocol

Does this timing rule allow the recorded native three-leg impulse to recover,
while a sustained violation still stops? Hypothesis: the brief measured hip torque,
hip speed and requested knee torque crossings clear far sooner than 500 ms.

1. Deterministic tests exercise exact500ms versus505ms, recovery, repeated impulses,
   independent joints, duplicate checks, state/target bounds, torque/coupling,
   knee-table domain, speed/attitude/tracking/slew, walking deviation, immediate
   invalid data/deadlines and immediate startup.
2. Replay all 9401 states/targets from the previous 47-second unlimited-actuator
   recording through the actual host Guard. Compare the75Nm requested-cap envelope
   with persistence0 versus0.5; all other limits identical. Source hashes verified.
   This uses an ideal control clock and zero inference time, not a timing benchmark.
3. Run exactly one 60-second MuJoCo dynamics rehearsal, seed 101, original standing
   capture/root placement0.44790276786489125m, zero initial velocities/actions,
   unchanged actor/gains, no entry blend/rate filter and two-second return blend.
   Actuator torque clipping remains disabled, but operating guards are enforced
   with500ms persistence. Keep contact/integrity checks and all requested limits.

Source `f719d107f3ed51eaf4b73c9a62e3fbe3867aaf9a`, clean at launch. Local host,
CPU 2, existing locked UV environment, no GPU/paid resources or robot access.
Exact commands, resources, inputs, seed and timeouts are in execution-contract.json.
Runtime packages, model and source/input hashes are retained in the raw simulation.

## Observed results

- Full suite: 521 passed on 2f582d6; Exploy demo passed. The subsequent audit-label
  repair and corruption regression passed all 29 physical-limit tests; Ruff passed.
- Immediate audit: rejected at5.025s on measured joint velocity/load.
- Persistence audit: all 9401 samples through 47 s passed; five limit-timer crossings
  recovered, longest 15 ms; no timers pending at the end.
- 60 s guarded, unlimited-actuator simulation: completed, 3001 policy samples, no fall,
  full lift/hold/IK/return, geometric/IK checks and final10s standing passed.
- Final10s max tilt0.833degrees; body-height span0.447micrometres; angular-speed
  p95 about3.02e-6rad/s. These are numerical simulator statistics, not hardware accuracy.

| Physical episode | Duration above bound | Monitoring entries |
| --- | ---: | ---: |
| FR HX measured torque | 5 ms | load_max and manufacturer torque |
| FR HX measured speed | 10 ms | velocity_max |
| FR knee requested PD torque | 15 ms | requested cap and manufacturer-profile demand check |

Five recovered timer entries represent three physical episodes, because the torque
checks are intentionally redundant. No sustained physical-limit violation occurred.
Separate tests verify stops beyond500ms; the dynamics run alone does not test that
failure branch. Timing thresholds, target commands and instantaneous bounds were
not raised to get a passing result.

The simulator uses simulation time for state/command scheduling. Its measured core
compute p99 was1.027ms, but one wall-clock outlier atsimulation52.475s was119.366ms.
The actual live command loop would reject that outlier against its unchanged10ms
computation limit. **This is not a live timing qualification or approval to deploy.**

## Retained failures and repairs

Validation001 stopped before any stage because an execution-wrapper timeout omitted
its duration. Validation002 ran tests and found four legacy crouch failures after
the shared torque-check signature changed. ReferenceGuard was updated to initialize
the timer and pass a timestamp, retaining immediate baseline qualification; all
24standalone tests passed. Validation003 then passed521tests and the demo, but the
new audit expected `passed` instead of the simulator's `completed` marker. All
recording hashes already matched. The audit now checks the correct marker and a
regression confirms corrupted data remains rejected. Validation004 completed the
audits and the one dynamics run. All earlier attempts and logs remain preserved.
There were no duplicate dynamics runs or unregistered parameter changes.

## Decision and continuation

The requested stop semantics are implemented and pass software, saved-data and
simulation checks. The local hardware envelope remains a preparation draft with
500ms persistence and75Nm requested leg demand; measured manufacturer limits and
other numeric caps are unchanged. Rebind to current source, perform fresh preflight
and resolve live timing readiness before requesting the exact physical trial
confirmation. No previous physical approval is reused.

Artifacts:
- `runs/relic-three-leg-persistence500-validation-20261013-004/`: audits, simulation,
  supervisor, raw recordings and reports.
- `local/relic-three-leg-persistence500-preparation-20261013-001/`: validated hardware
  draft, simulation envelope and pending-final-review record.
- Public sanitized `summary.json`, `artifacts.json`, `envelope.patch.json` and
  `execution-contract.json` accompany this record.

Reproduction requires the private standing capture and candidate artifacts at the
contract paths; they are not public robot data. Use pinned source, UV lock and fresh
output directories. No physical connection is needed or permitted by these commands.

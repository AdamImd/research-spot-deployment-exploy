# Qualification protocol and handoff

## Question and comparison

Can the exact candidate control twelve leg joints for stationary standing on the
repaired Spot with its arm attached and stowed, using RPM over the verified wired
route, while remaining inside the registered operating envelope?

The null operational decision is **do not activate live RL**. A software fixture
pass is evidence about implementation, not a test of that physical hypothesis.
No training, walking, paid compute or new seed campaign is part of this preparation.

## Required inputs and gates

| Gate | Required evidence | Existing team responsibility |
| --- | --- | --- |
| `post_repair_stock_check` | Robot identity/firmware, returned repair condition and successful stock operation, battery and payload checks | Robot operator / team |
| `rig_inspection` | Approved installation and current inspection for this robot configuration and test area | Adam |
| `arm_hold` | Stowed pose, all seven targets, gains/feedforward, drift and physical model validation | Control owner |
| `operating_envelope` | Reviewed per-joint position/velocity/load/tracking/slew limits and runtime watchdog thresholds | Xun / primary |
| `simulator_standing` | Candidate/config/robot-model hashes, simulator/environment pin, seeds, raw states, tracking/attitude/drift results and decisions | Minghao / primary |
| `transition_and_shutdown` | Exact native-to-policy blend and termination behavior under nominal and injected faults | Control owner / primary |
| `wired_timing` | Chosen route, representative RPM workload, inference/state/loop distributions and timing budget, plus a reviewed first-live acknowledgement gate | Runtime owner |
| `shadow_parity` | Training-reference observations/actions compared with this runtime on the same recorded states | Policy/runtime owner |

Slack assignments are recorded as context; no teammate has been messaged or assigned
new work by this implementation. Deliver the local handoff for review.

## Registered run record

Before any qualification trial, record the question/comparison, code commit and
dirty state, candidate and configuration hashes, exact environment, robot serial,
firmware, payload/model hashes, controller frequency, stream frequency, numerical
limits, duration, seed where applicable, data/source hashes, host and route, resource
allocation, operator and E-stop operator, rig inspection, stop policy and artifacts.

Use the existing team simulator first. Record the scientific acceptance thresholds
before evaluation and retain every failure. Do not infer a suitable numerical
hardware limit from the synthetic fixture or this package's software tests. Do not
retrain or add seeds to a failed method under this preparation authorization.

For real-time runs, retain raw states, network inputs/outputs, requested full-joint
commands, acquisition/send/expiry timestamps and command keys. Report timing
distributions and maxima, joint/body tracking, limit events and stop outcomes.
The requested duration includes the qualified transition and the remaining full-policy
standing interval. Acceptance requires both intervals to remain within all envelope
limits, followed by successful confirmed shutdown.
Keep observed results separate from interpretation. A clean process exit alone is
insufficient: verify `COMPLETE.json`, result status and hashes of all artifacts.

## Stages

1. Software contract and fault-injection suite; inspect and replay exact policy.
2. Stock robot and installed-rig inspection; read-only preflight.
3. Simulator parity, standing, transitions and injected failures with the exact
   arm/payload and envelope; qualify with a registered protocol.
4. Read-only shadow on RPM with representative workload. Validate inference, state
   age, policy scheduling and the reserved command acknowledgement budget.
5. Supervised native stand → initial joint hold → activation acknowledgement →
   registered blend → stationary policy standing → confirmed safe power-off.

No stage advances automatically. Failure preserves evidence and returns to primary
diagnosis. The initial hardware trial is a commissioning experiment, not already
validated performance. Missing command-latency evidence cannot be claimed from a
read-only trial; the live acknowledgement watchdog enforces the registered budget.

See [public status](STATUS.md) and [agent handoff](AGENT_QUICKSTART.md).

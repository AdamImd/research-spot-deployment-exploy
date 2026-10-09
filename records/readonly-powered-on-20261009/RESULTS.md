# Powered-on, motors-off Spot observations — October 9, 2026

## Question and scope

Which deployment assumptions can be checked while Spot is powered on and motor
activation is prohibited? Check identity, candidate bindings, high-rate telemetry,
read-only RPC timing and the validity of the initial policy state. These are
observations, not a physical policy rollout or a command-path timing qualification.

No motor-power requests, joint commands, control leases or E-stop writes were made.
Existing live services were left unchanged. No failed trial was retried. Cost: $0;
local CPU only, no GPU. The separate E-stop readiness gate stayed false.

## Pinned implementation and protocol

- Tested code: `aded413e3f299fdd1d8362a2f406aab8dca6dc58`, clean checkout.
- Environment: repository locked Python environment; precise Python/package/source,
  input and output hashes are in each run's `run.json` and `COMPLETE.json`.
- Host: RPM (`cs-u-rpm-dt-03`); verified direct registered Ethernet route.
- Candidate: bundled `policies/relic-exploy-standing/manifest.json`; unchanged graph
  `1940df208bf9e30e6134eeb71d1c4e67d3a0d66778726507d4886c62eae662e8`.
- Deterministic collection protocol; no randomized tests or random seeds.
- First read-only preflight, then a synchronized pose/cold-prediction attempt.
  The latter stopped on invalid ground-relative height; subsequent shadow inference
  was skipped, and another preflight confirmed motors off.
- A separate height-independent `spot-deploy watch --state-source stream --duration 60`
  measured stream delivery. Its recorder retains 20 Hz state samples while the
  receiver counts and measures all delivered stream messages.
- After streaming finished, 100 sequential unary state queries at 20 Hz measured
  RPC durations, state age, raw body/GPE transforms, contacts and motor power.
  Each response required motor power OFF. Snapshot checks bracketed this probe.
- Each network call was bounded by the profile's 3-second timeout. Supervisor limits
  were 35 seconds for short commands and 90 seconds for the 60-second watch. Identity,
  malformed/stale data and transport failures stop the relevant collection; no retry.
- The additional read-only probe source is preserved with its run. SHA-256:
  `3bf59a10a64b9bfcfd63d7832745bc8b2c4f4c8dc809fe543dd6758ad3d3864e`.
  It uses only `ReadOnlySpot` and state-query methods. It does not modify runtime
  source, candidate artifacts or a deployment guard.

## Observed results

| Check | Result |
| --- | --- |
| Firmware / SDK | Robot 5.0.1 / SDK 5.1.1; expected identity matched |
| Joint-level control entitlement | Enabled; command service not exercised |
| Joints / model / payload | All 19 joint names present; candidate hashes matched |
| Arm / faults / battery | Stowed; zero active faults; 97% |
| Motor power | OFF in preflight snapshots and all 100 unary measurements, including final snapshot |
| E-stop readiness | False; preflight remains failed on this check |
| Stream | 60 seconds completed, 20,001 received messages, approximately 333 Hz |
| Robot acquisition interval | p50/p99 3.000 ms; max 3.010 ms |
| Host receive interval | p50 2.994 ms; p99 3.440 ms; max 5.142 ms |
| Read-only state RPC duration | p50 4.063 ms; p99 5.422 ms; max 5.840 ms |
| Unary measured-state age | p50 6.191 ms; p99 8.596 ms; max 9.146 ms |
| SDK time-sync best-estimate RTT | 3.200 ms |
| Raw body height above estimated ground | Approximately -0.003474 m across 100 samples |
| Foot contact reports | `CONTACT_LOST` on all four feet throughout the unary probe |
| Live ReLIC predictions | Not run: initial-height capture failed before inference |

The ground-plane transform is present and finite. Its estimated plane lies about
3.47 mm above the body origin at that location. This is an unusable standing-height
estimate, not a measurement that the physical body is below the actual floor.
The policy contract requires an initial height of 0.3–0.7 m and four reported
contacts. Neither prerequisite is established by these observations. No positive
height was invented or substituted to make prediction startup pass.

The retained 20 Hz stream samples show very little leg encoder position variation:
maximum per-joint standard deviation 1.953e-5 rad, maximum range 1.278e-4 rad.
The maximum reported leg speed is nevertheless 0.07994 rad/s. These stationary
measurements do not establish the estimator-noise distribution during locomotion.

In the first unary sample, the four knee encoders are 0.00570–0.00963 rad
(approximately 0.33–0.55 degrees) below the live URDF lower bounds. The arm shoulder
pitch is about 0.00276 rad (0.16 degrees) below its bound. These are observed
model/encoder discrepancies in the folded pose; their cause is unproven. No bounds
were widened. The hardware envelope still needs review rather than blind adoption
of URDF limits.

## Interpretation and decision

The Ethernet connection, SDK compatibility, robot/candidate binding and state
stream are working on this boot. Powered-on observation can validate this part of
the deployment stack without energizing the motors.

The paper's command-to-confirmation delay cannot be inferred from unary state RPC
latency, state delivery intervals or time-sync RTT. There were no command writes,
so command acknowledgement timing, actuator response, friction, torque tracking,
startup stability and the full 200 Hz control deadline remain unmeasured.

Retain the failed height capture as evidence. Keep the existing requirement to
capture height and contacts after an explicitly authorized native stand in the
physical rollout. Whether that produces a valid estimate on this boot remains
untested. Do not change the requested no-fade startup, initial zero actions, policy
weights, physical envelope or stop configuration on the basis of these tests.

## Local raw records and reproduction

Raw records contain robot identity and stay in ignored local `runs/` directories:

- `runs/readonly-preflight-20261009-001/`
- `runs/readonly-observation-20261009-001/` — failed pose, skipped shadow, after-preflight;
  exact commands and tmux handle in `RUN_RECORD.md` and `supervisor.status`.
- `runs/readonly-stream-20261009-001/` — successful watch, exact command, durable
  supervisor identity and exit status. `watch/COMPLETE.json` hashes raw artifacts.
- `runs/readonly-rpc-probe-20261009-001/` — exact `probe.py`, before/after snapshots,
  first sample, raw query events, numerical results, source/runtime and completion.

The streaming and unary-probe completion records were verified after collection.
All work has completed; no new background observer was left running. To reproduce,
use the pinned code/environment, an explicitly supplied robot profile and the
recorded commands with fresh output directories. The archived probe has task-local
path constants; update them in a new recorded copy and record its new hash.

# Deployment preparation — October 8, 2026

Runtime/tool implementation commit: `abc1d1aeb2c1fecce8a26728666be743ce6394f0`.
The existing exported policy, gains, observation/action contract and physical
standing controller remain unchanged. New commands package local inputs and
check deployment prerequisites without constructing an SDK client.

## Observed software checks

- Full suite: 316 passed, including 13 new deployment preparation cases.
- Ruff and whitespace checks passed.
- SDK adapter imports are blocked in the offline-command boundary test.
- A real bundled graph is relocated and warmed with zero history preserved.
- Tests reject changed graph/source/packages, omitted digest, path escape,
  simulation envelope, dirty source, stale or differently bound preflight,
  altered artifacts and a replay mislabeled as preflight.
- Even a complete synthetic test-evidence set leaves live_ready false and sends
  zero commands. Physical activation remains in the existing gated stand path.

## Local operational preparation

A local bundle was created using the existing robot profile and exported policy.
Its joystick profile was checked against the exact file hash in the robot binding.
Candidate integrity, runtime identity and robot-profile validation passed.

One separately invoked read-only preflight timed out. No automatic retry was made.
The deployment check reports the missing reviewed hardware envelope/evidence,
failed preflight and unavailable current confirmed local physical-stop status.
All raw records, robot details, bridge profile and local bundle remain ignored.
No lease, motor, motion, E-stop registration, endpoint modification or rearm action
was performed. The existing running processes were left in place.

## Decision and reproduction

Prepared for hardware review, not physical activation. Complete the generated
19-joint worksheet and timing/body limits, supply reviewed evidence, establish a
fresh preflight and verify the independent stop before the existing supervised
standing sequence. The previous 5.920 ms host response still leaves strict 5 ms
release timing unqualified; this change does not retest or promote that result.

[Deployment workflow](../../docs/DEPLOYMENT.md) contains portable reproduction
commands. Each output directory must be new. No simulator rerun was required for
this packaging/check change; no controller dynamics changed.

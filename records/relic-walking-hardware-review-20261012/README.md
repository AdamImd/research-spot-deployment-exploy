# 0.30 m/s walking deployment preparation — 2026-10-12

Adam authorized preparing the faster walking profile for hardware review and
requires a separate explicit question immediately before deployment. Preparing
this review does not authorize motor power, leases or motion. Preserve the
manufacturer tablet as stop authority, the held-arm/harness observation profile,
manufacturer limits, original weights/gains and zero initial previous actions.

The requested goal is **0.75 m in front of the current orientation**, sampled
after native standing for each attempt. The live sequence obtains the current
stream state after `native_stand`, then initializes `ReLICRollout` and latches
`ForwardTravel`. It does not use orientation at boot or a previous attempt's
reference. The heading is fixed during this trial; a turn does not move the goal.
The recorded goal is `p_current + 0.75*[cos(yaw_current), sin(yaw_current), 0]`.
Tests check forward distance, zero lateral offset and acquisition timestamp at
four headings. The only runtime change is adding this explicit goal to
`walk_origin`; request generation and controller behavior are unchanged.

## Selected review profile and unresolved gates

Selected: maximum request 0.30 m/s, constant request law from the recent simulation
comparison, acceleration 0.10 m/s², initial zero request hold 3 s, goal tolerance 2 cm,
final measured stationary hold 1 s, policy-active cap 20 s. Native standing occurs
before the policy-active trial; direct policy startup has no joint-target fade.
These are proposed review parameters, not a qualified hardware configuration.

The existing accepted WalkingPlan permits at most 0.15 m/s and does not select a
constant request profile. That profile is currently limited to the explicit
simulation diagnostic. The hardware envelope's measured body-speed bound is
0.30 m/s, and live admission requires requested maximum strictly below that bound.
No schema, control path or envelope limit has been widened during this preparation.

The paired faster simulations have no falls, but all four exceed some unchanged
hip/knee speed references. Taper stops short; constant ends 5.79–6.72 cm past the
goal. Thus walking-distance/simulator qualification has not passed. An internal
Isaac completion flag followed by drift does not establish accurate stopping.
See [the faster-request record](../relic-faster-sim-20261012/README.md).

Existing hardware evidence predates these source/profile changes and cannot be
copied into newly passed gates. The review contains an empty evidence index and
an explicit blocked activation record, with no executable activation command.
The reference bundle preserves the earlier accepted 0.1325 m/s manifest solely to
exercise existing packaging/checks. It is not the selected faster candidate.

## Fresh robot observations

Read-only preflight from clean commit `a05adc4fd11c5ecf3afbe6a645fa7eb9ae0e61a1`
completed at `runs/walking-preflight-20261012-001/preflight`. Identity/firmware,
joint layout/control license, physical arm, stow and robot stop readiness passed;
motors off, zero active faults, battery 100%. Independently compared model/payload
hashes match the frozen policy. Payload registration list is empty; the physical
arm is confirmed separately. No lease acquired or motion commands sent.

This snapshot is preparation evidence. Final preflight must bind the final
approved candidate and remain fresh (at most 300 s). The native live gate checks
again before lease/motor work. Robot stop readiness does not verify the tablet
link/button or that the named operators are in position.

## Local bundle, recording and final approval

Local final review: `local/relic-walking-deployment-review-20261012-002/review/`.
The earlier 001 review and validation are preserved separately.
Includes copied policy artifacts and `manifest.review.json`, selected trial
parameters, unchanged hardware envelope, model/payload checks, input hashes,
simulated speed/distance failures, empty evidence and `activation.json`.
The review manifest intentionally fails the current accepted walking speed cap;
renaming it is not an approval or a working live controller.

Record the full raw robot state, issued targets, raw actions/ACK history, requested
and measured speed, frozen origin/goal, joint speeds/PD target offsets, estimated
PD torque, stream/ACK/expiry timing and shutdown result. After a trial use
`tools/record_rollout.py` for a gzip robot/action archive with `--state-hz 0`,
retaining every unique sampled state and the explicit goal. Videos are optional;
this review follows the selected robot-data/action recording scope.

Before the final question: resolve profile support and stopping/speed qualification,
review the exact limits without fabricating or widening evidence, freeze/commit
the final candidate and runtime, renew all bound evidence, confirm Adam/Minghao
and the following rig/clear path, then collect a fresh motors-off preflight.
Only after that ask explicitly for one physical trial, including distance, speed,
duration, tablet authority and the fact that motors will power and Spot will move.
Do not launch automatically after checks, resume, or retry a failed trial.

## Reproduction and execution scope

The offline script `prepare_review.py` uses the existing prepared reference bundle,
verified preflight and public faster-simulation summary to create a fresh review
directory. It imports no SDK or control adapter. It preserves source/config/hash
provenance; it does not create passed hardware evidence or a launch command.
Inputs and raw robot snapshots remain ignored; no endpoint, serial or credentials
are committed. Reproduce with the frozen local inputs and a fresh output:

```bash
uv run --no-sync python records/relic-walking-hardware-review-20261012/prepare_review.py \
  --reference-bundle local/relic-walking-deployment-review-20261012-002/reference-bundle \
  --preflight runs/walking-preflight-20261012-001/preflight \
  --simulation-summary records/relic-faster-sim-20261012/result-summary.json \
  --output local/NEW-REVIEW
```

Host `cs-u-rpm-dt-03`, locked UV Python 3.12/CPU environment, no GPU, no paid compute,
no retries. Luna owns each bounded persistent validation/preflight process.
All unresolved gates are reported as such; review completion is not live readiness.

## Completed validation

Final implementation pin: `36513ba59a7076cbfe096b5dedcfc4efb027e128`, clean at
launch. Campaign `runs/walking-review-validation-20261012-002` completed under
tmux of the same name: **461 tests passed**, Ruff and offline demo passed,
reference bundle and selected review package completed, and offline deployment
check returned exit 1 as expected with `live_ready=false`. All review artifact
hashes verified; recording template keeps all unique states, activation command
is null, approval false, hardware qualification false and motion commands zero.
The prior campaign001 passed 460 tests before the compact goal-field repair and
is preserved. Final validation repeated because recorder code changed.

Both campaigns used primary-authored exact command arrays, the locked CPU
environment and a 300 s maximum with no retries. Contracts/supervisors live in
`local/relic-walking-deployment-review-20261012-{001,002}/`; copied commands, logs,
exit statuses, PID, STATUS and finish records are in each durable run directory.
The selected review and prepared candidate remain separate; this expected blocked
check is not a deployment success. The runtime addition only records the goal:
no contact/controller/trajectory change or new dynamics experiment was made.
Sanitized observations and hashes are in `result-summary.json`.

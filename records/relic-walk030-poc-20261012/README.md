# Tapered 0.30 m/s physical proof-of-concept preparation — 2026-10-12

Adam requested execution with increased limits and selected the tapered 0.30 m/s
profile, accepting a short walk as proof of concept. The recent constant profile
is not selected. The user requires a question immediately before physical
deployment; no motor power, control lease or motion is authorized by this record
alone. Final exact settings and current operators/rig/tablet confirmation remain
pending. No simulation envelope is promoted to hardware evidence.

## Frozen trial proposal

Request 0.75 m in the current post-stand heading, taper `min(0.30, 0.8*remaining)`,
acceleration 0.10 m/s², initial zero-velocity hold 3 s, policy-active cap 20 s.
Native standing precedes direct activation; no fade, zero previous raw actions,
original graph/weights/gains, policy 50 Hz, targets 200 Hz. Raw actions advance on
acknowledgement. Position and current yaw are latched after native standing,
and the frozen goal is recorded in raw and lightweight data.

Proposed finite operating caps derived from observed simulated demands:
body speed 0.40 m/s, leg hips 6 rad/s, knees 12 rad/s, shared SDK backstop 12 rad/s,
front-left HY tracking 0.50 rad. Existing front-right HX tracking 0.75 rad and other
tracking bounds remain unchanged. Manufacturer torque, measured-position knee
torque table, coupled-arm torque and joint-position constraints remain enforced.
These speed caps are operating proposals, not manufacturer speed ratings.
No gain change, clipping, reverse correction or local E-stop implementation.
Manufacturer tablet remains stop authority; independent operator Minghao and
robot operator Adam are subject to final in-position confirmation.

Keep all body-attitude, health, state freshness, inference/ACK, command expiry,
stall, overshoot and shutdown guards. TTL 30 ms, native shutdown allowance 10 s,
no automatic retry. The goal tolerance remains 2 cm. A short outcome is explicitly
accepted by the operator as proof of concept, but **does not set goal completion
true**. If the goal is not reached, the existing bounded timeout/short-distance
error stops production and performs native shutdown; preserve the reason and
actual distance. Any other guard failure is reported separately. Tablet remains
available for immediate emergency stop; shutdown must be confirmed motors-off.

## Implementation and validation protocol

Extend the maximum accepted WalkingPlan request from 0.15 to 0.30 m/s; retain the
original taper, strict per-candidate bounds and live envelope/evidence admission.
The normal Exploy adapter now accepts the explicitly bound 0.30 plan without the
simulation-only wrapper. A real-graph regression compares observation, raw action
and target mapping with the independent native actor and rejects requests above
the bound. Original 0.10 candidates retain their tighter bound. Regenerate the
three affected JSON schemas; no graph export or weights change.

`audit_bounds.py` verifies recordings and replays all 12001 command/state samples
from each prior 60 s tapered simulation through the proposed numeric guard, with
recorded policy age/inference time. It cannot claim physical transport timing,
startup handover or goal completion. Those remain separate checks.

Then perform fresh 20 s guarded rollouts with the normal production core in
MuJoCo (no-slip 10) and native Isaac Lab; same captured standing pose, height,
zero velocities/history, seed 101 and 100 discarded inference prewarm calls.
The 20 s horizon matches the live timeout rather than inventing a longer live plan.
Prior 60 s traces supply the longer stability/demand comparison. Run full tests,
Ruff, offline demo and a paced 20 s host core/logger/SDK-encoding benchmark on
recorded states with ideal acknowledgements; record actual wall deadline misses.
The benchmark does not measure live command-network latency.

Preserve all failed checks and first causal errors. Numerical guard failure,
nonfinite/parity/integrity failure or a host budget miss blocks activation until
reviewed; do not widen more bounds or silently retry. Simulation short distance
remains an acknowledged limitation, not an exact-goal pass.

Local inputs/proposal: `local/relic-walk030-poc-20261012-001/`, original standing
capture `local/relic-walk-inputs-20261009-001/capture`. Model/payload and current
motors/fault/tablet stop state must be checked against this exact candidate again
before activation. Freeze and review source/config-bound evidence for one
supervised commissioning attempt; retain the limitations of old hardware records.
Raw robot identities, endpoints, captures and credentials stay ignored.

The final approval question must state the numeric caps, maximum request, travel,
duration, short-outcome interpretation, Adam/Minghao and following rig/clear path,
tablet link/button readiness, and that this powers motors and moves Spot. No
launch follows preparation automatically. Record full raw state/actions, targets,
PD offsets/torque estimates, odometry, timing/ACKs and shutdown, then produce the
lightweight gzip archive retaining all unique sampled states.

Host `cs-u-rpm-dt-03`, locked CPU ONNX/UV environment; GPU 0 for Isaac Kit only,
native Isaac overlay, no GPU 1, no paid compute. One Luna owner per bounded durable
validation run, exact primary-authored command arrays, no retries.

## Completed pre-activation checks

At clean runtime pin `c74bd12b4c70700043c4a0e14b363d75ccee67f8`, campaign
`runs/relic-walk030-poc-validation-20261012-001` passed 462 tests, Ruff/demo, both
full 60 s recorded numeric audits, two fresh 20 s guarded normal-core rollouts and
the 20 s paced host benchmark. The primary verified 4001 state/rollout rows and
1001 predictions per fresh engine (not 12001 rows, which belong to the 60 s audit).
MuJoCo/Isaac forward travel 0.689392/0.687834 m, stable final windows, no guard
stop/fall, goal completion false. Actor target parity max 2.3842e-7 rad.

Timing: 4001 commands, effective 200.043 Hz, no missed releases, observed 5 ms gate
true. Production max 4.033 ms, inference 0.515 ms, gap 8.403 ms and deadline-response
4.623 ms. Ideal state/ACK clocks remain a limitation; this is not fresh physical
command-transport qualification. Raw timing preserves every measured sample.

The fresh exact-candidate motors-off preflight passed with 100% battery, no active
faults and model/payload/identity/layout/license/robot stop-state checks passing.
No motion or lease was used. Local package/evidence binding review is prepared
for one supervised commissioning attempt with older successful stock/standing/
transport records and new guarded/parity/timing artifacts clearly distinguished.
Operator-confirmed rig/path/tablet and exact numerical limits remain unverified
until the final answer; hardware policy qualification remains false.

Pending check and raw preflight: `runs/relic-walk030-poc-pending-check-20261012-001`
and `runs/relic-walk030-poc-preflight-20261012-001`. Prepared bundle and offline
binding procedure: `local/relic-walk030-poc-20261012-001/bundle` and
`bind_review.py` beside it. The exact index/source/config/artifact hashes and
review limitations are retained locally. Proposed activation argv remains
withheld until approval; the final command will be one `spot-deploy walk` with
Adam/Minghao, bound inputs, duration 20 and explicit `--execute`, no retries.

Campaign contract/supervisor: `local/relic-walk030-poc-20261012-001/{contract.json,execute.py}`.
Durable tmux of the campaign name completed all stages. CPU OMP 2, GPU 0 Kit only,
600 s maximum/$0 budget, no recovery. Sanitized observations/hashes are in
`result-summary.json`; raw robot/preflight/input files remain ignored.

## Authorized physical trial outcome

Adam explicitly approved the final exact caps, one attempt, Adam/Minghao and
following rig/clear path/functioning tablet setup. Approval and frozen manifest/
envelope hashes are in the local final-approval record. A newly collected
exact-candidate motors-off preflight and approval-bound deployment check passed.
No additional approval, retries, fault clearing, lease takeover or E-stop writes.

One physical attempt at clean commit `ed1b407e1a48d51ed54a74b0966e20766da78ae0`
ran about 5.660 s of active policy before **command position limit** stopped it.
Native standing preceded direct activation with measured initial height 0.518284 m,
four reported foot contacts and zero initial raw actions. The goal was latched
from this current post-stand pose/heading. Distance at last emitted command state:
**0.208716 m forward, -0.036347 m lateral**; rejected-state forward 0.209334 m. These
are robot odometry measurements before shutdown, not independent metrology or
a claim of final resting displacement. Nominal 0.75 m goal was not reached.

Recorded 284 predictions include one rejected sample; 283 action-history ACKs and
1135 issued joint commands. Maximum requested forward velocity 0.266500 m/s,
maximum inference 0.720 ms, command gap 5.968 ms, issued-state peak leg speed 2.363 rad/s.
The position guard stopped before the request reached its 0.30 m/s maximum.

| Rejected joint | Target (rad) | Measured (rad) | Approved trial target range (rad) | Exceedance (rad) | Estimated PD torque (N·m) |
|---|---:|---:|---|---:|---:|
| Front-left knee | -2.174256 | -1.640778 | [-2.10, -0.80] | 0.074256 | -31.9855 |
| Front-right knee | -0.746815 | -1.739241 | [-2.10, -0.80] | 0.053185 | 59.3999 |

These are the unchanged trial position bounds, not an assertion of manufacturer
full travel ratings. Both measured angles were inside the range. The last
prediction is recorded before Guard.command rejects it; neither target was sent.
Torque values are estimates using unchanged gains/zero feedforward and observed
velocity; they are not independent torque measurements. The first failed check
was target position; no broader safety pass of this rejected prediction is claimed.

Native shutdown confirms motors OFF and lease returned, no cleanup errors,
6.0255 s total within the approved 10 s allowance. Passive read-only postcheck passed
with motors off and zero active faults. **No retry occurred.** This is a stopped
trial, not a successful goal or merely an accepted normal shortfall/timeout.

Raw run: `runs/relic-walk030-poc-hardware-20261012-001`. Supervisor: same prefix
`hardware-supervisor-20261012-001`, durable tmux `relic-walk030-poc-hardware-20261012-001`.
Compact: `runs/relic-walk030-poc-compact-20261012-001/rollout.jsonl.gz`, 1,045,161
bytes, complete capture with raw source status failed preserved. Postcheck:
`runs/relic-walk030-poc-postcheck-20261012-001`. Raw, compact and postcheck
completion/artifact hashes verified. `hardware-result-summary.json` stores
sanitized derived measurements, rejected targets, per-joint issued PD offsets/
torque estimates and verification hashes. Raw robot poses/identities stay local.

With frozen private inputs/logs present, regenerate the derived record from repo
root using `uv run --no-sync python records/relic-walk030-poc-20261012/summarize_hardware.py`.
No robot connection or commands are used by this analysis. Review the knee
target-angle envelope and actual-policy dynamics before any newly authorized
attempt; no position bounds were widened after this stop.

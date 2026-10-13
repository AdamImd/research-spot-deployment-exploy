# Selected-leg deployment preparation, 2026-10-12

Question: can the unchanged released ReLIC actor support the front-left Cartesian
IK sequence through an explicit selected-leg graph, within the accepted guard
profile? Adam approved ONE guarded offline rehearsal; no physical trial approved.

Protocol: build a ten-input graph by extracting the pinned Exploy native
observation graph, inserting the native 12-value leg command, and composing the
unchanged published ONNX actor. SDK-order targets override only front-left when
its command is nonzero; the remaining nine actor targets and held arm are retained.
Compare 64 seeded active/inactive cases with independent baseline observations,
raw actor and NumPy target mapping (seed101). Then one 60s MuJoCo rehearsal, CPU,
200Hz command /50Hz policy, direct startup, zero ACK-owned raw action history,
current accepted simulation envelope, no disabled guards or retries. Use the
captured standing pose, not a fabricated physical stand. MuJoCo no-slip iterations10
matches the reviewed walking simulation setting. Record physics diagnostics,
IK target/tracking, phases, native parity and final stability. Failed rehearsal
is not physical qualification and does not permit a silent guard change/retry.

Known system payload.fault code9 severity1 INFO may be recorded without stopping
ONLY when robot profile explicitly selects known-payload-info-observe and records
operator reason. Other faults, missing detail/count mismatch, and tablet stops
remain blocking. Nothing is cleared. Raw counts/messages are retained.

Inputs and raw results are ignored local/ and runs/; the execution contract records
source commit, simulator commit, hashes, paths, commands and environment. The
original nine-input standing graph and live task whitelist remain unchanged while
this new interface is under rehearsal. No motors/leases/robot commands in this run.

Result: export001 matched64 native cases exactly. Its rehearsal did not step
physics: the selected external simulator checkout lacked diagnostics. This setup
failure was preserved; vendored simulator fixed the dependency selection.
Rehearsal002 (sourcee71816a) stopped at32s on90.98Nm flHX request against44.88Nm,
exactly as native leg mode switched from selected IK to all-zero four-foot mode.
IK motion maxtracking error19.3mm, and foot had returned near the floor. This is
an output/mode handoff transient, not a graph joint-order mismatch. Raw selected-leg
actor actions still update history while its targets are overridden, matching native
ReLIC; no history zeroing or weight/gain change was introduced to mask the issue.

Adam approved2s output return blend and ONE new guarded rehearsal. Rehearsal003
(source7f28ee3) completed60s, full46s sequence, IK/clearance/final-standing checks.
Native maxima: observations7.45e-9, raw actions3.81e-6, targets2.38e-7rad.
Simulation wall-clock timing had3core and7total nominal5ms misses, not hardware
transport measurements. No guard was loosened; no physical qualification claimed.
Separate live schema4 adapter then matched all3,001 retained predictions exactly
against recorded200Hz states and native raw-history inputs. It does not reuse the
simulation wrapper as a hardware adapter. Targeted integration checks86passed;
fault/SDK focused checks58passed; these overlap and are not a full-suite claim.
Raw logs/videos/private profiles are under ignored runs/relic-three-leg-rehearsal-
20261012-{001,002,003} and local/relic-three-leg-live-review-20261012-001. Public
sanitized result files retain failures, source commits and actual numerical outcomes.

Next: freeze live review bundle, current motors-off preflight and exact approval
for ONE47s trial with Adam/Minghao and a clear supported three-foot workspace.
No physical activation has occurred during this preparation.

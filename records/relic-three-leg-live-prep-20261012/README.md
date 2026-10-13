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

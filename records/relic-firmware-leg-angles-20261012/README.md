# Firmware leg-position authority — October12,2026

Question: does the unchanged ReLIC continue a supervised0.75m tapered walking
proof of concept when conservative host measured leg-angle bounds are observed
instead of stopping, with remaining manufacturer and operating guards retained?
Adam requests the physical trial without offline tests and explicitly no retraining.

Implementation: envelope-bound `leg_position_guard=observe` with operator reason,
ReLIC-only adapter, manufacturer torque profile and explicit finite virtual target
ranges required. Only measured leg-angle stops become observations; mode/crossings
are recorded in raw and compact traces. Defaults stay enforced. No weights/gains,
rewards, frames, history or cadence change. Firmware saturates desired positions;
this does not establish measured overshoot/balance protection. Manufacturer torque
and knee table domain, speeds/SDK backstop, tracking, targets, body, timing, ACK,
expiry and tablet controls remain.

Proposal is ignored `local/relic-firmware-leg-angles-20261012-001/`. Same maxrequest
0.30m/s taper/0.10m/s² acceleration/3s zerohold/current-poststand0.75m goal/20s cap,
no fade, zero initial raw actions, shutdown10s. Speed caps body0.8m/s angular1rad/s,
hips12knees/SDK24rad/s, target slewlegs200arm1rad/s; PD-offset2rad. Short travel is
accepted without exact goal success. One distinct recorded attempt, no retry.

No tests, simulations, replays or benchmarks are run at operator request. Existing
results are historical evidence only; this source/guard change is untested. Bundle
creation/schema generation are configuration operations, not claimed test passes.
Fresh motors-off readiness and exact current Adam/Minghao/tablet/rig/path approval
remain pending. Raw identities/endpoints/world poses stay ignored and unpublished.

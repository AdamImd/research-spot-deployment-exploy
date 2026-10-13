# Front-left Cartesian IK demo

Use one unchanged released ReLIC actor through the ten-input selected-leg Exploy
graph. The selected front-left leg follows body-frame position IK; the other nine
leg joints remain actor controlled and the arm stays held. Body x points forward,
y left, z up. Input native joint order and SDK output joint order are independently
verified. No retraining, gain changes or local E-stop bridge. Manufacturer tablet
is the E-stop authority. Initial raw history iszero and updates only after ACK.

## Current sequence and command smoothing

| Phase | Seconds | Behavior |
|---|---:|---|
| Four-foot initial | 5 | Native direct policy startup; no new startup fade |
| Lift/reach | 6 | Quintic Cartesian lift from latched current foot to first waypoint |
| Three-leg hold | 3 | Hold first waypoint |
| IK square | 12 | Four3s quintic segments |
| Lower | 6 | Return to latched foot position |
| Four-foot return handoff | 2 | Selected-leg output blend back to actor |
| Four-foot final | 12 | All feet under actor control |

Total46s; future live margin1s gives47s. The8cm square uses body-frame foot-center
waypoints x0.53…0.61m, y0.22m, z−0.28…−0.20m. It follows current body orientation,
not boot/odom axes.3DOF position IK provides no foot orientation constraint.

At mode entry, raw actor targets can change abruptly; we do not claim those raw
predictions are smoother. `target_slew_rad_s=1` bounds the12 leg virtual targets
ACTUALLY commanded after the initial four-foot phase through finalhold. Initialize
from the preceding four-foot command. Exact mode-entry increment iszero; later
increments are atmost0.02rad per50Hz prediction, including after a delayed tick
(no catch-up). Arm targets unchanged. Preserve the actor's ORIGINAL raw output
for ACK-owned previous-action history; do not replace history with joint angles,
filtered target offsets or an unused alternate actor branch. Record raw proposals
and limited commands separately. This is explicit target postprocessing. Existing
manufacturer torque, measured state/body, deadline, ACK and TTL guards still apply;
a slew limiter is not torque or hardware qualification.

IK uses the manifest-hashed URDF, bounded damped least squares, analytic3x3 Jacobian,
previous seed,0.02rad geometry margin and0.2mm solve tolerance. Planned selected-leg
IK speed max0.8rad/s; commanded target rate max1rad/s. Required motion tracking p95
<=3cm. Reject unreachable/singular paths instead of silently clipping Cartesian
requests. Model geometry bounds are distinct from approved hardware envelopes.

## Evidence and failures

Original kinematic previews had no actor or physics. The first guarded dynamics
return stopped32s on90.98Nm flHX demand; the operator-approved2s return blend fixed
that simulator handoff. One subsequent physical attempt stopped5s at entry on
46.42Nm FRHX demand against44.88Nm; rejected command was not sent. Shutdown/OFF and
lease return confirmed5.017s; no fault/physical lift or retry claimed. See
records/relic-three-leg-hardware-20261012.

Smooth dual-actor entry blends (foot held, overlapping lift, slower4s transfer)
and a support lean still crossed the body angular-speed cap in recorded-pose
simulations. These failed experiments are preserved; they are NOT enabled in the
live contract. Analysis exposed~0.35rad knee target changes per20ms inside their
feedback loops. The final single-actor bounded-command method passed guarded60s
MuJoCo runs from BOTH the recorded physical pre-entry joint pose and the earlier
standing capture. Full lift/square/return/final-standing and clearance checks passed.
Max post-entry target step0.02rad; body angular peaks0.878/0.918rad/s below1rad/s.
IK tracking p95~1.92/1.91cm. Independent native observation/raw-action parity passed;
live adapter matched all3,001 recorded physical-pose-case predictions exactly.

Recorded-pose simulation root placement0.442334m is URDF stancefoot ground alignment,
not a measured physical floor height. Original torso-height request0.518528m was
retained independently; simulator starts withzero velocities/history. MuJoCo results
are not Isaac/hardware qualification; nominal5ms wall-clock misses remain in logs.
See records/relic-three-leg-entry-20261012 for source commits, protocol and results.

## Reproduction and deployment review

`tools/export_relic_selected_leg.py` builds the ten-input graph from the pinned
native Exploy observation subgraph and unchanged ONNX actor. Original nine-input
standing graph remains unchanged. Package a distinct schema4 relic-exploy-leg /
task=three-leg-demo candidate with `tools/prepare_selected_leg_candidate.py`.

`tools/simulate_relic.py --three-leg-plan configs/relic-three-leg-front-left-ik.json
--three-leg-graph PATH` rehearses the new graph offline using a standing baseline
and simulation envelope. `--initial-root-height-m` changes only simulated placement,
not the torso-height input. `tools/verify_selected_leg_adapter.py` compares a live
adapter against an existing recording without physics/robot IO. The separate live
adapter never dispatches the simulation wrapper.

CLI `three-leg` requires complete bound three_leg_sequence/three_leg_clearance
review, exact plan duration+1s, current motors-off preflight, two named operators,
rig/foot/arm clearance, functioning tablet and NEW exact physical approval. The
previous physical authorization is consumed; changed source/profile invalidate its
old evidence even though nominal duration is again47s. No physical rerun is part
of this fix. Known system payload.fault code9 severity1 INFO may be observation-only
only with explicit robot-profile policy/reason; preserve raw faults/messages, block
all other faults or incomplete counts, and never clear faults automatically.

The newly requested smoothed physical trial (October12) verified commanded target
steps<=0.02rad and rates<=1rad/s on hardware, including a~0.000016rad initial mode
step. It stopped~5.34s on body angular speed1.0047rad/s above1rad/s, shortly after
lift began; no completedthreeleg/IKsquare is claimed. Shutdown/OFF/lease confirmed
5.015s; passivepostcheck faultfree. No retry/limit change. This approval is consumed.
See records/relic-three-leg-smoothed-hardware-20261012 and compact recording.

The subsequent one-trial review increased only measured body angular speed to
2 rad/s, with no offline tests. The recorded attempt stopped at 5.485 s during
lift on pitch 0.179495 rad (10.284 degrees) above the retained 10-degree attitude
cap; angular speed was 1.061006 rad/s. Target steps/rates remained <=0.02 rad /
1 rad/s. Motors OFF and lease return confirmed in 5.015 s; fault-free passive
postcheck passed. No full sequence or hardware qualification; approval consumed,
no retry. See records/relic-three-leg-angular2-hardware-20261012.

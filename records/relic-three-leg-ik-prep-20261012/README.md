# Payload warning explanation and front-left IK preparation

The last walking proof of concept stopped in the generic health guard. A fresh
read-only full-state query (no lease/motor command/fault clearing) recovered the
exact active system payload.fault/code9/severity1 message:

> Center of mass estimated right of expected. Payload configuration may be inaccurate.

Severity1 is SEVERITY_INFO in the pinned SDK5.0.1 SystemFault enum. ReLIC's health
monitor aborts on ANY fault_count, so informational messages stop it too. This is
not a reported critical actuator fault or proof the tablet E-stop was pressed.
Robot fault codes are fault-specific, not a universal9code mapping. The exact
message is evidence of load/COM mismatch; arm pose or harness external loading
are plausible contributors, not demonstrated causes. No fault was cleared and
no payload configuration or ReLIC fault-filter policy changed. The separate stock/
crouch diagnostic's prior exact-known-INFO exemption does not apply automatically
to ReLIC. Official source: https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/protos/bosdyn/api/robot_state.proto .
Private diagnostic: local/relic-fault-detail-20261012-001/.

Prepare a simulation-only front-left IK square in front of the current bodyframe,
with unchanged released actor/gains/ACK history controlling the other three legs.
Default44s schedule:5s initial,6s lift,3s hold,12s square,6s lower,12s finalstanding.
Square x[0.53,0.61], y0.22, z[-0.28,-0.20]m; current body axes, not boot/odom.
Bounded analyticJacobian IK from the pinned URDF, previous-solution seed,
0.2mm residual tolerance/0.02rad model-limit margin,0.8rad/s planned command cap.
This is positional IK, not a foot-orientation or collision solver.

No new offline tests or physics rehearsal are automatically added. Generate a
kinematic plan/video only for visual review, labelled body/otherlegs fixed,
no actor/physics/robot execution. Previous failed three-leg mode transitions stay
visible; this geometric plan is not a stability or hardware qualification claim.
Current9input Exploy graph lacks selectedleg input/override; live hardware path
is unchanged and a new export/live contract plus support/contact/limits/tablet/
rig/fault review remains necessary before physical3leg demonstration.

See docs/THREE_LEG_IK.md and configs/relic-three-leg-front-left-ik.json. Use fresh
run directories, frozen source/model/config/capture hashes; never overwrite failures.

## Preparation artifacts

Clean source86c10d1 generated the44second kinematic plan with8801 samples at200Hz.
Max planned IK residual0.199997mm; peak planned selected-leg speed0.27195rad/s.
No actor, physics or hardware executed; other legs/body stay fixed for visualization.
Campaign001 renderer failed on SDKarm0/nativearm naming. That failed run is retained.
Primary repaired only the replay mapping and links at774191f; campaign002 reused
that SAME plan, producing2201 visual-mesh frames at50fps (encoded44.02seconds),
with explicit kinematic/noRL/no-dynamics labels and body-frame path/joint plots.
No automatic retries or extra tests/physics were added.

[Preparation summary](preparation-summary.json) binds source/config/model/input/
trajectory/completion/render hashes. [Fault explanation](fault-explanation.json)
retains the exact text/severity and read-only source without identity/endpoint.
Kinematic FK/IK feasibility is not three-leg support/balance qualification, and
there was no fresh RL transition test. Published weights are unchanged; the
standing nine-input Exploy graph still lacks selected-leg command semantics.

Private full artifacts: runs/relic-three-leg-ik-preparation-20261012-001/plan/;
render/plot/video: runs/relic-three-leg-ik-preparation-20261012-002/video/.
Supervised durable contracts/logs remain in those parentrun directories and
ignored local/relic-three-leg-ik-preparation-20261012-{001,002}/. Reproduce the
kinematic artifacts using the commands in docs/THREE_LEG_IK.md; public fixtures
suffice. No robot credentials, tests, policy inference or physics are needed.

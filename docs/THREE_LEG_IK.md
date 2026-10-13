# Front-left Cartesian IK demo — preparation only

Adam requested the unchanged ReLIC controller on three legs, with the fourth
moving by IK ahead of Spot. Use the previously selected front-left leg. The
existing nine-input standing Exploy graph fixes the leg command at zero and
cannot represent this demo; its live standing/walking paths remain untouched.
`SimulationThreeLegPolicy` uses the SAME released actor weights, inserts the
selected leg's absolute three-joint command into the native84D observation and
overrides only those three output targets, matching native ReLIC. Other nine leg
joints remain actor controlled; the arm stays held. No rewards or retraining.

## Sequence and reference frame

`configs/relic-three-leg-front-left-ik.json` defines a44second plan:

| Phase | Seconds | Action |
|---|---:|---|
| Four-foot standing | 5 | Zero leg command |
| Lift/reach | 6 | Quintic Cartesian path from latched actual foot to first waypoint |
| Three-leg hold | 3 | First waypoint |
| IK movement | 12 | Four3second quintic segments around an8cm square |
| Lower/return | 6 | Cartesian return to the latched foot location |
| Four-foot final | 12 | Zero leg command, actor controls all legs |

The square is in the body x-z plane at y=+0.22m, x=0.53…0.61m and
z=-0.28…-0.20m. Body x points forward, y left, z up. It follows CURRENT body
orientation rather than boot/odom axes; body translation/rotation does not turn
it into a world-fixed target. Positions refer to the foot-link/ball center, not the
bottom of the foot or a requested foot orientation. The 3DOF leg provides position
IK only. If a world-fixed target is desired, that needs an explicit different frame.

IK uses the manifest-hashed URDF chain and analytic translational Jacobian,
previous solution seed, bounded damped least squares, a **0.02rad joint-angle
margin** inside model bounds, and0.2mm solve tolerance. It rejects unreachable or
near-singular targets rather than silently clipping the Cartesian target. Planned
selected-leg command speed is capped at0.8rad/s; actual tracking tolerance is3cm
(p95 in the IK-motion phase). IK model bounds are geometry bounds, not approved
hardware operating limits. The actor's other joints still use the envelope guards.

## Review the intended motion without physics

```bash
uv run --no-sync python tools/prepare_three_leg_ik.py \
  --plan configs/relic-three-leg-front-left-ik.json \
  --manifest policies/relic-exploy-standing/manifest.json \
  --capture fixtures/standing-capture --output runs/front-left-ik-plan-NEW
MUJOCO_GL=egl uv run --no-sync python tools/render_relic_demo.py \
  --run runs/front-left-ik-plan-NEW --output runs/front-left-ik-video-NEW --fps 50
```

The artifact is a kinematic
plan: body and other legs held fixed, no actor execution, contact solver, `mj_step`
or robot connection. Video labels preserve this distinction. It illustrates foot
workspace and joint movement, NOT successful RL balance or physical clearance.
Visuals use the full restored model meshes. The cyan marker is the desired foot
center. `ik-plan.png/pdf`, video, trajectory, model/source/input hashes and completion
records support review without a physical rollout.

## Offline dynamics entry point when separately requested

Use a standing baseline and a SIMULATION-scoped envelope. A walking manifest is
rejected. The same `--three-leg-plan configs/relic-three-leg-front-left-ik.json`
argument works with `tools/simulate_relic.py` in MuJoCo or IsaacLab. Require a duration
covering the44second sequence; the existing60second protocol retains the final
standing window. Do not route this adapter to the live CLI or pretend the old
Exploy graph accepts selected-leg commands. The prior joint-pose three-leg runs
stopped at mode activation in both simulators; this new IK sequence does not fix
or qualify that transition by construction. Do not claim stability without a run.

Record Cartesian target/measured foot coordinates, IK residual/tracking errors,
all joint targets/measurements/torques, previous raw-action acknowledgements,
height/tilt and contact data. Existing clearance geometry is a proxy only, not
proof the three support feet carry load. Return-to-four-feet, IK tracking and final
standing must pass separately. Physics/collision/support-force review and a new
selected-leg Exploy export/live contract/stop review are still needed before a
physical demonstration. The current payload COM INFO warning also needs an
explicit live fault-policy review; it has not been silently cleared or ignored.

## Selected-leg graph under preparation (2026-10-12)

`tools/export_relic_selected_leg.py` composes the pinned Exploy native observation
subgraph with the unchanged released ONNX actor and a new `leg_command` input.
The graph uses `relic-exploy-leg-v1`, 12 SDK-order leg commands, SDK-order19targets,
and a front-left override only. A zero command means four-foot mode. The original
standing graph is unchanged. `tools/simulate_relic.py --three-leg-graph PATH`
can rehearse this graph offline with `--three-leg-plan`; it is not a live adapter.
Adam approved one guarded dynamics rehearsal before physical deployment review.
See records/relic-three-leg-live-prep-20261012. Live support must not be activated
or inferred from structural/numeric parity alone.

The separately explicit robot-profile `fault_policy=known-payload-info-observe`
with `fault_policy_reason` permits only system `payload.fault` code9 severity1.
Original counts/details/messages remain recorded; all other faults or incomplete
fault detail counts still block. This does not clear or acknowledge firmware faults.

## Guarded rehearsal and live preparation

The original handoff stopped at32s: fl_hx target1.721rad against actual0.205rad
requested90.98Nm, exceeding44.88Nm. Lift/hold/IK tracking passed; no complete
return was demonstrated. Adam approved a2s quintic return OUTPUT blend from the
last selected-leg IK pose to the current actor target, while the native leg
command switches to zero. Weights, gains, raw ACK history and startup are unchanged.
The updated plan is46s (plus1s live margin). A newly approved60s guarded MuJoCo
rehearsal completed the entire sequence and passed final standing/IK/clearance
checks. See records/relic-three-leg-live-prep-20261012. This is one simulator result,
not hardware qualification or Isaac verification. Recorded wall-clock outliers
remain; the same computation/command deadlines still stop physical execution.

Schema4 `relic-exploy-leg`, task `three-leg-demo`, is a distinct live contract.
It verifies the pinned base graph input joint order separately from SDK-order
outputs. `three-leg` requires the full46s sequence plus1s margin, bound additional
`three_leg_sequence` and `three_leg_clearance` evidence, fresh preflight, two named
operators and explicit execution. `stand`/`walk` reject this task. The live adapter
matched all3,001 retained rehearsal predictions exactly. It is a separate adapter;
the simulation wrapper is never dispatched to hardware. Initialization and
ACK-owned action history remain zero/unchanged. Demo telemetry retains Cartesian
IK targets, actual foot coordinates, tracking errors and phase/leg commands.

Package with `tools/prepare_selected_leg_candidate.py`; frozen preparation uses
`prepare-deployment`. Selected-leg candidates are deployment review inputs until
robot, rig/workspace, tablet and final exact one-trial approval are current.

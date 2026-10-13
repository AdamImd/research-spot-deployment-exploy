# USB joystick ReLIC arm preparation, 2026-10-13

## Question and protocol

Can the unchanged four-foot ReLIC actor accept body-frame X/Y/yaw commands while
the arm deploys to a front pose and returns, retaining the shared rollout guards,
zero initial actions and ACK-owned raw-action history?

Implement schema5 joystick-arm over the existing nine-input graph. Use the public
standing capture, zero simulated velocities, seed101, default saturated MuJoCo
actuators, 200 Hz physics/commands and 50 Hz inference. Keep gains/weights unchanged.
Use the explicitly simulation-scoped `fixtures/relic-direct-simulation/envelope.json`,
including the 0.15 rad arm-reference tracking guard, with guards enforced. Use the
existing 10-iteration no-slip setting, the registered 60-second script and final
10-second standing check. No robot access or physical deployment is part of this work.

Source branch: `feature/relic-joystick-arm`. Full-suite source `b4dad66`;
compact-pose simulation source `1a8752a`. A subsequent wrapper-only fix rejects a
live `--profile` override instead of silently ignoring it; controller/policy and
simulation configuration are unchanged by that fix. Its nine targeted tests pass.
Environment is locked UV/Python3.12 with simulation and visualization groups; exact
versions and input/raw-output hashes are in [artifacts.json](artifacts.json).

## Evidence

- Initial software campaign001: 525 passed, five failures. Three exposed missing
  new guard initialization in the legacy crouch subclass; two lacked the optional
  plotting environment. Fixed the subclass explicitly and included the already
  locked visualization dependency group. All failed logs remain under `runs/`.
- Campaign002: **530 tests passed**, Ruff and bundled Exploy demo passed. The initial
  longer front pose reached forward locomotion but stopped at19.24s on SH1 arm
  tracking0.1511rad against0.150. No fall occurred. Do not count this as completion.
  [Original failed result](extended-pose-failure.json).
- Geometry revision: use `[0,-2,2.4,0,-.4,0]` for the six arm joints, gripper unchanged.
  This brings the wrist closer to the body and reduces shoulder gravitational
  loading; no guards/gains/weights were weakened or altered. Model FK wrist position
  is `[.552,0,.408]m` and finger-link origin `[.669,0,.423]m` in the body frame.
- Campaign003: **60s completed**, no guard/fall, final standing passed,3001 policy
  samples. It consumed +X.15/−X.10m/s,±Y.10m/s,±yaw.20rad/s. Arm deployed over10.072s
  and returned to the stowed reference before the final standing window.
  [Compact-pose result](compact-pose-result.json).
- Command-to-measured peak arm errors, SDK arm order:
  `[.011931,.093316,.123744,.012827,.018819,.010036,.006718]rad`.
  Largest was EL0; SH1 stayed below.094rad.
- Per-step independent native parity: observation5.96e−8, raw action9.54e−7,
  target2.38e−7rad. Unit parity also exercises32 varied three-axis/moving-arm states.
- Core p99/max.644/2.864ms. **Full offline loop maximum47.495ms**, including two state
  processing outliers. These throughput measurements do not qualify live watchdog
  timing or wired transport. Final-window maximum tilt.163degrees; height span2.56e−6m.
- USB-only preview verified the reconnected Logitech Extreme3D by-id identity,
  6axes/12buttons, axis codes `[ABS_X,ABS_Y,ABS_RZ,ABS_THROTTLE,ABS_HAT0X,ABS_HAT0Y]`.
  Port change worked. Manual axis-sign/trigger manipulation remains to be verified.

## Reproduce

Follow [JOYSTICK_TELEOP.md](../../docs/JOYSTICK_TELEOP.md): prepare a fresh policy
from the checked-in profile, then run the documented guarded60s MuJoCo command.
The complete input script is `configs/joystick-relic-simulation-script.json` and is
also saved in the raw configuration. Raw campaigns001–003 are at
`runs/joystick-arm-validation-20261013-{001,002,003}`; each has durable supervisor
logs/metadata, with simulation rows/events/timing in002 and003. Preserve failures.

## Decision and remaining physical checks

Software and this one simulated compact-pose sequence pass. Default example now
uses that compact pose. This is not collision/workspace/harness qualification, a
full-speed joystick test, an Isaac parity run or hardware approval. The simulation
script exercised lower speeds than the configurable joystick maxima. The static
arm waypoint path is not the manufacturer's stow planner. Review it, the full
operating envelope, tablet stop, input mapping/disconnect behavior, native shutdown
from the deployed arm and travelling rig clearance before a physical trial.

No robot was connected, leased, powered or commanded. The provided tablet remains
the only E-stop authority; this joystick is a motion input. A new bound review and
explicit confirmation immediately before physical execution remain required.

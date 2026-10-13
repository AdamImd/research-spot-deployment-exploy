# USB joystick ReLIC locomotion and arm deployment

`tools/joystick_relic.py` uses the unchanged ReLIC Exploy policy, with explicit
body-frame X/Y/yaw commands and a reversible arm-joint path. The default invocation
is a USB-only preview: no SDK connection, lease, motors or E-stop registration.
The manufacturer tablet remains the sole E-stop authority. This does not run the
older `spot-estop` joystick bridge; trigger button 0 now requests arm movement.

## Controls (Logitech Extreme 3D)

Indices are Linux/SDL indices, starting at zero; printed button labels start at one.

| Input | Function | Default limit |
| --- | --- | --- |
| Stick forward/back, axis 1 negated | Body forward/back (W/S) | ±0.30 m/s |
| Stick left/right, axis 0 negated | Body left/right (A/D) | ±0.20 m/s |
| Twist, axis 2 negated | Body yaw, left positive | ±0.50 rad/s |
| Hold thumb button, index 1 / label 2 | Enable motion after a centered press | Release requests zero base velocity and pauses arm |
| Trigger, index 0 / label 1 | Toggle deployed/stowed arm, while enabled | Rising edge only |
| Button index 3 / label 4 | Finish session, normal native shutdown | Not an E-stop |
| Ctrl-C | End script and use normal shutdown if live | Tablet for emergency stop |

Center and release all buttons before live startup. Initial held buttons/INIT events
cannot enable or toggle. After the initial three-second zero-command hold, hold
button 2 and press the trigger. Trigger presses during the initial hold are discarded.
No window focus is needed. Deadzone is 12%, rescaled outside the deadzone. Requested
velocity acceleration limits are 0.10 m/s² in X/Y and 0.40 rad/s² in yaw. These are
joystick-command limits, not filters on the actor's leg actions. Release zeroes the
velocity request immediately; it cannot make actual robot motion stop instantly.

The by-id device path and vendor/product/name bind the USB identity, so a USB-port
change is supported. Reconnection during a live session requires a new invocation;
there is no automatic resume. Missing device, replacement, event resynchronization
and polling gaps over 150 ms terminate the session through the existing cleanup.
Idle joydev devices emit no events: availability is checked by ioctl, not inferred
from the time since the last stick movement. Physical signs still need preview
verification by moving the actual stick before commissioning.

## Arm behavior

The example contains an intermediate folded clearance waypoint and a front pose.
Final six arm angles in SDK order (SH0, SH1, EL0, EL1, WR0, WR1) are
`[0, -2.0, 2.4, 0, -0.4, 0]` radians. The gripper stays at its initial stow target.
Pinned model forward kinematics puts the final wrist approximately
`[0.552, 0, 0.408] m` from the body origin, with the finger-link origin at
`[0.669, 0, 0.423] m`. These are model positions, not measured hardware clearance.

Each segment uses a quintic trajectory with zero endpoint velocity/acceleration,
minimum four seconds and maximum 0.5 rad/s per arm joint. Another trigger press
reverses along the same path, including midway through a deployment. Releasing
enable pauses path progress. No catch-up motion follows a delayed tick. The path
is a setpoint trajectory, **not** the manufacturer's collision-aware stow planner.
It must be reviewed against the actual harness, arm, payload and body geometry.
Native arm-ready/stow RPCs are not mixed into the active joint-control session.

The nine-input graph is byte-identical. It already accepts `arm_command[7]` and
`velocity_command[3]`; no re-export, gain change or retraining is required.
The actor runs at 50 Hz and receives the current arm reference. The arm's native
position override follows that reference at 200 Hz between actor ticks. The twelve
leg targets remain unfiltered, and raw previous actions advance only after ACK.
Previous actions start at zero and body height is captured after native stand.

Schema 5, `task=joystick-arm`, `morphology=spot-arm-mobile` binds the complete input
profile and arm path into the manifest/evidence hashes. Legacy standing/walking/
three-leg manifests retain their held-arm restriction. The new task enforces arm
position, speed, tracking, load and command bounds; the old harness observation
exception cannot be used to authorize moving the arm. Torque/timing thresholds are
read from the separately reviewed envelope, never replaced by this input script.

## Run the preview or prepare a candidate

From the repository, using the existing locked Python 3.12 UV environment:

```bash
uv run --no-sync python tools/joystick_relic.py preview --duration 60
uv run --no-sync python tools/joystick_relic.py prepare \
  --manifest policies/relic-exploy-standing/manifest.json \
  --profile examples/joystick-relic-arm.json \
  --output local/joystick-policy-001
```

Preparation copies the pinned artifacts into a new directory and writes a bound
manifest. It does not copy prior approval or change any hardware envelope. Editing
the profile afterward does not alter this frozen manifest; prepare a new candidate.
`--profile` applies to prepare/preview only. Live control uses the manifest profile.

A reproducible offline input sequence deploys, exercises both signs of all three
axes, stows and holds for the final ten seconds:

```bash
uv run --no-sync python tools/simulate_relic.py \
  --simulation-source simulation --capture fixtures/standing-capture \
  --manifest local/joystick-policy-001/manifest.json \
  --envelope fixtures/relic-direct-simulation/envelope.json \
  --teleop-script configs/joystick-relic-simulation-script.json \
  --backend mujoco --mujoco-noslip-iterations 10 --duration 60 \
  --output runs/joystick-sim-001
```

That simulation uses explicit simulation limits and actuator saturation. It does
not establish hardware qualification. The input sequence is included in its saved
configuration; the usual state/action/target/PD torque and ACK records are retained.

## Physical commissioning (separate operator approval)

Use `spot-deploy prepare-deployment` on the prepared manifest to create the review
bundle. Review the arm path and operating envelope, and obtain bound evidence for
`joystick_control`, `arm_deployment`, and `teleop_clearance`, plus the existing
identity, dynamics, timing, transition/shutdown and tablet gates. Earlier walking
approval with a stowed arm does not qualify the moving-arm candidate.

After review, current preflight and explicit approval immediately before execution:

```bash
uv run --no-sync python tools/joystick_relic.py run \
  --manifest local/REVIEWED/policy/manifest.json \
  --envelope local/REVIEWED/envelope.json --robot local/REVIEWED/robot.json \
  --evidence local/REVIEWED/evidence.json \
  --operator Adam --safety-operator Minghao \
  --output runs/NEW-joystick-trial --execute
```

The wrapper delegates to `spot-deploy joystick`; both use the same readiness gates,
fresh preflight, native stand, watchdogs, recording, bounded shutdown and lease
return. Default duration is the exact manifest duration (60 seconds in the example).
There are no retries. On error or expiry, shutdown may move the arm/body; review
that path as well. Tablet emergency stopping remains the independent operator's job.

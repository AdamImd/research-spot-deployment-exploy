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

The live link is checked before power-on, then reopened immediately after native
stand to discard the unread kernel event queue accumulated during blocking startup.
The physical device identity must remain identical. Fresh INIT snapshots never
enable or toggle; release/center and press enable after standing. Once active,
event loss/resynchronization still stops the session; there is no automatic resume.

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

For an explicitly requested compliant manipulation profile, set
`arm_tracking_guard=observe` and provide `arm_tracking_reason` in the reviewed
envelope. This logs all seven arm/gripper reference and target tracking crossings
instead of stopping for them. Measured arm position/speed, requested and measured
torque (including coupled motor limits), command bounds/slew, leg tracking, timing,
body and tablet guards still enforce. `arm_motion_guard` stays `enforce`; the
broader legacy harness exception remains incompatible with this mobile-arm task.
Default arm tracking remains enforced. Mode and crossing events are included in
the compact recording as well as the original state/action/command recording.

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
The plan and envelope duration contracts support at most 300 seconds. For an operator-requested five-minute
session, prepare with `examples/joystick-relic-arm-five-minute.json` and review a
hardware envelope with the same 300-second maximum. A live duration override
cannot extend an already bound 60-second manifest. The longer session needs a
new bound one-trial review; the completed 60-second run is historical evidence.
There are no retries. On error or expiry, shutdown may move the arm/body; review
that path as well. Tablet emergency stopping remains the independent operator's job.

For an operator-started session from a frozen reviewed bundle:

```bash
uv run --no-sync python tools/run_joystick_bundle.py --bundle local/REVIEWED --execute
```

Without `--execute` it only displays the profile. Supplying `--execute` confirms
that Adam and Minghao (or the supplied `--operator`/`--safety-operator`) are ready
with the functioning tablet and reviewed travelling rig/arm/shutdown clearance.
The command performs fresh preflight, creates a unique `runs/joystick-operator-*`
recording, runs the bound duration, then checks motors off and creates a compact
recording. Ctrl-C requests the controller's native shutdown. It never retries.
Failures print their recorded reason and any blocking robot fault directly in the
terminal, followed by the motors-off confirmation when available.
Evidence must match the frozen source/configuration and remain within its reviewed
validity; source changes require a new bundle. Preparation and the default display
do not authorize agent-initiated physical motion.

## Joystick demonstration recorder

On the control desktop, from the repository root, open the recorder with:

```bash
tools/joystick_demo_ui.py
```

USB-only mode starts by default. **Start recording** captures body-frame forward,
lateral and yaw commands. Center the Logitech Extreme 3D, release and then hold
printed button **2**, and drive after the initial three-second hold. Release button
2 to zero all three commands. For a calibration trace, leave the stick released
for at least a few ticks before **Stop and save**. Printed button **4** also finishes.
The window displays the effective command and the recording directory. The USB
recording contains timestamped commands and a summary. The arm stays stowed
during demo collection and enabled trigger presses mark the recording.

To also offer physical collection, pass the directory of a reviewed joystick
bundle with `--bundle PATH`. Opening the window still selects USB-only. Choose
**Physical Spot** and click **Connect** for a read-only identity and tablet-status
connection. Enter the two operators and confirm **Stand up** to begin a bounded
manual session under Spot’s native standing controller. **Enable ReLIC** activates
joint control. **Disable ReLIC · native stand** requests a stock stand, then
waits for standing feedback. Enable again for a new ReLIC segment starting
with zero previous actions and a freshly armed joystick. **Release · sit down**
requests safe sit/power-off and lease return from either mode. The duration
limit starts at the initial stand and includes time in native stand. The runtime checks the bundle, device identity, tablet and preflight.
Window close also requests native
shutdown, waits for the motors-off check and saves the recording. The tablet is
the emergency stop. The physical run directory includes full trial events,
50 Hz decisions, 200 Hz streamed commands, acknowledgement and fault records,
compact data and shutdown outcome. A partial or failed run retains its status.
The UI uses system Python with GTK 3 on Linux and the checkout's UV environment
for workers. Launch it on the computer with the joystick and the reviewed robot
connection. The normal arm-toggle launcher remains available for other tasks.

For repeated calibration sessions use a separately reviewed bundle prepared with
a registered family profile. Use **Start recording** for its uninterrupted run. The confirmation identifies that family. Its frozen
base commands run in order at policy decisions, with button 2 held. Releasing
enable during movement ends the repeat. USB-only mode always records manual input.
See [the frozen ReLIC calibration record](../records/relic-sysid/README.md).

## Drive the simulator with the USB stick

On the RPM desktop:

```bash
/home/adam/repos/research/research-spot-exploy-public/tools/joystick_demo_ui.py --sim
```

Click **Connect**, optionally **Start camera streams**, then **Stand up** and **Enable ReLIC**.
Center the stick, release then hold the thumb button. The first three seconds of each
segment request zero velocity. **Release · sit down**, button 4 and closing the window end the session. The default simulation lasts up to 60 s.
It uses the actual USB device, the frozen ReLIC actor at 50 Hz, the guarded
rollout and 200 Hz MuJoCo PD commands. Native stand and sit are kinematic poses in MuJoCo. Handback preserves planar
location and heading. Each ReLIC segment runs the policy and contact dynamics.
The initial standing fixture uses captured body orientation and zero velocity. Control acknowledgements are
idealized with one tick of delay. Events, the sampled room geometry, latest
camera frames and result are saved in the displayed run directory.

| Physical input | Demo recording and simulator |
| --- | --- |
| Thumb button 2 | Hold to enable. Release immediately requests zero velocity. |
| Push/pull stick | Forward/back, up to ±0.30 m/s |
| Move stick sideways | Strafe, up to ±0.20 m/s |
| Twist handle | Turn, up to ±0.50 rad/s |
| Trigger (1) with enable held | Increment the recording marker. The arm stays stowed. |
| Hat left/right | Previous/next camera. Center the hat between clicks. |
| Hat up/down | Return to the seven-camera grid. |
| Button 4 | Finish and save. The physical tablet retains emergency-stop authority. |
| Throttle lever | Unassigned in the demo so the registered speed scale is repeatable. |

The recorder checks the manufacturer's USB identity and Linux joystick axis map.
RPM reported 046d:c215, six axes (X, Y, Rz, throttle, hat X/Y) and twelve buttons.
Linux indices start at zero, printed labels at one. The manual describes the
controls; the assignments above belong to this recorder.
[Joystick manual](https://manuals.plus/asin/B00009OY9U.pdf) ·
[Logitech controls](https://www.logitechg.com/en-us/shop/p/extreme-3d-pro-joystick)

## Room-camera monitoring

**Start camera streams** shows both FLIRs, both Kinects, both D415s and the L515.
In MuJoCo it enables the calibrated renderer as soon as the simulator is connected.
In physical mode it works independently of the Spot connection or motion session.
On RPM it attaches to an existing fleet service or starts a bounded 20-minute
acquisition using the fleet repository and its saved calibration. The fleet
checkout must support `fleet-track run --preserve-host-network`. For a matching older camera checkout, run `git apply --check` and `git apply` there with the [fleet patch](../integrations/flir-apriltag/fleet-shared-ethernet.patch). This leaves
the shared Ethernet MTU, receive ring, interrupt coalescing and system receive
buffer unchanged. FLIR synchronization and remote camera startup can take over
a minute. The panel shows the time waiting for the first frame for each view.
For read-only monitoring on startup, pass `--live-monitor --bundle PATH`.
Transient SDK timeouts disconnect the read-only display and permit up to three
consecutive attempts. Physical control retains its existing deadlines. Stop streams or
close the UI to finish an acquisition started here. When attached to someone
else’s service, Stop only detaches this viewer. Freshness is checked per frame
and frames over one second old are cleared. Cameras provide monitoring only.
Use `--fleet-url` to view an existing receiver or `--fleet-repo` to locate the fleet
checkout. The acquisition logs are in the fleet repository’s `data/fleet-tracking`.

The simulation renders separate, fixed cameras from the seven-camera November 3
calibration used by the November 10 room twin. It uses the saved floor transform,
the archived fitted Spot XY/heading as a placement assumption, the RGB intrinsics,
native image aspect ratios, lens distortion and display orientations. The
renderer models full-field RGB; a physical feed may advertise a cropped ROI.
The simulated floor and Spot are the walking scene. The FOVs do not imply current
Spot localization, synchronized exposures, reconstructed room clutter or verified
absolute accuracy. At the archived spawn the 5 MP FLIR cuts off part of Spot.

To prepare another checkout, install the project's simulation/visualization
groups with UV, then import the geometry from the FLIR owner's saved datasets:

```bash
uv sync --locked --inexact --group simulation --group visualization
uv run --no-sync python tools/room_array.py --fleet-repo /home/adam/repos/flir-apriltag
```

This writes a new `local/room-cameras/array.json` with hashes of the source
calibration, floor, archived Spot pose and display settings. Each simulated run
snapshots it. `RELIC_ROOM_ARRAY` can point to another exported snapshot.

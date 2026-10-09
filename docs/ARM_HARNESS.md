# Arm supported by the trial harness

Adam reported that the harness pushes the arm slightly and explicitly requested
that arm motion not stop the ReLIC leg controller. Select this exception in the
reviewed envelope; the default remains full motion enforcement:

```json
{
  "arm_motion_guard": "observe",
  "arm_motion_reason": "Operator reports harness pushes the arm; arm motion must not stop the leg policy"
}
```

The reason is required, and only the ReLIC adapters accept this mode. It is bound
by envelope/source hashes and must be selected in a new candidate bundle and
trial review. It is not inferred from the arm configuration or a failed run.

Arm measured position, velocity, distance from the held stow target, measured
tracking error and target-to-measured tracking error become observational. The
handover speed check covers the twelve leg joints. The SDK's raw stow flag remains
in the snapshot; preflight explicitly records `arm_harness_observation` instead of
claiming a displaced arm is stowed. Policy observations retain actual arm angles
and velocities, and the seven commanded arm targets remain held at their registered
values with unchanged gains. This mode does not free, stow or otherwise move the
arm through a separate command.

All leg motion/tracking checks, body attitude/speed, timestamps, finite-state
validation, inference/ACK/expiry deadlines, battery, faults and tablet stop state
remain enforced. Measured and predicted arm torque limits, coupled arm-motor
limits, command position/slew bounds and the SDK shared velocity backstop also
remain active. Thus this exception does not accept arbitrary torque or change the
manufacturer's robot protections.

`arm_motion_mode` records the selection and operator reason. Changes in arm-motion
violations produce `arm_motion_observation` events with the measured seven-joint
state and arm-local indices (0--6). They do not terminate the policy. Ordinary
command/state records retain full readings. A subsequent producer guard failure
now records the rejected state in `guard_failure`, avoiding ambiguity about which
joint crossed a boundary.

Tests reproduce a narrow arm-position boundary, displacement, velocity and
tracking violations through supported hold and direct handover, verify the
default mode still stops, and verify leg/load/command protections stay active.
Run the standard suite and bounded 60-second simulation and host timing checks
before the user-authorized physical repeat. No automatic retries are added.

The first supervised repeat completed ten seconds, 500 predictions and 2,003
commands, including observed shoulder-pitch boundary crossings. Arm targets
remained held; native shutdown and a separate postcheck confirmed motors OFF.
See the [trial record](../records/relic-standing-harness-20261009/README.md) for
validation, timings, measurements and limits of this result.

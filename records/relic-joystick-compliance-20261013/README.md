# Arm tracking observation for compliant manipulation

## Question and change

The prior extended joystick session stopped after229s when elbow EL0 tracking
error exceeded0.15rad for502ms. Adam requested that arm tracking not stop compliant
manipulation, and requested a single command to start the same controller himself.

Add optional `arm_tracking_guard=observe` with a required operator reason, only
for the joystick-arm task. The three arm tracking checks (current reference,
previous command and proposed command) become observations for all seven arm/
gripper entries. Leg tracking remains enforced. The mobile arm still requires
`arm_motion_guard=enforce`; measured position/speed, measured/requested/coupled
torque, command bounds/slew, body/timing/tablet guards remain enforced. Weights,
gains, arm path, history, 50/200Hz scheduling and default tracking enforcement
are unchanged. Recorded crossing-mask transitions include measured/reference
values; compact recordings retain tracking mode, crossings and joystick inputs.

## Validation protocol

Unit regression holds0.16rad elbow offset for600ms: enforced mode stops, the
explicit observation mode continues and reports all three tracking crossings.
Separate cases verify arm position/speed/load/predicted-PD torque and leg tracking
still stop. The initial new-test failure used a simulation fixture with2rad generic
tracking bounds; corrected the test's threshold to0.15rad rather than changing
guard behavior. Targeted91tests and Ruff passed before full validation.

Run the required bounded60s MuJoCo sequence from public standing capture with
seed101, zero initial velocity, default actuator saturation and10noslip iterations.
Use a simulation-scoped copy of the prior envelope with only explicit arm tracking
observation enabled, the same compact arm and deterministic three-axis/deploy/stow
script. Record raw results, configuration and code commit. Simulation and history
are commissioning evidence, not physical robustness qualification.

## Operator command

`tools/run_joystick_bundle.py` displays the bound profile by default. Human
`--execute` attests current named operators/tablet/rig/arm and shutdown clearance,
performs fresh preflight, runs one session, verifies motors off and saves a compact
recording. Every invocation has a unique output directory; no retry. The local
prepared launcher supplies this Spot's ignored frozen bundle. No physical motion
is executed while preparing or validating the command.

## Validation result

Full source `b9cb3c2` passed540tests, Ruff and the Exploy demo. The guarded60s
MuJoCo sequence completed without a fall proxy and passed stable final standing.
Observation/raw-action/target parity maxima were5.96e-8/9.54e-7/2.38e-7rad.
Compute p99/max0.656/0.879ms. Default-only launcher display also passed without
robot/USB access. Full logs and data remain under
`runs/relic-joystick-compliance-validation-20261013-001`; see artifact hashes and
`simulation-result.json`. Subsequent documentation-only commit records results;
controller/launcher source unchanged. No physical trial was performed for this
change. Fresh operator-start commissioning remains separate from qualification.

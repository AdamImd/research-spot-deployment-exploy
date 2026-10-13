# Operator-selected firmware leg-position authority

After the right-front knee crossed the conservative measured operating bound
-2.10rad at -2.125rad, Adam requested relying on the robot firmware for joint
positions, another trial without tests, and using the existing policy without
retraining. This change applies to host measured leg-angle operating checks only.

An envelope can explicitly select `leg_position_guard: observe` with a nonempty
`leg_position_reason`. Default `enforce` retains existing behavior. Observation
requires a ReLIC adapter, manufacturer torque profile and separate finite virtual
PD target bounds. Existing measured bounds stay in the configuration as named
references; violations are recorded in `leg_position_observation` when the set of
out-of-bound leg joints changes. All actual positions remain recorded at command
rate. `leg_position_mode` records the operator reason and retained torque/command
protections. Compact recording preserves these events.

The [SDK5.0.1 documentation](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/docs/concepts/joint_control/README.md)
states desired positions and loads are saturated to robot kinematic/load limits.
That describes command saturation, not a guarantee about measured overshoot,
balance or contact dynamics. No firmware setting is changed and no measured
sensor value or network action is clipped by this host change.

Keep virtual targets knees[-3,0]rad and hips[-3,3]rad, PD-offset cap2rad, all
manufacturer measured/predicted/coupled torque limits, doubled speed/slew caps,
body-attitude/travel/timing/ACK/expiry guards, held arm and tablet authority.
Measured knees outside the manufacturer torque table still stop torque evaluation;
unknown torque capability is never extrapolated. This change is not a bypass of
all hardware safety or approval for manipulation.

The policy graph, trained weights, gains, action scale, initialzero/ACK history,
current-poststand travel frame, directstartup and 50Hz/200Hz scheduling are
unchanged. There is no online reward learning or extension penalty; adding a
training penalty would require retraining, which the operator explicitly declined.

No offline tests/simulations/replays/benchmarks are run for this change at Adam's
request. Prior pass/failure records retain their original source/envelope hashes;
they do not qualify this new guard branch. Before an operator-approved one-attempt
activation, bind the new source/graph/envelope, retain the lack of new experimental
qualification, check current motors-off/health/tablet state and obtain Adam's
requested exact final confirmation with distinct Adam/Minghao and a following
rig/clear path. Preserve failures and recordings; no automatic retries.

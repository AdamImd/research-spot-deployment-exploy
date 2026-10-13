# Virtual PD targets and measured joint-angle bounds

The ReLIC target is nominal joint angle plus 0.2 times the raw network action.
Target minus measured angle contributes to torque through Kp. It is not itself
a measurement of the physical joint. The first physical walking attempt stopped
at targets -2.174 rad (front-left knee) and -0.747 rad (front-right knee) outside
the shared trial bounds [-2.10,-0.80], while measured angles stayed in range.
See the [recorded trial](../records/relic-walk030-poc-20261012/README.md).

An envelope can now separate these quantities:

- `position_min`, `position_max`: measured joint-angle limits, still required.
- `command_position_min`, `command_position_max`: optional paired finite
  19-element virtual target bounds, in the same canonical SDK order.
- Omit or set both command fields to null to preserve legacy shared bounds.
  Supplying only one, an unordered range or a nonfinite/wrong-length vector fails.

`Guard.check_state` checks actual angles against measured limits.
`Guard.command` checks targets against the command limits, then retains PD-offset
tracking, predicted PD-plus-feedforward torque, manufacturer measured-position
knee/coupled-arm torque constraints, held arm, target slew and timing checks.
Broader target ranges do not alter or clip the policy, scale, gains or measured
angle limits. New source/configuration hashes require new reviewed evidence.

Adam requested substantially broader targets and selected preparing 2.0 rad
target-minus-measured caps for the leg joints. The proposed configuration is:

| Quantity | Proposed bounds |
|---|---|
| Leg knee virtual target | [-3.0,0.0] rad |
| Leg HX/HY virtual target | [-3.0,3.0] rad |
| Leg absolute target-minus-measured offset | 2.0 rad |
| Measured angle ranges | unchanged |
| Arm targets/ranges and held-arm contract | unchanged |
| Manufacturer torque, speeds/body/timing/stop controls | unchanged |

These are virtual command ranges, not approved mechanical travel. The knee
torque table remains evaluated at the actual measured angle and never
extrapolated to a virtual target. The supplied current profile's measured knee
range stays [-2.10,-0.80] rad; the manufacturer table covers a different interval
and must not be interpreted as permission to widen measured limits implicitly.
The [SDK 5.0.1 Joint Control documentation](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/docs/concepts/joint_control/README.md)
also states that firmware saturates desired positions and loads to kinematic/load
limits. Wider host targets cannot override that firmware behavior; raw host PD
torque estimates are not independent applied-torque measurements or a guarantee
of exact firmware response outside its kinematic limits.

Validate the actual recorded failed prediction and full guarded normal-core
simulations before a new trial. Preserve new measured-angle/torque/tracking stops;
do not call accepting this old prediction a successful physical rollout.
New command events include `guard_monotonic_s` (computation start) separately from
`monotonic_s` (finish), both preserved by compact recording. Historical recordings
without the former cannot prove an exact rejected-step slew interval; the audit
reports that limitation rather than mixing start and finish clocks.
The operator requires a separate exact approval immediately before another
physical attempt; the prior one-attempt authorization has been used.

The [fresh validation record](../records/relic-wide-pd-targets-20261012/README.md)
reports both guarded 20-second walks and 60-second standing tests passing, with
the walks still about 6 cm short. Its host benchmark missed the selected 5 ms
computation deadline once (5.272 ms) and two nominal releases. The proposed bounds
are prepared; the timing gate has not passed and no second physical trial ran.

Adam subsequently selected a **10 ms inference/core computation limit**, retaining
the 200 Hz schedule and accepting the observed nominal misses. The
[revised trace review](../records/relic-computation-10ms-20261012/README.md) passes
that selected limit without rerunning or erasing the old 5 ms result. Command-gap
and ACK limits remain 15 ms; expiry remains 30 ms. The subsequently confirmed one
physical attempt moved about 0.18 m before front-left HY measured speed 6.042 rad/s
exceeded its 6.0 rad/s cap. Motor shutdown and lease return were confirmed.
No retry or goal-success claim; that single-use approval is consumed.

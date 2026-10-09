# First standing trial: hardware-limit draft

**Status: unreviewed proposal, not approved for activation.** This review uses the
operator's October 9 answers, firmware-matched manufacturer references, the
motors-off model capture, and two existing simulated rollouts. No motor command
was sent for this review. The released weights, gains and runtime guards are unchanged.

The editable numerical proposal is
[proposal.json](../records/hardware-limit-review-20261009/proposal.json).
Its format deliberately cannot load as a hardware `Envelope`. Do not rename it
to `envelope.json`. The [computed audit](../records/hardware-limit-review-20261009/summary.json)
contains all 19 model limits, joint demand, hashes and comparison failures.

## Trial assumptions confirmed by Adam

- All four feet support Spot; the rig is slack fall arrest, becoming taut after
  approximately **20 cm** downward travel.
- Clearance, including harness stretch, was reported checked. The minimum
  remaining body/arm-to-floor/frame gap has not yet been supplied.
- Spot's stowed arm is the only payload; no known joint or repair restrictions.
- **10 seconds of active ReLIC**, after native standing and initialization.
  Direct startup, no fade-in, zero initial raw actions, zero base-velocity command.
- Manufacturer tablet is the stop authority. A separate operator will hold it.
  Stop functionality and rig inspection still need their existing evidence records.

The 20 cm rig slack is not an implemented software drop limit or a stopping-distance
guarantee. Both previous simulations dropped about 8–8.5 cm at startup. The
approximately 11.5 cm difference is geometric slack remaining, not a demonstrated
dynamic arrest margin. The current streaming guard checks orientation and velocity,
but has no body-height/drop field. A height-based abort would require implementation
and validation; this draft does not claim one exists.

## Manufacturer reference versus proposed operating limits

Boston Dynamics' [SDK 5.0.1 transmission data](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/docs/concepts/joint_control/supplemental_data.md)
gives motor torques and reductions. Ideal joint-side products are shown below;
these are not continuous-duty ratings or approved trial caps.

| Joint family | Reference maximum (N m) | Draft load cap (N m) |
| --- | ---: | ---: |
| Hip abduction / flexion | 44.88 each | 30 / 25 |
| Knee | Position dependent | 60, within proposed position range only |
| Arm SH0 | 89.89 | 8 |
| Arm SH1 / EL0 | Coupled, see below | 15 each |
| Arm EL1 / WR0 / WR1 | 23.23 each | 5 each |
| Gripper F1X | 11.31 | 2 |

From the published coupling Jacobian, virtual work gives
`abs(tau_el0) <= 89.89` and `abs(tau_sh1 - tau_el0) <= 89.89` N m.
The proposed 15 N m individual caps imply at most 30 N m for their difference;
independent manufacturer-scale caps would not generally capture the coupling.

The [5.0.1 knee table](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/docs/concepts/joint_control/knee_torque_limits.md)
reaches about 113.24 N m in mid-range but only 37.17 N m at -2.7929 rad and
30.60 N m at -0.2471 rad. A fixed 60 N m cap therefore cannot cover the full
mechanical range. This draft restricts knees to [-2.1, -0.8] rad, where adjacent
tabulated values remain above the proposed cap. This is a screening calculation,
not a thermal or actuator qualification.

The captured live URDF supplies per-joint position limits. Its effort and velocity
entries are uniformly **1000**, so they are not used as physical ratings. ReLIC's
simulator arm effort limits also differ from these manufacturer-derived values.
No verified physical maximum-speed or continuous-load specification was obtained.

## Joint proposal and observed demand

All angles are radians; speeds are rad/s. These discussion values leave margins
over recorded standing demand while keeping loads below the reference motor-side
conversions. That choice does not establish a safe operating envelope on this robot.
Limits abort rather than clip or modify the policy output.

| Joint group | Proposed position range | Speed cap | Load cap (N m) | Tracking cap | Target-rate cap |
| --- | --- | ---: | ---: | ---: | ---: |
| All HX | [-0.40, 0.40] | 4 | 30 | 0.50 | 100 |
| All HY | [0.50, 1.45] | 4 | 25 | 0.40 | 100 |
| All KN | [-2.10, -0.80] | 8 | 60 | 1.00 | 100 |
| SH0 | Fixed stow ±0.05, inside model limits | 1 | 8 | 0.05 | 0.5 |
| SH1 / EL0 | Fixed stow ±0.05, inside model limits | 1 | 15 | 0.05 | 0.5 |
| EL1 / WR0 / WR1 | Fixed stow ±0.05, inside model limits | 0.5 | 5 | 0.05 | 0.5 |
| F1X | Fixed stow ±0.05, inside model limits | 1 | 2 | 0.05 | 0.5 |

Arm targets remain exactly the manifest stow values; the ranges are monitoring
bounds, not an arm-motion command. The JSON expands these groups in SDK joint order.

For unchanged PD gains, a knee tracking gap around 0.816 rad can occur while actual
movement remains much smaller. Both the new target and preceding target are included
in this audit's tracking maximum. Load and measured-speed checks remain separate.

The first hold-to-policy target jump is **73.50 rad/s** when divided by the 5 ms
command period. Later target changes peak at about 17.90 rad/s. The 100 rad/s draft
target-rate bound includes that direct-start discontinuity; it is **not** permission
for a physical joint to rotate at 100 rad/s. A 25 rad/s target bound would reject
the requested direct startup. Acceptance of that discontinuity remains a review decision.

Proposed handover torque-step caps are 12 N m for each leg joint, then
`[5, 5, 12, 3, 3, 3, 1]` N m for SH0 through F1X. The current manifest instead has
200 N m everywhere, inherited from simulation. The observed largest leg step is
8.70 N m; the elbow step is **10.96 N m** when supported-hold feedforward is removed.
Reviewing only policy-phase arm torques would miss that step. Changing these caps
requires a new manifest/candidate binding and renewed evidence.

## What the archived traces pass and fail

These are sampled offline comparisons, not new simulation runs or hardware tests.
There are 2,001 recorded samples in each first-ten-second window, at 200 Hz.
The full 60-second traces were checked as well. Both ended in their registered
stable-standing window under the original broad **simulation** envelope.

| First 10 seconds | MuJoCo | Isaac Lab |
| --- | ---: | ---: |
| Maximum body drop from initialization | 8.497 cm | 8.011 cm |
| Maximum tilt | 2.650° | 3.089° |
| Maximum linear speed | 0.1655 m/s | 0.1632 m/s |
| Maximum angular speed | 0.1386 rad/s | 0.1695 rad/s |
| Largest knee requested PD torque | 49.001 N m | 49.626 N m |
| Maximum gripper speed | 0.626 rad/s | **9.501 rad/s** |
| Maximum gripper requested PD torque | 0.255 N m | **3.737 N m** |
| Sampled joint bounds in this proposal | **Fail** | **Fail** |

All sampled leg bounds pass. Specific unresolved failures and integration issues:

1. **Speed-threshold mapping.** The [Joint Control API](https://dev.bostondynamics.com/docs/concepts/joint_control/readme)
   has one shared velocity threshold: any joint exceeding it triggers a behavior
   fault. Our `sdk_control.command_proto` currently sends `min(velocity_max)`.
   This draft would therefore impose **0.5 rad/s on every joint**, despite allowing
   knees up to 8 rad/s on the host. A separately reviewed global SDK threshold and
   retained host per-joint guards need an explicit contract and regression tests.
   Simply deploying this heterogeneous draft would trigger unintended faults.
2. **Measured positions versus targets.** MuJoCo SH1 goes 0.00105 rad below -π,
   EL0 0.00609 rad above π, and F1X 0.000238 rad above zero. Isaac's SH1/EL0 also
   cross the model boundaries, by much smaller amounts. All commanded arm targets
   remain inside the model. The current envelope applies the same strict bounds
   to measured and commanded positions. Solver compliance, model accuracy and
   encoder tolerance need separate review; do not widen command bounds to hide it.
   Motors-off hardware capture also showed up to 0.00963 rad knee discrepancy
   below the live URDF limit, so this is not solely a simulator concern.
3. **Gripper transient.** Isaac exceeds the proposed 1 rad/s and 2 N m bounds at
   5 ms despite a constant target; MuJoCo does not. This audit verifies the order
   and PD arithmetic but does not establish the cause or hardware relevance.
   Investigate initialization/contact/constraint dynamics before increasing bounds.

The SDK computes PD plus feedforward torque from desired and measured state;
the host's `load_max` guards both the predicted total and reported load. This draft
also compares preceding applied simulation torque and supported-hold feedforward.
It is not a direct measurement of physical motor torque or stop performance.

## Body and timing proposals

| Field | Draft | Basis and remaining check |
| --- | ---: | --- |
| Absolute roll / pitch | 10° each | Observed tilt <3.1°; physical bound unreviewed |
| Linear / angular speed | 0.30 m/s / 0.50 rad/s | Above observed simulated transients |
| Arm deviation from stow | 0.05 rad | Includes recorded displacement; speed failure remains |
| Inference deadline | 5 ms | Archived policy inference max 1.59 ms |
| Maximum state age / gap | 30 / 15 ms | Motors-off state stream ~333 Hz; receive gap max 5.142 ms |
| Allowed future skew | 1 ms | Requires clock-sync validation, not inferred from RTT |
| Maximum policy age | 50 ms | Policy nominal period 20 ms |
| Maximum command gap / ACK delay | 15 / 15 ms | Fault thresholds, not permission to abandon nominal 5 ms cadence |
| Command TTL | 30 ms | Effective expiry also limited by sample timestamp + maximum state age |
| Full-state health age | 250 ms | Verify health poll scheduling and RPC latency |
| Shutdown timeout | 2 s | Cleanup bound, not a demonstrated physical stopping time |
| Battery floor | 50% | Conservative proposed trial-entry floor; unreviewed |
| Policy duration / transition | 10 s / 0 s | Operator-selected trial and direct startup |

These values satisfy the configuration inequalities: command gap < TTL <= policy
age, ACK < TTL, command/policy periods below their respective gaps/ages. They do
not prove transport performance. The prior host benchmark had one **5.920 ms**
response against its 5 ms nominal budget and one missed release. Physical command
ACKs and stop times are unmeasured; read-only RPCs cannot qualify them.

## Review outcome and next decisions

Keep the active bundle's approved envelope absent. Resolve the three concrete
integration/transient issues above; review the proposed per-joint, body, timing
and handover values; then prepare a new bound candidate and collect the existing
readiness evidence. Clearance's minimum gap remains an operator input. The draft
does not alter the tablet selection, activate motors, or authorize a rollout.

See [analysis protocol and reproduction](../records/hardware-limit-review-20261009/RESULTS.md)
and [deployment procedure](DEPLOYMENT.md).

# Selected manufacturer-based torque limits

On October 9, Adam selected **manufacturer-based limits** in place of the lower
draft torque caps and stated that the other parts had been tested and were
acceptable. [Operator confirmation](../records/manufacturer-limits-20261009/operator-review.json).
This supersedes the torque selection in the earlier hardware-limit draft.

The selected configuration is
[spot-manufacturer-standing-envelope.json](../configs/spot-manufacturer-standing-envelope.json).
It uses the existing ten-second, direct-start standing scope, unchanged policy
weights/gains, and the draft's non-torque operating bounds. Preparation remains
offline; selecting limits does not send robot commands.

## Torque constraints

The [SDK 5.0.1 motor/transmission data](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/docs/concepts/joint_control/supplemental_data.md)
is converted to joint coordinates. These reference maxima are not continuous-duty
or thermal ratings.

| Joint | Maximum magnitude, N m |
| --- | ---: |
| HX and HY | 44.88 |
| Knee | Interpolated at measured position, maximum 113.236015 |
| SH0 | 89.89 |
| SH1 | 179.78 bounding box, subject to coupling |
| EL0 | 89.89, subject to coupling |
| EL1, WR0, WR1 | 23.23 |
| F1X | 11.31 |

The shoulder/elbow transmission additionally requires
`abs(tau_sh1 - tau_el0) <= 89.89` N m. For example, SH1 = 160 and EL0 = 80
passes, while SH1 = 160 and EL0 = -80 fails although individual bounds pass.
This follows from applying the transpose of the published velocity Jacobian to
joint torque.

The [knee table](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/docs/concepts/joint_control/knee_torque_limits.md)
is evaluated by linear interpolation at **measured joint position**, not the
desired target. All 101 entries were checked against the copy already bundled
with ReLIC; source hashes are in [sources.json](../records/manufacturer-limits-20261009/sources.json).
Angles outside the table are rejected rather than extrapolated. Near full flexion
the available torque is about 37 N m, so using 113 N m everywhere would be incorrect.

`actuator_limit_profile: spot-sdk-5.0.1` enables these constraints in `Guard` for
measured loads, commanded feedforward and predicted total PD-plus-feedforward
loads. Per-joint `load_max` remains an additional bound. Violations stop command
production; the host does not clip the policy or change its gains. The profile is
source-hashed with the runtime and included in the envelope binding.

## Shared velocity threshold

`sdk_velocity_safety_limit: 8.0` explicitly sets the robot's shared backstop to the
largest selected host speed bound. The host still enforces HX/HY 4, knees 8,
SH0/SH1/EL0 1, EL1/WR0/WR1 0.5, and F1X 1 rad/s. These are retained operating
bounds, not manufacturer speed ratings. An arm exceeding its tighter bound still
stops the host even below the shared robot threshold.

An explicit shared threshold below any configured host speed cap is rejected.
Older envelopes that omit the field retain their existing `min(velocity_max)`
serialization; they are not silently reconfigured. The selected configuration
resolves the prior unintended 0.5 rad/s global cap.

## Validation and operational state

The earlier draft and its failed sampled comparisons remain historical records.
Raising torque caps does not change its measured-position boundaries or turn the
archived 9.5 rad/s gripper spike into a pass under the retained 1 rad/s bound.
Adam's confirmation of the other tests is recorded as operator-reported evidence;
we do not substitute fabricated per-test measurements or timestamps.

The torque-specific simulator regression uses
[spot-manufacturer-torque-simulation.json](../configs/spot-manufacturer-torque-simulation.json):
the original broad simulation monitoring bounds plus the new manufacturer torque
checks and table-covered knee range. It runs the established 60-second protocol
in each engine. This isolates the changed torque guard; it does **not** assert a
pass of the complete, tighter hardware envelope. The simulation file is explicitly
simulation-scoped and cannot authorize hardware.

See [results](../records/manufacturer-limits-20261009/RESULTS.md). Final deployment
still uses a fresh motors-off preflight, the final bound candidate and the existing
supervised activation path. No motor activation occurs during this update.

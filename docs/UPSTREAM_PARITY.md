# Joint order and critical runtime audit

## Scope and references

This is an offline comparison with the runtime Minghao linked in Slack, pinned to
[`spot_rl_inference` at `dee987ab`](https://github.com/SaxionMechatronics/spot_rl_inference/tree/dee987abc8c27e6542ba95fefa8e16390af5c1f9).
The linked training tree was inspected at
[`spot_rl_train` at `6bd031d8`](https://github.com/SaxionMechatronics/spot_rl_train/tree/6bd031d8f89aa99ad7a4b2050a792ddb40345f6d).
That training commit is **not** established as the origin of the bundled model.
The reference is armless handstand; it is not our standing candidate.

The audit uses independently generated synthetic SDK messages, seed `20261004`,
the reference's actual observation/command modules, and its actual ONNX model.
No robot adapter, demo entrypoint, simulator or hardware is run. Source modules and
the model remain ignored under `local/reference`; only hashes, test contracts and
results are versioned. The reference model SHA-256 is
`3277dbe235a4616cec4c102e7fefd3cde9f86c212d5c5679fb66113ecf3e7e7f`.

## Exact joint mapping

Both SDK arrays use leg-major order; the reference network uses joint-type-major
order. Our manifest must explicitly name the reference order in **both**
`observations.joint_order` and `actions.joint_order` when importing that convention.
The existing zero-output fixture uses SDK order and 48 observations; it must never
be copied as an assumed contract for a real policy.

| Index | SDK joint at this index | Reference policy joint at this index | SDK index read/written for this policy joint |
| --- | --- | --- | --- |
| 0 | fl_hx | fl_hx | 0 |
| 1 | fl_hy | fr_hx | 3 |
| 2 | fl_kn | hl_hx | 6 |
| 3 | fr_hx | hr_hx | 9 |
| 4 | fr_hy | fl_hy | 1 |
| 5 | fr_kn | fr_hy | 4 |
| 6 | hl_hx | hl_hy | 7 |
| 7 | hl_hy | hr_hy | 10 |
| 8 | hl_kn | fl_kn | 2 |
| 9 | hr_hx | fr_kn | 5 |
| 10 | hr_hy | hl_kn | 8 |
| 11 | hr_kn | hr_kn | 11 |

Thus `q_policy = q_sdk[[0,3,6,9,1,4,7,10,2,5,8,11]]`, followed by subtraction
of defaults in policy order. Conversely, after scale/offset,
`target_sdk = target_policy[[0,4,8,1,5,9,2,6,10,3,7,11]]`.
These are different permutations. No additional sign reversal is applied.

Our arm/gripper slots 12–18 are `arm0_sh0`, `arm0_sh1`, `arm0_el0`, `arm0_el1`,
`arm0_wr0`, `arm0_wr1`, `arm0_f1x`. They come from the stow contract, never the
12-action policy. SDK 5.1.1's `JointIndex` enum is tested directly. Audit arm values
are deliberately distinct synthetic sentinels, not a physical stow pose.

## Other critical contracts

| Item | Pinned reference | Our verified path / remaining distinction |
| --- | --- | --- |
| Input/output | float32 `[1,45]` `obs` → `[1,12]` `actions` | Exact named shapes; the audit config has 45 inputs |
| Observation layout | body linear 0:3, angular 3:6, gravity 6:9, relative q 9:21, dq 21:33, previous action 33:45 | Same sequence; no extra velocity command in this contract |
| Frame convention | inverse of body-to-odom quaternion; wxyz; gravity `[0,0,-1]` | Same conversion, including mixed-axis rotations and quaternion sign equivalence; accepted rounding drift is normalized |
| Defaults in policy order | `[.1,-.1,.1,-.1,.9,.9,1.1,1.1,-1.5,-1.5,-1.5,-1.5]` rad | Subtracted from q and added to targets by joint name |
| dq defaults | zero | Raw dq is equivalent for this reference; nonzero training defaults need explicit term offsets |
| Scaling/history | no observation scale/clip/normalization/history; raw last action | Identity transforms, history length 1, zero reset; previous action stays in network output order even if q observation order differs |
| Action conversion | `default + 0.2 * raw_action`; no clipping | Explicit null clip fields now preserve this exactly; existing bounded-clip manifests retain their behavior |
| Leg gains and feedforward | kp 60, kd 1.5, velocity/feedforward zero | Exact in the audit; real gains must come from the actual candidate's effective training setup |
| Policy timing | state event divider 6; nominal 333/6 ≈55.5 Hz | Training config says `.002 × 10 = .020 s` (50 Hz); our audit contract uses 50 Hz independently of command streaming |
| Command dimension | 12 leg values | 19 values; seven fixed arm/gripper targets and explicit gains |
| Startup | native stand, then ten-call ramp from 0.1 to 1.0 of the policy offset around defaults | Native stand, hold measured pose, require active feedback, then time-based blend from that pose; intentionally different and requires qualification |
| Command lifetime | acquisition +100 ms, 5 ms extrapolation | Reviewed envelope TTL capped by state freshness, zero extrapolation |
| Gains on wire | first command only | All 19 gains on every command |
| Stop path | lease/E-stop/expiry and power-off attempt | Additional freshness, health, acknowledgement and limit guards; bounded cancellation/power-off confirmation |

The state stream and command stream are distinct from inference scheduling. The
reference's event divider may coalesce updates, so 55.5 Hz is a nominal rate, not
a measured guarantee. Matching tensor values does not qualify either scheduler.

The inspected training code has a stiffness curriculum from 60 to 40 and a PLAY
override of 40; the bundled runtime JSON uses 60 and has no curriculum. It also
uses delayed hip actuators and remotized knee actuators in simulation. Runtime PD
gains alone do not reproduce those simulated torque/delay dynamics. Obtain the
effective gains, actuator settings and export metadata for the real checkpoint;
do not infer them from a repository default or silently substitute 40 or 60.

## Regression coverage and repairs

`tests/test_upstream_parity.py` covers distinct SDK state sentinels, all twelve
one-hot action channels, action magnitudes beyond ±1, separate observation/action
orders, raw previous action/reset, mixed-axis body frames, q/−q and quaternion
rounding drift. Tests compare actual SDK command arrays including the arm tail.

The audit found and repaired an inability to express *no clipping*. The manifest
now requires either explicit finite clip bounds or explicit nulls, with both
action bounds disabled together. Conversion still rejects non-finite values and
float32 observation overflow; physical envelope checks remain separate.

Startup tests now verify a nonzero measured pose is held until activation feedback
and then blended to the policy target. A stale `IS_STANDING` substatus is rejected
when the native command's parent status says it was overridden/interrupted.
The SDK descriptor audit also corrected activation feedback access to
`response.feedback.full_body_feedback` (SDK 5.1.1). Tests now construct actual
feedback protobufs for active, unknown, error and overridden states; fake
controller tests alone had not exposed the invalid former field name.

`tools/verify_upstream.py` additionally runs 24 signed basis cases through the
actual reference postprocessor and a 24-step sequence through both ONNX pipelines.
It records both observations/actions, full targets and reference ramped commands.
The ten-call reference ramp is checked separately rather than claiming our startup
behavior is identical. Absolute/relative tolerances are `2e-6` / `1e-6`.
See [current status](STATUS.md) for the final run and observed numerical errors.

Final run at `92a1845`: 24 basis and 24 sequential ONNX cases passed. Observation,
raw-action, full-scale-target and steady wire-target differences were zero. The
largest ramp/wire rounding difference was `1.0491e-7` rad; the complete package
suite passed 172 tests. Numerical results (historical archive; see [public validation](VALIDATION.md))
and raw comparisons (historical archive; see [public validation](VALIDATION.md)) preserve
both pipelines' outputs. The synthetic sequence produced raw magnitudes up to
10.94627, so the no-clipping distinction is material even for this bundled model.

## Reproduction

From the source commit recorded by the result:

```bash
uv sync --locked --all-groups
.venv/bin/python tools/verify_upstream.py --fetch --output runs/upstream-parity-new
.venv/bin/python tools/validate_package.py --output runs/parity-acceptance-new
```

Use fresh output paths. `--fetch` downloads only missing hash-pinned reference
files; modified files fail verification. Without it, the audit is offline and
requires the previously downloaded files. `spatialmath-python` is pinned in the
`audit` dependency group solely to execute the reference quaternion implementation.
The ordinary regression suite needs no external source, model or network.

This comparison cannot satisfy `shadow_parity` for an absent standing candidate.
Repeat with that checkpoint's original preprocessing/export contract and recorded
robot states, then qualify the arm/payload dynamics, effective gains, transition,
timing, E-stop and shutdown under the existing protocol. No hardware readiness is
inferred from these numerical tests.

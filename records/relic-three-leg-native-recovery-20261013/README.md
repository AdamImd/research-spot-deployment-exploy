# Native three-leg transition recovery: 47-second MuJoCo diagnostic

## Question, hypothesis and protocol

Does the unchanged, unfiltered ReLIC controller recover from its discrete
four-to-three-leg transition when actuator saturation is retained and host
operating stops no longer end the simulation? The hypothesis was that the
49.9N·m demand rejected in the previous guarded run is a recoverable transient.

One run, seed101, source `5f1346811757599e6ea40c93bd1608f95b559417`, clean at launch.
Existing locked UV/Python environment; CPU2, GPU0 EGL rendering only, no paid
resources, no retries or robot access. Exact stage commands, resources, timeouts
and measurement definitions are in execution-contract.json and the private
supervision record. Existing unfiltered47s profile: 5s four feet,6s lift,3s hold,
12s IK square,6s lower,2s return blend,12s final four feet plus1s margin. No new
entry blend, rate filter, gains, weights or history rule. Policy50Hz/control200Hz.
The sole dynamics change relative to the preceding guarded video is --no-guards.
Finite/integrity checks and released actuator/contact physics remain active.

Initialization uses the latest recorded hardware standing joints/orientation,
with simulated velocities and initial raw actions zero. The5s policy hold then
runs normally. Root placement0.44790276786489125m is a URDF median foot-ground
estimate; the original torso-height request is retained separately. This is not
a reconstruction of harness forces or an exact continuation of the physical run.
Input capture, graph and model hashes are retained with the run/configuration.

## Observed result

The full47s run completed without a fall proxy. It finished lift, hold, square,
lower and return; the existing geometric/IK sequence checks and final10s standing
check all passed. Runtime completion alone was not used as the success criterion.

| Measurement | Result |
|---|---:|
| Front-right HX peak requested torque |49.9213N·m|
| Front-right HX peak applied torque |45.0N·m|
| FR HX saturation duration |0.005s (one control interval)|
| FR knee torque-speed saturation duration |0.020s (four intervals)|
| Maximum absolute roll |3.069degrees|
| Maximum absolute pitch |4.350degrees|
| Maximum body angular speed |1.5372rad/s|
| IK square position error p95 / maximum |1.9006 /1.9088cm|
| Three-foot geometric hold/IK fraction |100%|
| Final four-foot geometric fraction |100%|
| Final10s maximum tilt |0.8863degrees|
| Final10s angular-speed p95 |0.0001255rad/s|
| Final10s body-height span |0.0463mm|

FR HX and FR knee are the two leg joints showing saturation above the registered
1e-5N·m difference threshold. Knee saturation is from the torque-speed curve:
reported knee speed reaches -15.794rad/s, with requested negative torque clipped
to zero at5.025/5.030s; further speed clipping occurs at5.035/5.060s. Total knee
saturation is20ms. This is retained actuator physics, not host target filtering. During the hold and square, FL normal force is below
the diagnostic5N loaded threshold throughout, while FR/HL/HR exceed it throughout.
These are actual simulated contact forces in addition to the geometric checks.
The simulator clips hip effort at45N·m; the current host hardware envelope uses
44.88N·m. The recorded measured-load crossing follows the clipped45N·m interval;
velocity at that state remains below its cap. This result does not verify actual
firmware saturation or change the physical envelope.

## Measurement alignment and artifacts

Requested torque at sample k is paired with preceding_applied_torque_Nm at sample
k+1, since the latter describes the intervening5ms physics step. Saturation duration
sums those interval lengths where the paired values differ by more than1e-5N·m.
Arm and leg vectors retain SDK ordering. Body roll/pitch derive from root quaternion;
angular rates are body-frame components and their norm. Guard logs record the first
occurrence per reason, not exhaustive counts. Full per-joint torque statistics,
sequence/parity/standing metrics and artifact hashes are in summary.json.

Private raw result/trajectory/contacts/timing:
`runs/relic-three-leg-native-recovery-20261013-001/rollout`.
Video: sibling `video/replay.mp4`,100fps,4701frames,47s, labelled UNGUARDED diagnostic.
Analysis: sibling `analysis/recovery.png`, `entry-torques.png` and PDF copies,
`aligned_metrics.npz` and `summary.json`. Native dynamics plots also retained.
Video at20s and full recovery plot were visually inspected. Raw data and private
capture stay ignored; only sanitized records and analysis source are published.

Reproduce analysis without new dynamics or robot IO:

```sh
uv run --no-sync python records/relic-three-leg-native-recovery-20261013/analyze.py \
  --run runs/relic-three-leg-native-recovery-20261013-001/rollout \
  --output NEW_ANALYSIS_DIRECTORY
```

## Interpretation and decision

This one simulation supports allowing the native transition to complete instead
of treating its brief unsaturated PD demand as evidence of failed balance. It
recovered without the global target limiter and completed the task. It does not
establish robustness across initial conditions or hardware equivalence. Keep
physical limits unchanged. Any physical transient allowance still requires a
separate concrete review and explicit approval immediately before execution.

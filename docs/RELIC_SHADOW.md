# Read-only ReLIC on live Spot state

This runner loads the released ReLIC ONNX checkpoint, observes the physical robot,
and logs predicted leg joint angles for the status board. It has no execution flag,
command client, lease acquisition, motor-power or E-stop-write operation. The
existing gated standing controller is unchanged. Stopped motors and an unarmed
E-stop do not prevent read-only inference.

## Rates and interpretation

- SDK state stream: approximately 333 Hz (measured previously at 3 ms intervals).
- ReLIC inference: 50 Hz, matching 5 ms simulation steps with decimation four.
- Prediction log: one sample per inference, with the newest streamed state.
- Browser data refresh: approximately 4 Hz, with retained 50 Hz history.
- Commands sent to the robot: zero.

ReLIC predicts 12 leg actions. Targets are the released defaults plus `0.2 × raw
action`, reordered into SDK joint order, without extra clipping. The seven arm
targets are held at the first measured arm pose as policy inputs and display
context; they are not learned outputs. This session requests zero base velocity,
four-foot support and level torso roll/pitch. By default the requested height is
latched from the initial SDK `body`/`gpe` transforms. It is the vertical distance
from the body origin to the estimated ground plane, independent of odometry's
arbitrary origin. The value stays fixed throughout the run and appears on the
board. Missing, stale (>250 ms), nonfinite or invalid ground data stops startup;
there is no guessed-height fallback. Use `--body-height 0.55` to reproduce the
earlier fixed-height sessions. The Python policy helper retains 0.55 m as its
explicit offline baseline default; the live CLI defaults to `--body-height initial`.

The [SDK frame definitions](https://dev.bostondynamics.com/docs/concepts/geometry_and_frames.html)
describe `gpe` as a ground-plane estimate. Height provenance, plane geometry and
measurement time are saved in `initial-height.json` and policy metadata. This is
an estimate, not a terrain/contact qualification. Previous raw actions still
initialize to zeros; changing the height does not seed them from measured angles.

The 84-input observation follows the previously audited simulation contract at
`research-spot-relic-sim` commit `47cde41d6251a818a6a887d37cead9b83d47fce6`, derived
from upstream ReLIC `27f8033c5064d32f049a17accb71cd1091422878`:
body velocity/angular velocity/gravity; commands; all 19 relative joint positions
and velocities in native articulation order; previous raw 12-action output.
Arm names map from `arm_*` to SDK `arm0_*`. SDK body-origin linear velocity is
shifted to the root rigid-body COM using `v_com = v_body + omega × r_com`, then
expressed in the body frame. The live URDF root COM must match `[0,0,-0.00496172]`
within 1 micrometre. This is the root body's COM, not whole-robot COM.

Model SHA-256: `039b542d7e833961b7602b4f933c10415589aa1ff1fa1a8295979f6a20c2f000`.
CPU ONNX Runtime runs with one intra-op and one inter-op thread. No simulator,
Torch runtime, GPU or dependency installation is needed.

The actual resting/stowed state is fed to the policy. No artificial standing pose
is substituted. Outputs can be far from the resting posture and are diagnostic;
they are not evidence of valid standing control. Previous-action observations
contain previous predictions even though those predictions were never applied.
The existing unresolved cross-simulator falls and hardware qualification remain.

The released checkpoint is a feedforward MLP (Gemm/Elu ONNX operations), with no
recurrent hidden state or stacked observation buffer. Its only temporal input is
the previous 12 raw actions in observation indices 72–83. A new policy instance
initializes these to float32 zeros, matching the simulation action manager's reset.
The first valid prediction replaces that vector; a skipped delivery tick leaves it
unchanged. Restarting the browser does not reset policy history; a new capture does.
Zero raw actions are not zero joint angles: target conversion adds the nominal
joint-position offsets. This reset convention does not make a physical resting
pose equivalent to the policy's training reset state.

## Run

From this repository with the credential environment already configured:

```bash
.venv/bin/python -m spot_deploy.relic_shadow \
  --robot local/robot.json \
  --checkpoint simulation/source/relic/relic/assets/spot/pretrained/policy.onnx \
  --duration 3600 --output runs/relic-shadow-new
# After capture startup, in a separate terminal:
.venv/bin/spot-view --run runs/relic-shadow-new --port 8765
```

Each output directory must be new. Duration is bounded to one hour. The runner
stops on RPC/health failure, stale state (>100 ms), duplicate/reordered received
timestamps, nonfinite output or inference exceeding 50 ms. If a prediction tick
finds the same still-fresh sample in the mailbox, it skips inference until the
next scheduled tick. It does not advance previous-action history on that sample.
The robot-clock and local-receive freshness limits remain 100 ms; a delivery pause
cannot keep old data alive. `shadow_wait` events and `skipped_state_ticks` record
these skipped ticks. Health is read asynchronously at
approximately 1 Hz so unary RPCs do not block the policy loop. It never retries,
changes robot state or falls back to a synthetic pose. Use durable supervision and
the repository executor procedure for operator sessions.

Records include the checkpoint hash, source and dependency identity, robot snapshot
and URDF, policy settings, every observation/action/target, health, inference/state
timing and final statistics. `shadow-status.json` reports current observed rates.
The viewer labels predictions separately from commands; sent-command fields stay
absent throughout a shadow run.

## Offline verification

To save one synchronized physical pose for an offline simulator comparison:

```bash
.venv/bin/python tools/capture_relic_pose.py \
  --robot local/robot.json \
  --checkpoint simulation/source/relic/relic/assets/spot/pretrained/policy.onnx \
  --output runs/relic-pose-capture-new
```

This captures joints, orientation, velocities and ground-relative height from one
fresh SDK message and saves a cold prediction with zero previous actions. It is
read-only and requires a new output directory. The paired simulation harness and
standing-capture result (historical archive; see [public validation](VALIDATION.md))
are maintained in the simulation repository. That result distinguishes initial
prediction agreement, PD-hold sag, large torque-producing target offsets, and
eventual stability; the MuJoCo 60-second standing gate remains failed.

```bash
.venv/bin/pytest -q
.venv/bin/python tools/verify_relic_shadow.py \
  --reference simulation \
  --output runs/relic-shadow-parity-new
```

The parity comparison runs 32 sequential synthetic states (seed 20261005) against
the existing simulator adapter and actual released checkpoint, checking all 84
observations, 12 raw actions and 19 targets with absolute tolerance `2e-5`.
Joint permutations, COM velocity correction, omitted clipping, stale-state stop,
metadata filtering and absence of control imports also have regression tests.

At clean source commit `46b5fc7`, **188 tests passed** in 3.09 seconds and Ruff
passed. The 32 sequential real-checkpoint comparisons returned maximum absolute
error **0** for observations, raw actions and targets. The complete hash-checked
parity record (historical archive; see [public validation](VALIDATION.md)) includes
source/model/reference provenance (historical archive; see [public validation](VALIDATION.md))
and all raw comparisons (historical archive; see [public validation](VALIDATION.md)).

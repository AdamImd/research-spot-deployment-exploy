# Faster ReLIC requests — 2026-10-12

## Question, hypothesis and decision

Adam asked whether the simulated robots could move faster. Test a forward request
maximum of 0.30 m/s instead of the frozen walking plan's 0.1325 m/s. Hypothesis:
a larger request elicits faster locomotion, but retaining the near-goal taper can
still leave the robot stationary short of the goal. Compare the same maximum with
and without that taper in MuJoCo and Isaac Lab. This task is simulation-only.

Observed: both engines respond with faster locomotion and remain upright for the
full 60 s. Removing the taper reaches the goal region but final positions overshoot
the 0.75 m target by 5.79–6.72 cm. Neither final position satisfies the 2 cm goal
tolerance. Retaining the taper leaves both stationary near 0.69 m. The result
supports a larger simulated speed request; accurate stopping remains unresolved.
No claim about physical deployment or universally stable speeds follows from one
seed, one initial pose and these unguarded diagnostics.

## Registered protocol and pinned state

- Implementation tested: `e6b0c1650743b60121c15d87a76a85deb4bebf3f`, clean at launch.
  Earlier attempt: `1429ed56db767275a2b0f30b371225e1944d8765` (failure below).
- Four paired cells: MuJoCo/Isaac Lab × taper/constant. Seed 101 in all cells.
  Same captured standing joints/root orientation/body height (0.523164 m),
  zero initial simulated velocity, direct startup, zero previous actions, ideal
  acknowledgements and 100 discarded inference prewarm calls.
- Frozen policy SHA256:
  `1940df208bf9e30e6134eeb71d1c4e67d3a0d66778726507d4886c62eae662e8`.
  Same weights, joint order, observation transforms, held arm and action scale 0.2.
  Leg Kp 60/Kd 1.5; feedforward 0. Policy 50 Hz, command/physics 200 Hz.
- Three-second initial hold, acceleration0.1 m/s², distance0.75 m,
  tolerance0.02 m, final hold1 s. Taper uses `min(0.3, 0.8*remaining)`.
  Constant requests 0.3 until the existing goal logic zeros the request, subject
  to the same initial hold and acceleration ramp. It does not change joint gains.
- `--no-guards`: operating/handover/deadline/walking-monitor stops disabled.
  Finite/shape/parity checks, history acknowledgement, held-arm contract,
  physical torque saturation and joint constraints retained. A fall proxy
  (height<0.25 m or tilt>0.9 rad) is recorded without ending the horizon.
- MuJoCo 5 ms physics with no-slip 10, corrected collision mesh names; Isaac Lab
  native 5 ms/TGS CPU physics. No attempt to make their contact solvers identical.
  See the [physics attribution record](../relic-physics-attribution-20261012/README.md).
- Frozen local inputs: `local/relic-walk-inputs-20261009-001/capture`,
  `local/relic-unguarded-walk-20261012-001/candidate/manifest.json`, and
  `local/relic-unguarded-walk-20261012-001/reference-simulation-envelope.json`.
  Capture completion hash, source hashes and packages are in each `source.json`;
  manifest/recording/completion hashes are in `result-summary.json`.
  Robot-specific raw inputs remain excluded from public Git.

## Results

All four cells recorded 3001 predictions and 12001 state samples over 60 s.
All ended with `reason=completed`, finite predictions, no fall proxy, and a stable
final 10 s window. This is diagnostic horizon completion, not accurate walking.
Measured speed is body-origin odometry velocity projected along the initial
heading; displacement uses the same latched frame. The median is restricted to
samples with requested forward speed ≥0.25 m/s, not the whole60 s.

| Engine/profile | Max request (m/s) | Measured median (m/s) | Measured peak (m/s) | First within goal (s) | Final travel (m) |
|---|---:|---:|---:|---:|---:|
| MuJoCo taper | 0.2945 | 0.2680 | 0.3316 | never | 0.68940 |
| Isaac taper | 0.3000 | 0.2653 | 0.3370 | never | 0.68784 |
| MuJoCo constant | 0.3000 | 0.2871 | 0.3451 | 7.13 | 0.81717 |
| Isaac constant | 0.3000 | 0.2864 | 0.3453 | 7.21 | 0.80787 |

The taper cells end requesting 0.0485/0.0497 m/s with negligible measured speed.
Increasing the maximum alone therefore does not eliminate this near-goal stall.
The prior 0.1325 m/s taper runs ended at 0.66956/0.67678 m (MuJoCo/Isaac).
Constant cells end with zero request. Isaac's internal `walking_completed=true`
latched at 8.445 s, followed by another motion transient around 12.7 s and drift
beyond tolerance. MuJoCo's internal flag remains false. The Isaac flag must not
be interpreted as final-distance success; the unguarded recording exposes drift
that the normal walking monitor would evaluate.

| Engine/profile | Peak leg speed (rad/s) | Peak target error (rad) | Peak requested PD torque (N·m) |
|---|---:|---:|---:|
| MuJoCo taper | 11.2436 | 0.82953 | 50.1126 |
| MuJoCo constant | 11.2436 | 0.83470 | 50.3008 |
| Isaac taper | 10.5080 | 0.83225 | 50.5073 |
| Isaac constant | 10.5080 | 0.87800 | 53.2629 |

All table demand maxima occur on front-right knee. Some hip/knee speeds exceed
the unchanged simulation reference envelope (hips 4, knees 8 rad/s); these are
observed demands, not newly approved limits. Full per-joint results preserve
requested PD torque and preceding applied torque separately. Independent native
policy parity max raw error≤1.1921e-6 and target error≤2.3842e-7 rad.

## Implementation and failure record

Two explicit simulation-only CLI options select the maximum and taper/constant
profile. The override requires `--no-guards`, a simulation envelope and walking
task. A separately bounded `SimulationVelocityPolicy` replaces only the named
velocity input, using the existing Exploy graph decoder. Regular live policy
input validation and frozen manifests remain unchanged. Tests exercise the real
graph and reject hardware scope, reverse/lateral/yaw and over-maximum requests.

First campaign `runs/relic-faster-sim-20261012-001` preserved all four integrity
stops at 4.34 s: `velocity command outside the explicit forward walking plan`.
The travel generator allowed a larger request, but the Exploy adapter still
enforced the original 0.1325 cap. Those are configuration failures, not faster
60 s gait tests. The primary repaired the simulation-specific input path, ran
validation, and explicitly authorized fresh campaign002; no automatic retry.

At clean implementation e6b0c16: **456 tests passed**, Ruff and offline demo pass.
All four corrected cells, two constant-profile replays, full video decode checks
and previews completed. Primary inspected both t6 s previews and comparison plot.
Replays render recorded state; they do not run new dynamics.

## Environment, execution and artifacts

Host `cs-u-rpm-dt-03`; local CPU physics/inference, OMP threads2, GPU0 only for
Isaac Kit/EGL visualization; no GPU1 or paid compute. UV locked Python3.12.3,
NumPy2.2.6, ONNX Runtime1.23.2, MuJoCo3.3.7. Native Isaac overlay Python3.11.15,
NumPy1.26.4, Isaac Lab0.54.3, Isaac Sim5.1.0.0, Torch2.7.0+cu128.
Native audit dependencies are pinned in `configs/native-physics-audit-requirements.lock`.

One Luna execution owner launched a primary-authored exact command-array
supervisor in tmux `relic-faster-sim-20261012-002`; campaign completed and exited.
Contract/supervisor are in `local/relic-faster-sim-20261012-002/`; copied contract,
source hash, command arrays, per-stage exits/logs, PID, STATUS and finish time in
`runs/relic-faster-sim-20261012-002/`. Maximum campaign 1200 s, bounded stages,
stop on error/integrity/nonfinite/timeout, no automatic recovery; cost $0.

Raw data under that run directory: `{mujoco,isaaclab}-{taper,constant}/`
with `configuration.json`, `source.json`, `events.jsonl`, `rollout.jsonl`,
`timing.jsonl`, `result.json` and integrity `COMPLETE.json`.
Videos/graphs: `{mujoco,isaaclab}-constant-replay/replay.mp4` and `dynamics.png/pdf`.
Comparison: `runs/relic-faster-sim-analysis-20261012-002/comparison.png/pdf`,
full summary and completion hashes. Public `result-summary.json` contains only
simulation metrics and input/artifact hashes, not raw robot identity or capture.

## Reproduction

At the pinned implementation with the recorded frozen local inputs/environment,
run each cell with a fresh output directory; never overwrite prior results:

```bash
env OMP_NUM_THREADS=2 uv run --no-sync python tools/simulate_relic.py \
  --simulation-source simulation \
  --capture local/relic-walk-inputs-20261009-001/capture \
  --manifest local/relic-unguarded-walk-20261012-001/candidate/manifest.json \
  --envelope local/relic-unguarded-walk-20261012-001/reference-simulation-envelope.json \
  --prewarm-inference 100 --no-guards --simulation-forward-speed 0.3 \
  --duration 60 --backend mujoco --simulation-speed-profile constant \
  --mujoco-noslip-iterations 10 --output runs/NEW-DIRECTORY
```

For taper change only `--simulation-speed-profile taper`. For Isaac use
`--backend isaaclab`, omit no-slip, use the native overlay interpreter, and set
`PYTHONPATH` to this repo's `src` and `simulation/source/relic`.
The exact environment/commands are also preserved in the campaign contract.
Recompute the figure/summary from the completed campaign without new physics:

```bash
uv run --no-sync python records/relic-faster-sim-20261012/analyze.py \
  --output runs/NEW-ANALYSIS-DIRECTORY
```

Raw captures are intentionally not public; reproducing this particular pose
requires the frozen local inputs. The public code permits another capture and
records its hashes. No hardware connection or activation occurred.

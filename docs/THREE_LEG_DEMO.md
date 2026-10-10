# Front-left lift, hold and return — simulation preparation

Adam requested a front-left leg lift, hold, then return to four feet. This tool
prepares and records that sequence offline; it cannot activate Spot. The raw
released actor, all PD gains and action scaling are unchanged. The existing
nine-input Exploy graph embeds zero leg commands and cannot execute this sequence.

## Native ReLIC semantics

ReLIC's 84 observations contain twelve leg-command values in leg-major order
(fl, fr, hl, hr; hx, hy, kn). Leave non-selected triples zero and put the selected
leg's requested absolute joint positions in its triple. Run the unchanged actor
and construct ordinary leg targets as default position + 0.2 * raw action; replace
only the selected leg's three targets with the requested pose. Hold the arm.
This mirrors the native SpotJointPositionAction process_actions implementation in
simulation/source/relic/relic/tasks/loco_manipulation/mdp/actions/spot_joint_actions.py.
Previous actions retain all twelve raw actor outputs, including the overridden
leg's outputs, and advance only after ideal simulated command acknowledgments.

The offline wrapper uses the baseline standing manifest for numeric guards and
initialization only. Its execution is explicitly labeled raw actor with native
selected-leg override; it is not a new Exploy export or a qualified live manifest.
Simulation requires an explicitly simulation-scoped envelope. All existing
position, speed, torque, body, target and history guards remain enforced.

## Sequence

configs/relic-three-leg-front-left.json records:

| Phase | Duration | Command |
| --- | ---: | --- |
| Four-foot initial standing | 5 s | Zero leg-command vector |
| Front-left lift | 4 s | Quintic path from measured fl pose to [0.12, 1.10, -1.90] rad |
| Three-leg hold | 5 s | Hold the lifted pose; other legs follow RL |
| Front-left lower | 4 s | Quintic path back to the latched standing fl pose |
| Four-foot final standing | 12 s | Release selected-leg override and restore zero leg commands |

The requested lifted pose is inside the current draft leg position bounds. This
is not approval of its torque, tracking, support geometry or physical execution.
Lift/lower interpolation applies only to the explicitly commanded front-left
pose; policy startup remains direct. Requested base velocity stays zero.

Measure body height/tilt, all joint velocities, target-minus-measured offsets,
requested PD torque, requested leg pose and foot clearances. A geometric hold
check requires fl clearance >=0.05 m and the other three foot clearances <=0.015 m
for at least 90% of hold samples. Foot clearance uses radius 0.036 m and is a
geometry proxy, not a foot-force/contact-load measurement. Record final four-feet
proximity and the ordinary ten-second stable standing window. A collected failed
run is not a successful demo.

## Run and render

Use UV's locked CPU environment with visualization dependencies:

```bash
uv sync --locked --group simulation --group visualization
uv run --no-sync python tools/simulate_relic.py \
  --simulation-source simulation --capture fixtures/standing-capture \
  --manifest policies/relic-exploy-standing/manifest.json \
  --envelope fixtures/relic-direct-simulation/envelope.json \
  --backend mujoco --mujoco-noslip-iterations 10 --duration 60 \
  --three-leg-plan configs/relic-three-leg-front-left.json --output runs/three-leg-001
MUJOCO_GL=egl uv run --no-sync python tools/render_relic_demo.py \
  --run runs/three-leg-001 --output runs/three-leg-replay-001 --fps 50
```

Use a frozen local capture and simulation copy of the reviewed hardware envelope
for commissioning comparisons; the fixture command above uses simulation limits
and does not qualify hardware. Existing isolated Isaac Lab can run the same
simulator command with --backend isaaclab --physics-device cpu. See SIMULATION.md.
Run one bounded process at a time with source/config/input hashes and seed 101;
preserve failures and do not widen guards to force a pass.

The renderer creates replay.mp4, dynamics.png and dynamics.pdf from measured
recorded states using visual meshes. It performs kinematics only and never
advances dynamics. Labels preserve a source guard stop or failure. COMPLETE.json
binds the source recording, rendered files, frame count and tool hash. Compact
recording includes demo_prediction and requested leg commands.

## Remaining physical preparation

The 2026-10-09 commissioning comparison stopped at mode activation in both
backends: MuJoCo body angular speed exceeded its bound at 5.02 s, and Isaac's
fr_hx target exceeded +0.4 rad at 5 s. No hold/return phase was reached. Native
parity passed; a smooth selected-leg path did not prevent supporting-leg target
jumps when the leg-command observation became nonzero. See
../records/relic-walk-three-leg-prep-20261009/README.md for metrics, provenance
and preserved failures. This is an implemented offline prototype, not a
successfully validated three-leg motion.

Keep this simulation-only until selected-leg Exploy inputs/output overrides are
explicitly exported, native parity passes through lift/return, SDK/live history
and limb/stop semantics are reviewed, support/contact measurements are available,
and manufacturer/hardware bounds and the supervised rig review cover the demo.
Do not send the nine-input standing graph a synthetic lifted-leg command or
activate the demo on Spot through the standing/walking CLI.

# ReLIC on Spot with Exploy

Research deployment and simulation tools for **four-foot standing with Spot's arm
attached and stowed**. The default branch is `main`.

Exploy exports native ReLIC observation construction, the released actor and all
19 joint position targets into one ONNX graph. The host retains named joint/frame
conversion, **50 Hz inference**, **200 Hz commands**, zero initial previous actions,
acknowledged action history, direct startup and the existing stop/qualification gates.
This version uses Exploy's exporter with a Python/Spot SDK host. The Exploy C++
controller is outside this implementation.

**Start here:** [agent guide](docs/AGENT_QUICKSTART.md), [Exploy graph and export](docs/EXPLOY.md),
[simulator instructions](docs/SIMULATION.md), [validation and limitations](docs/VALIDATION.md).

Explicitly requested forward travel uses the [distance-based walking protocol](docs/WALKING.md).
Standing remains the default; walking requires its own manifest, plan and evidence.
Prepare a simulated [front-left lift/hold/return demo](docs/THREE_LEG_DEMO.md)
with the original actor, unchanged gains and recorded visual-mesh replay.

For a supervised hardware trial, start with [deployment preparation](docs/DEPLOYMENT.md).
`prepare-deployment` creates a local candidate bundle and hardware-limit review template;
`deployment-check` reports unresolved inputs without contacting Spot.

Use the [lightweight rollout recorder](docs/RECORDING.md) to save compressed joint
state, actions, command targets and timing from a live or completed run.

The next supervised test is the [standalone joint-API hold/crouch diagnostic](docs/JOINT_API_CROUCH.md).
[Saved progress and resume instructions](docs/HANDOFF_JOINT_API.md) describe its draft settings and remaining checks.

## Fresh clone: offline first

Requires Git, [UV](https://docs.astral.sh/uv/), Linux and Python 3.12. UV can obtain
Python when it is missing. No robot, GPU, Isaac installation or private files are
needed for the following commands. A C++ compiler enables the firmware-latch test;
Node.js enables the browser kinematics test.

```bash
git clone https://github.com/AdamImd/research-spot-deployment-exploy.git
cd research-spot-deployment-exploy
uv sync --locked --group simulation
uv run --no-sync pytest -q
uv run --no-sync ruff check src tests tools
uv run --no-sync python tools/demo_exploy.py
uv run --no-sync spot-deploy inspect-policy \
  --manifest policies/relic-exploy-standing/manifest.json --output runs/inspect-001
```

The demo prints the 19 predicted joint angles from a bundled historical standing
pose with zero previous actions. It opens no connection and sends no commands.
The exported graph, original checkpoints, required robot meshes and simulator
adapters are included in the clone with their licenses. No Git LFS, private sibling
checkout or extra model download is required.

## Run the shared controller in MuJoCo

```bash
uv run --no-sync python tools/simulate_relic.py \
  --simulation-source simulation --capture fixtures/standing-capture \
  --manifest policies/relic-exploy-standing/manifest.json \
  --envelope fixtures/relic-direct-simulation/envelope.json \
  --backend mujoco --mujoco-noslip-iterations 10 --duration 60 \
  --output runs/mujoco-001
uv run --no-sync spot-view --run runs/mujoco-001 \
  --visual-urdf simulation/source/relic/relic/assets/spot/spot_with_arm.urdf
# Open http://127.0.0.1:8765
```

Each output directory must be new. [Isaac and export setup](docs/SIMULATION.md)
requires a separate compatible Isaac Lab environment. Use that environment only
for Isaac/export; the deployment runtime remains on the UV lock.

## What is verified

The original Exploy integration passed 303 software tests, 6,130 numerical parity
cases and 60-second Isaac Lab/MuJoCo standing trials. Those are recorded offline
results, not physical qualification. The CPU paced test had one **5.92 ms** response
against a **5 ms** budget and one missed release. Timing qualification remains open.
See [results, provenance and reproduction](docs/VALIDATION.md).

The included graph is an unqualified research candidate. Simulation envelopes
cannot authorize hardware, and this checkout contains no robot credentials,
endpoint profile, live E-stop configuration or current hardware evidence.
Walking, manipulation and three-leg hardware operation are outside this standing
specialization. [Operator runbook](docs/OPERATIONS.md).

## Repository map

| Path | Purpose |
| --- | --- |
| `src/spot_deploy/` | Controller, SDK adapters, qualification, read-only viewer and stop bridges |
| `policies/relic-exploy-standing/` | Hash-bound exported graph and provenance |
| `simulation/` | Bundled simulator adapters and pinned ReLIC source/assets |
| `fixtures/standing-capture/` | Historical pose for repeatable offline testing |
| `tests/`, `schemas/` | Runtime regression suite and configuration contracts |
| `tools/` | Export, import, parity, rollout, timing and capture commands |
| `docs/` | Agent setup, architecture, usage and hardware runbooks |
| `records/` | Public numerical summaries and source provenance |

Contributions use branches and pull requests; see [CONTRIBUTING.md](CONTRIBUTING.md).
ReLIC source, models and derived graphs retain the **RAI Institute Research License
(non-commercial research)**. See [third-party notices](THIRD_PARTY_NOTICES.md).

### Unguarded simulation diagnostic

`tools/simulate_relic.py --no-guards` explicitly disables software operating-envelope,
handover, inference-deadline and walking-monitor stops **in simulation only**.
Use a direct-start manifest and an envelope whose `scope` is `simulation`.
The fixed recording horizon continues after the fall proxy (body height below
0.25 m or tilt above 0.9 rad), without changing weights, gains, action history,
walking request law, simulated actuator saturation or model joint constraints.
Finite-data, artifact/parity, held-arm and ACK integrity checks remain active.
There is no such flag on the hardware CLI. The original hardware envelope is
unchanged. `result.json.demands` reports per-joint speed, tracking offset,
requested PD torque, preceding applied torque, target slew and joint/target ranges,
with peak times and reference-bound exceedances. Peaks before the first fall
are separate from all samples. These are observed demands, not safe hardware limits;
completion means the recording horizon finished, not that walking succeeded.

Example (private frozen standing capture and candidate required):

```bash
uv run --no-sync python tools/simulate_relic.py \
  --simulation-source simulation --capture /absolute/path/to/capture \
  --manifest /absolute/path/to/direct-walking/manifest.json \
  --envelope /absolute/path/to/simulation-envelope.json \
  --backend mujoco --duration 60 --no-guards --output runs/unguarded-diagnostic
```

The [2026-10-12 diagnostic record](records/relic-unguarded-walk-20261012/README.md)
compares guarded-prefix parity, two MuJoCo friction settings and Isaac Lab, with
observed demands and a clear separation between recording completion and gait success.

### Physics attribution diagnostics

The simulation runner accepts `--physics-diagnostics` to record model properties,
foot normal forces/velocities, and MuJoCo contact pairs/constraint torques. Isaac
normal forces are impulse averages over the preceding 5 ms; MuJoCo reports its
instantaneous solved contact force, so they are not pointwise equivalent.
`--audit-native-actuators` independently compares 400 Isaac/NumPy actuator cases
after an Isaac rollout. Defaults preserve existing physics and behavior.

Two explicit offline interventions require `--no-guards`: `--mujoco-substeps 4`
uses 1.25 ms physics while holding one PD torque/delay update per 5 ms control tick;
`--initial-height-offset 2` places the same joint pose two metres above the floor,
retaining the original requested torso height for a short no-floor-contact probe.
These are attribution experiments, not deployment settings. Normal policy and
command rates, original gains and physical saturation remain unchanged.

For an audit without repeating physics, use `tools/audit_relic_physics.py` with
`--simulation-source simulation --output runs/NEW-native-audit` in the existing
Isaac Python 3.11 overlay. Install its extra packages using UV and
`configs/native-physics-audit-requirements.lock` (NumPy remains 1.26.4, MuJoCo 3.3.7).
Auxiliary audit errors preserve the completed rollout's `result.json`; a missing
COMPLETE or present ERROR still indicates a failed stage, regardless of a Kit
process returning exit zero during shutdown.

MuJoCo conversion gives generated collision OBJs a `collision__` basename prefix.
The URDF importer otherwise aliases visual and collision meshes with matching
basenames (the wrist visual mesh extends about 9 cm beyond its intended collider).
Source assets and visual meshes stay untouched. A regression checks every compiled
collision mesh's transformed bounds against the referenced URDF collision vertices.

The [physics attribution record](records/relic-physics-attribution-20261012/README.md)
isolates ground contact, friction/integration and gripper self-contact, with an
independent actuator audit and measured impact of the repaired mesh alias.

### Faster requests in simulation

Use `tools/simulate_relic.py --no-guards --simulation-forward-speed 0.3` to override
only the simulation's forward request maximum. The original manifest and live
walking contract retain their limits. Default `--simulation-speed-profile taper`
keeps the distance-dependent slowdown; `constant` retains the acceleration ramp,
initial hold and immediate zero request inside the goal tolerance, while removing
the near-goal taper. The policy still predicts targets at 50 Hz; commands remain
200 Hz with unchanged weights/gains. Configuration and events identify the
separate override. Requested base speed is distinct from measured robot speed.

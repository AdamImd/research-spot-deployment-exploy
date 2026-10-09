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

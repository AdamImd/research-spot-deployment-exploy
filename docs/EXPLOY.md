# Exploy graph: use, contract and re-export

The bundled graph at `policies/relic-exploy-standing/policy.onnx` is immediately
usable on the locked CPU runtime. [Fresh-clone commands](AGENT_QUICKSTART.md).
It is a modified research export with the original trained weights unchanged.

## Contract

Exploy `05f9dc3b2e589abdae8de942e8c6749e7a123c88` exports native ReLIC observation
and action methods. The host supplies nine float32 batch-one tensors:

| Input | Width | Meaning |
| --- | ---: | --- |
| `base_quaternion` | 4 | Body-to-world quaternion, wxyz |
| `base_linear_velocity` | 3 | Root COM velocity in body frame |
| `base_angular_velocity` | 3 | Angular velocity in body frame |
| `joint_positions`, `joint_velocities` | 19 each | Export metadata's native named joint order |
| `arm_command` | 7 | Held stowed-arm joint positions |
| `torso_command` | 3 | Roll, pitch, captured body height |
| `velocity_command` | 3 | Zero base-velocity command in this standing host |
| `previous_actions` | 12 | Previous acknowledged raw actions; zero at activation |

Outputs are `obs` [1,84], `actions` [1,12] and
`output.joint_targets.robot.all.pos` [1,19]. The host maps names to SDK order,
converts SDK velocity at the body origin to root-COM velocity, holds the arm and
checks gains/metadata/finite outputs. See [architecture](ARCHITECTURE.md).

The graph is one **policy step**. The host owns 50 Hz inference and 200 Hz commands;
it does not run Exploy's automatic memory feedback or its internal decimation.
Manifest v3 binds the graph, configuration, original checkpoints and exporter
commit. Raw actor v2 and synthetic fixture v1 remain independently available.
A raw actor cannot be relabeled as a v3 graph.

## Re-export with a compatible Isaac Lab installation

Only re-export requires Exploy, Torch or Isaac. Use an existing, independently
installed Isaac Lab runtime with Python 3.11, Torch 2.7.0+cu128 and Isaac Lab 0.54.3.
Set `ISAAC_PYTHON` to that environment's interpreter. Its full simulator dependency
set is not part of the Python 3.12 runtime lock.

```bash
bash tools/setup_exploy_export.sh "$ISAAC_PYTHON"
PYTHONPATH="$PWD/src" .venv-exploy-export/bin/python tools/export_relic_exploy.py \
  --simulation-source simulation --output runs/export-001
```

The setup script installs a pinned no-dependency overlay in `.venv-exploy-export`
without replacing the shared lab environment. The exporter reconstructs the
released Linear/ELU network because nested TorchScript cannot be traced by Exploy,
strictly copies all weights and checks 32 seed-101 observations against both the
original TorchScript and ONNX before exporting. It runs native observation/action
methods from bundled ReLIC, with four-foot commands and implicit drives disabled.

`policy.onnx` is the full policy-step graph. `native.onnx` retains Exploy's combined
decimation graph for inspection. `configuration.json`, `graph-contract.json` and
`export.json` capture provenance. Graph bytes need not match across toolchains;
verify numerical parity and update the bound manifest through the importer.

After creating `local/relic-reference` with the import command in the agent guide:

```bash
uv run --no-sync python tools/import_exploy_relic.py \
  --reference-manifest local/relic-reference/manifest.json \
  --export runs/export-001 --output local/relic-exploy-new
uv run --no-sync python tools/audit_exploy_relic.py \
  --manifest local/relic-exploy-new/manifest.json \
  --reference-manifest local/relic-reference/manifest.json \
  --height-record fixtures/standing-capture/initial-height.json \
  --output runs/new-graph-parity-001
```

Add `--events runs/mujoco-001/events.jsonl` to include a recorded rollout. Tolerances:
observation 1e-5, raw action 1e-4, target 3e-5 rad. Verify zero history after priming
and repeat the shared-core simulations for a changed graph.

## Provenance and scope

ReLIC source pin: `27f8033c5064d32f049a17accb71cd1091422878`.
The bundled graph SHA-256 is
`1940df208bf9e30e6134eeb71d1c4e67d3a0d66778726507d4886c62eae662e8`.
It retains the original export configuration and hashes, including historical
build-machine paths inside that provenance file. Those paths are not opened at
runtime. The initial import's model/payload hashes are historical bindings and
do not qualify another physical robot. See [validation](VALIDATION.md) and
[third-party notices](../THIRD_PARTY_NOTICES.md).

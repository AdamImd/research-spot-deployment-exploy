# Handoff to Minghao and his coding agent

The public repository is self-contained for offline inference and MuJoCo. Start
with `main`; create a branch before editing. This is a modified research integration
of ReLIC with Exploy, with the Python Spot SDK host retained.

## First commands

Follow the clone/install block in [README](../README.md), then:

```bash
uv run --no-sync python tools/demo_exploy.py
uv run --no-sync python tools/import_relic.py \
  --simulation-source simulation --capture fixtures/standing-capture \
  --settings fixtures/relic-direct-simulation/settings.json \
  --output local/relic-reference
uv run --no-sync python tools/audit_exploy_relic.py \
  --manifest policies/relic-exploy-standing/manifest.json \
  --reference-manifest local/relic-reference/manifest.json \
  --height-record fixtures/standing-capture/initial-height.json \
  --output runs/parity-001
```

Expected: 19 finite target angles, 128 varied parity cases passing, zero action
history after discarded warmup, and no hardware access. The default output is
an offline prediction, not a command. A large position target error can be part of
PD control; inspect simulated dynamics instead of declaring safety from a drawing.

Run the 60-second MuJoCo command in README next. The rollout uses the production
core and independently compares each prediction to the original ReLIC adapter.
It produces `events.jsonl`, `rollout.jsonl`, `timing.jsonl`, `result.json` and a
hash-bearing `COMPLETE.json`. Completion alone is insufficient: check the recorded
reason and final-standing criterion. Failed runs retain their evidence.

## Code entry points

| Task | Start here |
| --- | --- |
| Graph I/O, metadata, named-joint/frame conversion | `src/spot_deploy/exploy_policy.py` |
| Manifest dispatch, common policy interface | `src/spot_deploy/relic_contract.py` |
| Zero history, acknowledgement, handover and scheduling | `src/spot_deploy/relic_rollout.py` |
| Hardware command generation | `src/spot_deploy/relic_control.py`, `sdk_control.py` |
| Native observation/action export | `tools/export_relic_exploy.py` |
| Simulator acceptance and per-step parity | `tools/simulate_relic.py` |
| Independent actor/observation implementation | `src/spot_deploy/relic_policy.py` |
| Viewer and mesh rendering | `src/spot_deploy/viewer.py`, `web/` |

## Current limits and useful next work

The graph is CPU-only at deployment. Export/Isaac require a separate Python 3.11
Isaac Lab environment; no re-export is needed to use the included graph. Exploy's
C++ host has not been ported. Automatic memory/decimation from Exploy is intentionally
replaced by explicit host scheduling and acknowledged action history.

The original host benchmark missed one 5 ms release deadline. Investigate scheduling
and command transport under a registered timing protocol before any hardware
promotion. Keep runtime and simulator throughput distinct. Public evidence has no
current robot identity, route, live-stop health or rig inspection; those must be
collected by the actual operators through the existing runbook.

## Suggested agent prompt

> Work in this repository on a new branch. Read AGENTS.md and docs/AGENT_QUICKSTART.md.
> Install the locked UV environment and run the offline tests, bundled Exploy demo,
> independent parity audit and the documented MuJoCo rollout. Report the exact
> commit, numerical results, timing limitations and any failures. Keep the policy
> weights, joint mapping, zero-initialized acknowledged action history and hardware
> gates intact. Do not connect to Spot. Propose and test changes in a pull request.

Push access requires accepting the GitHub collaborator invitation. Use the user's
normal GitHub authentication; never place tokens in this repository or agent prompts.

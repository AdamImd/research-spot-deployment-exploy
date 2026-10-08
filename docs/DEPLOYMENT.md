# Prepare an Exploy policy deployment

Preparation produces a local candidate bundle and an explicit list of unresolved
checks. It never contacts Spot, changes E-stop state, acquires a lease or moves
the robot. The existing `stand` command remains the physical activation interface.
Scope is four-foot standing, arm attached/stowed, zero base velocity, 50 Hz policy,
200 Hz commands, direct startup and zero initial previous actions.

## 1. Prepare the local bundle

Start from a clean committed checkout and the locked environment:

```bash
uv sync --locked --group simulation
uv run --no-sync spot-deploy prepare-deployment \
  --manifest policies/relic-exploy-standing/manifest.json \
  --robot local/robot.json --output local/deployment-001
```

Omit `--robot` if its profile is not available yet, then create `robot.json` inside
the bundle using `schemas/RobotConfig.json`. If hardware limits have already been
reviewed, supply `--envelope PATH`. Simulation envelopes are rejected. No numerical
hardware limits are guessed or copied from simulation.

| File | Purpose |
| --- | --- |
| `policy/` | Relocated candidate graph, source/config hashes, support model and license |
| `bundle.json` | Frozen policy-file hashes and runtime source/Python/package identity |
| `robot.json` | Optional operator-supplied profile; credentials remain environment-only |
| `envelope.review.json` | Editable review template; missing values intentionally null |
| `REVIEW.md` | Named-joint worksheet, units, gains and physical evidence checklist |
| `evidence.json` | Initially empty evidence index; no automatic pass entries |
| `commands.json` | Argument templates for preflight, shadow, readiness and gated standing |

Each output directory must be new. Candidate artifacts are immutable within a
prepared bundle; prepare a new bundle after changing source, runtime packages or
policy. Robot/envelope/evidence files are review inputs. Their final hashes are
bound by the normal readiness system. The initial `COMPLETE.json` records bundle
creation; changing review inputs changes that historical record, so subsequent
checks use `bundle.json` for immutable artifacts and readiness for review inputs.

## 2. Review the physical envelope

Fill every null in `envelope.review.json` and save the reviewed result as
`envelope.json`. Document the reviewer, source of limits and relevant model/rig
assumptions in a review artifact. The template lists all 19 joints in SDK order.
Its gains and transition duration describe the fixed policy contract; they do
not establish safe physical joint/load/tracking/slew limits.

Review state freshness, inference budget, command release/ACK timing, expiry and
shutdown as separate limits. The current graph predicts quickly, but a previous
host benchmark produced one 5.920 ms response and one missed 5 ms release. That
result has not been promoted into wired hardware timing evidence.

## 3. Check the bundle offline

```bash
uv run --no-sync spot-deploy deployment-check \
  --bundle local/deployment-001 --output runs/deployment-check-001
```

Read `deployment-check.md` and `result.json`. Exit status is nonzero while required
inputs, evidence, preflight or local physical-stop status remain unresolved. A
blocked check is expected while filling the review template. The check verifies
the relocated graph, discarded warmup/zero history, frozen files, clean runtime,
hardware envelope and hash-bound evidence. It sends no robot packets.

## 4. Read-only robot verification

With the operator-supplied profile and credential environment, run a separate
bounded preflight. It is usable before the envelope review is finished:

```bash
uv run --no-sync spot-deploy preflight \
  --manifest local/deployment-001/policy/manifest.json \
  --robot local/deployment-001/robot.json --output runs/preflight-001
```

Compare actual model/payload bindings with the candidate. A mismatch requires a
reviewed new candidate/configuration; do not simply rewrite hashes to bypass it.
Preserve a failed attempt and diagnose it before retrying. Once a reviewed envelope
is present, the shadow command in `commands.json` can collect bounded live state
and predicted angles without sending targets. Shadow history is previous prediction
history; it does not emulate robot command acknowledgements or measure their latency.

Once evidence is supplied, check a fresh preflight and the existing local bridge:

```bash
uv run --no-sync spot-deploy deployment-check --bundle local/deployment-001 \
  --preflight-run runs/preflight-001 --check-estop \
  --output runs/deployment-check-002
```

Preflight must have passed with this exact candidate/profile, unchanged artifacts
and age at most 300 seconds. `--max-preflight-age-s` may tighten that limit.
`--check-estop` only reads the configured local status file and validates its
freshness/profile/robot/boot binding. It does not register, allow, rearm or modify
any endpoint. Current robot endpoint health is checked again by physical `stand`.
Use the separate [joystick](JOYSTICK_ESTOP.md) or [ESP32](ESP32_ESTOP.md) procedure
when operators explicitly commission the physical stop input.

## 5. Final supervised activation

Run readiness on the final manifest, robot, envelope and evidence paths. Supply
reviewed artifacts for every gate using `schemas/EvidenceIndex.json`. Creating
this bundle or passing a simulator run never inserts passed hardware evidence.

The standing argument template intentionally omits `--execute`. Only the operators
in position complete it with distinct robot/E-stop operator names, reviewed trial
duration, a fresh output directory and explicit `--execute`. The normal standing
path then checks its gates again, performs fresh preflight, acquires the lease,
uses native stand, captures height, verifies the hold/activation and runs ReLIC.
It can power on and move Spot. No preparation/check tool invokes that path.

No five-second fade-in is inserted. Initial actions stay zero and subsequent raw
action history advances after acknowledgement. Failure stops command production
and invokes the existing bounded shutdown procedure; physical operators retain
the independent stop. There is no automatic restart or fault clearing.

See [OPERATIONS](OPERATIONS.md), [PROTOCOL](PROTOCOL.md), [VALIDATION](VALIDATION.md)
and the generated `REVIEW.md` for the remaining decisions and evidence.

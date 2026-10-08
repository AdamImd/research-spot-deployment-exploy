# Portable simulator rollout

The repository bundles `simulation/spot_relic_sim`, its model configuration and
the licensed ReLIC source/assets. No private sibling repository is required.
The main controller and tests use Python 3.12 from the locked UV environment.

## MuJoCo

Use the 60-second command in [README](../README.md). It runs on CPU without a
renderer or GPU. The fixed protocol uses seed 101, 0.005 s physics/commands,
0.02 s policy steps, captured standing pose 002, zero initial simulated velocity,
zero previous actions, zero requested base velocity and no interpolation.
Use explicit `--mujoco-noslip-iterations 10`; this is the previously validated
contact-drift setting, not an inferred hardware parameter.

A pass requires finite states/outputs, independent per-step numerical parity,
no controller guard stop and a passing final ten-second standing window.
The final window needs height >=0.30 m, tilt <10 degrees, height span <0.02 m,
tilt span <2 degrees, planar-speed p95 <0.05 m/s, angular-speed p95 <0.1 rad/s,
and joint-speed p95 <0.5 rad/s. See `simulation/spot_relic_sim/settling.py`.
A 60-second run is an offline standing check, not robust real-world qualification.

## Isaac Lab

Prepare `.venv-exploy-export` as described in [EXPLOY](EXPLOY.md) using a compatible
existing Isaac installation. From the repository root:

```bash
PYTHONPATH="$PWD/src:$PWD/simulation/source/relic" \
  .venv-exploy-export/bin/python tools/simulate_relic.py \
  --simulation-source simulation --capture fixtures/standing-capture \
  --manifest policies/relic-exploy-standing/manifest.json \
  --envelope fixtures/relic-direct-simulation/envelope.json \
  --backend isaaclab --physics-device cpu --duration 60 --output runs/isaac-001
```

Set `--physics-device cuda:0` only for a separately recorded GPU-physics comparison.
This does not change the bundled graph's CPU inference provider. Do not change
shared drivers or simulator packages to run a smoke test. The bundled upstream
compatibility changes are listed in `simulation/UPSTREAM.md`.

## Reproducibility

Use a committed source snapshot and fresh output directory. Record command,
commit/dirty state, manifest and asset hashes, dependency versions and seed. Run
one simulator at a time, cap each at 15 minutes, and stop on nonfinite state,
unrecoverable exception, parity failure, guard stop or timeout. Preserve the
failure. No automatic retry or parameter relaxation. Simulation acknowledgements
are ideal and actuator/contact correspondence to hardware remains unverified.

The public pose fixture has newly computed integrity records after removing
robot identity and operational metadata. Its numerical starting pose and height
are unchanged. No connection or current robot state is needed.

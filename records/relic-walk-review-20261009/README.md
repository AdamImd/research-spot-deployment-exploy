# ReLIC forward-walking review — 2026-10-09

Physical walking was not started. The separate distance-based walking path is
implemented; neither simulator completed 0.75 m within the existing joint guards.
The operator confirmed the rig can follow the full move and the path is clear.

## Protocol and provenance

Question: can the unchanged released ReLIC actor walk 0.75 m slowly and stop
based on robot odom/body poses? See ../../docs/WALKING.md for the bound plan.
Seed 101; 50 Hz policy / 200 Hz commands; direct startup; initial action history
zero, then ACK-based updates; held arm targets and all torque protections retained.
Initialize simulation from the last successful physical standing capture, height
0.523164 m, with simulated velocities zero. Weights, joint mapping, action scaling
and gains are unchanged: all leg Kp=60, Kd=1.5, zero feedforward.

The hardware-derived numeric envelope is unchanged except the planned duration
cap; simulation copies are explicitly scoped simulation. Hardware hip velocity
bounds remain 4 rad/s and knees 8 rad/s. Arm motion is observed under the previously
requested harness exception; this did not cause the walking stops.

Full suite: 434 passed at d2ebfa17268f0b94955d5aa3c590115c050f4377; Ruff/demo passed.
An earlier run found a legacy-recording compatibility bug, fixed before these tests.
Exact rejected-state logging was added at c55b718c06fa85e7372b3fdfed71160d9c2840b3.
No physical control commands, motor activation, lease acquisition or E-stop writes
were issued for this walking review. CPU policy/physics on cs-u-rpm-dt-03, existing
isolated Isaac environment; $0. Raw logs, environment/package records, commands,
configuration and hashes remain in the ignored run directories listed below.
Robot-specific inputs stay private; this summary is not a fully portable dataset.

## Observed results

| Backend / request | Outcome | Measured forward travel |
| --- | --- | ---: |
| MuJoCo no-slip10 / 0.10 m/s | 20 s distance timeout; settled nearly motionless | 0.2503 m |
| Isaac Lab / 0.10 m/s | 20 s distance timeout; settled nearly motionless | 0.2548 m |
| MuJoCo no-slip10 / 0.15 m/s | Joint velocity guard; fr_kn -8.6814 rad/s exceeds 8 | 0.2489 m |
| Isaac Lab / 0.15 m/s | Joint velocity guard; fl_hy -4.4728 rad/s exceeds 4 | 0.4750 m |

Nonzero commands are present in the observation and predictions agree with the
independent raw actor, with maximum action error below 1e-6. A paired five-cell
MuJoCo diagnostic found that no-slip0 crosses measured joint-position bounds even
with zero command; no-slip10 zero-command standing passes its 20 s stable window.
This shows contact sensitivity but does not explain away the Isaac walking failure.

Inference: increasing the timeout alone will not fix the 0.10 m/s stationary
attractor. Faster commands trigger stepping but exceed the existing velocity
bounds. The exact physical response remains unknown. Do not widen limits or
claim walking qualification from the completed recording markers: these runs
completed their data collection with failed controller outcomes.

## Saved runs and decision

- runs/relic-walk-validation-20261009-001: preserved failed recorder regression.
- runs/relic-walk-validation-20261009-002: tests/style/demo, failed walking simulation;
  real-time encoding benchmark correctly skipped after that gate failed.
- runs/relic-walk-diagnostics-20261009-001: wrapper path typo; no simulation began.
- runs/relic-walk-diagnostics-20261009-002: five paired CPU MuJoCo cells.
- runs/relic-walk-isaac-check-20261009-001: two Isaac cells and exact MuJoCo guard state.
- local/relic-walk-inputs-20261009-001: manifests, hardware/simulation envelopes,
  captured pose and operator request; local/relic-walk-075-20261009-001 is a
  prepared unqualified bundle with empty evidence.

No process is left owning a physical walking run. No automatic retry or guard
relaxation is authorized. Next: resolve low-speed gait behavior and velocity-bound
compatibility in simulation, then rerun host timing and fresh motors-off preflight
before reviewing one supervised walking attempt. result-summary.json contains
observed values, hashes, pinned commits and the plan for audit.

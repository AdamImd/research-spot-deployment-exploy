# Bounded ReLIC forward walking

Standing remains the default. `spot-deploy walk` is a separate physical command;
`stand` rejects walking manifests. No automatic retries or fault clearing.

## Protocol and parameters

The operator requested 0.75 m forward at a slow speed and confirmed that the
fall-arrest rig can follow the full travel and the floor/arm path is clear.
This is operator-reported clearance, not an independent inspection. Adam operates
Spot; Minghao holds the manufacturer tablet E-stop. No local E-stop is started.

`configs/relic-walk-075.json` supplies the required plan embedded in the manifest:

| Parameter | Value |
| --- | ---: |
| Forward distance | 0.75 m |
| Maximum requested forward speed | 0.10 m/s |
| Requested speed acceleration | 0.10 m/s² |
| Distance tolerance | 0.02 m |
| Maximum lateral drift | 0.10 m |
| Maximum heading drift | 0.15 rad |
| Maximum pose/joint timestamp difference | 20 ms |
| Maximum consecutive planar pose step | 0.025 m |
| Initial zero-velocity hold | 1 s |
| Final stationary hold | 1 s |
| Maximum measured speed during final hold | 0.03 m/s |
| Maximum policy-active duration | 20 s |
| Progress stall window | 5 s |
| Required progress per stall window | 0.005 m |

Native standing precedes policy activation. Capture initial height and the robot
odom/body pose, start previous raw actions at zero, and activate directly without
joint interpolation. Request `[vx, 0, 0]` at 50 Hz policy inference while emitting
joint targets at 200 Hz. Raw action history advances only after acknowledgment.
The graph, weights, gains, arm targets and joint-order mapping remain unchanged.
The exported graph metadata retains its original standing export provenance;
nonzero command support is verified separately, not relabeled as qualification.

## Distance and termination

Use robot `odom_tform_body` position and its kinematic acquisition timestamp.
Freeze the initial body yaw after native standing. For displacement `(dx, dy)`
in odom, compute forward `cos(yaw0)*dx + sin(yaw0)*dy` and lateral
`-sin(yaw0)*dx + cos(yaw0)*dy`. This horizontal starting-body frame excludes
body pitch/height from forward distance. Body x is forward; see the
[official frame reference](https://dev.bostondynamics.com/docs/concepts/geometry_and_frames.html).
This is robot odometry, with its drift limitations, not independent metrology.

The heading is sampled from the current post-stand state, not the orientation at
boot or the odometry frame's x-axis. `walk_origin` records the explicit
`goal_odom_position_m`: current position plus distance times
`[cos(current_yaw), sin(current_yaw), 0]`. It is a fixed goal for that attempt;
turning during travel does not move the goal. The zero-velocity hold precedes
forward travel, and the same captured heading is used to measure progress.

After the initial hold, request `min(0.10, 0.8 * remaining_distance)` m/s with
bounded acceleration. At the distance tolerance request zero immediately.
Require measured planar velocity below 0.03 m/s for one continuous second before
success. Continued motion resets that hold. No elapsed-time distance estimate or
reverse correction is used. Missing/stale/replayed/backwards pose timestamps,
odometry jumps, excessive drift, overshoot or timeout stop the trial. A forward
request of at least half the maximum speed (capped at 0.05 m/s) must produce
0.005 m of measured progress within five seconds; otherwise stop with a recorded
`walk_stall`. This detects a stationary policy before the total distance timeout.

Retain all numeric joint/load/body/timing guards. The existing explicit harness
arm-observation exception retains held arm targets and measured/predicted torque
checks. Existing fr_hx tracking is 0.75 rad; other tracking bounds remain unchanged.
Command expiry stays 30 ms; native shutdown allowance stays 10 s. Completion
initiates native safe-power-off and requires observed OFF plus lease return.
If shutdown is unconfirmed, Minghao uses the tablet immediately.

## Validation and activation

The 2026-10-09 preparation compared slower 0.025 m/s² request ramps. At
0.125 m/s MuJoCo stalled after about 0.25 m; at 0.14 m/s both MuJoCo and Isaac
exceeded the 8 rad/s knee-speed bound. The 0.15 m/s MuJoCo trial also exceeded
that bound. Native actor parity passed, so a slower request ramp alone did not
resolve the gait failure. No physical walking was started. Full provenance and
results are in ../records/relic-walk-three-leg-prep-20261009/README.md.

A walking manifest requires `task: walking` and a non-null `walking` plan; a
standing manifest cannot carry that plan. Fresh evidence binds the plan through
the manifest hash. All existing gates remain required, plus `walking_distance`
(nonzero policy parity, measured-distance logic, simulation/timing review) and
`walking_clearance` (rig travel and clear path). Standing evidence alone cannot
activate walking. `prepare-deployment` generates a walking command template.

Before one supervised attempt: run tests and independent graph/reference parity,
roll out in MuJoCo from a recorded successful standing pose with hardware-derived
numeric guards, verify measured-distance completion and the final stable window,
and benchmark the shared core/logger/SDK encoding on the host. These tests do not
qualify real transport or physical walking. Perform a fresh motors-off preflight
and reviewed source/config-bound evidence before issuing `walk --execute` with
both operator names and `--duration 20`. A simulation failure is preserved and
investigated; do not widen hardware limits to pass it.

`walk_origin`, 20 Hz `walk_progress`, per-policy velocity commands and timestamped
odom/body positions are included in raw and compact robot/action recordings.
Use `tools/record_rollout.py` after the run to create the gzip JSONL artifact.

## Offline onset diagnostics

`tools/analyze_relic_walk.py` separates measured body travel from world-frame
foot displacement and identifies motion before the first nonzero velocity
request. Foot clearance/displacement are geometric stepping proxies; neither
proves loaded support or independently validates robot odometry. The report
compares simulator joint speed with position-derived speed, and plots issued
targets, requested PD torque and the preceding applied torque. An exact rejected
state is shown separately from the last accepted physics tick.

```bash
uv sync --locked --group simulation --group visualization
uv run --no-sync python tools/analyze_relic_walk.py \
  --run runs/walking-001 --output runs/walking-onset-analysis-001
```

The tool verifies recording integrity before analysis and creates diagnostics.json,
walking-overview.png/pdf, walking-onset.png/pdf and hash-bound COMPLETE.json.
It cannot construct a robot client or change controller output. When comparing
initial zero-velocity hold duration, retain direct policy startup and zero raw
history: delaying the base-velocity request is distinct from fading joint targets.
Keep new manifests/protocols frozen separately; a diagnostic comparison does not
replace the physical walking plan or its qualification evidence.

The October 12 follow-up kept preparation simulation-only at the operator's
explicit instruction. Startup holds and speed/ramp brackets all failed to
complete 0.75 m. A separately approved fl_hy tracking-cap comparison (0.4 to
0.5 rad in simulation only) passed its prior tracking stop, then exceeded the
unchanged knee-speed bound after 0.3408 m. Hardware limits remain unchanged.
See ../records/relic-walking-onset-20261012/README.md for the PD-offset review,
foot/body displacement, exact rejected states, 200 fps replay and scope.

The subsequent [0.30 m/s hardware preparation review](../records/relic-walking-hardware-review-20261012/README.md)
keeps activation blocked: the accepted host speed cap, body-speed headroom,
guarded joint speeds, goal stopping and renewed bound evidence remain unresolved.
The operator requires a separate explicit approval immediately before any
physical deployment. Preparation never grants that approval.

The [tapered 0.30 m/s proof-of-concept trial preparation](../records/relic-walk030-poc-20261012/README.md)
supersedes the earlier constant-profile selection. The accepted host schema now
allows an explicitly bound request up to 0.30 m/s; live envelope/evidence guards
still apply. Short travel is operator-accepted as proof of concept, with actual
distance and normal timeout/shutdown recorded rather than claiming goal success.
Exact proposed operating caps and final physical approval remain separate.

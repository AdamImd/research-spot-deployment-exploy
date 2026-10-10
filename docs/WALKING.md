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

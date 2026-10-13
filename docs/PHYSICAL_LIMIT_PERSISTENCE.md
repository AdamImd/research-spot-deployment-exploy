# Physical operating-limit persistence

On 2026-10-13 Adam selected a continuous **greater than 500 ms** stop condition
for all physical operating limits, including torque, speed, attitude, position
and tracking. The explicit envelope fields are:

```json
{
  "physical_limit_persistence_s": 0.5,
  "physical_limit_persistence_reason": "Operator-approved continuous physical-limit window"
}
```

Omitted/zero duration retains immediate stops. A positive duration requires a
reason and is bounded to at most 0.5 seconds. Limits, gains, weights, native raw
ACK history and command values are unchanged. The separate requested leg torque
cap remains 75 N m; measured manufacturer bounds remain their existing values.
These are changes to host stop timing, not firmware limits or actuator clipping.

## Continuous per-limit timers

Every enforced physical limit has its own timer, separately for each joint or
body component. The first violating observation starts it; continued violation
at exactly 500 ms is allowed. The first control check strictly after 500 ms raises
the existing stop error. At 200 Hz this is normally the next 5 ms tick. An in-bounds
observation resets that timer. Repeated short impulses do not accumulate; another
joint or another type of limit cannot inherit or clear its duration. Repeated
checks at the same timestamp do not advance elapsed time.

The window covers measured and requested positions, measured speeds and loads,
PD-plus-feedforward demand, feedforward load, manufacturer joint and coupled-arm
torque, body roll/pitch and linear/angular speed, measured/command tracking,
arm-stow displacement and enforced command slew. Walking lateral/heading,
backwards/overshoot and final position drift use the same mechanism. Existing
arm/leg-position/target-rate observation-only selections remain observation-only.

The manufacturer knee-table domain is also a physical-position condition. During
an out-of-domain grace interval, torque monitoring uses the nearest table endpoint
without extrapolation. The domain timer still stops a sustained violation. This
endpoint check does not establish physical safety outside the manufacturer's table.

## Immediate checks and activation boundary

Tablet E-stop, firmware/robot faults and other health/readiness checks retain their
existing behavior. State validity, non-finite inputs/torque, timestamp order,
freshness, inference/computation deadlines, command gaps, acknowledgements and
expiry remain immediate. The held-arm output contract is immediate. Trial timeout,
walking timeout/stall, odometry integrity, artifact/identity/license gates and
shutdown deadlines are unchanged.

ReLIC startup qualification and one-time handover pose/velocity/torque-step gates
remain immediate. The persistence window is enabled when the rollout activates;
it is not permission to start from an invalid state. This option cannot delay a
stop enforced independently by Spot firmware or the tablet.

## Recording and offline validation

The live shared rollout logs `physical_limit_persistence` at activation and
`physical_limit_started`, `physical_limit_recovered`, `physical_limit_stop` events,
including the limit, component index, local timestamp, start, elapsed duration and
observed value when available. Full command/state recording is unchanged.

`tools/audit_physical_limits.py` checks a hash-verified simulator recording through
these actual host guards without rerunning dynamics or contacting a robot. It
uses the recorded states/targets and an ideal control clock; it does not qualify
real-time scheduling. A failed audit is recorded as `passed=false` rather than
silently ignored. Source recordings and previous failed trials remain immutable.

`--no-actuator-torque-limits` is independent of `--no-guards` in MuJoCo: ideal
unlimited actuator dynamics can now run with the persistence guards enforced.
An unguarded diagnostic still explicitly needs `--no-guards`. All runs require a
simulation-scoped envelope. Simulated fall proxies use the persistence window for
termination, but any observed fall still fails scientific recovery acceptance.

No physical motion is authorized by this change. A new source-bound bundle, fresh
preflight and the requested exact confirmation remain necessary before a trial.

The [validation record](../records/physical-limit-persistence-20261013/README.md)
contains the passing47s audit and60s guarded unlimited-actuator simulation.
The three physical crossing episodes recovered in5,10and15ms. A119ms host compute
outlier remains separate from these physical impulses and would trip the unchanged
live10ms deadline; the record does not establish live timing readiness.

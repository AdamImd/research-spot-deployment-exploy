# Recorded standing after operator leg adjustment

Adam requested “Try again, I fixed the leg positions” after approving the
front-right hip abduction tracking bound increase from 0.50 to 0.75 rad.
One additional supervised physical ReLIC trial completed all ten seconds.
Adam operated Spot and Minghao remained the independent manufacturer-tablet
E-stop operator under the previously confirmed session rig setup. The leg
adjustment is an operator report; the actual native-standing startup joint
state is retained in the raw `relic_initialized` event.

Runtime was clean commit `0aae29a0d6993213d689aa42cffa8c26a6a9588a`. The controller,
weights, graph, gains and other envelope fields matched preceding trials. Only
`tracking_error_max[3]` (`fr_hx`) increased to 0.75 rad. All torque protections,
body/leg guards, faults and tablet state checks stayed active. Arm motion was
observed under the explicit harness exception; held arm targets were unchanged.

Fresh candidate, one-attempt review, motors-off preflight and deployment-check
passed before activation. Previously validated runtime content hashes and the
offline recorded-target audit were checked; no new simulation or timing campaign
is claimed. Full hardware qualification was not declared by the one-trial review.

Native standing preceded direct policy startup without a fade. Initial body
height was measured at 0.52316 m and four foot contacts were reported. The policy
requested zero base velocity, ran at 50 Hz, and produced 200 Hz joint commands
with 30 ms expiry. Previous actions began at zero and advanced on acknowledgements.

| Recorded measurement | Result |
| --- | ---: |
| Policy-active duration | 10.0 s |
| Predictions / action-history acknowledgements | 500 / 500 |
| Joint commands | 2,003 |
| Maximum inference | 0.521 ms |
| Command-gap p99 / maximum | 5.471 / 5.694 ms |
| Observed acknowledgement p99 / maximum | 9.640 / 9.663 ms |
| Peak front-right hip target-to-measured gap | 0.476369 rad |
| Maximum absolute roll / pitch | 1.174 / 2.578 degrees |
| Final two seconds maximum absolute roll / pitch | 1.174 / 0.563 degrees |
| Final two seconds maximum leg speed | 0.06923 rad/s |
| Native shutdown including lease return | 5.016 s |
| Compact gzip bytes | 1,560,035 |

No first-failure or guard-failure event occurred. The front-right hip gap stayed
below even its former 0.50 rad threshold on recorded emitted commands. This
observation does not isolate whether the operator adjustment or other initial
state differences account for the changed outcome. The previous failed repeat
remains preserved; this new success does not erase it or establish robust startup
across initial poses.

The shutdown event explicitly confirms motors OFF, lease return and no cleanup
errors within the approved ten-second shutdown allowance. A separate read-only
postcheck passed with motors OFF and no active faults. There was no automatic retry,
fault clearing, E-stop write or separate arm command.

Private artifacts:

- Raw: `runs/relic-standing-tracking-075-20261009-001`.
- Compact state/actions: `runs/relic-standing-tracking-075-compact-20261009-001/rollout.jsonl.gz`.
- Postcheck: `runs/relic-standing-tracking-075-post-preflight-20261009-001`.
- Supervisor: `runs/relic-standing-tracking-075-supervisor-20261009-001`.
- Candidate: `local/relic-standing-tracking-075-20261009-002`.
- Operator request and review: `local/relic-tracking-trial-review-20261009-001`.

Raw metadata records exact command, host/environment, source and configuration,
model and weights hashes. Raw, compact and postcheck completion hashes were
verified. `result-summary.json` contains sanitized measurements and per-leg-joint
position offsets, P/D terms and estimated torque maxima. Estimated PD torque
is not an independent torque measurement; command states do not record body
translation, so measured drift is not asserted.

With private artifacts present, recompute the sanitized summary from the repo:

```bash
uv run --no-sync python records/relic-standing-tracking-075-20261009/summarize.py
```

Summary reproduction does not connect to Spot. The ten-second attempt is finished;
there is no active physical-control process or pending automatic retry.

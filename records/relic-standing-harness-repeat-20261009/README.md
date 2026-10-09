# Recorded ReLIC standing repeat, October 9

Adam explicitly requested another recorded standing trial after the successful
ten-second harness run. The repeat used the same controller, weights, graph,
gains and envelope, with Adam operating and Minghao on the manufacturer tablet
E-stop under the previously confirmed session rig arrangement. This attempt
did not complete ten seconds. No automatic retry or limit change was made.

Runtime commit `541eca743f00602aaca3d8808d5a3ab4c5a9db33` only added records and
documentation after the validated controller commit `85e1d1f`. Exact runtime
content, model and configuration hashes matched the successful trial. A fresh
candidate bundle, explicit one-attempt request, review and motors-off preflight
were recorded. The initial review assembly used the wrong simulator metadata
schema and stopped before producing reviews; its offline deployment-check
failure is preserved. Corrected review assembly and deployment-check then passed
before the single physical activation. No fresh simulation campaign is claimed.

The protocol remained direct startup without a fade, zero first previous actions
advancing only after acknowledgements, 50 Hz policy, 200 Hz commands, zero base
velocity, held arm targets, harness arm motion observed, and arm torque protection
enforced. Command expiry remained 30 ms and native shutdown allowance ten seconds.
Native standing preceded policy activation. Initial measured requested body height
was 0.52147 m and all four initial foot contacts were reported.

The recording contains 10 policy predictions, 9 action-history acknowledgements
and 39 commands. First-to-last prediction time was 0.18001 s. Maximum inference
was 0.523 ms and maximum recorded command gap 5.465 ms. The tenth predicted target
failed the command tracking-error guard and was not transmitted:

| Joint | Measured angle | Rejected target | Difference | Reviewed limit |
| --- | ---: | ---: | ---: | ---: |
| Front-right hip abduction (`fr_hx`) | -0.141409 rad | +0.362810 rad | 0.504219 rad | 0.500000 rad |

The rejected state and policy target are both recorded, so this triggering joint
is established directly. The difference is 28.890 degrees versus a 28.648-degree
limit. This was a leg tracking stop; the arm observation exception was active.
The actual target-to-measured gap includes the PD controller's commanded offset;
it is not solely an actuator lag measurement.

Compared with the successful run, the first leg target differed by at most
0.04013 rad; by prediction ten the largest target difference was 0.39956 rad.
These recordings demonstrate that this startup outcome was not repeatable under
the two observed initial conditions. They do not establish the underlying cause
or justify increasing the guard. Inspect paired initial states and early closed
loop response before any proposed controller/protocol revision.

The shutdown event explicitly confirmed motors OFF and lease return in 6.01580 s,
with no cleanup errors and within the approved budget. A separate read-only
postcheck passed with motors OFF and no active faults. The compact gzip is
38,870 bytes and preserves the failed source status. Raw, compact and postcheck
artifact hashes were verified; sanitized values/hashes are in `result-summary.json`.

Private raw paths:

- `runs/relic-standing-harness-20261009-002`
- `runs/relic-standing-harness-compact-20261009-002/rollout.jsonl.gz`
- `runs/relic-standing-harness-post-preflight-20261009-002`
- `runs/relic-standing-harness-supervisor-20261009-002`
- Candidate `local/relic-standing-harness-20261009-003`
- New request/review procedure `local/relic-harness-review-20261009-002`

The summary's triggering-joint calculation is the absolute difference between the
last `policy.targets` and `guard_failure.state.positions`, compared with the
envelope's per-joint `tracking_error_max` for the twelve legs. Other counts and
shutdown values come directly from the events. Private raw metadata retains the
exact command, source, environment, policy/model and input hashes. No additional
physical attempt is authorized by this record; full hardware qualification is
still incomplete.

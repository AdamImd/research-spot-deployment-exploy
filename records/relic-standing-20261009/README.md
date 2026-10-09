# First physical ReLIC attempt, October 9

The user explicitly requested ReLIC activation, accepted the known arm discrepancy,
and selected the manufacturer tablet for emergency stopping with up to ten seconds
for native safe-power-off. Adam was robot operator and Minghao the independent
stop operator; the previously confirmed slack fall-arrest rig was the session setup.
A new local hardware envelope changed only native shutdown timeout from2 to10seconds;
command TTL stayed30ms, policy50Hz, command200Hz, zero basevelocity, direct startup,
zero initial previousactions and zero policy feedforward. Source was cleanb8ecd62.

The hash-bound local review records were explicitly acceptance for one supervised
commissioning experiment. They retained partial hardware timing/model qualification,
operator-reported rig/tablet checks, known arm discrepancy and previous shutdown
failures. They do not declare prior physicalReLIC qualification. Fresh motors-off,
no-fault, tablet-ready preflight passed before the single attempt.

Spot powered on and stood natively, then ReLIC entered its policy phase. The run
recorded29jointcommands,7policy predictions(first-to-last0.12504s), and6policy
acknowledgement-history updates. Initial previousactions were allzero. Maximum
inference0.543ms; maximum recorded commandgap5.536ms. It aborted on a measured
joint-position limit before completing the10secondtrial. The last recorded arm
shoulder-pitch position was -3.141332rad,0.015degree above the configured -pi bound.
This is the likely trigger, but the rejected state was not logged, so the causal
joint is not conclusively established. No full-duration stability claim is made.

The shutdown_result event explicitly confirms motorSTATE_OFF and lease return,
with no cleanup errors. Native safe-power-off6.00989s, lease return0.00297s, total
6.01295s, within the approved10secondbudget. The top-level failed result.json does
not repeat these fields; absence there is not absence of shutdown evidence. A
separate read-only post-preflight confirms motorsOFF and no activefaults.

The initial executor summary incorrectly described an immediate failure and
unavailable shutdown evidence based on result.json alone. The event log provides
the more complete observations reported here. No repeat or faultclearing occurred.

Raw: runs/relic-standing-20261009-001. Compact lossless action/state capture:
runs/relic-standing-compact-20261009-001/rollout.jsonl.gz; source_status remainsfailed.
Postcheck: runs/relic-standing-post-preflight-20261009-001. Supervisor:
runs/relic-standing-supervisor-20261009-001. Source/input/model/weights hashes and
package environment are retained in raw run.json; completion hashes verified.
Policy deployment was attempted, but the10secondcommissioning trial is incomplete.

# Doubled speed caps — October 12, 2026

After the recorded10ms trial stopped on front-left HY measured speed6.042rad/s
against6rad/s, Adam requested "Double the speed caps universally and deploy again"
and "No tests." Prepare one new attempt, with no additional offline test,
simulation, replay or benchmark. Existing strict-limit failures remain preserved.

| Configured bound | Previous | New |
|---|---|---|
| Leg HX/HY measured speed | 6 rad/s | 12 rad/s |
| Leg knee measured speed | 12 rad/s | 24 rad/s |
| SDK shared velocity backstop | 12 rad/s | 24 rad/s |
| Body linear speed | 0.40 m/s | 0.80 m/s |
| Body angular speed | 0.50 rad/s | 1.00 rad/s |
| Leg virtual target slew | 100 rad/s | 200 rad/s |
| Arm target slew | 0.5 rad/s | 1.0 rad/s |
| Declared arm/gripper measured speeds | [1,1,1,0.5,0.5,0.5,1] rad/s | [2,2,2,1,1,1,2] rad/s |

The explicit harness arm-motion observation mode and held-arm commands remain
unchanged; doubling declared speed vectors does not authorize manipulation.
Keep policy maximum request0.30m/s, taper/0.10m/s² acceleration/3s initialhold,
goal0.75m in the current post-stand heading, <=20s activepolicy, nofade,
zero initial previousactions/ACKhistory, fixed gains/weights and nominal50Hz/200Hz.
Virtual targets knees[-3,0], hips[-3,3], PD-offset caps2rad, manufacturer torque/
measured-angle limits, inference/core10ms, gap/ACK15ms, expiry30ms and native
shutdown10s are unchanged. Short goal distance is accepted, not a successful
goal; other guard failures retain failed status. No automatic retries.

Proposal/bundle/explicit delta/operator request remain in ignored
`local/relic-speed-caps-2x-20261012-001/`. Retain earlier graph/guarded simulation,
host trace and prior physical records with their ORIGINAL source/envelope hashes.
No fresh experimental evidence is claimed for the doubled speed envelope. Any
one-attempt review accepting the change must state this limitation and the
operator-requested lack of additional tests. Live readiness and stop/lease guards
remain enforced. Exact final profile/current Adam/Minghao/tablet/followingrig/
clear-path confirmation is pending under Adam's earlier instruction to ask
exactly before activation; no physical motion is implied by creating this record.

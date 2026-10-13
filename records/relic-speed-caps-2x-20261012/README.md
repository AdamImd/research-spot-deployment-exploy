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

## One confirmed physical repeat

The first read-only readiness snapshot passed health checks but reported motors
ON, so no motion was launched from it. Adam then confirmed motors OFF and the
exact one-attempt profile/current operators/tablet/rig/path. The fresh motors-off
snapshot and bound deployment review passed. One recorded physical repeat ran
from clean commit `90a4edbf0b1c593f30bdd67fc4b7ea6c2bfacb5a`, without additional
offline tests, simulations or benchmarks. Its single-use approval is consumed.

The repeat issued policy commands for **6.070 s** and advanced **0.305277 m** along
the current post-native-stand heading, with about -0.0051 m lateral displacement.
The stopping state gives0.306775m forward. The policy request reached0.30m/s;
the0.75m goal was not reached. It stopped on **measured joint position limit**:
front-right knee **-2.125037 rad** crossed its unchanged configured lower bound
**-2.10 rad** by0.025037rad (about1.43degrees). This is a configured operating-angle
bound, not a claim that the manufacturer mechanical limit was exceeded. No speed
or torque limits were changed in response, and no retry occurred.

Recorded core-command computation maximum was2.696ms, within the10ms limit.
Raw records contain304 predictions,304 acknowledged histories and1218 commands.
Native shutdown confirmed motors OFF and lease return in **6.014 s**, within the
10s allowance; passive postcheck passed with motors off and no active faults.
Compact recording completed at **1,122,829 bytes**, preserving failed source status
and `hardware_qualified=false`. A guard-stopped partial walk is not goal success.

[Sanitized hardware summary](hardware-result-summary.json) and
[analysis script](summarize_hardware.py) bind the recorded commit and completion
hashes without publishing raw endpoints, identity or world pose. Reproduce with:

```bash
uv run --no-sync python records/relic-speed-caps-2x-20261012/summarize_hardware.py
```

Raw trial, passive postcheck and compact archive remain in ignored
`runs/relic-speed-caps-2x-{hardware,postcheck,compact}-20261012-001/`.
Supervisor handle/approval/command/exit records:
`runs/relic-speed-caps-2x-hardware-supervisor-20261012-001/`.

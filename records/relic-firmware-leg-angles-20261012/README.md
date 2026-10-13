# Firmware leg-position authority — October12,2026

Question: does the unchanged ReLIC continue a supervised0.75m tapered walking
proof of concept when conservative host measured leg-angle bounds are observed
instead of stopping, with remaining manufacturer and operating guards retained?
Adam requests the physical trial without offline tests and explicitly no retraining.

Implementation: envelope-bound `leg_position_guard=observe` with operator reason,
ReLIC-only adapter, manufacturer torque profile and explicit finite virtual target
ranges required. Only measured leg-angle stops become observations; mode/crossings
are recorded in raw and compact traces. Defaults stay enforced. No weights/gains,
rewards, frames, history or cadence change. Firmware saturates desired positions;
this does not establish measured overshoot/balance protection. Manufacturer torque
and knee table domain, speeds/SDK backstop, tracking, targets, body, timing, ACK,
expiry and tablet controls remain.

Proposal is ignored `local/relic-firmware-leg-angles-20261012-001/`. Same maxrequest
0.30m/s taper/0.10m/s² acceleration/3s zerohold/current-poststand0.75m goal/20s cap,
no fade, zero initial raw actions, shutdown10s. Speed caps body0.8m/s angular1rad/s,
hips12knees/SDK24rad/s, target slewlegs200arm1rad/s; PD-offset2rad. Short travel is
accepted without exact goal success. One distinct recorded attempt, no retry.

No tests, simulations, replays or benchmarks are run at operator request. Existing
results are historical evidence only; this source/guard change is untested. Bundle
creation/schema generation are configuration operations, not claimed test passes.
Fresh motors-off readiness and exact current Adam/Minghao/tablet/rig/path approval
remain pending. Raw identities/endpoints/world poses stay ignored and unpublished.

## Approved physical result and operator acceptance

Adam confirmed the exact one-attempt/current crew/tablet/rig/path profile. Fresh
motors-off readiness and source/envelope-bound deployment checks passed. ONE
physical trial ran from clean `b1fc5a9ea34c453b94905304c86fc4de61b2bba8`, with no
offline tests, retraining, source edits during execution or retry. The approval is
consumed. Afterward Adam assessed it as "Good job, sucess!"; record this as
operator acceptance of the walking proof of concept, not full policy qualification.

Latest issued-state travel was **0.692539 m forward**, **-0.021512 m lateral** along
the post-native-stand heading. Active issued-policy span was **12.920 s**; about
**0.05746 m** remained to the0.75m nominal goal. The finite virtual targets and
manufacturer/speed/body/watchdog constraints stayed enabled. Five angle-observation
events recorded right-front and right-hind knee operating-bound crossings without
stopping solely for those angles. Weights, gains and history behavior stayed fixed;
no online reward or extension penalty was introduced.

The trial ended on `E-stop or robot fault stopped ReLIC`, retaining raw `failed`
status and incomplete nominal goal. The passive postcheck also retained `failed`
status: motors were OFF, but one active system `payload.fault` code9, severity1,
was present. Do not infer the physical fault cause or that an emergency tablet
press occurred solely from the combined stop message. No fault clearing was
performed. Further activation would require current fault/readiness review and
new explicit authority; operator proof-of-concept acceptance does not erase faults.

Native shutdown confirmed motors OFF and lease return in **6.016 s**, within the
10s allowance. Raw records contain647 predictions,646 acknowledged histories and
2588 commands; maximum recorded core computation was2.507ms, within10ms. Compact
recording completed at **2,399,409 bytes**, preserving failed source status and
`hardware_qualified=false`. Shutdown success is distinct from the faulting postcheck.

[Sanitized hardware summary](hardware-result-summary.json) records numerical
results, fault name/code, operator acceptance and original completion hashes.
[Analysis script](summarize_hardware.py) reads existing records without replaying
or controlling the robot. Reproduce from the repository root:

```bash
uv run --no-sync python records/relic-firmware-leg-angles-20261012/summarize_hardware.py
```

Raw trial/postcheck/compact records remain ignored:
`runs/relic-firmware-leg-angles-{hardware,postcheck,compact}-20261012-001/`.
Supervisor/approval/command/exit provenance:
`runs/relic-firmware-leg-angles-hardware-supervisor-20261012-001/`.

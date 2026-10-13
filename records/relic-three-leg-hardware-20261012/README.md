# One approved three-leg physical commissioning trial, October12,2026

Source: ff6333f1bfc9960ae26d0a4b34c0feeb4516256d, clean. Adam explicitly
approved ONE47s attempt with Minghao on connected tablet and attached/clear
three-foot rig/workspace. Frozen source/configuration binding and exact approval
are private local/relic-three-leg-live-review-20261012-001. CPU UV locked runtime;
no weight/gain/history/limit changes. No local E-stop, fault clearing or lease takeover.

Immediate motors-off preflight003 passed. Native stand and direct policy startup
ran. At about5s, the first selected-leg prediction was rejected for predicted
PD-plus-feedforward load: front-right HX actual−0.11125rad, target+0.66267rad,
velocity+0.00800rad/s gives46.4231Nm withkp60/kd1.5, against44.88Nm. This rejected
command was never sent. The front-left selected target still matched actual joints
and IK residual waszero; the physical leg lift/square/return was not demonstrated.

Offline counterfactual used the EXACT retained state, same preceding raw-action
history, unchanged graph/weights/gains/height, changing only the leg command tozero.
Four-foot FRHX target0.05846rad requests10.1705Nm. Restoring the selected-leg command
reproduces46.4231Nm. This identifies a discrete mode-entry prediction transient,
not tablet/fault stop, ordering error, high measured joint speed or the previously
fixed return handoff. Why this physical stance differs from the earlier MuJoCo
transition remains unqualified; do not treat one simulator pass as hardware parity.

251 predictions,250ACKs,1004issuedcommands preserved. Native shutdown confirmed
motorsOFF and lease returned in5.017s total (safe power-off5.013s), below10s allowance.
Passive postcheck passed, motorsOFF and faultfree. Compact archive is complete,
with failed raw-run status retained. No automatic retry or additional motion.
See result-summary.json for numerical outcomes and compact completion hash.

Raw supervisor/command/config/source/approval logs and compact actions/state/IK
recordings remain ignored under runs/relic-three-leg-{hardware,compact,postcheck,
hardware-supervisor}-20261012-001. No private identities/endpoints/poses are published.

Next engineering gate: replay the measured physical pre-transition stance offline
and evaluate a continuous mode-entry strategy while retaining manufacturer torque
limits and unchanged policy. A new physical attempt requires new exact approval;
the47s authorization is consumed. Do not silently widen torque limits or relabel
this trial as successful three-leg motion.

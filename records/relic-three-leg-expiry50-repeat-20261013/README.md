# Unchanged expiry-50 ms repeat — completed commissioning sequence

Adam approved one unchanged47s trial and confirmed Adam operating, Minghao on the
functioning connected tablet, and the rig/workspace setup. The attempt ran once
from clean commit `a840ef2a4b14a688496529164d8ae35ae2b44f7c`. No offline tests,
runtime changes or automatic retries occurred.

The complete47s controller schedule executed: initial four feet, front-left lift,
three-leg hold, IK motion, lowering, four-foot handover and final four feet.
Recorded2350policy predictions,9397commands and2350policy-history ACK events.
The command-ACK latency summary separately contains6668acknowledgement measurements.

| Measurement | p99 | Maximum |
| --- | ---: | ---: |
| Command gap | 5.765 ms | 14.251 ms |
| Command ACK latency | 9.486 ms | 17.205 ms |

Seven physical-limit timers started and all recovered, with maximum continuous
duration30.000ms, below the500ms persistence window. These include duplicate
requested-torque checks; seven timers are not seven independent physical events.
Full requested/measured values and timestamps are in `result-summary.json` and
the raw recording. No physical-limit stop event or first-failure event occurred.

Native motors-off and lease return completed in6.015s, within the10s allowance.
The read-only postcheck passed: motors off, zero active faults, clear tablet stop
states. Raw successful-run/postcheck completion manifests and compact artifact
hashes were verified. The compact recording completed successfully.

This establishes completion of one supervised commissioning sequence. Recorded
phase labels establish the commanded schedule; independent measured Cartesian
tracking/contact-load acceptance and general hardware robustness are not claimed.
The50ms expiry and45ms gap/ACK limits remain; half-second producer stalls are not
tolerated merely because physical persistence/computation thresholds are500ms.

Artifacts: `runs/relic-three-leg-expiry50-repeat-hardware-20261013-001`, including
`trial`, `postcheck`, `compact` and supervisor logs/status. The private frozen
bundle/approval/contract is under
`local/relic-three-leg-expiry50-repeat-20261013-001`. See `artifacts.json` for
hashes. Approval is consumed; no further physical run is authorized automatically.

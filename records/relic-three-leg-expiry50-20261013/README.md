# Correcting the rejected joint-command expiry

Adam requested a fix and another attempt, with the proper timing checked against
the documentation. The previous 505 ms expiry was rejected by firmware 5.0.1
with `STATUS_TOO_DISTANT`; that repeat produced zero policy predictions and
confirmed motors off and lease return afterward. Its approval is consumed.

## Source and correction

The [official SDK 5.0.1 Joint Control example](https://github.com/boston-dynamics/spot-sdk/blob/v5.0.1/python/examples/joint_control/joint_api_helper.py#L151-L155)
sets end_time to synchronized robot time corresponding to local now plus **50 ms**.
That example supplies an appropriate starting value, not evidence of the maximum
firmware ceiling. The [5.0.1 protocol](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/protos/bosdyn/api/robot_command.proto)
defines STATUS_TOO_DISTANT as an end time too far in the future; inspected
protocol comments do not specify the numeric maximum.

This draft changes only three envelope fields:

| Field | Rejected configuration | Corrected draft |
| --- | ---: | ---: |
| command_ttl_s | 505 ms | 50 ms |
| max_command_gap_s | 500 ms | 45 ms |
| max_command_ack_s | 500 ms | 45 ms |

The 5 ms margin is one nominal command tick. The host requires gap and ACK
budgets strictly below expiry. Policy remains 50 Hz and commands 200 Hz; no
change to extrapolation (zero), gains, weights, action history or target filtering.
All other settings remain, including 500 ms physical-limit persistence,
500 ms computation ceiling, 500 ms state-gap/health budgets and 505 ms state/
policy age. The synchronous producer's **45 ms watchdog and 50 ms firmware
expiry mean a half-second producer stall is still not tolerated**. Do not claim
this configuration makes the prior 119 ms compute outlier survivable. It fixes
the rejected expiration time only; no asynchronous command renewal was added.

## Attempt boundary

The proposed trial is the same recorded 47-second front-left lift/hold/IK square/
lower/return, with no entry limiter, two-second return blend, 75 N·m requested
leg torque threshold, retained measured manufacturer bounds, existing observation
exceptions and 10-second native shutdown allowance. Tablet is sole E-stop.

No offline tests, simulation or benchmark were run, retaining Adam's no-tests
instruction. Packaging and motors-off preflight remain required. Prepare a new
source-bound bundle and obtain the requested exact final confirmation with Adam
operating and Minghao on the tablet, rig attached and workspace clear. One
attempt only; no retries. Local packet:
`local/relic-three-leg-expiry50-preparation-20261013-001`.

## First corrected attempt outcome

The approved trial ran once at clean commit
`cc853566f76d949c2f902fc88bbf8c98f3e5e179`. The50ms expiry was accepted:
1,162 commands,290 policy predictions and289 acknowledgements were recorded,
with5.780seconds between first and last policy predictions. Initial four-foot
standing and lifting were observed; the IK square and return were not reached.
The first error was `joint control no longer processing`. The saved feedback
does not identify whether an operator command, firmware condition or another
event caused that status. Do not infer the cause from absent active faults.

Three physical-limit episodes recovered: front-right hip speed in5.001ms,
front-right knee manufacturer torque in5.005ms and body angular speed in15.007ms.
None reached the500ms persistence threshold. These crossings were not the
recorded stop reason. Motors off and lease return were confirmed in7.021s;
postcheck passed with zero active faults and clear stop states. No retry was
made under that approval. Raw and compact recording hashes were verified.
See `result-summary.json` and `artifacts.json`.

Adam then stated "We jumped the gun" and requested another run as-is. That
statement is operator context, not independently verified stop causality.
Prepare one separate repeat with identical runtime source, manifest, robot
profile and envelope; no offline tests. The new local packet is
`local/relic-three-leg-expiry50-repeat-20261013-001`. Fresh final confirmation
and motors-off preflight remain required; the preceding approval is consumed.

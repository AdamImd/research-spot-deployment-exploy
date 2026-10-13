# Recorded three-leg trial: angular-speed cap 2 rad/s

## Question and protocol

Does the unchanged smoothed three-leg controller progress through lift with the
measured body angular-speed cap increased from 1 to 2 rad/s? Adam approved one
47-second physical attempt, with Minghao holding the manufacturer tablet E-stop,
rig attached and workspace clear. No offline tests were requested or run for this
profile. No retries were authorized.

Source: `6b5c46a969286cc493c1a567f5964f65192b526d`, clean at launch.
Environment: existing locked UV deployment environment on the local RPM host.
Pretrained actor, gains, zero initial raw-action history, ACK-owned history updates,
50 Hz policy / 200 Hz commands, 1 rad/s post-entry leg-target limiter and two-second
return blend were unchanged. The only envelope change was `max_angular_speed=2.0`.
Native stand preceded the 5 s four-foot hold, 6 s lift, 3 s hold, 12 s IK square,
6 s lower, 2 s return blend and 12 s final hold, plus 1 s margin. Other protections
were retained, including 10-degree roll/pitch caps and 10 s shutdown allowance.
Physical execution has no assigned random seed; this is a recorded commissioning
attempt, not a replicated simulation experiment.

## Observed result

Stopped at 5.485 s, during lift, on the body-attitude guard. Pitch was
0.179495 rad (10.284 degrees), above the unchanged 0.174533 rad (10-degree) cap.
Roll was -0.041406 rad; body angular speed was 1.061006 rad/s, below the new 2 rad/s
cap. There were 275 policy predictions, 1,099 commands and 274 policy ACKs.
Recorded post-entry target steps remained at most 0.02 rad and rates at most
1 rad/s. No completed tripod, IK square or return is claimed.

Native shutdown confirmed motors off and lease returned in 5.015 s. Passive
postcheck passed with motors off, no active faults and tablet stop ready.
Recording integrity was verified; the source run remains FAILED while compacting
completed successfully. Compact artifacts total 998,505 bytes. See
[result-summary.json](result-summary.json) for exact metrics and completion hash.

Private raw data: `runs/relic-three-leg-angular2-hardware-20261012-001`.
Compact recording: `runs/relic-three-leg-angular2-compact-20261012-001/rollout.jsonl.gz`.
Supervisor: `runs/relic-three-leg-angular2-hardware-supervisor-20261012-001`.
Postcheck: `runs/relic-three-leg-angular2-postcheck-20261012-001`.
Exact bound config, command, human approval and environment evidence are retained
under ignored `local/relic-three-leg-angular2-hardware-20261012-001` and supervisor
records. Robot identity, endpoint and world-pose records are not published.

## Interpretation and decision

The command limiter operated as configured; this attempt reached the attitude cap
instead of the previous angular-speed cap. This does not establish stable
three-leg behavior. The one-trial authorization is consumed. No retry or additional
limit change was made. Keep the failed recording for subsequent review.

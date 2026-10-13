# Three-leg latency allowance — commissioning draft

Adam selected a 500 ms computation allowance and then explicitly included
communication and stale-data limits. The 119.366 ms simulation computation
outlier is within that allowance; it remains a recorded observation, not a
real-time or wired-transport qualification. No runtime algorithm, policy, gain,
action-history rule, limiter or firmware setting is changed.

The private draft is
`local/relic-three-leg-latency500-preparation-20261013-001/envelope.proposed.json`.
Its frozen deployment bundle will be `bundle` in that directory. The earlier
`bundle.compute-only` is superseded and must not be used for this request.

| Timing field | Proposed value |
| --- | ---: |
| Computation/inference | 500 ms |
| State timestamp gap | 500 ms |
| Command gap / producer watchdog | 500 ms |
| Command acknowledgement | 500 ms |
| Full-state health freshness | 500 ms |
| Command expiry | 505 ms |
| State and policy maximum age | 505 ms |

The extra 5 ms is one command tick. The existing contract requires gap/ACK to
be strictly less than expiry, and expiry no greater than policy age. Effective
expiry is also bounded by the state timestamp plus maximum state age. Expiry
does not renew merely because computation stalls. A stalled host can therefore
leave the last joint target valid for approximately half a second; this must be
explicit in the final trial confirmation. These are maximum stop thresholds,
not a change from the nominal 50 Hz policy and 200 Hz command rates.

Physical operating-limit persistence remains strictly greater than 500 ms,
independent per limit/component and reset on recovery. This is distinct from the
absolute timing budgets above. Tablet E-stop, reported faults, non-finite data,
timestamp ordering, held-arm output contract, startup qualification and trial
deadlines retain their existing checks. The exact known payload INFO exception
and existing observation modes remain unchanged.

The [previous preparation record](../relic-three-leg-deployment-preparation-20261013/README.md)
describes the unchanged 47-second front-left lift, hold, IK square and return,
75 N·m requested leg demand threshold, manufacturer measured-torque bounds,
unfiltered entry, two-second return blend and 10-second native shutdown allowance.

A new motors-off read-only preflight passed on this date, with no active faults,
stowed arm, clear robot stop states and matching identity/model/payload/joints/
license. No lease or motion command was issued. Preflight freshness must be
rechecked immediately before any authorized attempt.

This record does not authorize motion. Obtain the requested exact final
confirmation with the timing values above and current named operator, tablet,
rig and workspace setup. Review the bound commissioning evidence without
relabeling simulation or historical transport measurements as new hardware
qualification. One physical attempt only; no automatic retries.

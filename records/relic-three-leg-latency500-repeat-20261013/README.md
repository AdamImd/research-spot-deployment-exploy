# Latency-500 ms repeat preparation

Adam requested another attempt with no tests after the previous stream closed
before any policy prediction. This revision preserves the stream response status
enum, numeric status and header error code in the existing stop error. It does
not log raw server messages/headers. Control behavior and all timing, policy,
gain, history, physical limit and shutdown settings are unchanged.

No offline test suite, replay, simulation or benchmark is run for this revision,
as requested. Packaging, required deployment gates and motors-off preflight remain
part of preparation. The source change is explicitly untested.

Prepare a fresh source-bound bundle for the identical 47-second commissioning
sequence, with 500 ms computation/gap/ACK/health budgets, 505 ms expiry/state/policy
age and the previously reviewed physical limits. The previous approval was
consumed. Obtain the requested final one-trial confirmation before physical
execution. No automatic retries.

The prior failed trial and confirmed shutdown remain recorded in
`records/relic-three-leg-latency500-hardware-20261013`.

## Approved attempt outcome

Adam subsequently approved one repeat and confirmed the operator/tablet/rig
setup. The repeat launched exactly once from clean commit
`a2888381ad7fd207b7884eb1b6425c7ea34dbe2a`; no offline tests were run.
Native stand completed. Two preparation commands and zero policy predictions
were recorded before **STATUS_TOO_DISTANT (5), header error code 1**.
The newly retained status identifies an end-time rejection; the front-left demo
did not run. The [firmware-matched 5.0.1 protocol definition](https://raw.githubusercontent.com/boston-dynamics/spot-sdk/v5.0.1/protos/bosdyn/api/robot_command.proto)
defines this status as a command end time too far in the future.

Both recorded preparation commands had expiry approximately 505 ms after the
associated robot state. The agent-selected 505 ms expiry did not work with this
robot. The exact supported firmware ceiling is not established by this attempt
or the inspected protocol comments. Restore a firmware-compatible command expiry
before proposing another run. Host computation and communication allowances do
not override firmware expiry; a shorter expiry cannot tolerate a half-second
producer stall merely by retaining the longer host watchdog.

Native shutdown and lease return completed in **6.019 seconds**, within budget.
Postcheck passed with motors off, zero active faults and clear stop state.
No retry occurred; approval is consumed. Raw, postcheck and compact records are
under `runs/relic-three-leg-latency500-repeat-hardware-20261013-001`; completion
and compact artifact hashes were verified. See `result-summary.json` and
`artifacts.json`. The durable supervisor finished with failure, not rollout
success. Further physical execution requires a new exact approval.

# One approved latency-500 ms trial — stopped before policy predictions

The exact one-trial approval confirmed Adam operating, Minghao holding the
connected functioning tablet, and the rig and three-foot/foot/arm workspace.
The attempt used clean commit `101f6926d3c64a67193a62f08e92b370c39a2f3c`
and the [reviewed timing configuration](../relic-three-leg-latency500-20261013/README.md).
No parameters changed during execution and no retry occurred.

The final motors-off preflight and bound deployment check passed. Native power-on
and stand completed. The recorder captured two preparation commands and **zero
policy predictions**, followed by `command stream ended unexpectedly`.
The front-left lift/hold/IK/return sequence did not run. This is a failed
commissioning attempt, not a successful 47-second rollout.

Native safe-power-off and lease return were confirmed in **5.017 seconds**, within
the 10-second allowance. The subsequent read-only check passed with motors off,
zero active faults and clear robot stop states. Tablet authority was unchanged.

## Diagnostic limit

`ControlSpot.check_stream()` calls `future.result()` but discards the returned
stream response before raising its generic completion error. The saved trace
therefore does not establish why the server ended the stream. Do not label this
as a confirmed expiry rejection, network timeout, torque stop or policy failure.
Capture the stream response status and sanitized error details before another
approved attempt; no diagnostic change or new physical attempt is included here.

## Artifacts and continuation

Raw, postcheck and compact recordings are under
`runs/relic-three-leg-latency500-hardware-20261013-001`.
The durable supervisor session was
`relic-three-leg-latency500-hardware-20261013-001`; its final status is
`finished_with_failure`. Raw failed-run and postcheck completion manifests, plus
compact artifact hashes, were verified. Compact capture completed successfully;
that status describes recording integrity, not trial success.

See `result-summary.json` and `artifacts.json` for sanitized outcomes and hashes.
The private deployment/approval/runner remain under
`local/relic-three-leg-latency500-preparation-20261013-001`.
Approval is consumed. Any new physical attempt requires the requested fresh
exact confirmation and a new run directory; do not reuse this attempt or retry.

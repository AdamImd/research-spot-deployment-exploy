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

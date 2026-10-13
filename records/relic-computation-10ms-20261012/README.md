# Selected 10 ms computation limit — October 12, 2026

Adam accepted a few long delays and selected a new 10 ms limit after the retained
20-second host benchmark's 5.272 ms core computation exceeded its old 5 ms limit.
Set the new hardware proposal's `max_inference_s` to 0.010 seconds. This field
bounds inference and the entire core-command computation in the live loop.
Only this field differs from the previous wider-PD-target envelope. The nominal
command schedule remains 200 Hz (5 ms), with no catch-up bursts. Command-gap and
ACK limits remain 15 ms, expiry 30 ms, and native shutdown allowance 10 seconds.

The selected virtual targets remain knees [-3,0] rad, hips [-3,3] rad, leg
PD-offset caps 2.0 rad. Measured joint-angle and manufacturer torque protections,
weights/gains, direct startup, zero action history and tablet stop authority stay
unchanged. The walking profile remains maximum request 0.30 m/s tapered toward a
0.75 m goal in the current post-stand heading, acceleration 0.10 m/s², initial
zero-speed hold 3 s and policy duration at most 20 s. Short travel is accepted as
a proof of concept, not exact goal completion.

## Retained evidence, not a new timing run

[Trace review](trace-review.json) binds the original execution commit and raw
artifacts to the revised envelope and analysis-tool hashes. Against 10 ms, the
retained trace has zero computation/inference deadline misses; its maximum core
computation is 5.272 ms. Maximum command gap is 10.108 ms, within the unchanged
15 ms gap limit. It still contains two nominal missed releases, maximum production
5.419 ms and maximum release-to-finish 5.771 ms. These are accepted for this
observed trace and remain visible; no assertion of strict 200 Hz or absence of
future misses is made. The original 5 ms failure record remains immutable.

No simulation/benchmark is repeated for this change. Earlier guarded20s walking
and60s standing results are retained with their original source/envelope hashes;
no new execution or hardware/firmware equivalence is claimed. Public raw identities,
endpoints and world positions remain excluded. The proposal and new bundle are in
ignored `local/relic-computation-10ms-20261012-001/`.

Adam then requested deployment without further testing. Skip additional offline
experiments. Deployment still needs the requested exact final confirmation of
this one-attempt profile and current Adam/Minghao, rig/path and tablet readiness.
The deployment command retains its live motors-off preflight and all guards; no
E-stop registration/rearm, fault clearing, lease takeover or automatic retry.
This record does not authorize activation by itself or fabricate current health.

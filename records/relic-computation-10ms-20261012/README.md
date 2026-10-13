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

## One confirmed physical attempt

Adam subsequently confirmed the exact profile and current Adam/Minghao/tablet/
following-rig/clear-path setup. Motors-off preflight and bound deployment checks
passed. One recorded physical attempt ran from clean commit
`2afa4a22ff9ffc63b9da61f8954af1e905bde345`; no additional simulation or benchmark
ran and no retry occurred. The requested single-use approval is now consumed.

The policy ran for **5.385 s** before `measured joint velocity or load limit` stopped
it. The offending sample was **front-left HY velocity -6.04235 rad/s** against the
selected absolute **6.0 rad/s** cap. No measured-load bound was exceeded in that
stopping sample. This is a measured-speed stop, not a command-position or 10 ms
computation stop. Recorded core computation maximum was 2.810 ms; the wider
bounds do not remove measured-speed protections.

Latest issued-state travel was **0.180435 m** forward, about **-0.026 m** lateral
along the robot-current post-native-stand heading. The stopping state gives
0.181065 m forward; the last lower-rate progress record gives 0.1781 m. These
measurements have distinct timestamps. Maximum request was 0.236 m/s, before the
0.30 m/s maximum. The 0.75 m goal was not reached. Preserve raw status `failed`;
do not label a speed-guard stop as accepted shortfall or successful rollout.

Native shutdown confirmed motors OFF and lease return in **7.017 s**, within the
10-second allowance. Passive postcheck passed with motors off and no active
faults. Full raw events contain 270 predictions, 269 acknowledged histories and
1080 issued commands. Compact state/action recording completed at **999,891 bytes**,
retaining failed source status and `hardware_qualified=false`.

[Hardware summary](hardware-result-summary.json) and
[analysis script](summarize_hardware.py) preserve hashes and sanitized relative
measurements. Run from the repository root with:

```bash
uv run --no-sync python records/relic-computation-10ms-20261012/summarize_hardware.py
```

Raw inputs remain in ignored `runs/relic-computation-10ms-{hardware,postcheck,compact}-20261012-001/`.
The supervisor record is `runs/relic-computation-10ms-hardware-supervisor-20261012-001/`.
Do not expand speed limits or automatically reattempt based on this result.

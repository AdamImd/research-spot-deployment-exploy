# Wider virtual PD targets — 2026-10-12

## Question, design and scope

Adam requested wider boundaries to accommodate PD scaling: knees [-3,0] rad,
similarly wider hips. Interpret these as virtual PD target bounds and prepare
hips [-3,3] rad, preserving measured-angle safety bounds. Adam separately selected
preparing 2.0 rad target-minus-measured limits for all leg joints. Another physical
attempt requires a new exact approval; the previous approved attempt was used
and ended on the command-position guard. No automatic retry or physical motion
is authorized by preparation.

Add optional paired command-position vectors to Envelope, with legacy fallback
to the measured bounds when both are absent/null. Manufacturer torque table,
actual position/speed/load/attitude, held arm, expiry/health/ACK/slew guards remain.
No clipping, policy/weights/gain/order/history/rate/startup change. Arms retain
their existing command ranges. Broad virtual targets can be outside mechanical
range without allowing measured angles there. Firmware retains its own desired
position/load saturation; see [the contract explanation](../../docs/PD_TARGET_BOUNDS.md).

## Proposed configuration

`local/relic-wide-pd-targets-20261012-001/` freezes the original walking candidate,
proposed hardware envelope, simulation envelope and operator request. Same 0.30
m/s tapered maximum, 0.10 m/s² acceleration, 3 s zero-speed hold, 20 s walking cap,
0.75 m nominal goal in the current post-native-stand heading, no fade, zero initial
actions/ACK history. Short distance remains an accepted proof of concept, not
goal completion. Body 0.40 m/s, hips 6 rad/s, knees/shared SDK 12 rad/s remain the
previously approved operating caps; measured positions and manufacturer loads
stay unchanged. Leg PD-offset cap 2.0 rad and virtual bounds are the new proposal.
Arm/harness observation mode retains commanded stow and all torque checks.

## Validation protocol

Unit tests cover finite/paired/ordered vectors, unchanged legacy behavior,
unclipped accepted virtual target outside a measured bound, unchanged measured
guard, retained tracking/PD torque/slew, and evaluating manufacturer torque at
measured knee angle despite a virtual target outside the table.

`audit_hardware.py` verifies failed raw-record integrity and replays all 1135
issued command/state samples through the proposed guard using recorded finish
timestamps as a legacy timing proxy. The rejected prediction passes numeric
guards only: its exact slew/scheduling interval cannot be reconstructed because
the previous command's computation-start timestamp was not recorded. It preserves
recorded policy age/inference and previous-target measured tracking, checks targets
are not clipped, and does not claim new physical motion or exact transport timing.
New command records include `guard_monotonic_s`, preserved in compact recordings,
for exact future start-to-start audits. Raw inputs remain ignored; public results
contain hashes/counts.

The preserved validation attempt `runs/relic-wide-pd-targets-validation-20261012-001`
passed 470 tests/Ruff/demo, then stopped before simulations in the historical
audit. It compared the rejected policy START to the previous command FINISH:
0.459933 rad / 4.591180 ms = 100.1776 rad/s against 100 rad/s. Only 8.153 microseconds
separate that mixed-clock interval from the minimum allowed interval; the previous
computation duration is unknown, so this cannot establish a real slew violation
or an exact pass. The corrected audit labels this timing gap explicitly. The live
100 rad/s slew guard is unchanged; fresh guarded simulation/benchmark gates still
apply. No bounds were widened in response to this audit failure.

Run full tests/Ruff/demo, the recorded audit, both fresh 20 s guarded walking
rollouts and both 60 s guarded standing rollouts under the new target bounds.
Standing runs provide the full 60 s regression horizon without disabling the
20 s walking timeout. They use the same frozen graph/gains/capture and zero base
request. Walking runs use the exact proposed walking candidate, original captured
standing state/height/zero velocities/history, seed 101 and 100 discarded inference
calls. MuJoCo uses no-slip 10; Isaac uses native TGS/CPU physics. Neither simulation
claims a calibrated physical model or exact hardware response to firmware clipping.

Run a fresh 20 s paced host core/logger/SDK-encoding benchmark with ideal clocks/
ACKs; retain actual response/deadline gaps and misses. No robot command network
test is implied. Preserve any new guard/nonfinite/parity/timing failure with no
automatic retry or additional bound widening. Renew source/config-bound review
and collect a fresh motors-off preflight before presenting the exact final
approval. No E-stop bridge/setup/fault clearing/lease takeover.

Current host `cs-u-rpm-dt-03`, locked UV Python 3.12 CPU ORT environment and native
Isaac overlay; GPU 0 for Kit only, no GPU 1 or paid compute. One Luna operational
owner per primary-authored bounded campaign, durable exact commands/logs/hashes,
no retries. Keep raw robot identities/endpoints/captures/credentials ignored.

## Results and decision

Fresh campaign `runs/relic-wide-pd-targets-validation-20261012-002` ran from clean
commit `cc44fb88c81d9f5f503c47e3c29133b5e5161f3b`, preserving attempt 001. All 470
tests, Ruff and demo passed. The corrected historical audit accepted the 1135
issued commands and rejected prediction's numeric checks, with rejected-step
timing explicitly unverified. Full guarded simulation results:

| Backend/task | Duration | Relative forward travel | Stable final window | Falls/guard failures |
|---|---:|---:|---|---|
| MuJoCo walking | 20 s | 0.689392 m | passed | none |
| Isaac Lab walking | 20 s | 0.687834 m | passed | none |
| MuJoCo standing | 60 s | zero commanded velocity | passed | none |
| Isaac Lab standing | 60 s | zero commanded velocity | passed | none |

Both walks retain `walking_completed=false`: approximately 6 cm short is the
previously accepted proof-of-concept behavior, not exact goal completion. Graph
raw-action parity maximum was 1.1921e-6; gains/weights/history remain unchanged.

The 20-second host-only probe produced 3998 commands, averaged 199.934 Hz and
missed two nominal releases. Two production responses exceeded 5 ms. Maximum core
computation was **5.2719 ms**, maximum production 5.4186 ms, maximum release-to-finish
5.7715 ms, and maximum command gap 10.1077 ms. Inference maximum was 0.4836 ms.
The first slow core interval had inference 0.2786 ms; its cause outside inference
is not isolated. Another slow interval spent 4.6370 ms in logging/encoding; do not
infer that either interval identifies an OS, GIL or storage cause.

The benchmark previously displayed hardcoded 20 ms inference/gap budgets. The
actual selected envelope has **5 ms inference/core computation** and **15 ms gap**
limits. Reanalysis of the retained timing trace against those limits finds one
core computation miss, zero inference misses and zero gap misses. The benchmark
now uses configured limits and reports computation/inference/gap gates separately;
two focused regression tests verify the budget distinction and missing intervals.
After this reporting repair, the full suite passed **472 tests**, Ruff and the
offline Exploy demo. Simulator execution remains pinned to `cc44fb8`; the later
repair changes benchmark reporting and records, not the executed runtime/guards.
No timing run was repeated to select a better outcome, and original artifacts
are immutable. The supervisor completed its stages and prepared a draft bundle;
that process completion does **not** establish that the timing gate passed.

**Decision:** the wider virtual targets and 2.0 rad PD-offset proposal pass these
guarded simulations. The timing gate failed, so no fresh physical activation
approval was requested, no hardware evidence was promoted and no new robot access
or motion occurred. The current live computation watchdog could stop another
attempt despite accepting the wider targets. Address timing before a new physical
trial, retaining these misses and collecting a fresh motors-off preflight and
exact operator approval at that point. Torque/actual-angle limits still take
precedence over the wider targets; a 2.0 rad offset is not a guarantee of acceptance.

[Sanitized results and hashes](validation-summary.json) exclude raw identities,
endpoints and world positions. Analysis is separate from execution: the summary
binds the executed commit, raw completion/result/timing hashes and analysis-tool
hashes. Reproduce from the private registered campaign with:

```bash
uv run --no-sync python records/relic-wide-pd-targets-20261012/summarize_validation.py \
  --run runs/relic-wide-pd-targets-validation-20261012-002 \
  --envelope local/relic-wide-pd-targets-20261012-001/envelope.proposed.json \
  --output local/reproduced-wide-pd-targets-summary.json
```

Raw inputs/configurations, captured height, commands, environment and logs remain
in the ignored registered run/proposal directories. Public clones can reproduce
the contract tests and offline tools; the physical-capture comparison requires
those private hash-bound inputs and is not claimed to run from public files alone.

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
issued command/state samples plus the rejected prediction through the proposed
guard. It preserves actual recorded policy age/inference where present, checks
targets are not clipped, and does not claim further physical movement or new
transport timing. Raw inputs remain ignored; public results contain hashes/counts.

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

# Smaller physical joint-API crouch

## Question and protocol

Does reducing the original +5/-10 degree hip/knee reference to +1/-2 degrees
allow the same ten-second diagnostic to complete? The user explicitly requested
this smaller physical attempt. One run, no retry, no ReLIC, no random seed.
Runtime commit: 800b49e7c2b0ef42d902a564545a7e5ac4dedc99, clean.
Settings: configs/joint-api-crouch-small.json; locked PEP 723 UV environment.
Adam operated Spot; Minghao held the independent tablet E-stop, with the previously
confirmed fall-arrest rig. CPU execution on cs-u-rpm-dt-03.

The exact INFO system payload.fault/code 9 was explicitly accepted and logged.
Other fault, tablet, timing, joint and body guards remained active. Fixed native
load feedforward and gains were unchanged. The protocol was 2 s hold, 3 s lower,
1 s crouch, 3 s return, 1 s hold at 200 Hz.

## Observations

The trial aborted on `measured joint position limit` after 1,126 recorded commands;
last sampled trial time was 5.6175 s. The saved rejected state identifies front-right
hip pitch at 0.4994475 rad, below its configured lower bound of 0.5 rad.
The last target was 0.7410685 rad: a 13.844 degree target-to-rejected-state gap.
This was a configured operating-envelope boundary, not evidence of a mechanical
hard stop. Maximum sampled leg tracking error was 15.201 degrees.
Rejected-state roll/pitch were -2.654/-6.158 degrees, within the 10 degree guard.

Maximum recorded command gap was 6.373 ms; maximum sampled state age 6.917 ms;
maximum reference computation time 0.507 ms. No timing guard was reported as the
stop cause. These are partial-run observations, not ten-second qualification.

Automatic safe-power-off completed in 6.022 s, and explicit motor-OFF plus lease
return were confirmed. This exceeds the existing 2 s ReLIC shutdown budget.
The separate read-only post-inspection passed with motors off and **zero active
faults**. No faults were cleared; no E-stop writes or supplemental motion issued.

## Evidence and interpretation

Raw run: runs/joint-api-crouch-small-20261009-001. COMPLETE.json artifact hashes
were independently verified. It contains run/source/input hashes, compressed
joint targets/states/loads, PD terms, timing, baseline, events and rejected state.
Sanitized summary: small-physical-result.json beside this report.
The supervisor and post-inspection directory names mistakenly contain 20260909;
their recorded execution timestamps are October 9. Preserve the original paths:
runs/joint-api-crouch-small-supervisor-20260909-001 and
runs/crouch-small-post-inspection-20260909-001.

The preceding offline validation passed 395 tests, Ruff, demo and the smaller
plan. Its wrapper copied stale commit metadata; the plan independently recorded
the actual clean runtime above. The correction and originals are preserved in
runs/crouch-small-validation-20260909-001/METADATA_CORRECTION.json.

Reducing reference amplitude did not resolve tracking drift. The measured hip
motion substantially exceeded the requested one-degree change. Fixed feedforward
and the absence of body-balance feedback remain hypotheses, not proven causes.
Decision: incomplete diagnostic; no automatic repeat or limit expansion. Diagnose
tracking/support behavior and shutdown latency before claiming deployment readiness.

## What transfers to ReLIC deployment

The diagnostic Kp/Kd arrays exactly match the frozen manufacturer candidate:
leg Kp=60 N m/rad, Kd=1.5 N m s/rad. ReLIC updates at 50 Hz and streams at
200 Hz; the diagnostic observed 200.004 Hz over its recorded interval. Command
gap p99/max was 5.096/6.373 ms, state age p99/max 5.897/6.917 ms, reference
compute p99/max 0.348/0.507 ms. Host-observed ACK delay reconstructed using
increasing acknowledged keys was p99/max 9.711/9.764 ms. This is not isolated
network latency. Inference and production ReLIC logging were not exercised.

During active hold from 0.2 to 2 s, maximum leg tracking error was 1.314 degrees;
maximum absolute roll/pitch was 0.706/0.126 degrees. Across the subsequent crouch
samples, absolute measured-load minus contemporaneous PD-plus-feedforward estimate
had median 0.443 and p95 2.085 N m over the 12 leg joints. At the final recorded
front-right hip sample, P=14.462, D=0.283, feedforward=5.969, total=20.714 N m;
measured load was 20.737 N m. These unaligned sampled comparisons support the
expected PD/load convention but do not establish a calibrated actuator model.

The candidate ReLIC policy phase explicitly uses zero feedforward. Under a
quasistatic approximation with Kp=60, the captured native knee loads of about
21--24 N m correspond to positive position offsets of about 20--23 degrees;
hip-pitch loads correspond to about 5--6 degrees. Therefore large policy target
minus measured-angle offsets can be physically meaningful support commands.
This does not prove any particular ReLIC output is correct: assess its signed
PD torque, damping, operating limits and subsequent closed-loop behavior.
Do not carry the diagnostic's native-load feedforward into the policy phase
without revisiting the training/deployment contract.

The configured 0.5 rad front-right hip bound stopped this test by only 0.000553 rad
(0.0317 degrees); it is not a manufacturer mechanical-stop measurement. The
boundary and posture envelope need to be assessed against intended policy motion,
separately from the substantial tracking drift. Fresh checks, actual policy
handover/inference/ACK-history/recording under load, and reliable bounded shutdown
remain to be verified physically. The successful 6.02 s shutdown does not satisfy
the current 2 s budget, and the preceding attempt did not confirm shutdown.

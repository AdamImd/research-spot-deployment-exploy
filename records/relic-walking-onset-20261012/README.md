# Walking onset and PD tracking review — 2026-10-12

All work was offline. Adam explicitly instructed: **Do not deploy on real
hardware.** No robot connection, lease, motor command, E-stop write or physical
walking deployment occurred. No candidate completed the requested 0.75 m under
its simulation monitoring profile. The hardware envelope remains unchanged.

## Question, hypothesis and protocol

Can separating direct policy startup from the forward request, and changing the
request ramp, produce gait within the existing position/speed/torque bounds?
Does the tracking-error stop indicate an ordinary loaded-joint PD offset?

The first comparison changes the initial zero-velocity hold from 1 to 3 s with
0.14 m/s maximum request and 0.025 m/s² acceleration, then brackets 0.13 m/s.
The next stage compares 0.135 m/s and a faster 0.1 m/s² request ramp after 3 s.
A final narrow bracket tests 0.13 and 0.1325 m/s with that faster ramp. All use
0.75 m initial-body-frame travel, 20 s walking deadline, 1 s final stationary
hold, direct policy startup, zero initial raw actions and ideal simulated ACKs.
The 3 s hold delays only the base-velocity request; it is not a joint fade-in.

Weights, graph, action scaling, arm targets and gains are unchanged: leg Kp=60,
Kd=1.5, zero feedforward. The original numeric monitoring profile keeps hip
speeds at 4 rad/s, knees at 8 rad/s, manufacturer torque checks and all other
guards. These speed caps are operating bounds, not manufacturer speed ratings.
Use the same successful physical standing capture from October 9, height
0.523164 m with simulated velocities zero. Seed 101; 50 Hz actor, 200 Hz commands
and physics. MuJoCo no-slip iterations 10, CPU policy/physics. The archived Isaac
0.14 m/s record is analyzed separately; its dynamics were not rerun.

Every new profile is separately frozen under local/relic-walking-onset-20261012-
{001,002,003} with hypothesis/configuration/seed/capture hashes. The simulation
envelope has explicit simulation scope. Research is deterministic and uses a
single standing capture; it does not establish robustness across poses, robot
calibration or environments. Private robot-specific inputs are not published.

## Observed comparisons

Travel values below are the last accepted trajectory tick; the exact failure
state can be one physics tick later. Foot travel is the mean forward displacement
of four foot centers, a geometric proxy rather than contact-force evidence.

| Request m/s | Hold s | Ramp m/s² | Stop s | Body m | Mean feet m | Outcome |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0.14 | 1 | 0.025 | 8.065 | 0.2532 | 0.0452 | fr_kn -8.3001 rad/s |
| 0.14 | 3 | 0.025 | 12.900 | 0.2714 | 0.1535 | fl_hy -4.4118 rad/s |
| 0.13 | 3 | 0.025 | 17.130 | 0.2485 | 0.0243 | Progress stalled |
| 0.135 | 3 | 0.025 | 18.500 | 0.2520 | 0.0254 | Progress stalled |
| 0.14 | 3 | 0.1 | 6.230 | 0.2503 | 0.0595 | fr_kn -8.1475 rad/s |
| 0.135 | 3 | 0.1 | 6.740 | 0.2748 | 0.1684 | fl_hy -4.1261 rad/s |
| 0.13 | 3 | 0.1 | 6.845 | 0.2561 | 0.0571 | fr_kn -9.5573 rad/s |
| 0.1325 | 3 | 0.1 | 9.700 | 0.3306 | 0.0994 | fl_hy target offset -0.40225 rad exceeds 0.4 |

The initial 3 s zero-request interval already moves the body forward 0.0872 m.
Body travel alone therefore overstates walking progress when interpreted as
stepping: the 0.13 slow case reaches 0.2485 m while mean foot advance is only
0.0243 m. Clearance and displacement are not proof of loaded support, but they
make this difference visible. The archived Isaac comparison likewise records
0.2547 m body travel, 0.0517 m mean foot advance and a knee-speed stop at 9.085 s.

All independent native actor comparisons pass; maximum raw-action differences
remain below 1e-6. Position-derived speeds agree with the simulator's velocity
field, including exact rejected states. This is observed simulated motion, not
an inference/joint-order discrepancy. Timing results remain in result-summary;
no physical SDK transport performance is established by these faster-than-real-
time, ideal-ACK simulations.

## Approved simulation-only tracking comparison

At the 0.1325 fast case's 9.70 s stop, fl_hy is 1.129945 rad and the rejected
prediction requests 0.727692 rad: offset -0.402252 rad versus a 0.4 cap. Measured
velocity is -0.005189 rad/s. Predicted PD torque is -24.1274 Nm and preceding
measured simulated load is -23.9740 Nm, below the 44.88 Nm hip torque bound.
This is consistent with a load-producing PD offset rather than rapid closing
of that particular joint gap. The rejected prediction was not issued.

The draft proposes **only fl_hy tracking 0.4 → 0.5 rad**, keeping every other
guard unchanged. A conservative algebraic bound under the 4 rad/s speed cap is
60*0.5 + 1.5*4 = 36 Nm. This calculation does not establish safe hardware tracking
tolerance, thermal capability or contact-force behavior. Adam explicitly approved
evaluating this separate draft in simulation and prohibited real deployment.
The original unapproved proposal is preserved alongside a separate approval;
neither changes or authorizes the hardware envelope.

The approved comparison gets past the tracking stop, then stops at 9.97 s on
fr_kn velocity -8.64156 rad/s versus 8. Body travel is 0.3408 m, mean foot travel
0.1111 m. The rejected-step position derivative independently gives -8.64158
rad/s. Both walking completion and final stability remain false. Raising this
single tracking threshold does not resolve the gait dynamics failure.

## Artifacts, environment and decision

Simulation source: f7b414a6a55c124bd5a08f66da300d28ef69ccec. Final diagnostic,
plot and replay acceptance: 799a0824e29f2ba276660660f95e1f116c3f0311.
445 tests, Ruff and offline Exploy demo pass. Host cs-u-rpm-dt-03, UV locked
Python 3.12 with simulation/visualization groups, ONNX Runtime 1.23.2, NumPy
2.2.6, MuJoCo 3.3.7, SDK packages 5.1.1. $0; CPU two threads; GPU0 only for
kinematic video rendering. No automatic recovery, new seeds or guard changes
beyond the explicitly approved one-field simulation comparison.

- runs/relic-walking-onset-20261012-{001,002,003}: all original-profile comparisons.
- runs/relic-walking-tracking-20261012-001: approved simulation tracking comparison,
  plots and 50 fps visual-mesh replay of its failure.
- runs/relic-walking-review-artifacts-20261012-001: 445-test acceptance and updated
  analyses for stepping/PD-offset/proposal cases; native 200 fps stepping replay.
- runs/relic-walking-slow-replay-20261012-002: verified 8× slow playback and previews.
- local/relic-walking-tracking-review-20261012-001: draft, approval and frozen inputs.

The representative 0.135 m/s fast stepping clip retains 1,348 frames at native
200 fps, 6.74 s; its 25 fps slow copy retains every frame and lasts 53.92 s.
Both decode and source/artifact hashes verify. Correct yellow/black visual
materials and all authored triangles are preserved. Playback and plots reuse
recorded states and never advance dynamics. Native and approved-proposal clips
show different experiments; their source stop labels remain intact. A first
slow-copy command failed because -r25 and passthrough mode contradicted each
other; that failure is preserved, and a separate corrected CFR conversion passed.

Recompute the sanitized summary from the private recordings:

```bash
uv run --no-sync python records/relic-walking-onset-20261012/summarize.py
```

The new tools/analyze_relic_walk.py reports feet/body travel, PD offsets, exact
velocity/tracking violations and rejected versus issued targets, with integrity-
bound overview/onset plots. Research records include code/config/data hashes,
packages, seed, raw paths, timing, failures and user scope.

Decision: preserve the original hardware contract and keep walking preparation
in simulation. No passing candidate exists under these profiles, so the planned
new Isaac and live-core timing promotion stages are not launched. Further work
needs an explicit controller/actuator-model investigation rather than another
small speed/ramp search or automatic guard relaxation. Normal-force/contact
instrumentation and a separately reviewed command-shaping experiment are possible
next comparisons; no modified output processing is implemented or qualified here.

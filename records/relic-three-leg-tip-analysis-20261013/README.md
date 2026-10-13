# Why the front-left transition pitched: recorded-data analysis

## Question and protocol

Why did the recorded 2 rad/s angular-cap physical trial tip during lift entry?
Compare its 4.98–5.48 s samples with the existing successful recorded-pose MuJoCo
rehearsal. This is analysis of saved data, with no new physics, actor inference,
robot connection or motion. No controller/limit change is made.

Analysis prepared from repository commit `2f8565a` with the new script and record
tracked in the subsequent analysis commit.

Hardware source: `6b5c46a969286cc493c1a567f5964f65192b526d`.
Prior simulation source: `96501f8644cce15fa911c8d97dec3debbbece6f8`.
Analysis uses the existing locked UV environment, NumPy and the pinned support URDF.
No randomness is used. Run from repository root:

```sh
uv run --no-sync python records/relic-three-leg-tip-analysis-20261013/analyze.py
```

Raw input paths and SHA-256 hashes, model hash, named metrics and sampled results
are in [analysis.json](analysis.json). Raw files remain private ignored `runs/` and
`local/`; reproduction requires those retained artifacts. No world coordinates,
robot serials or network endpoints are published. The script projects URDF mass-
weighted COM and foot-link locations into the gravity-horizontal plane using each
measured orientation. It computes barycentric coordinates for FR/HL/HR and signed
margin to their FR–HL diagonal; positive is the remaining triangle's side. This is
an approximate quasi-static diagnostic, not measured contact forces or stability
qualification. PD estimates are kp*(target-position)-kd*velocity; reported joint
loads are separate. No contact/no-slip or harness-force assumption is proven.

## Direct observations and code mechanism

Four-foot standing was steady immediately before entry: pitch -0.73 degrees,
angular speed 0.008 rad/s. At 5 s, the graph leg-command observation changes from
12 zeros to the measured front-left joint pose. The graph detects nonzero command
and replaces the front-left actor target with that IK pose immediately. The
Cartesian lift trajectory is smooth, but this support-mode observation is discrete.
The target limiter prevents a joint-target jump; it does not interpolate the
observation or preserve the supporting PD offset.

At entry, FL actual knee is -1.688 rad, while its four-foot virtual target is
-1.331 rad. The approximately +0.357 rad supporting offset becomes a proposed zero
offset when IK begins at actual pose. Applied targets converge at 1 rad/s.
Estimated knee PD effort declines from 21.42 N·m to 7.29 N·m by 5.30 s; reported
knee load likewise falls from 18.68 to 7.85 N·m. These are joint torques, not foot
normal force. The other support legs' requested targets differ from applied
limited targets by up to 1.022 rad at the 5.10 s sample. Their balance response
is therefore materially modified by the all-leg limiter.

The intended IK foot point moves only approximately 1.51 mm during the recorded
first 0.48 s of the six-second lift. Actual body-frame IK tracking error nevertheless
reaches 13.14 cm. Thus a large commanded Cartesian foot lift is not the immediate
cause of the observed motion.

| Time after policy start | Hardware pitch | FL estimated knee PD torque | Max other-leg target/filter difference |
|---|---:|---:|---:|
| 4.98 s | -0.73° | 21.42 N·m | 0 rad |
| 5.10 s | -0.77° | 15.43 N·m | 1.022 rad |
| 5.20 s | -0.50° | 10.17 N·m | 0.819 rad |
| 5.30 s | 1.33° | 7.29 N·m | 0.146 rad |
| 5.40 s | 5.96° | 9.31 N·m | 0.745 rad |
| 5.48 s | 10.00° | 12.58 N·m | 0.838 rad |

The following guard state crossed pitch10.284degrees; angular speed1.061rad/s
remained under2rad/s. The guard detected the developing motion; it did not cause
its onset. Shutdown and postcheck are documented in the hardware trial record.

## Why the earlier simulation looked better

The successful simulation used an earlier recorded physical pose, not this trial's
new settled stance. At entry, its model COM is only 0.9 cm outside the remaining
support triangle; this hardware stance is 4.5 cm outside, toward the removed-foot
side. The simulator moves inside by5.40s (margin+0.6cm) and5.48s (+2.1cm), while
hardware remains outside (-3.9/-3.5cm). Simulator pitch reaches6.14degrees at5.40s
then drops to4.60degrees at5.48s; hardware continues to10degrees. Both show a
transition transient, but only the simulated stance recovers in this interval.
These comparisons use the same URDF mass model; real payload/harness forces and
actuator/contact dynamics are not reconstructed. Earlier simulation root placement
was model-derived and its velocities/history initialized separately; it was not an
exact replay of the new trial's settled stance and external forces.

## Interpretation and next decision

The leading supported mechanism is premature loss of FL supporting PD effort
while the COM is outside the remaining three-foot support triangle, together with
limited application of the actor's other-leg corrective targets. A smooth foot
trajectory and bounded joint-target increments do not guarantee smooth weight
transfer. The discrete leg-command observation is an additional potential trigger
for the large actor response. We cannot uniquely separate these contributions or
prove actual contact loss using this recording: foot contact forces and harness
loads are absent. Do not claim a completed tripod or physical fall from a10degree
attitude stop alone.

Before another hardware attempt, evaluate an actual load-transfer handoff from
this trial's settled state: retain supporting PD effort until the body/support
condition is established, and independently assess whether the all-leg limiter
suppresses balance corrections. Use one bounded offline comparison at a time;
do not restore the failed dual-actor/support-lean method blindly. Reproduce the
new stance and record foot forces/contact state and torso motion. Raising attitude
limits alone would allow this developing transient to continue; it would not fix
the demonstrated handoff mismatch. No new limit change or physical trial is
performed or authorized by this analysis.

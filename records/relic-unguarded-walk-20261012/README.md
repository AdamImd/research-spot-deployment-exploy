# Unguarded ReLIC walking diagnostic — 2026-10-12

## Question, authorization and comparison

Adam requested: “Run in sim with no guards to see what the limits are.”
His immediately preceding prohibition on real hardware remains in force.
Question: do the software stops truncate otherwise stable locomotion, and what
joint/PD demands occur if the controller continues? Compare both simulators,
including a MuJoCo cell matched to the preceding guarded tracking comparison.
This is diagnostic evidence; it does not qualify or change hardware limits.

## Registered protocol

Frozen clean simulation/tool commit: **8f11bcd689d31c1c2397aa8b9bce1b913336435d**.
All cells use seed 101, 60 simulated seconds, captured standing joints and body
height approximately 0.523164 m, zero simulated initial velocity, direct startup,
zero previous raw actions followed by ideal one-tick acknowledgement updates.
No fade-in, graph/weight/action-scaling/gain changes. Nine-input Exploy policy
SHA256 `1940df208bf9e30e6134eeb71d1c4e67d3a0d66778726507d4886c62eae662e8`;
leg Kp=60, Kd=1.5, feedforward=0; arm targets held. Policy 50 Hz, commands/physics
200 Hz. Body-frame forward distance goal 0.75 m; request max 0.1325 m/s,
acceleration 0.1 m/s², initial hold 3 s, stopping tolerance 0.02 m, final hold 1 s.
Preserve the distance-dependent command taper, including the request continuing
if the robot stalls. A successful recording horizon does not mean goal completion.

`--no-guards` uses an isolated simulation-only rollout/command implementation.
The shared hardware Guard and hardware CLI have no bypass setting. Envelope,
handover, inference deadline, walking timeout/stall/deviation operating stops
are disabled. A first fall proxy (height <0.25 m or tilt >0.9 rad) is logged;
physics continues to the recording horizon. Finite data, shapes, artifact/parity,
ACK/history and held-arm invariants remain. Model actuator saturation, torque-speed
behavior and joint constraints remain physical model properties. Both simulators
run on CPU; Isaac/render uses GPU0 only. No Wi-Fi/SDK transport is simulated.
The original simulation envelope remains a monitoring reference, including fl_hy
tracking 0.4 rad, hips speed 4 rad/s and knees 8 rad/s. These speed caps are operating
bounds, not manufacturer speed ratings. The result's per-joint `reference_bound`
for torque is the envelope's scalar load_max, not the position-dependent knee table.

Private frozen inputs: `local/relic-unguarded-walk-20261012-001/{candidate,
reference-simulation-envelope.json,protocol.json,protocol-amendment.json}`;
standing capture `local/relic-walk-inputs-20261009-001/capture`. Protocol and source,
config, capture, data and artifact hashes are recorded privately. Raw hardware
captures and identity records are excluded from the public repository.

## Observed results

| Cell | MuJoCo no-slip iterations / prewarm | Fall proxy | Final forward travel | Final standing window | Peak leg speed | Peak requested/applied leg torque |
|---|---|---|---|---|---|---|
| MuJoCo default friction | 0 / 100 | 8.995 s | 0.2461 m | fails | 26.8559 rad/s | 90.2235 / 69.1408 Nm |
| Isaac Lab | not applicable / 100 | none | 0.6768 m | passes | 10.0758 rad/s | 51.2639 / 51.2639 Nm |
| MuJoCo matched friction | 10 / 0 | none | 0.6696 m | passes | 16.8958 rad/s | 49.9125 / 49.9125 Nm |

All three recorded the full 60 s; **none completed the 0.75 m goal**. MuJoCo's first
cell launched before the primary's amendment arrived. Its full recording was
preserved. The separate matched cell was then authorized and launched without
retrying/overwriting the first. Prewarm outputs are discarded and never initialize
policy history; differences in prewarm and friction remain explicit in the table.
This is one seed and one requested speed, not a robustness sweep or a safe-limit search.

In the matched MuJoCo cell, all 1,994 accepted samples through 9.965 s exactly
match the preceding guarded cell's measured q/dq, targets, requested PD torque and
body position. The guarded cell stopped at 9.97 s on fr_kn speed. Continuing gives
fr_kn **−16.895765 rad/s at 9.985 s**; the preceding 5 ms position difference gives
**−16.895747 rad/s**, confirming the spike in the simulated joint trajectory.
fr_kn exceeds its 8 rad/s operating bound for six sampled ticks (30 ms total
sampling weight; this does not establish continuous-time crossing duration).
Peak hip-pitch speed is **6.93380 rad/s**. Peak target offsets: hip abduction
0.40280 rad, hip pitch **0.47451 rad**, knee **0.83073 rad**. Peak requested hip
pitch torque is **28.6224 Nm**; the overall 49.9125 Nm peak belongs to the knee.
Global maxima of requested and applied torque need not occur at the same instant
or on the same joint. Applied torque in each row precedes its newly issued target.

The matched MuJoCo final 10 s has mean height 0.481919 m, maximum tilt 1.6881°,
planar speed p95 2.38e-7 m/s and joint-speed p95 1.26e-6 rad/s. It is stationary
with about 0.08044 m remaining and a nonzero forward request, not a completed walk.
Isaac likewise becomes stationary with about 0.07322 m remaining; final request
0.05858 m/s and planar speed p95 6.21e-5 m/s. The final standing-window checks are
post-run evaluation metrics, not active guards or a walking qualification.

Default-friction MuJoCo's maximum 90.2235 Nm hip request occurs after the fall;
its peak before the first fall is 69.1408 Nm on a knee. Post-fall peaks must not be
mistaken for upright operating demand. This model clips hip effort to 45 Nm,
which explains an applied hip load much smaller than the 90 Nm request. The
simulated saturation is not proof of physical hardware equivalence.

## Validation and artifacts

450 tests pass, Ruff and the offline Exploy demo pass at the frozen commit.
Native parity remains within thresholds: max raw action error 4.77e-6
(dimensionless), max target error 9.54e-7 rad. Locked UV Python 3.12.3,
NumPy 2.2.6, ONNX Runtime 1.23.2, MuJoCo 3.3.7; existing Python 3.11 Isaac overlay
`/home/adam/repos/research/research-spot-deployment-exploy/.venv-exploy-export`.
Matplotlib 3.11.2, Pillow 12.3.0. Host cs-u-rpm-dt-03, two CPU threads; $0.
Durable tmux supervisors, 120 s MuJoCo / 240 s Isaac / 300 s replay caps,
no automatic retries. All simulation/render/decode exits 0; integrity checks pass.
A gait fall/stall is preserved as a valid diagnostic outcome.

- `runs/relic-unguarded-walk-20261012-001`: tests, MuJoCo default and Isaac,
  configurations, events, trajectories, timing, hashes, source packages and videos.
- `runs/relic-unguarded-walk-20261012-002`: amended matched MuJoCo and replay.
- Each `*-replay` contains `replay.mp4`, `dynamics.png/pdf`, completion hashes and
  all original authored visual meshes/materials. Videos are **kinematic playback**,
  3,001 frames at 50 fps (60.02 s encoded), with no new dynamics.
- `result-summary.json`: sanitized metrics, joint peaks/crossings/ranges, foot
  displacement, independent speed check and exact guarded-prefix comparison.

Rebuild the sanitized report from preserved private recordings:

```bash
uv run --no-sync python records/relic-unguarded-walk-20261012/summarize.py
```

Reproduce the matched MuJoCo physics (fresh output; private bound inputs required):

```bash
OMP_NUM_THREADS=2 uv run --no-sync python tools/simulate_relic.py \
  --simulation-source simulation \
  --capture local/relic-walk-inputs-20261009-001/capture \
  --manifest local/relic-unguarded-walk-20261012-001/candidate/manifest.json \
  --envelope local/relic-unguarded-walk-20261012-001/reference-simulation-envelope.json \
  --backend mujoco --duration 60 --prewarm-inference 0 \
  --mujoco-noslip-iterations 10 --no-guards --output runs/NEW-matched-unguarded
```

## Interpretation and decision

Observed: the two upright continuations show that crossing the existing speed
bounds does not necessarily produce a fall in simulation. Their observed demand
is above those bounds. The default MuJoCo physics result is a counterexample to
assuming guard removal alone resolves the behavior. The confirmed 5 ms speed spike
is joint motion in the simulator, not merely a large PD target offset.
Interpretation: friction/contact behavior and low-speed command tracking need
investigation; the evidence does not isolate a hardware cause. Both upright runs
stall during the tapered request, so the distance goal remains unresolved.
Decision: retain this offline diagnostic for controller/contact analysis; do not
promote the policy, alter hardware limits, access Spot or deploy on hardware.

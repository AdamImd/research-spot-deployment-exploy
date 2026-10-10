# Walking and front-left demo preparation — 2026-10-09

The offline sequence and recording tools are implemented. Neither walking nor
the three-leg transition passed the existing numeric guards. No robot access,
lease acquisition, motor commands or E-stop writes occurred.

## Protocol

Question: can slower velocity-request ramps avoid the walking transient without
changing weights, gains or limits, and can a smooth front-left lift/hold/return
succeed under that contract? Adam chose front-left lift, hold, then return.

Seed 101; 50 Hz inference / 200 Hz commands; direct policy startup, zero initial
raw-action history and simulated acknowledgment updates. Initialize from the
last successful physical standing capture, height 0.523164 m, with simulated
velocities zero. The hardware-derived numeric envelope is copied with explicit
simulation scope and a 60 s duration cap. Hip velocity bounds remain 4 rad/s,
knees 8 rad/s. All leg gains remain Kp=60, Kd=1.5 with zero feedforward. Arm
targets, gains and torque limits are unchanged; the requested harness exception
observes arm motion. These failures did not arise from the arm-motion exception.

Walking requests 0.75 m measured in the initial robot frame, acceleration
0.025 m/s², one-second initial/final holds and a 20 s distance deadline. New
progress monitoring stops a request producing less than 5 mm net forward
progress over five seconds. Initial speeds are 0.125 and 0.15 m/s; a subsequent
bracket diagnostic tests 0.14 m/s with all other parameters fixed.

configs/relic-three-leg-front-left.json specifies five-second initial standing,
four-second lift to [0.12, 1.10, -1.90] rad, five-second hold, four-second return
to the pose latched at lift onset, and final standing. Base velocity stays zero.
The raw released actor follows native selected-leg observation/target semantics.
The nine-input Exploy graph embeds zero leg commands and cannot run this demo.
Only the explicit selected-leg lift/lower trajectory uses quintic interpolation;
policy startup has no fade-in.

Host cs-u-rpm-dt-03; CPU physics/inference, GPU0 rendering only; $0. MuJoCo
no-slip iterations 10. Isaac uses the existing isolated Python 3.11 lab environment
with current-repository imports. UV Python 3.12 uses locked simulation and
visualization groups. SDK 5.1.1, NumPy 2.2.6 and ONNX Runtime 1.23.2. Full
commands, environments and source/input hashes are preserved in the raw runs.
Robot-specific capture inputs remain private; the public record is not a fully
portable copy of the dataset. No automatic retries or guard relaxation occurred.

## Observed results

| Test | Stop time | Forward travel | Outcome |
| --- | ---: | ---: | --- |
| MuJoCo zero-command standing | 60 s | — | Completed; stable final window |
| MuJoCo walking 0.125 m/s | 14.175 s | 0.2512 m | Progress stalled; final speed 0.00019 m/s |
| MuJoCo walking 0.15 m/s | 8.025 s | 0.2660 m | fr_kn -11.7875 rad/s exceeds 8 |
| MuJoCo walking 0.14 m/s | 8.065 s | 0.2532 m | fr_kn -8.3001 rad/s exceeds 8 |
| Isaac walking 0.14 m/s | 9.085 s | 0.2547 m | fr_kn -8.1431 rad/s exceeds 8 |
| MuJoCo front-left demo | 5.020 s | — | Body angular speed limit at mode activation |
| Isaac front-left demo | 5.000 s | — | fr_hx target 0.53871 rad exceeds +0.4 rad |

All independent native observation/action/target comparisons passed, with raw
action errors below 1e-6. The demo never reached hold/return; zero hold samples
is a failed demo, not support evidence. Completed recording markers mean intact
data collection, not successful controllers or hardware qualification.

At 5 s, the front-left observation triple switches from zero to the current
absolute joint pose. The selected-leg target starts at that measured pose, but
the actor immediately changes supporting-leg targets. Isaac's fr_hx target
changes by +0.46007 rad in that update and violates its bound. MuJoCo's first
targets stay within position limits, then body angular speed exceeds 0.5 rad/s
20 ms later. This establishes a mode-activation transient despite the smooth
lift. A smaller final pose or longer lift duration has not been shown to fix it.

Inference: ordering and inference parity are not the observed failure. Slowing
the walking ramp alone did not resolve stall/stepping behavior. Interpolating
the selected leg alone does not smooth the actor's supporting-leg response to
the mode input. Physical response and complete learned-controller diagnosis
remain unknown.

## Provenance and decision

Simulation commit: abe17c27230b1dcaba21adf1e9e7371d313a2c16.
Bracket/repaired-renderer commit: 92545cb0730e3407a650c47e8272d2f7bf9011fd.
440 tests passed; Ruff and offline Exploy demo passed. Preserved infrastructure
failures: campaign001 lacked adapter runner provenance (fixed before002), and
campaign002's renderer gave the hash helper a string path (fixed before003).
Render-only003 reused the recorded states and did not rerun dynamics.

- runs/relic-walk-three-leg-prep-20261009-001: preserved adapter failure.
- runs/relic-walk-three-leg-prep-20261009-002: tests, baseline, walking and both demos.
- runs/relic-walk-bracket-20261009-001: paired 0.14 m/s diagnostic.
- runs/relic-walk-three-leg-replay-20261009-003: 50 fps MP4 and dynamics PNG/PDF for both failed demos.
- local/relic-prep-comparison-20261009-001 and local/relic-prep-walk-bracket-20261009-001: frozen protocols/inputs.

Videos decoded; frame counts matched (MuJoCo 251, Isaac 250). Kinematic visual-mesh
replay preserves source guard_stop labels and introduces no dynamics. Graphs end
at the last accepted physics tick; exact rejected states/targets are retained in
events.jsonl and result-summary.json. Foot clearance is a geometry proxy, not a
measurement of loaded support contacts. Recompute the public summary locally:

```bash
uv run --no-sync python records/relic-walk-three-leg-prep-20261009/summarize.py
```

Decision: keep both motions offline and preserve original weights/gains/guards.
Next investigate gait onset and selected-leg activation/return against native
ReLIC, including loaded contact measurements. Physical three-leg work also needs
an explicit selected-leg Exploy export and live contract review. Repeat host
timing and fresh motors-off preflight after the motion simulation gate passes.
No physical trial evidence was created or promoted here.

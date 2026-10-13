# Unfiltered ReLIC three-leg rollout video

Question: what does the newly requested unfiltered selected-leg profile do from
the latest recorded settled hardware joint pose in MuJoCo?

One bounded offline simulation was run; no robot connection or motion. Source
`a5fad50cfc91be5f39dc5436dc71670c16c868b2`, clean at launch; existing locked UV
runtime, CPU2 and GPU0 EGL rendering only, seed101, no paid compute/no retries.
Exact command arrays, environment, artifact paths and input evidence are retained
in private `local/relic-three-leg-unfiltered-video-20261013-001/contract.json`.
Latest hardware state at4.981s from the cap2 trial is used for starting joints and
orientation. Simulation resets velocity/history to zero and performs its5s policy
hold; this is not an exact reproduction of physical external/harness forces.
URDF median foot-ground placement estimates root height0.447902768m while retaining
the captured torso-height request separately. Capture hashes are in its COMPLETE.
The47s plan, weights/gains,50Hz policy/200Hz commands, native ACK-owned raw history,
IK path and2s return blend are retained. Target filtering is null and command-rate
crossings are observation-only; other guards remain active. Physics diagnostics
are enabled, MuJoCo noslip iterations10, prewarm100 discarded outputs.

The rollout stopped5.02s, immediately after entering lift, on predicted PD-plus-
feedforward load. Front-right HX requested49.921N·m against44.88N·m. The video is
therefore the recorded partial rollout, not a completed lift/square/return. Native
observation/action/target parity errors remain below2.4e-7rad for targets. Stable
final windowfalse. This guarded failure does not qualify the unfiltered profile.

`runs/relic-three-leg-unfiltered-video-20261013-001/video/replay.mp4`:100fps,
502frames,5.02s; visual-mesh state replay with failure label. It generates no new
physics. Dynamics PNG/PDF are in the same directory. Video/trajectory/model/source
hashes are preserved in video/COMPLETE.json; compact public metrics and video hash
are in summary.json. Full logs and durable supervision records are in the parent
run directory; raw robot data remains private ignored. Frame near4.8s was inspected
to verify robot meshes and outcome annotation. No physical approval was requested
or consumed; no guard was widened and no unguarded simulation was substituted.

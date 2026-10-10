# ReLIC replay visual correction — 2026-10-09

Adam reported gold coloring and holes in the simulation recording. The replay
used the simulation converter's gold fallback and its direct OBJ import. That
import kept only one material subset from several authored meshes: the body
retained 26,748 yellow-wrap triangles but lost 6,459 black-plastic triangles.
Other omitted groups affected upper legs, arm segments and the gripper.

The replay now creates a separate visual MJCF with every material group encoded
as an explicit MSH asset. It preserves the original diffuse colors, PNG texture,
normals, UV scaling and visual transforms. All 162,762 authored triangles across
30 groups survive compilation; a count mismatch fails rendering. Every body
surface is restored (33,207 triangles). Source and generated mesh hashes are
recorded in visuals/visual-manifest.json; COMPLETE.json also binds the visual
model, manifest and helper. Links lacking authored visuals are explicitly listed.

Regression checks compare collisions, joint axes/frames/ranges, inertials,
degrees of freedom and articulated poses against the original canonical model.
The simulation converter and the recorded trajectories remain unchanged. Only
kinematic replay is performed; no new physics, robot access or motor commands.

Validation at df92b89a6ae24d3b394ea142dfe6711f72bf3440: 441 tests and Ruff pass.
Both original guard-stop recordings were re-rendered at 50 fps, decoded and
verified against their artifact/source hashes. MuJoCo: 251 frames, encoded
duration 5.02 s (last recorded tick 5.015 s). Isaac: 250 frames, encoded duration
5.00 s (last recorded tick 4.995 s). The original body-speed/target-position
stops remain labeled; this visual fix does not change their controller outcomes.
Primary inspection of both t=4.98 s frames confirms restored solid yellow/black
surfaces. Full commands, environments and timings remain in the private runs.

- runs/relic-replay-visuals-20261009-001 preserves the first acceptance attempt:
  both renders passed, then an ffmpeg decode check timed out (exit 124).
- runs/relic-replay-visuals-20261009-002 completed with explicit noninteractive
  input and bounded decoder/filter threads. Encoder threads are also bounded.
- Source recordings: runs/relic-walk-three-leg-prep-20261009-002/three-leg-mujoco
  and three-leg-isaac. No dynamics rerun, seed or controller configuration change.
- Corrected MP4/PNG/PDF files are under acceptance002/{three-leg-mujoco,three-leg-isaac}.

Reproduce rendering using docs/THREE_LEG_DEMO.md with a fresh output directory.
The source capture remains private; public fixtures support portable examples.
result-summary.json preserves sanitized counts, hashes and outcomes. Decision:
use the corrected renderer for future recorded-state replays; the walking and
three-leg deployment blockers documented in the preceding review are unchanged.

# Chair perception: initial camera commissioning, 2026-10-15

Question: can a user-selected chair be segmented in Spot's front/hand views and
converted into a calibrated, auditable partial point cloud without robot motion?
Hypothesis: registered RGB-D plus acquisition-time transforms support multi-view
fusion; depth gaps must remain missing rather than become invented surfaces.

Implementation/measurement commit: `ed17e6d`, clean at live server launch.
Pinned environment, source/checkpoint hashes, reproduction and UI/API commands:
[CHAIR_PERCEPTION](../../docs/CHAIR_PERCEPTION.md). No random sampling; no training.
The server records module hashes and source identity. Camera inputs remain private.

Protocol: inspect live SDK sources, select an object through the browser's box
prompt, inspect the SAM2 mask, accept it and propagate via calibrated 3D projection.
Repeat captures on a stationary chair. Observe a separate object-motion case.
The pipeline never acquired a lease or issued motor/arm/gripper commands. Initial
state inspection reported motors off and no system faults. Later external scene
and camera-pose changes are not attributed to this process.

Observed results:

| Case | Result |
|---|---|
| Near mesh chair, initial hand selection | 7,780 voxel points, all three views contributed |
| Near chair mask depth coverage | hand 78.5%, front-left 52.8%, front-right 21.7% |
| Near chair subsequently moved substantially | lost association, empty current cloud; no stale geometry retained |
| Stationary blue chair, 10 subsequent captures | all 10 tracked; 1,710–1,931 voxel points |
| Stationary chair gripper view | visual mask available; only 9–10 depth samples, excluded from cloud |
| Static capture/processing HTTP round trips | see exact mean/min/max in results.json; approximately 0.36 s, 2.8 Hz |
| Automated validation | 560 tests passed, Ruff and existing Exploy demo passed |

The 10-frame timing is a short warmed-system measurement including RPC, model,
recording and API serialization. It is not an endurance benchmark, a guaranteed
frequency or the paper's approximately 10 Hz tracker claim. Front/hand clocks,
source depth scale and pixel/transform conventions were exercised; no metrology
calibration accuracy claim is made.

Failures preserved: initial dependency distribution name mismatch (`sam-2`, not
`sam2`) was corrected; first launch incorrectly precreated the output directory;
next launch rejected capture timing without enough diagnostic data. Age is now
checked at receipt before decoding and raw rejected responses are retained; the
original failure did not retain enough data to prove its exact timing cause.
No age/skew thresholds were widened. Failed initial hand selection on a distant
chair motivated distinct 2D-only masks versus depth-supported cloud contributions.

Private artifacts on RPM:

- `runs/chair-perception-setup-20261015-001` and `002`: dependency failure/success.
- `runs/chair-perception-validation-20261015-001`: initial 556-test pass and launch diagnostics.
- `runs/chair-camera-timing-20261015-001`: three read-only raw timing samples.
- `runs/chair-perception-validation-20261015-002`: final validation, live sequence
  summaries, artifact hash manifest and server launch handle.
- `runs/chair-perception-live-20261015-003`: early blue-chair selection.
- `runs/chair-perception-live-20261015-004`: near-chair three-view fusion, motion/loss,
  and stationary-chair sequence. Frame NPZs/protobufs, revision masks/clouds and
  prompts support offline replay. `latest.json` points to the latest artifact.

Interpretation/decision: ready for interactive camera-only collection and visual
review. The cloud contains only observed surfaces; the chair is not reconstructed
as a complete mesh. Geometry-based association deliberately fails on substantial
object motion. Use explicit reselection; full temporal video tracking and dynamic
manipulation qualification are future work. Perception is not connected to ReLIC
commands and does not authorize autonomous chair manipulation.

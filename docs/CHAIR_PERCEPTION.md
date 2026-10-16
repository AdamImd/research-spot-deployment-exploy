# Chair selection and multi-camera point cloud

This independent, camera-only pipeline reads Spot's front-left, front-right and
hand RGB-D streams. It does not acquire a lease, power motors, move the arm/gripper,
modify tablet E-stop authority or send any output to ReLIC. It can run with motors
off. Native standing/gripper positioning, if needed, is a separate operator action.

## Relation to Visual Whole-Body Control

[Visual Whole-Body Control for Legged Loco-Manipulation, §§3.3/G/J](https://arxiv.org/html/2403.16967v5)
uses click-prompted SAM and AOT-Tiny trackers for head/wrist masks, with masked depth
as policy input. This implementation adopts interactive object selection but uses
**SAM 2.1 tiny image segmentation and measured-depth reprojection** for association
between views and successive captures. It adds a metric point-cloud output; it is
not a reproduction of that paper's tracker, learned high-level policy or performance.
No chair manipulation policy is included.

SAM2 source is pinned in `perception/pyproject.toml` and `perception/uv.lock` to
`2b90b9f5ceec907a1c18123530e92e794ad901a4`. The official SAM2.1 tiny checkpoint URL is
https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
and the downloaded SHA256 is
`7402e0d864fa82708a20fbd15bc84245c2f26dff0eb43a4b5b93452deb34be69`.
SAM2 is Apache-2.0; source/license: https://github.com/facebookresearch/sam2.
Weights stay in ignored `local/`. Existing control dependencies and weights are unchanged.

## Environment and launch

Use the isolated Python 3.12 environment (Torch and SAM2 are not added to the
control runtime):

```bash
SAM2_BUILD_CUDA=0 uv sync --project perception --locked
mkdir -p local/chair-perception/models
curl --fail --location \
  https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt \
  -o local/chair-perception/models/sam2.1_hiera_tiny.pt
sha256sum local/chair-perception/models/sam2.1_hiera_tiny.pt
CUDA_VISIBLE_DEVICES=1 uv run --project perception --no-sync python \
  -m spot_deploy.perception.server \
  --robot /absolute/path/to/private/robot.json \
  --checkpoint local/chair-perception/models/sam2.1_hiera_tiny.pt \
  --output runs/chair-session-001 --port 8767
```

Choose an available GPU; `cuda:0` is relative to CUDA_VISIBLE_DEVICES. CPU is
available with `--device cpu`, at reduced throughput. The loader verifies the checkpoint hash
above before loading. The camera client checks robot serial against RobotConfig.
Authentication uses only BOSDYN_CLIENT_USERNAME and BOSDYN_CLIENT_PASSWORD.

Open **http://127.0.0.1:8767/** (the command above selects this port). The server is loopback-only, lasts 30 minutes by
default (`--duration`, maximum two hours), and never starts continuous capture
until the user checks its box. Use a new output directory each launch.

1. Choose the camera where the desired chair is clearest. Drag a box around it.
2. Click **Preview mask**. Add foreground/background points to refine the same
   prompt, then preview again. **Clear prompts** starts over. The green overlay
   must represent the intended chair, not the desk, floor or neighbouring chair.
3. Click **Use this chair**. It selects that object and attempts mask propagation
   into the other cameras using calibrated projected points. Pixels with missing
   depth may seed a visual-only mask, but cannot contribute 3D measurements.
4. Inspect each proposed mask and the 3D cloud. If a view is ambiguous, select in
   that view to replace the object anchor. This is single-object selection.
5. Use **Capture / track once**, or **Continuous capture**. Selecting a new prompt
   pauses continuous capture. A lost track clears the current cloud and requires
   reselection. Current-frame mask predictions are not ground-truth annotations.

**Polygon mode** supplies an exact hand-drawn mask without any model. Start the
server without `--checkpoint`; automatic propagation is then unavailable.

The image/selection API is also usable by an agent: GET `/api/state` returns the
current `sequence` and displayed image sizes; GET `/image/{frontleft,frontright,hand}`
returns the displayed pixels. POST `/api/preview` takes
`{view, sequence, prompt: {box: [x0,y0,x1,y1], points: [[x,y]], labels: [1]}}`.
Labels are 1 for foreground, 0 for background; polygon mode uses `polygon` vertices.
POST `/api/accept` accepts the preview; POST `/api/step` captures and tracks once.
POSTs require the per-session `X-Session-Token` embedded in the local page and a
loopback Host header. GET `/api/cloud` is a bounded JSON display stream. There are
no robot-motion HTTP routes, arbitrary path inputs or cross-origin permissions.

## Geometry and data integrity

| View | Visual source | Registered depth source |
|---|---|---|
| Front left | frontleft_fisheye_image | frontleft_depth_in_visual_frame |
| Front right | frontright_fisheye_image | frontright_depth_in_visual_frame |
| Gripper | hand_color_image | hand_depth_in_hand_color_frame |

Require matching optical-frame names, dimensions and pinhole calibration. Reject
unimplemented camera models rather than treating raw distorted pixels as pinhole.
Native front streams are rotated clockwise for display and segmentation; the
same pixel transform multiplies K, so metric rays remain correct. Geometry uses
`depth / ImageSource.depth_scale`, never a hardcoded millimetre scale. This Spot
reported 999 for fronts and 1000 for hand during initial inspection.

Use the **depth acquisition snapshot's odom_T_camera**, including the moving hand
chain, not a fixed gripper extrinsic or the latest unrelated robot state. Require
RGB/depth skew ≤80 ms, inter-depth skew ≤100 ms and robot-clock image age at receipt ≤500 ms. The UI shows elapsed snapshot age separately.
Reject exposure-to-exposure camera displacement >2 cm or rotation >0.05 rad.
These thresholds are initial perception quality settings, not control deadlines.

Depth codes 0/65535, nonfinite values and depths outside 0.2–4 m are excluded.
Erode masks one pixel to reduce foreground/background edge bleed. Do not fill
depth holes: missing geometry is preferable to fabricated chair surfaces.
Backproject with K inverse and transform to odometry; voxel-average at 1 cm.
The PLY/NPZ cloud is a **current capture**, not an accumulating reconstruction;
no old points survive a failed capture or lost association. Per-voxel camera
provenance uses bits 1=front-left, 2=front-right, 4=hand. RGB from front cameras
is repeated grayscale. Axes in the viewer are odometry X red/Y green/Z blue.

Reprojection verifies available current depth within 5 cm of projected surface
depth. Missing-depth pixels may additionally seed a 2D mask proposal. Such pixels
do not establish visibility, so those masks require visual inspection. A view
contributes to the cloud only with at least 20 depth-verified projected pixels
and 20 actual measured masked depths; otherwise its status is
`mask_only_no_reliable_depth` and its 3D contribution is zero.
Require 20 support pixels, SAM score ≥0.6, ≥60% seed coverage and ≥80% candidate
points within the preceding cloud's axis-aligned region expanded by 25 cm.
Out-of-view or known depth-occluded regions remain absent. These gates reduce identity drift but do not
prove that a mask is the right chair. The object must be static or slowly moving;
fast manipulation needs a proper temporal tracker and motion compensation.
Normal tracking gaps may not exceed 5 seconds. Immediately after explicit manual
selection, a seed up to 300 seconds old may be re-associated against *fresh depth*;
the seed itself is never copied into a new capture. Longer gaps require reselection.

## Recording and replay

Private `runs/` sessions contain raw SDK protobuf responses, displayed RGB and
metric depth NPZs, K, transforms, timestamps, prompts/model scores, masks, merged
`chair.npz`/`chair.ply`, JSON summaries and events. Selection revisions preserve
earlier masks; `latest.json` identifies current artifacts. Clouds are measured
partial surfaces, not watertight meshes. The JSON browser cloud is capped at
30,000 points; PLY/NPZ keep all voxel points. Session provenance includes source
commit/dirty state, module hashes, model hash and Torch version. Raw robot camera
images and frame trees are private; do not commit them to the public repository.

To replay captured frames without connecting to Spot:

```bash
CUDA_VISIBLE_DEVICES=1 uv run --project perception --no-sync python \
  -m spot_deploy.perception.server --replay runs/chair-session-001 \
  --checkpoint local/chair-perception/models/sam2.1_hiera_tiny.pt \
  --output runs/chair-replay-001
```

Replay advances one recorded bundle per capture; end-of-replay clears the cloud
rather than pretending old frames are live. Reapply saved prompts through the same
API. No robot credentials are needed. Prompt coordinates refer to saved displayed
pixels, after front-camera rotation.

Focused validation: `uv run --no-sync pytest -q tests/test_chair_perception.py`.
Coverage includes known metric rays, rigid transforms, rotated-image consistency,
invalid depths, occlusion, view provenance, SDK units/skew/exposure motion, stale
selection rejection, empty cloud on capture failure, and camera-only SDK imports.

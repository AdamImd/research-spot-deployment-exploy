# Robot state viewer

Run from the repository's locked environment. All viewer examples are read-only.

```bash
.venv/bin/spot-view --demo
```

Open **http://127.0.0.1:8765** on the RPM desktop. Drag the robot to orbit, scroll
to zoom, or choose Side/Top. Select any joint row to plot its position. Toggle
measured pose, predicted target and sent-command overlays independently. Pause,
scrub or change playback speed; **Latest** follows incoming samples.

The measured robot uses the URDF's visual meshes and materials. Predicted and sent
poses use translucent amber/purple meshes. **Visual meshes** and **Link frames**
are independent view controls; the **Links** disclosure lists every link, including
fixed links and the root, with per-link visibility and **Show all links**. A link
without visual geometry displays a frame marker rather than an invented shape.
Missing or unsupported visuals are explicitly marked in this list.

For the current saved Spot URDF, its `link_models/*.obj` files are not present.
Use the already-local ReLIC visual model, which has matching measured joint frames:

```bash
.venv/bin/spot-view --run runs/relic-live-shadow-001/telemetry \
  --visual-urdf simulation/source/relic/relic/assets/spot/spot_with_arm.urdf \
  --estop-status /tmp/spot-joystick-status.json
```

This keeps all 19 measured joints from Spot's saved URDF, maps the ReLIC arm aliases,
and adds five fixed leaf frames: 25 displayed links, 20 visual meshes. Template
joint parents, types, origins and axes must match; a mismatch refuses startup.
The template's mesh geometry is a visual reference, not calibration evidence for
the physical robot. It never changes policy observations, targets or commands.

The demo generates illustrative standing motion and mock predictions. No policy,
Spot connection, lease, command adapter or serial port is involved.

## Recorded and live runs

```bash
.venv/bin/spot-view --run runs/replay-example
# In one terminal, after completing the read-only robot setup in OPERATIONS.md:
.venv/bin/spot-deploy watch --robot local/robot.json --duration 60 --output runs/watch-001
# In a second terminal after run.json exists:
.venv/bin/spot-view --run runs/watch-001
```

`watch` records measured state at up to 20 Hz and full health approximately once a second.
It can observe a stopped or faulted robot; identity, firmware and the 19-joint arm
morphology still have to match. It does not need a policy, command lease or enabled
motors. Stream captures are bounded to 60 seconds and have normal completion records.

For the observed firmware 5.0.1, use `--state-source poll` with a
`WatchRobotConfig` profile containing its exact serial, firmware, host and wired
route. This calls the unary measured-state API, maps joints by name, and obtains
body orientation from the odom/body frame snapshot. Polling captures can last up
to 3600 seconds; RPC failure or a stale robot timestamp ends the capture with
saved evidence and no automatic restart. The viewer then shows disconnected/stale
or completed data. Both 5.0.1 and 5.1.x profiles are accepted after protocol
review; streaming access and joint-control entitlement are separate capabilities.
See [firmware compatibility](FIRMWARE_COMPATIBILITY.md).

```bash
.venv/bin/spot-deploy watch --robot local/robot-watch.json --state-source poll \
  --duration 3600 --output runs/live-status
.venv/bin/spot-view --run runs/live-status
```

This mode displays measured state and health. RL predictions require a separate
shadow inference source; the board does not fabricate predictions for a watch run.

The [ReLIC live shadow runner](RELIC_SHADOW.md) supplies 50 Hz predictions from the
high-frequency state API. The table includes all target joint angles in radians;
select a leg row to compare its measured and predicted angles over time. Amber is
the target overlay. ReLIC's seven arm target values are fixed policy inputs, not
network predictions. The policy panel names each of its 12 leg actions and reports
observed state/inference rates. Browser data refresh remains approximately 4 Hz;
the retained 24,000 samples cover about eight minutes at 50 Hz.

Use the same viewer with the existing `shadow` command to see policy predictions
without actuating. A `stand` run additionally records sent joint commands. A sent
command is not proof that the robot executed that target. Raw action bars show
network index order, not SDK joint order; the plotted targets are the converted
19-joint SDK-order positions in radians. Bars saturate visually at magnitude 1;
their numerical labels retain the actual value.

```bash
.venv/bin/spot-view --run runs/shadow-001 --estop-status /tmp/spot-esp32-status.json
```

The hardware panel labels bench mode separately. Current bridge status expires
after 0.5 seconds in the viewer. This display threshold is not the standing gate's
stricter bound and does not change any watchdog. Without a selected bridge file,
the GUI shows recorded aggregate SDK E-stop health when available. Battery values
from a preflight snapshot are labeled as such; they are not live measurements.

## Model, time and limits

- New preflight/watch/shadow/stand runs save `robot.urdf` alongside its snapshot hash.
  The viewer loads this automatically, or accepts `--urdf PATH`. A mismatch against
  the snapshot hash is visibly flagged. URDF parsing accepts a connected joint tree
  with all 19 canonical movable joints, including every declared link, fixed link,
  visual origin, mesh scale, material color and supported primitive geometry.
- The WebGL renderer uses a local, pinned Three.js 0.180.0 build. OBJ and STL meshes,
  local MTL diffuse materials, PNG diffuse textures and texture scale are supported.
  Box, sphere and cylinder URDF visuals are supported. Other mesh/texture formats
  are reported as unavailable; collision geometry is never substituted for visuals.
  Asset paths stay under the URDF directory unless `--mesh-root DIR` explicitly
  supplies another root. Symlinks cannot escape that root, remote URLs are rejected,
  and only validated geometry/PNG assets are served through opaque content hashes.
- The view remains body-centred with measured orientation and a reference floor;
  it is not an odometry map. Without a URDF the explicitly approximate schematic
  remains available. Meshes do not establish contact, collision or calibration.
- Predicted/sent overlays disappear when more than 250 ms older than the displayed
  robot sample. While following live data, stale state (>0.5 s) or a disconnected
  viewer suppresses overlays. The last measured pose remains visible with a stale
  label. Paused/replayed frames intentionally show historical observations.
- Up to 24,000 frames are retained, covering about 20 minutes at 20 Hz.
  Long captures are a rolling window. Invalid/oversized log lines
  are counted, partial writes are buffered, and replaced/truncated logs are rejected.
- Service binds only `127.0.0.1`. The viewer has no control/arming endpoints and
  does not load scripts, fonts or other assets from a CDN. Use `--port` for a
  second instance. Ctrl-C ends the viewer without touching the robot.

Checks cover partial logs, malformed state, complete URDF trees, visual-frame parity,
OBJ/STL decoding, missing visuals, local asset containment and HTTP origin boundaries.
`node tests/test_kinematics.mjs` additionally
checks renderer FK for fixed, revolute and prismatic links and body orientation.

# Lightweight rollout recording

`tools/record_rollout.py` creates a compact, compressed robot-data recording from
an existing run's `events.jsonl`. It works with native stock rehearsals, ReLIC
hardware rollouts, read-only shadow predictions and shared-core simulator runs.
It never connects to Spot or issues control commands. Run it in a separate process;
it reads the already-written log and leaves the control-loop recording unchanged.

## Record a rollout

Once the rollout has created its run directory, start the recorder. It reads from
the beginning, including startup events already written, then follows new events:

```bash
uv run --no-sync python tools/record_rollout.py \
  --run runs/rollout-001 --output runs/recording-001 \
  --manifest local/deployment/policy/manifest.json \
  --follow --duration 60
```

It stops after reading the source's completion marker and draining the final events,
or after the requested wall-clock duration. Ctrl-C finalizes a partial recording.
Use a new output directory outside the source directory for every recording.
Starting this command does not start a rollout or operate the robot.

To compact a completed run, omit `--follow`:

```bash
uv run --no-sync python tools/record_rollout.py \
  --run runs/stock-rehearsal-20261009-001 --output runs/stock-recording-001
```

## What is retained

- Measured joint positions, velocities and loads, body orientation/velocities and
  command acknowledgement state. Redundant measured-state events are sampled at
  50 Hz by default; use `--state-hz 0` for every unique source state, or another rate.
- **Every policy prediction**, including all 12 raw actions, 19 joint targets,
  inference time and its exact measured input state. Prediction input states are
  retained even when `--state-hz` is lower than the policy rate.
- **Every logged command**, including its key, 19 target positions, feedforward,
  expiry and clock fields, plus the measured state's acknowledgement key and time.
- Policy acknowledgements, phase transitions, startup height/history, health,
  stock posture feedback and failure events. Source-line numbers preserve ordering.

Commands are host-produced targets, not proof that actuators applied them. Look at
acknowledgement fields/events separately. Shadow predictions were never commands.
Robot time, host monotonic time and wall time retain their distinct field names;
they must not be subtracted without the corresponding clock conversion.

`recording.json` labels SDK joint order, units, quaternion convention, source and
recorder commit/hash. Raw-action order comes from `--manifest`, simulator manifest
metadata or shadow policy metadata; it is **null when unavailable**, never guessed.
No rounding is applied to retained numerical values. Full observation vectors,
duplicated high-rate state copies, route and identity events are omitted.

## Files and reading

- `rollout.jsonl.gz`: gzip-compressed JSON Lines, streamed with compression level 1.
- `recording.json`: format, source, ordering and capture options.
- `summary.json`: counts, bytes, source-prefix hash and completion/partial status.
- `COMPLETE.json`: recording artifact hashes. This marker exists for partial and
  failed captures too; inspect its status and `summary.json.capture_complete`.

```python
import gzip, json

with gzip.open("runs/recording-001/rollout.jsonl.gz", "rt") as stream:
    for line in stream:
        event = json.loads(line)
        if event["event"] == "policy":
            print(event["state_robot_time_s"], event["raw_actions"], event["targets"])
```

This recorder uses bounded 256 KB reads and event-size limits, holds no full-rollout
array in memory, and buffers incomplete live writes until newline. Invalid JSON,
oversized events, source replacement/truncation and a completed source's event-hash
mismatch fail explicitly. Output directories are never overwritten. A complete
capture of a failed source remains labeled with `source_status: failed`.

It is a convenient data artifact, **not a replacement for the required safety log
or deployment evidence**. It cannot recover source events that were never logged,
and recorder failure does not stop the robot. Compression still consumes some
host CPU/I/O; separate-process operation does not guarantee zero scheduling impact.
The script records no video and does not directly replay the compressed file in
the current viewer; the original run remains usable by the viewer.

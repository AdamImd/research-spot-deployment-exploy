# Supervised stock-motion baseline

`tools/rehearse_stock.py` collects native-controller measurements before policy
deployment. It uses only Boston Dynamics power, stand and sit commands; it does
not instantiate a policy or joint-command client. It neither qualifies ReLIC nor
changes its evidence and all-faults gates.

Physical execution requires an explicit operator request, two distinct named
operators in position, the inspected rig and clear area, and a tablet E-stop
operator. Use the selected tablet profile, locked UV environment and a clean
committed branch. Credentials remain environment-only. Do not take another
client's lease: acquisition failure stops the run without motion.

First inspect with no execution flag:

```bash
uv run --no-sync python tools/rehearse_stock.py \
  --robot local/deployment/robot.json \
  --manifest local/deployment/policy/manifest.json \
  --envelope local/deployment/envelope.json \
  --operator ROBOT_OPERATOR --safety-operator TABLET_OPERATOR \
  --output runs/stock-inspection-001
```

Only for the expressly requested physical rehearsal, add `--execute` and choose
a new output directory. If already powered, the sequence first sits and safely
powers off; then it powers on, stands, observes for 10 seconds, sits, observes for
2 seconds and safely powers off. No walking or arm commands are sent.

The separately explicit `--allow-known-payload-info` option accepts only the
observed system `payload.fault`, code 9, severity INFO for this stock diagnostic.
The warning is recorded, never cleared or relabeled as healthy. Other faults
abort. This exception does not apply to ReLIC activation.

State-stream age is limited to 100 ms for this diagnostic (not the stricter RL
control envelope); full health and tablet stop state are polled about 10 Hz.
The acquired lease is retained every second. Posture commands have a 15-second
completion deadline, power-on 20 seconds, and safe power-off 15 seconds.
The overall operational deadline is 120 seconds, checked between bounded SDK
calls. The tablet remains the independent stop authority during blocking calls.
Cancellation or failure after a power/motion request triggers one safe-power-off
cleanup attempt, followed by lease return. There is no automatic rerun, lease
takeover, E-stop write or fault clearing. An unconfirmed shutdown must be reported
to the tablet operator; process exit alone is not shutdown evidence.

Records include input/source hashes, operator names, initial snapshot, phases,
native command RPC and posture completion times, streamed joint positions,
velocities and loads, body state, standing height, foot contacts, full health,
stream timing and exact final motor-off confirmation. The observer samples the
latest mailbox about 100 Hz; receiver statistics count all incoming packets.
Native command acceptance timing is not joint-stream acknowledgement timing.
These measurements do not test an actual tablet-stop press, network outage,
policy handover, joint-level transport or the ReLIC shutdown deadline.

Keep robot-specific raw records under ignored `runs/`, and preserve failed runs.

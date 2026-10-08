# RPM joystick stop

The Logitech Extreme 3D at
`/dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-joystick` is supported as a
USB input to Spot's software E-stop service. The actual RPM device reports USB
`046d:c215`, 12 buttons, and index 0 mapped to Linux `BTN_TRIGGER` (288).
Physical **button 2 (index 1, Linux `BTN_THUMB`) is CUT**. Physical
**button 4 (index 3) is rearm**. Index 2 remains cockpit motion-enable.
The trigger (index 0) is no longer the stop or rearm control. Axes never command the robot.

The adapter reads Linux joystick events directly; it needs neither Pygame nor
window focus. No extra dependency or system installation is required. It uses
the existing `spot-estop` CLI with explicit `--input joystick`; ESP32 remains the
default for existing commands and profiles.

## Operator behavior

- Startup is stopped. Press/release index 1, leave it released for at least
  0.5 seconds, then freshly press index 3 to allow operation.
- **Pressing index 1 latches STOP.** Releasing it does not clear the latch.
  Holding index 3 through a stop does not clear it. A fresh index 3 press after
  the release interval is required. Rearm permits power-on; it does not power
  motors or resume an RL trial.
- The provided profile sends **CUT**, which immediately requests actuator-power
  removal. A standing robot can fall. Keep the physical robot stop available and
  use the reviewed support/test setup. `settle_then_cut` is an explicit alternate
  profile choice, not a fallback silently substituted at runtime.
- USB removal, replacement, event overflow/resynchronization, malformed input,
  stalled polling, RPC failure, SIGINT/SIGTERM and bridge exit request STOP.
  Errors terminate the bridge; there is no automatic reconnect/rearm/retry.
- The input loop nominally runs at 50 Hz. It rejects polling/cycles older than
  150 ms. Profile robot watchdogs are both **1 second**, matching the durations
  returned by this Spot 5.0.1 after it normalized our first 400/800 ms request.
  RPC timeout is 80 ms. These are configured deadlines, not measured stop times.
  The local status file refreshes at 10 Hz or immediately on button/latch changes;
  input polling and robot check-ins remain at 50 Hz. Timing maxima are recorded.

This is a consumer joystick feeding a **software E-stop**, not an ESP32 firmware
latch or safety-rated wired switch. An idle joystick produces no events: the
adapter checks the kernel device with ioctl/poll each cycle, but this cannot
detect every stuck button/USB firmware failure. End-to-end stop qualification
is separate from input detection and unit tests.

## Identify and bench without robot access

```bash
cp examples/joystick-estop.json local/joystick-estop.json
.venv/bin/spot-estop identify --input joystick \
  --profile local/joystick-estop.json --output runs/joystick-identify-new
.venv/bin/spot-estop bench --input joystick \
  --profile local/joystick-estop.json --status /tmp/spot-joystick-status.json \
  --duration 1800 --output runs/joystick-bench-new
```

Bench mode does not import a robot adapter, cannot stop Spot, and never satisfies
the live-control interlock. It records every button edge and latch transition.
Verify button 2 CUT, release without rearm, separate rearm, held rearm, and USB
loss before robot setup. Process ownership uses both a status-file lock and an
advisory lock on the actual input node; other joystick applications are not grabbed.

## Robot configuration and registration

The `hardware_estop` object in the robot configuration must bind the profile
SHA-256 and absolute status path, exactly as for ESP32. A separate
`local/robot-joystick.json` keeps the observation-only robot configuration intact.
Use the actual profile hash, status file `/tmp/spot-joystick-status.json`, and a
250 ms maximum status age. Qualification remains required for motion.

```bash
.venv/bin/spot-estop inspect --input joystick \
  --profile local/joystick-estop.json --robot local/robot-joystick.json \
  --output runs/joystick-inspect-new
```

Configuration and registration require **motors off** and matching robot identity.
The default joystick profile uses `PDB_rooted` and can initialize an **empty**
E-stop configuration. It never replaces an existing endpoint. If an independent
`PDB_rooted` endpoint already exists, explicitly use `spot_joystick_rpm` as the
joystick's role; the adapter preserves every existing endpoint. Configuration
changes invalidate registrations, so verify/re-register any independent clients.
The ESP32 path still requires an existing independent `PDB_rooted` endpoint.

```bash
# Use the inspected current ID; for an empty configuration this is an empty string.
.venv/bin/spot-estop configure --input joystick --execute \
  --expected-config-id CURRENT_CONFIG_ID \
  --profile local/joystick-estop.json --robot local/robot-joystick.json \
  --output runs/joystick-configure-new
# Keep this separate process running for the entire operator session.
.venv/bin/spot-estop bridge --input joystick --execute \
  --expected-config-id NEW_CONFIG_ID --expected-endpoint-id SLOT_ID \
  --profile local/joystick-estop.json --robot local/robot-joystick.json \
  --status /tmp/spot-joystick-status.json --output runs/joystick-bridge-new
```

The bridge starts stopped after registration. Repeat the physical CUT/rearm
sequence. It retains the endpoint on exit so the robot watchdog still applies.
Starting this bridge neither acquires a motion lease nor powers on the robot.
Changing the input configuration does not qualify or enable ReLIC control.
E-stop setup uses unary state reads and does not require the RL streaming-state
service. Failure records identify the setup phase without retaining credentials.
Failed authentication cleanup does not start a time-sync connection.

Pass `--estop-status /tmp/spot-joystick-status.json` to `spot-view` to show the
joystick state alongside predictions. The board explicitly labels bench mode
and only shows a robot-confirmed state after an acknowledged E-stop check-in.

The October 5 implementation has passed the USB polling bench, but registration
was blocked by loss of Ethernet contact. Do not treat the installed configuration
or a bench label as proof that button 2 can stop Spot. See the
[qualification procedure](OPERATIONS.md).

References: [Linux joystick event and initialization semantics](https://docs.kernel.org/input/joydev/joystick-api.html),
[Spot E-stop levels, configuration and registration](https://dev.bostondynamics.com/docs/concepts/estop_service.html).

## Optional cockpit IPC

Set `SPOT_JOYSTICK_IPC` in the bridge environment to an absolute socket path in an
owner-only (0700) directory. Restart the bridge through the normal motors-off
procedure; this feature does not configure/register an endpoint itself. The Unix
datagram socket accepts only `CUT` and `STATE`. CUT wins over same-cycle rearm,
latches STOP, and requires a new physical rearm. Replies include confirmed state,
bridge session/robot binding, monotonic time, sequence, axes and buttons. No IPC
request can allow power or command motion. Publishing is bounded/nonblocking and
an unavailable subscriber is dropped. The separate autonomy cockpit may consume
axes with its own deadman; this bridge itself still issues no motion commands.

Cockpit IPC validation on October 6, 2026: 32 focused joystick/IPC tests passed,
including CUT latching before check-in, no ALLOW verb, nonblocking dropped clients,
owner-only directory checks, axes publication and immediate endpoint stop if IPC
startup fails. No hardware bridge was restarted or configured for this change;
physical end-to-end timing remains unqualified.

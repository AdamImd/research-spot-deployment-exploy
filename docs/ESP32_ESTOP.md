# ESP32 physical stop input

This implementation assumes a **classic ESP32 DevKit (`esp32dev`) connected to RPM
over USB serial**. An existing board/protocol, ESP32-S3/C3, or Wi-Fi transport needs
an explicit adapter and fresh qualification. No board was connected or flashed
during software implementation.

The button feeds a dedicated endpoint in Spot's **software** E-stop service. It is
not a direct motor-power cut, dual-channel safety circuit or certified controller.
Keep the independent manufacturer E-stop available. Qualify the actual wiring,
rig and stop behavior before relying on this input around a powered robot.

## Wiring and physical behavior

| Signal | Classic ESP32 pin | Wiring |
| --- | --- | --- |
| STOP | GPIO27 | Normally closed latching mushroom contact to GND |
| REARM | GPIO26 | Separate normally open momentary switch to GND |
| Reference | GND | Both contacts share the board ground |
| Pullups | 3.3 V | External 10 kΩ pullup per input; internal pullups also enabled |
| Host | USB | Data-capable cable to RPM; identify the stable `/dev/serial/by-id/…` path |

Do not apply 5 V or motor/control supply voltage to GPIO. These are dry contacts.
An open STOP contact (button pressed or a broken wire) immediately latches the
firmware stopped. A short to ground can defeat this single-channel input and is
not detected by this design. Inspect the actual switch terminals and board pinout.

Startup is stopped. Release the mushroom, keep the closed circuit stable for at
least 500 ms, then release and press the separate REARM switch while host queries
are healthy. Holding REARM at startup or through a stop cannot rearm. Releasing
the mushroom alone cannot rearm. A 250 ms query gap latches stopped; communication
restoration does not rearm it. Reboot, malformed frames or host bridge failure
require restarting the bridge and another physical rearm.

## Build, identify and bench test

```bash
uv sync --locked --all-groups
PLATFORMIO_CORE_DIR="$PWD/local/platformio" .venv/bin/pio run -d firmware/esp32_estop -e esp32dev -j 2
cp examples/esp32-estop.json local/esp32-estop.json
```

Build is pinned to PlatformIO Core 6.1.18, Espressif32 6.9.0 / Arduino 2.0.17.
Review the GPIO mapping before flashing `firmware/esp32_estop/.pio/build/esp32dev/firmware.bin`.
Upload is a separate deliberate hardware operation; the build never uploads.

Set `serial_device` in `local/esp32-estop.json` to the board's stable path. Opening
the port can auto-reset a DevKit; the bridge waits 1.5 seconds for boot, then discards
boot chatter while still stopped. Close serial monitors before using the bridge.

```bash
# USB only. Press STOP first. This reports the board MAC-based identifier.
.venv/bin/spot-estop identify --profile local/esp32-estop.json --output runs/estop-identify-001
# Copy the reported device_id into local/esp32-estop.json, then:
.venv/bin/spot-estop bench --profile local/esp32-estop.json --status /tmp/spot-esp32-status.json --duration 60 --output runs/estop-bench-001
```

Bench mode never imports/connects a Spot adapter and cannot satisfy a standing
interlock. Inspect the status file/transitions while testing press, release,
physical rearm, held reset, open wire, USB loss and reboot. Wrong device/CRC,
partial/oversized frames, repeated sequence, changed boot ID and missed response
deadlines terminate the bridge. There is no automatic restart. A normal physical
STOP keeps the bridge alive, checking in STOP until a deliberate physical rearm.

## Bind the robot profile and evidence

Calculate `sha256sum local/esp32-estop.json` and add the following object to the
otherwise valid `local/robot.json`. Use that exact hash and an absolute status path:

```json
"hardware_estop": {
  "profile_sha256": "REPLACE_WITH_64_HEX_SHA256",
  "status_file": "/tmp/spot-esp32-status.json",
  "max_status_age_s": 0.25
}
```

The new `hardware_estop` evidence gate must pass along with the existing gates.
The gate's artifact should record board identity, firmware binary/source hashes,
pin/connector wiring, profile hash, independent endpoint configuration, button and
link-loss trials, observed latencies, robot/rig state, operator and reviewer, and
the accepted stop level/timeouts. Hash this artifact in the existing evidence index.
Changing firmware, source, profile hash or the bound robot config invalidates the evidence.

The sample profile's timing values are engineering starting points, **not reviewed
hardware limits**. `stop_level: cut` requests immediate actuator-power cut when
STOP is detected and can make Spot fall into the rig. `settle_then_cut` requests a
settle before cut and has different behavior. Both need physical review.

## Dedicated Spot endpoint setup (motors off)

Use the existing environment-only credentials and wired-route profile from
OPERATIONS.md. Keep the manufacturer E-stop independently connected.

```bash
.venv/bin/spot-estop inspect --profile local/esp32-estop.json --robot local/robot.json --output runs/estop-inspect-001
# Read config.unique_id from the inspection result. This next command changes config:
.venv/bin/spot-estop configure --execute --expected-config-id CURRENT_CONFIG_ID --profile local/esp32-estop.json --robot local/robot.json --output runs/estop-configure-001
```

Configure verifies serial/firmware and motors off, preserves existing endpoint
specifications and adds one unique `spot_esp32_*` role. It refuses an existing role
or name, rather than replacing it. Config changes may invalidate registrations;
re-register/verify independent endpoints using their normal clients afterward.
Never replace the configuration with `force_simple_setup`. Capture the returned
config and endpoint IDs. Setup neither acquires a command lease nor powers motors.

```bash
# Separate terminal/process; it must remain running for the whole trial.
.venv/bin/spot-estop bridge --execute --expected-config-id NEW_CONFIG_ID --expected-endpoint-id SLOT_ID --profile local/esp32-estop.json --robot local/robot.json --status /tmp/spot-esp32-status.json --output runs/estop-bridge-001
```

Registration again verifies motors off, exact slot identity/timeouts and absence
of a recently checking-in owner. Replacing an expired registration explicitly targets
its unique endpoint ID; the last requested NONE level alone is not proof of a live
owner. It starts with STOP. Physically rearm only after all independent
E-stops and the operating area are ready. Registration can change the endpoint ID;
the acknowledged ID is written to bridge status. Re-inspect before a later bridge
invocation to obtain the current IDs. A status-file lock and exclusive serial open
prevent duplicate local bridge ownership. Never manually forge an armed status file.

`stand` checks fresh status before acquiring its lease, throughout native standing
via its health monitor and before each streamed joint command. It rejects bench,
wrong profile/robot/host/boot, stale or non-armed status, changed bridge sessions,
and an endpoint absent from Spot's status. A stopped trial never auto-resumes.

## Failure semantics and qualification

On serial/host error the bridge attempts a bounded STOP RPC, writes stopped/fault
status, exits nonzero and preserves logs. SIGINT/SIGTERM and bounded completion
also send STOP. It never deregisters the endpoint on exit. If RPM or the bridge
dies before sending STOP, the robot endpoint watchdog applies. The configured
`endpoint_timeout_s` initiates Spot's soft stop; `cut_power_timeout_s` specifies the
hard-cut watchdog. These are different from the ESP32's 250 ms communication latch.
Neither an RPC deadline nor a status-file timestamp is a measured physical stop time.

Test actual response for button press, severed wire, USB unplug, ESP32 reset,
bridge SIGTERM, forced process loss, host/network loss, held reset and independent
E-stop activation. Measure end-to-end latency and rig behavior with the selected
stop level and both timeouts. Do not infer hardware qualification from unit tests,
successful firmware compilation or a green GUI label.

## Wire format v1

ASCII at 115200 baud, LF terminators, at most 160 bytes:

```text
host: Q,<16 lowercase hex random challenge>\n
ESP:  E1,<12 hex device>,<8 hex boot>,<uint32 sequence>,<uint32 uptime_ms>,<challenge>,<stop 0|1>,<armed 0|1>,<8 hex CRC32>\n
```

CRC32 is the standard reflected IEEE/zlib CRC over all response bytes preceding
the last comma. Exactly one of STOP/ARMED is true. Sequence and uptime must increase
or stay monotonic as applicable; a wrap causes the host to stop and require restart.
CRC detects corruption, not malicious devices; USB and local status are trusted
local channels. No network service, remote rearm command or GUI control is exposed.

References: [Spot E-stop service](https://dev.bostondynamics.com/docs/concepts/estop_service.html),
[Spot protobuf fields](https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html),
[ESP32 GPIO](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/gpio.html),
[Espressif32 6.9.0 framework versions](https://github.com/platformio/platform-espressif32/releases/tag/v6.9.0),
[classic esp32dev target](https://docs.platformio.org/en/latest/boards/espressif32/esp32dev.html).

# Manufacturer tablet as E-stop authority

The operator may select the provided Spot tablet as the independent stop control.
The local joystick/ESP32 bridge is then inactive. This does not disable robot stop
monitoring, command/state watchdogs, shutdown handling or the operator evidence gates.

Use this explicit robot-profile selection:

```json
{
  "estop_authority": "tablet",
  "hardware_estop": null
}
```

Retain the profile's actual identity, endpoint, firmware, host and wired interface.
Tablet mode and a local `hardware_estop` binding are mutually exclusive. Existing
profiles default to `estop_authority: sdk_endpoint`, preserving the earlier SDK
endpoint requirement. Never remove a binding merely to pass the earlier mode.

## Switching from the local bridge

With motors off, identify and stop only the repository-owned E-stop bridge and any
supervisor that would restart it. Normal bridge exit requests STOP; it does not
silently unregister or allow the endpoint. A previously registered endpoint may
still block robot operation after its process exits. Read the actual E-stop
configuration/status and hand that state to the tablet operator. Do not replace
the robot's configuration or clear another endpoint automatically.

Use the tablet profile for subsequent read-only checks and a newly prepared
deployment bundle. `spot-estop configure` and `spot-estop bridge` reject tablet-mode
profiles before device or network writes. Inspection and offline device bench
checks remain available. Do not launch an old bridge with its old profile while
the tablet is the selected authority. Retain old profiles only for explicit rollback.

## Robot status and tablet qualification

In tablet mode, preflight and the runtime health monitor require:

- E-stop service aggregate level `NONE`;
- full-state hardware and software stop entries;
- every reported stop entry to be `STATE_NOT_ESTOPPED`.

Unknown, missing or stopped states fail the check. Any existing SDK endpoint still
contributes to the aggregate stop level. The code does not register an endpoint,
send an allow/check-in, acquire a lease or change motor power while checking status.

Spot firmware 3.3 and later permits an empty SDK E-stop endpoint configuration.
Consequently, endpoint count alone cannot determine whether this mode is usable.
See the manufacturer's [E-stop service documentation](https://dev.bostondynamics.com/docs/concepts/estop_service.html).

A clear robot stop state is not proof that the tablet is connected, its button
works, or its connection-loss behavior is suitable for the trial. The existing
`hardware_estop` evidence key now describes the selected tablet's stop control,
connection loss and stop response. It remains unverified until operators supply
the corresponding evidence. The offline checker marks the *local bridge file*
as not applicable; it does not mark the tablet evidence or live readiness passed.
The independent E-stop operator uses the tablet for the physical trial.

On October 9, 2026, the operator selected tablet authority. Inspection found no
running repository E-stop bridge, no registered SDK E-stop endpoints or Keepalive
policies, motors off, and clear hardware/software stop states. No robot E-stop
configuration needed changing. Absence of Keepalive policies does not verify tablet
connection-loss behavior. These are historical observations, not current readiness.

Reverting to a local joystick/ESP32 input requires an explicit mode/profile change,
its normal commissioning procedure, a new bundle and evidence for that configuration.

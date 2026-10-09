# Tablet authority selection — October 9, 2026

The operator selected the provided manufacturer tablet as the E-stop authority
and requested that the internal E-stop code be paused. Motor activation remained
prohibited throughout this work.

## Implementation and verification

Implementation commit: `d0bab05` (full hash in the local run records).
The operator-selected local profile now uses `estop_authority: tablet` and
`hardware_estop: null`. Original joystick profiles and the old review bundle are
preserved for explicit rollback. No registered robot E-stop configuration was
changed. The active profile is local to the maintainer's host, not published.

No repository-owned E-stop bridge or user service was running when inspected;
there was no process to terminate. The new profile rejects internal bridge
configuration/registration before any device or network access. The robot's own
stop states, health and other control watchdogs remain monitored. The existing
stop-evidence gate describes the tablet's stop control and connection-loss behavior.

The complete offline suite passed: **329 tests in 6.93 seconds**, Ruff passed,
and the bundled Exploy prediction demo passed. An additional snapshot/health
integration assertion was subsequently added; all 14 targeted SDK/tablet tests
and Ruff passed again. Policy graph/weights, observations, gains, targets, history
and numerical rollout code were unchanged, so no simulator rerun was performed.

## Observed robot state

Read-only inspection found an empty SDK E-stop configuration, aggregate stop level
NONE, no Keepalive policies, and all three reported hardware/payload/software
stop states clear. Motors were off. This explains the preceding endpoint-based
preflight failure: that profile required an SDK endpoint even though modern Spot
permits none. Empty endpoints alone never enable the new mode; it is explicitly
selected and also requires clear, complete full-state stop telemetry.

The new tablet-profile preflight passed all nine checks, including identity,
firmware, joint layout, license, stowed arm, no active faults, model/payload hashes
and robot stop state. The final snapshot still reported motors off. No lease,
motion command, motor-power request, E-stop check-in or E-stop write was issued.

Passing this preflight does **not** establish tablet connectivity, button operation,
connection-loss behavior or physical policy readiness. Those facts were not tested.
The refreshed offline deployment check passed policy/runtime/profile, fresh
preflight and the not-applicable local bridge check. It remains blocked on the
unfilled hardware envelope and unreviewed evidence, including tablet stop evidence.
No gate was automatically promoted and no existing evidence was relabeled.

## Local artifacts and continuation

- Active profiles: `local/robot.json`, `local/robot-tablet.json`.
- New review bundle: `local/spot-deployment-tablet-20261009-001/`.
- Raw inspection: `runs/tablet-estop-inspect-20261009-001/` and
  `runs/tablet-estop-keepalive-20261009-001.json`.
- Passed read-only preflight: `runs/tablet-preflight-20261009-001/`.
- Offline readiness result: `runs/tablet-deployment-check-20261009-001/`.

Raw records remain ignored because they contain robot-specific identity/configuration.
Each normal CLI run records source/package/input identities and artifact hashes.
No new persistent processes were left running. For subsequent work use the new
bundle's command templates and [tablet operation guide](../../docs/TABLET_ESTOP.md).
The old joystick bundle is not the selected operating profile. Configuration or
source changes require fresh bound evidence; historical preflight records expire.

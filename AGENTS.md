# Agent instructions: ReLIC / Exploy / Spot

This file is self-contained. No maintainer home directory, private Slack channel,
private repository or hidden memory is required. Read README.md,
docs/AGENT_QUICKSTART.md, docs/EXPLOY.md, docs/ARCHITECTURE.md and docs/VALIDATION.md.
For deployment preparation read docs/DEPLOYMENT.md. Keep editable review inputs
separate from frozen candidate artifacts. Null hardware limits are intentional.

## Scope and default actions

- Work on `main` through a feature branch; use Git to track changes and open a PR.
- Use UV and the committed lock. Python runtime is 3.12, CPU ONNX Runtime, SDK 5.1.1.
- Offline tests, model inspection, replay and simulation are the default workflow.
- Never connect to a robot, acquire a lease, change motors, issue robot commands,
  register/rearm an E-stop or flash hardware as part of build/test/demo/CI.
- There is no qualified hardware policy here. Do not fabricate evidence, relabel a
  simulation envelope or remove a guard to get a test to pass.
- Physical work needs an explicit operator request and the existing standing gate,
  bound evidence, inspected rig and two distinct named operators. See OPERATIONS.
- Default scope is four-foot standing, stowed arm, zero base velocity. Explicit
  operator-requested walking requires task=walking, a bound WalkingPlan, and the
  walking_distance/walking_clearance evidence gates. See docs/WALKING.md. No
  automatic retries, fault clearing or manipulation in the hardware path.
- Three-leg dynamics rehearsal is offline via tools/simulate_relic.py and
  docs/THREE_LEG_DEMO.md and docs/THREE_LEG_IK.md. tools/prepare_three_leg_ik.py
  creates a body-fixed kinematic preview only, not RL balance evidence.
  The current nine-input Exploy graph fixes leg commands
  at zero; do not pretend it supports a selected lifted leg or dispatch the offline
  adapter to hardware. The new schema4 relic-exploy-leg / task=three-leg-demo uses
  an explicit ten-input graph and separate live adapter. CLI three-leg requires
  complete bound three_leg_sequence / three_leg_clearance evidence, exact plan.duration_s+1
  duration (currently47s for46s withtarget-rate limit), tablet stop, two operators and new one-trial approval.
  The smoothed profile uses one native actor and target_slew_rad_s=1 for all12leg
  targets afterinitialfour-foot phase through finalhold. First exact entry
  incrementzero, laterincrements<=.02rad/50Hztick, no catch-up. Preserve native
  raw ACKed history and weights/gains. Raw proposals and limitedcommands differ
  and must both be recorded. Two-second return output blend remains. Failed
  dualactor/supportlean experiments are offline-only, not enabled inlivecontract.
  Current plan46s/live47s; previous physical approval is consumed. Two60s guarded
  MuJoCo runs passed, fromrecordedphysicalpose andearlierbaseline. A smoothed
  hardwaretrialverified targetlimiting, butstopped5.34s on1.0047rad/s bodyangular
  speed against1rad/s; OFF/lease returnconfirmed. No fulltripod qualification;
  Latest cap2rad/s physical attempt stopped5.485s onpitch10.284degrees
  against10degrees; OFF/lease returnconfirmed5.015s. No tests/retries forcapchange;
  latest physicalapprovalconsumed. See records/relic-three-leg-angular2-hardware-20261012
  and records/relic-three-leg-entry-20261012. MuJoCo60s passed after preserving the original32s
  torque stop; other simulator/hardware qualification remains separate.
  The new configs/relic-three-leg-front-left-ik-unfiltered.json draft sets
  target_slew_rad_s=null. Live adapter accepts that explicit unfiltered profile;
  command_rate_guard=observe with a reason disables the target-slew stop only for
  selected-leg commissioning and records crossings. Default remains enforce.
  Existing smoothed dynamics evidence does not qualify this new profile.
  A subsequent47s unfiltered --no-guards MuJoCo diagnostic completed the full
  sequence and stable final standing without a fall; FR HX saturated45Nm for5ms.
  See records/relic-three-leg-native-recovery-20261013. This is simulation-only
  evidence and does not qualify firmware saturation or authorize physical motion.
  Retain measured speed, torque, attitude, timing and tablet stop checks. No
  physical activation is authorized by preparing this candidate; fresh exact
  one-trial approval and operator/setup confirmation are required before execution.
- Keep secrets in environment variables and robot-specific files in ignored `local/`.
  Exclude raw hardware/identity/endpoint records from public commits.

## Contracts that must remain explicit

- Standing uses nine named state inputs; selected-leg adds leg_command12.
  Explicit native input order and SDK output order differ in the selected graph.
  Native84-value observation -> 12 raw actions -> 19
  position targets. Names, quaternion wxyz, COM velocity and joint mapping matter.
- Start previous actions at zero. Rollout advances raw-action history only after
  acknowledgement. A warmup output is discarded. Do not add automatic graph memory.
- Policy rate is 50 Hz; command rate is 200 Hz. No catch-up bursts after a missed tick.
- Direct startup means zero interpolation duration. Capture initial body height.
- Preserve pinned weights, graph/manifest hashes, gains and the held arm contract.
- Measured leg-angle bounds default to enforcement. The operator-selected
  `leg_position_guard=observe` requires an explicit reason, ReLIC controller,
  manufacturer torque profile and separate finite virtual target bounds. It logs
  operating-angle crossings; it does not remove torque, speed, body, tracking,
  timing or tablet controls. See docs/FIRMWARE_LEG_POSITIONS.md. Never enable by default.
- The operator-requested harness exception is explicit `arm_motion_guard=observe`
  with a reason in the envelope. See docs/ARM_HARNESS.md. Arm motion is logged;
  all torque limits and held arm commands remain enforced. Never enable it by default.
- Read-only and command SDK adapters remain separate. Viewer stays loopback/read-only.
- The operator selected the manufacturer tablet as E-stop authority. Use an explicit
  tablet profile (`estop_authority=tablet`, `hardware_estop=null`); do not start the
  local joystick/ESP32 bridge unless the operator changes that selection. Continue
  monitoring robot stop state and retaining the stop evidence gate. See docs/TABLET_ESTOP.md.
- Joystick STOP is index 1 (button 2), separate rearm index 3 (button 4); these are
  this profile's mappings. Do not infer trigger mappings on a different device.

## Validation and handoff

Run `uv sync --locked --group simulation`, then `pytest -q`,
`ruff check src tests tools` and `python tools/demo_exploy.py` through `uv run --no-sync`.
For a changed graph, also run the independent parity procedure in EXPLOY.md.
For controller/simulator changes, use the bounded 60-second simulation protocol.
Do not change the small-pose assumptions or tolerances merely to pass a gate.

Keep code/config/model hashes, seeds, environment, raw results, failures and
interpretation. Use fresh `runs/` directories; preserve failed attempts. Separate
observed simulation results from deployment decisions. A legacy summary does not
prove a new commit passed. Update docs for material behavior changes.

For long simulations/benchmarks, use a durable process and one operational owner.
When available, delegate execution to `long_run_executor_luna` with exact command,
commit, resources, success/stop criteria and no automatic retry. Otherwise use a
local persistent process and durable logs. No paid compute is authorized by this
file. No external messaging or automatic physical activation is authorized.

## Torque diagnostics and requested-demand draft (2026-10-13)

MuJoCo's explicit `--no-actuator-torque-limits` removes
all actuator clipping while retaining contact physics/integrity. Default simulated
actuators are unchanged. Unlimited dynamics are not hardware evidence.
`leg_requested_torque_max` plus an operator reason separately caps the predicted
12-leg PD-plus-feedforward demand; Adam selected 75 N·m for a hardware draft.
Measured manufacturer, arm/coupling, feedforward and knee-table-domain checks stay
unchanged. No new physical approval; rebind source and preflight before review.
The paired unlimited-torque47s diagnostic completed/recovered; see
records/relic-three-leg-unlimited-torque-20261013. Requested knee peak75.85Nm
exceeds the new75Nm draft, and measured hip torque/speed also cross retained
bounds. No physical approval or hardware qualification. Video and precision-aware
analysis were repaired from saved data without rerunning dynamics.

## Physical-limit persistence draft (2026-10-13)

Adam selected physical_limit_persistence_s=0.5 for all physical operating limits:
continuous per-limit/per-joint violation must be strictly longer than500ms.
Recovery resets that timer; impulses do not accumulate. Default0 remains immediate.
See docs/PHYSICAL_LIMIT_PERSISTENCE.md for scope, endpoint knee-table checking,
recorded events and immediate checks. ReLIC enables the window at activation;
startup/handover qualification, data/transport, health/tablet and trial deadlines
stay immediate. Unlimited MuJoCo actuators can now run with persistence guards
enforced; --no-guards remains a separate explicit operating-stop diagnostic.
No physical approval; keep historical evidence and rebind before exact approval.
Validation is recorded in records/physical-limit-persistence-20261013:521fulltests
passed, saved47s500ms audit passed, and one60s guarded unlimited-actuator simulation
completed/no fall/stable. Three physical episodes recovered5/10/15ms (five timer
entries due redundant torque checks). Recorded119ms core outlier would violate
unchanged live10ms computation gate; no live timing qualification or physicalapproval.
Current draft: local/relic-three-leg-persistence500-preparation-20261013-001.

The subsequent operator-selected latency draft includes communication/stale data:
500ms computation, gap, ACK and health budgets;505ms TTL/state/policy age for one
command-tick margin. See records/relic-three-leg-latency500-20261013. This changes
host timing thresholds only; the119ms outlier is retained, not a timing pass.
Nominal50/200Hz rates and all other physical settings remain. Final exact approval
must disclose that the last target may remain valid for about half a second.
No physical approval is supplied by this draft or its motors-off preflight.

The subsequently approved ONE latency500 hardware attempt is consumed:
records/relic-three-leg-latency500-hardware-20261013. Native stand completed;
two preparation commands, zero policy predictions, then command stream ended
unexpectedly. Stream response status was discarded, so server cause is unknown.
OFF/lease return confirmed5.017s; fault-free motors-off postcheck passed.
No retry. Next diagnosis should preserve stream response status; do not infer
a confirmed expiry rejection or claim three-leg success.

The separately approved repeat added protocol-status reporting (no offline tests)
and now confirms STATUS_TOO_DISTANT(5) for505ms expiry; no policy predictions.
See records/relic-three-leg-latency500-repeat-20261013. Exact firmware ceiling is
unknown. Native OFF/lease return confirmed6.019s, fault-free postcheck. Repeat
approval consumed; do not run this505ms expiry configuration again unchanged.

The next draft follows the official firmware-matched SDK5.0.1 example:50ms TTL,
45ms command-gap/ACK limits. See records/relic-three-leg-expiry50-20261013.
This fixes end-time rejection only;500ms compute/stall tolerance is not achieved
with the synchronous producer and50ms firmware expiry. No offline tests requested;
new exact physical approval remains required before the proposed repeat.

The50ms expiry trial reached lifting with290policy predictions, then stopped on
"joint control no longer processing" at~5.8s; no saved specific feedback cause.
Three physical episodes recovered5/5/15ms, none triggered500ms stop. Native OFF/
lease return7.021s and fault-free postcheck confirmed. Adam requested an unchanged
repeat, saying "We jumped the gun"; no source/envelope change or offline tests.
Record: records/relic-three-leg-expiry50-20261013. Previous approval consumed.

The separately approved unchanged repeat completed the47s schedule:2350policy
samples,9397commands, full lift/hold/IK/lower/handover/final-fourfeet phases.
Seven timer crossings all recovered (max30ms); no stop. OFF/lease return6.015s
and fault-free postcheck confirmed. See records/relic-three-leg-expiry50-repeat-20261013.
One completed commissioning attempt, not blanket hardware/contact-load or IK
tracking qualification. Approval consumed; no automatic further physical trial.

Latest approved forward repeat walked0.715016m (goal0.75,tolerance0.02), lateral
0.029140m, then20s timeout before goal-and-settle completion.1000policy samples/
4001commands. Short proof-of-concept scope accepted; retain raw failed status.
OFF/lease return7.018s; postcheck passed with known payload.fault9 INFO only.
No offline tests/retry. Record: records/relic-forward-repeat-20261013. Approval
consumed; no further physical motion authorized automatically.

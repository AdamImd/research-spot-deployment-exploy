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
- Current scope is four-foot standing, stowed arm, zero base velocity. No automatic
  retries, fault clearing, walking or manipulation in the hardware path.
- Keep secrets in environment variables and robot-specific files in ignored `local/`.
  Exclude raw hardware/identity/endpoint records from public commits.

## Contracts that must remain explicit

- Nine named state inputs -> 84-value native observation -> 12 raw actions -> 19
  position targets. Names, quaternion wxyz, COM velocity and joint mapping matter.
- Start previous actions at zero. Rollout advances raw-action history only after
  acknowledgement. A warmup output is discarded. Do not add automatic graph memory.
- Policy rate is 50 Hz; command rate is 200 Hz. No catch-up bursts after a missed tick.
- Direct startup means zero interpolation duration. Capture initial body height.
- Preserve pinned weights, graph/manifest hashes, gains and the held arm contract.
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

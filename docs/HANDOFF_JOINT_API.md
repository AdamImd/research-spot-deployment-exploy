# Saved progress: next session is the joint-API crouch test

Saved October 9, 2026. The user is taking a break and requested preparation only.
**Do not start motion automatically after an hour.** Resume with the user present.

## Next objective

Use the standalone [joint-API diagnostic](JOINT_API_CROUCH.md) to hold the measured
standing pose, make a small crouch and return over ten seconds. Record joint
tracking/PD offsets, measured loads and API timing. This is deterministic reference
control without ReLIC, and the manufacturer tablet remains the E-stop authority.
Do not start an SDK/joystick/ESP32 E-stop implementation.

The script is `tools/joint_api_crouch.py`, with adjacent UV script lock and
`configs/joint-api-crouch.json`. Draft: 200 Hz; 2 s hold, 3 s lower, 1 s crouch,
3 s return, 1 s hold; hip pitch +5 degrees, knees −10 degrees; roll and arm fixed.
Leg gains 60/1.5. Constant median native load feedforward measured during a
one-second settled baseline. Review these proposed parameters before the trial.
The user specified a slight crouch, not those exact amplitudes or feedforward.

## Existing results and code

- [Native stock rehearsal](../records/stock-rehearsal-20261009/VALIDATION.md): one
  power-on/stand/sit/power-off cycle completed, exact motors-off and lease return
  confirmed at completion. Mean standing height 0.515259 m, all four feet contacting,
  no faults. Native shutdown took 3.114 s. This is a historical state observation.
- [Compact recorder](RECORDING.md): state/actions/command/timing sidecar, no robot
  connection. A saved 60 s ReLIC rollout reduced from 42.34 MB to 5.46 MB while
  preserving all 3,001 predictions and 12,001 command events exactly.
- Manufacturer-based torque profile, policy graph and hardware envelope are
  preserved. Full policy qualification and the evidence index remain incomplete.
  No physical ReLIC or joint-level crouch rollout has run.

Branches/PRs are stacked: manufacturer review [#3](https://github.com/AdamImd/research-spot-deployment-exploy/pull/3),
stock rehearsal [#4](https://github.com/AdamImd/research-spot-deployment-exploy/pull/4),
compact recorder [#5](https://github.com/AdamImd/research-spot-deployment-exploy/pull/5),
then `feature/joint-api-crouch`. Source implementation began at `ad43c8f`; repaired
clock ordering at `7078024` passed 388 offline tests, Ruff, the offline demo and
the isolated-environment plan. Later cleanup reporting refinements require their
own validation; consult the current result record, not this historical count alone.

The first mocked lifecycle validation failed because host time was sampled before
reading the mailbox. A newly arriving state appeared newer than the tick. The
repair samples host time after reading state and preserves causal startup errors.
Failed run 001 and successful run 002 remain preserved under ignored `runs/`.
Mocked perfect tracking validates orchestration, not dynamics or real-time deadlines.

## Resume checklist and local evidence

Read `local/joint-api-crouch/HANDOFF.md` for the machine-specific continuation record,
operator names, prior processes, source commits, local paths and failures. Local
`binding.json` holds only model/payload hashes; `commands.json` contains inspection
and eventual execution templates. No secrets are embedded. The selected robot and
envelope remain under `local/spot-deployment-manufacturer-20261009-001/`.

On return, review draft parameters, verify actual human/rig/tablet readiness and
run fresh read-only `inspect`. A current physical request plus explicit `execute
--execute` is required for motion. Preserve two separate named operators. Never
reuse historical preflight, steal a lease, clear faults or loosen limits to pass.

All numeric data is buffered during commands and compressed after cleanup; a host
crash can lose those samples. Preserve the original source run logs when testing
the full policy. Use one run owner, fresh directories and no automatic retries.
Shutdown measurement has a separate 10-second native timeout; the existing
2-second RL budget remains a comparison, not an attained qualification.

No paid resources, GPU or remote host is required. Do not disturb unrelated tmux
sessions. No physical motion process was started during this preparation.

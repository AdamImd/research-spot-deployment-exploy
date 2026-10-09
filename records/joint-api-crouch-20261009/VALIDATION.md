# Joint-API diagnostic preparation validation

Final runtime commit: `e16ca14771fbf0b066cfafe384b3f82d4315b670`, clean checkout.
Full suite: **388 passed in 19.27 s**. Ruff, offline Exploy demo and the isolated
locked UV `joint_api_crouch.py plan` all passed. Plan reports no hardware access,
zero commands and no RL shutdown qualification. No physical run was performed.

Validation includes mocked normal 10-second stream lifecycle and activation failure
with shutdown/lease return, geometric trajectory/continuity, named-joint wire data,
PD decomposition, bounded memory and exact motor-OFF checks. Mock tracking is ideal
and timing limits are relaxed only inside the unit-test fixture; no physical timing
or dynamics conclusion follows. Physical configuration remains unchanged by tests.

Failed validation 001 is preserved: 386 passed, 2 lifecycle failures from clock
ordering before mailbox read. Commit 7078024 repaired it and validation 002 passed.
Final validation 003 covers normal-stop race handling and shutdown-reporting
refinements too. All three directories remain under ignored runs/.

Durable final artifacts: runs/joint-api-crouch-validation-20261009-003 and
runs/joint-api-crouch-plan-20261009-003. Commands, source/environment, outputs, logs,
exit codes and hashes are saved there. The durable session exited after completion.
No paid compute, GPU, SSH, robot connection or physical commands were used.

The last native-standing capture fits the proposed joint-position path with a
minimum remaining margin of 0.00661 rad. This is a geometric check only; use fresh
preflight and review the draft amplitude/feedforward before the next session.

Reproduce offline: `uv run --no-sync pytest -q`, `uv run --no-sync ruff check src tests tools`,
`uv run --no-sync python tools/demo_exploy.py`, and
`uv run --script --locked tools/joint_api_crouch.py plan --output runs/crouch-plan-NEW`.
The test has no random seed. Read docs/JOINT_API_CROUCH.md for physical protocol.

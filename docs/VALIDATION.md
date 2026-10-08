# Validation and remaining limits

## Exploy integration, October 8, 2026

These measurements were collected before publication from frozen deployment
commit `0a023ef7d49fd4c6bd6e4fc5e582541605bd58ab` and simulator commit
`74cb498719164d4c6e698a1f056db5ca348f60d2`. The exported graph remains byte-identical.
The public snapshot records source provenance in `records/source-provenance.json`;
private historical commits and robot operational records are not in public Git history.

| Check | Observed result |
| --- | --- |
| Software | 303 tests passed; Ruff passed |
| Parity | 128 varied + 6,002 recorded states passed |
| Maximum observation error | 1.639e-7, tolerance 1e-5 |
| Maximum raw-action error | 4.768e-6, tolerance 1e-4 |
| Maximum target error | 9.537e-7 rad, tolerance 3e-5 rad |
| MuJoCo | 60 s completed; final standing passed; 7.067x real time |
| Isaac Lab CPU physics | 60 s completed; final standing passed; 1.303x real time |
| Policy input/inference/output | p99 0.424 ms, max 1.590 ms |
| Full command production | p99 0.925 ms, max 4.026 ms |
| Scheduled release to command completion | p99 0.998 ms, **max 5.920 ms** |

Numerical summaries: parity (historical archive; see [public validation](VALIDATION.md)),
MuJoCo (historical archive; see [public validation](VALIDATION.md)),
Isaac (historical archive; see [public validation](VALIDATION.md)),
CPU timing (historical archive; see [public validation](VALIDATION.md)).
Source summaries contain omitted local-path markers; full historical raw traces
remain in the maintainer's experiment archive. The clone supports a new recorded
reproduction through the agent guide and simulator instructions.

Both simulator trials start directly from the same captured standing pose with
zero velocities/history, unchanged weights/gains, four-foot commands and held arm.
Each produced 3,001 policy samples. Per-frame parity maxima were 5.960e-8 observation,
9.537e-7 action and 2.384e-7 rad target. Final ten-second mean heights were 0.476536 m
and 0.481797 m; maximum late-window tilts 0.956 and 0.621 degrees. These results do
not show that startup movement disappears or establish physical actuator equivalence.

The 60-second host benchmark produced 11,999 commands and 3,000 predictions,
with **one response over 5 ms and one omitted nominal release**. The late tick woke
4.913 ms late, then spent 1.007 ms producing its command. This locates most of that
response in wake-up delay, without isolating the OS/thread cause. State/ACK timing
was ideal; SDK serialization and async recording were real; network transport was
absent. There was no retry to select a better result. Isaac's average throughput
also hides individual intervals over 5 ms. Strict 200 Hz physical timing remains
unqualified. No C++ scheduling improvement has been demonstrated by this branch.

## Interpretation and next gate

The Exploy graph preserves the released controller numerically and supports
standing in these two simulated trials. The host, network, E-stop response, physical
model and full operator evidence still need their registered checks. Simulation
results and a built support rig do not automatically satisfy those gates.

Reproduce software and graph checks via [AGENT_QUICKSTART](AGENT_QUICKSTART.md).
Reproduce the simulation protocol via [SIMULATION](SIMULATION.md).
Read [OPERATIONS](OPERATIONS.md) before any expressly requested physical work.

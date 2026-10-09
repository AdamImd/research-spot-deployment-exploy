# Stock rehearsal tool validation

Implementation commit: `dac6746`. Full offline suite: 358 passed in 6.95 seconds;
Ruff and bundled offline Exploy prediction demo passed. An additional explicit
STATE_OFF assertion was then added to the existing shutdown test; the targeted
suite passes 14 cases and Ruff passes. This test-only addition changes no runtime.

The tool is a native-controller baseline recorder, separate from ReLIC deployment.
A requested read-only robot inspection timed out before a snapshot; physical
execution was not launched. No stand/sit timing, power transition or native
standing measurements were obtained. Connection failure is not qualification
evidence. Robot-specific raw logs remain in ignored local run directories.

Protocol and limitations: [stock rehearsal](../../docs/STOCK_REHEARSAL.md).

## Physical stock rehearsal after connection restoration

Runtime commit `96a7052e1320e6cde40ac400847c4fa37794e3ea`, clean. Fresh read-only
inspection passed with motors off, arm stowed, stop state ready and no active
faults. One authorized native power-on/stand/10-second observation/sit/2-second
observation/safe-power-off run completed with motor-off confirmed and lease returned.
No policy inference, joint-command activation, E-stop writes or fault clearing.

Power-on 4.286 s, stand 1.896 s, sit 1.684 s, native safe-power-off 3.114 s.
Standing mean height 0.515259 m; range 1.098 mm; maximum tilt 0.198 degrees.
All 97 standing health samples reported all four feet in contact. All 223 health
samples were fault-free. Incoming state stream: 7,676 packets, median acquisition
interval 3 ms; receive gap p99 4.368 ms, max 8.292 ms. Sampled state age max 7.405 ms.

[Sanitized measured summary](hardware-summary.json) includes per-joint standing
statistics and artifact binding. Raw robot-specific records and reproducible
analysis remain locally in ignored runs/stock-rehearsal-20261009-001 and
runs/stock-rehearsal-analysis-20261009-001. The earlier failed inspections are
preserved, not replaced. No retry of a physical execution occurred.

This establishes the native baseline only. Native RPC acceptance is not joint-stream
ACK timing; no actual E-stop press, outage, RL handover or RL shutdown was tested.
The 3.114-second native shutdown does not qualify the separate 2-second RL budget.

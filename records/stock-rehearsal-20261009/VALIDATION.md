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

# Lightweight recorder validation — October 9, 2026

Question: can a separate-process recorder preserve policy/command data while
reducing the size of a rollout artifact without altering the control path?

Implementation: clean commit `974fd1003cadddba81ab99bd089839f575038567`.
Environment: existing locked UV Python 3.12 environment; gzip compression level 1;
50 Hz redundant-state sampling; no rounding, seeds or robot connections.

All 372 tests passed in 7.26 seconds. Ruff and the offline prediction demo passed.
Recorder tests include exact prediction/input retention, full command retention,
state sampling, live partial writes, failed source status, bounded duration,
interruption, malformed input, hash mismatch and overwrite protection.

| Source | Source events | Compressed events | Reduction | Conversion time |
| --- | ---: | ---: | ---: | ---: |
| Completed physical native rehearsal | 3,890,857 B | 736,574 B | 81.07% | 0.205 s |
| Existing 60 s MuJoCo ReLIC rollout | 42,337,349 B | 5,458,340 B | 87.11% | 2.090 s |

The ReLIC check matched every retained prediction's input state, raw actions,
targets and inference time to the original, and every command's targets,
feedforward, key and expiry. All 3,001 predictions, 12,001 command events and 3,000
policy acknowledgements matched exactly. Native rehearsal has no policy actions;
none were synthesized. Source completion event hashes were verified during capture.
[Machine-readable validation](validation.json) binds source event hashes.

Reproduce from the preserved local raw runs with:

```bash
uv run --no-sync python tools/record_rollout.py \
  --run runs/stock-rehearsal-20261009-001 --output runs/stock-recording-NEW
uv run --no-sync python tools/record_rollout.py \
  --run runs/manufacturer-limits-validation-20261009-001/mujoco \
  --output runs/relic-recording-NEW
```

The actual outputs are `runs/stock-recording-20261009-001` and
`runs/relic-recording-20261009-001`. These paths and raw records are local, not
public fixtures. The public unit tests reproduce the boundary checks without
hardware or those archives. A deterministic conversion does not require a seed.

Decision: the recorder is usable as a compact data sidecar. Existing mandatory
control logs remain unchanged. File-size results cover the compressed event file,
excluding small metadata files. Conversion timing is an offline observation, not
a benchmark of simultaneous recording's effect on hardware control deadlines.
No new physical rollout, real-time qualification or safety-evidence promotion
was performed.

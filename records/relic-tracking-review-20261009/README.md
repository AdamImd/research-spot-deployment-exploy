# Front-right hip tracking review

After the recorded repeat stopped on `fr_hx` target-to-measured tracking error,
Adam explicitly instructed: “Increase to 0.75.” The revised private envelope
changes only `tracking_error_max[3]` for front-right hip abduction from 0.50 to
0.75 rad. Other hip limits, knee limits, position bounds, all torque protections,
timing, arm targets and the explicit arm-observation mode remain unchanged.

The old envelope and both physical recordings are preserved. New review input:
`local/relic-tracking-review-20261009-001/envelope.json`; new prepared candidate:
`local/relic-standing-tracking-075-20261009-001`. The candidate's evidence index
starts empty, so it cannot reuse approval bound to the preceding envelope.
No physical trial, motor command or robot access occurred in this review.

Offline audit at clean commit `eb175663e1b4014826eca4818fdcdae6079b0749` verified
both source recordings' hashes. All 2,003 commands from the successful trial
and all 39 emitted commands from the failed repeat pass numeric guards under
both envelopes. The failed repeat's recorded rejected prediction still fails
the original 0.50 rad tracking guard, and passes all command guards with the new
0.75 rad limit. Its actual gap was 0.504219 rad. Recorded target, velocity and
feedforward values were checked, including predicted PD load and manufacturer
actuator limits; there was no policy recomputation or dynamics rollout.

This establishes that the revised bound admits that particular recorded command.
It does not establish how later predictions or physical motion would evolve.
Historical sender times and idealized age/policy clocks do not validate live timing.
Any new physical attempt requires an explicit request, fresh preflight and new
matching review evidence. No automatic retry is introduced.

Sanitized output is `audit.json`. Private source/procedure/artifact hashes and
completion are in `runs/relic-tracking-audit-20261009-001`. Recompute with private
artifacts present using the saved script and a fresh output directory:

```bash
uv run --no-sync python local/relic-tracking-review-20261009-001/audit_recorded_targets.py
```

The saved script's default output directory is the existing audit; change it to a
new directory before reproduction. The original audit/completion remain immutable.

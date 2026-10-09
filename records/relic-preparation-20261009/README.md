# ReLIC deployment preparation, October 9

## Protocol and results

Question: does the recorded native standing baseline produce plausible, guarded
direct-start ReLIC commands, and can the current CPU host execute the real policy,
recorder and SDK encoder at 200 Hz? No physical activation was authorized or used
for this preparation. Runtime 35e20271e190804ae1065e161a54e1ec6e399cb7 was clean;
UV locked Python 3.12 environment, CPU only, no paid compute. No parameter sweep
or randomized replication. Simulator configuration/seeds and source hashes are
preserved in the raw campaign, with the standard captured pose and 60 s protocol.

| Check | Result |
| --- | --- |
| Unit/regression suite | 405 passed; Ruff and demo passed |
| Independent zero-history startups | 197/197 recorded native samples pass current guards |
| Predicted leg PD torque range | -9.09 to +23.28 N m |
| Largest absolute handover step, legs / all joints | 9.20 / 10.96 N m |
| MuJoCo, 60 s | Completed; final standing window stable |
| CPU command benchmark, 60 s | 12,001 commands, 3,000 predictions, zero missed releases |
| Command production p99 / max | 0.949 / 4.023 ms |
| Release-to-completion p99 / max | 1.022 / 4.424 ms |
| Independent actor action parity in benchmark | Maximum error zero |

The startup audit uses captured height 0.5191 m and actual velocities. Each sample
is an independent startup with zero action history, a supported measured-pose
command, and ideal next-tick acknowledgement. It is not a closed-loop hardware
rollout. The MuJoCo and host benchmark use the simulation envelope with selected
manufacturer actuator limits; those limits do not replace the hardware envelope.
The benchmark includes real async logging/serialization but ideal clocks and ACKs,
without network transport. The read-only live dashboard was concurrently active.

The original campaign wrapper incorrectly expected `passed/success` from a
simulator artifact whose valid status is `completed`, and stopped scheduling
before timing. The unrun timing stage was subsequently launched exactly once.
Original bookkeeping is retained; FINAL_SUMMARY.json and the stage-specific
hash-bearing completion records establish the actual outcome. No experiment retry.

## Findings still requiring review

The first policy commands pass the existing envelope, but the candidate's 200 N m
per-joint handover-step cap still needs review independently from absolute torque
limits. Do not infer that the configured step threshold was physically qualified.

The static-support preparation model has an arm discrepancy worth resolving:
at the final captured native state, predicted supported-hold elbow torque is
-11.09 N m versus measured +2.45 N m. The 13.54 N m difference is much larger
than the other contemporaneous joint differences. This is before the zero-
feedforward policy phase; it needs investigation of support-model versus hardware
load convention/arm behavior, not an unreviewed sign flip. It did not violate the
existing numerical guards. Passing those guards alone does not validate support.

Native safe-power-off remains a timing/consistency issue: the previous successful
diagnostic took 6.02 s, beyond the 2 s policy envelope, and the earlier attempt was
unconfirmed after 10 s. The code now records stages and requires explicit STATE_OFF;
that improves evidence, not shutdown speed. No physical verification of this code
change has been performed. Hash-bound reviews, combined policy/transport behavior
and a fresh immediate pre-trial preflight remain outstanding.

## Reproduction and local operational state

Raw campaign: runs/relic-preparation-validation-20261009-001. Its run.sh and timing
stage artifacts contain exact commands, logs, timings and hashes. See results.json
here for a sanitized summary; all original robot records remain ignored locally.
The protocol and audit command are in docs/RELIC_TRIAL_PREPARATION.md.

Local candidate: local/relic-trial-preparation-20261009-001. Its session-review.json
contains concrete operator names, ten-second parameters and launch/recording
argument templates. No launch command was executed. The new evidence index remains
unverified. Fresh preflight runs/relic-prep-preflight-20261009-001 passed motors-off;
runs/relic-prep-check-20261009-001 records the outstanding evidence gates.

The read-only watch/viewer and explicitly requested eduroam gateway run in separate
bounded tmux sessions for up to one hour, without restart. Exact private network
address, PIDs and shutdown commands are in local/ACTIVE_DEPLOYMENT.md. The dashboard
shows measured state only. It does not send policy commands or E-stop requests.

Decision: preparation tests passed; physical activation remains unqualified.

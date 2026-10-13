# Unfiltered three-leg commissioning preparation

Adam requested removal of target rate limiters and preparation for another trial.
This is a review draft; no robot connection, lease, power or command was issued.

The new `configs/relic-three-leg-front-left-ik-unfiltered.json` sets the demo
filter to null. The selected-leg adapter passes native actor/IK targets through;
no entry target interpolation or rate clipping is applied. The two-second return
blend and smooth six-second Cartesian lift/lower remain trajectory definitions.
Raw ACK-owned action history, weights, graph, gains and 50/200 Hz rates are unchanged.

A private proposed envelope sets `command_rate_guard=observe`, with the user's
reason, to log rather than stop on target-rate crossings. It retains the prior
2 rad/s angular cap, 10-degree attitude caps, measured speeds, virtual target/PD
bounds, manufacturer torque limits, timing/ACK/expiry and tablet E-stop contract.
There is no removal of measured-speed or timing safeguards. No infinite envelope
numbers or giant substitute rate caps are used. Default command-rate enforcement
remains unchanged for all existing profiles. This exception requires an explicit
reason and the selected-leg demo adapter/task; compact recordings retain its mode
and crossing events.

Focused software checks: 91 passed. Full regression initially480passed/1failed on
the newly changed JSON schema; schema regenerated and that check passed afterward.
Ruff and diff checks passed. No new dynamics rehearsal or physical test is part
of this preparation. Existing smoothed simulation evidence is not promoted to
unfiltered profile qualification. The profile can reproduce abrupt virtual targets
and the earlier torque stop; no guard has been widened to avoid that outcome.

Private artifacts: `local/relic-three-leg-unfiltered-preparation-20261013-001`.
The deployment bundle will freeze the committed source, graph, manifest, envelope,
robot profile and runtime. Evidence remains empty pending exact review; the
previous one-trial approvals are consumed and are not copied. Before activation:
fresh motors-off preflight, explicit review of this new behavior/evidence scope,
and exact human approval immediately before the one physical attempt. Adam and
Minghao/tablet/rig/workspace must be freshly confirmed. No automatic retries.

Nominal review profile remains47s: native stand;5s four-foot hold,6s lift,3s hold,
12s IK square,6s lower,2s return blend,12s final hold plus1s margin. Shutdown10s.
This document and packaging do not authorize that physical sequence.

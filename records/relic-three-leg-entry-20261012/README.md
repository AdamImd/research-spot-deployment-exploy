# Smooth entry into selected-leg ReLIC, October12,2026

Question: remove the deterministic mode-entry target discontinuity seen in the
single physical trial, without changing weights/gains/manufacturer torque limits.
No physical execution or new physical approval is part of this task.

Method: add2s three-leg-handover after5s initial standing. Hold selected-foot body
position while evaluating the SAME graph withzero andselected-leg command, same
measured state and ACKed previous raw vector. Quintic blend ALL leg targets and raw
actor actions from four-foot toselected branch. The returned mixed raw vector only
advances history after SDK ACK, so the unused selected branch is not prematurely
fed back as if fully emitted. Held arm identical inboth branches. Subsequent6s lift,
3s hold,12s square,6s lower,2s return blend,12s final hold unchanged. Total48s;
future live timeout49s. This is a documented adapter/history-mixture change, not
retraining or input interpolation through nonsensical near-zero absolute leg poses.

Same-state replay of rejected physical prediction: atblend entry, max target jump
and raw jump zero; FRHX request10.1705Nm, previously46.4231Nm. This proves initial
continuity, not closed-loop balance or torque compliance throughout transition.

Protocol: first one60s guarded MuJoCo rehearsal from recorded physical pre-entry
joints/orientation, zero simulated velocities and zero initial policy history.
Root placement0.442334m inferred from medianURDF stancefoot/world vertical plus
foot radius; this is model-ground alignment, NOT measured floor height. Retain
original latched torsoheight request0.518528m as a separate input. If that passes,
run the earlier physical standing capture as a baseline60s regression. Same guards,
no-slip10, unchanged graph/actor/gains, seed101. Compare independent native BOTH
branches and blended targets/actions, full sequence/tracking/final standing; render
recorded dynamics. Preserve failures, no automatic retry or guard relaxation.

Ignored raw inputs/candidates/contracts: local/relic-three-leg-entry-20261012-001.
Raw results: runs/relic-three-leg-entry-20261012-001. Source commit and input/config
hashes are recorded in the bounded execution contract; no robot clients or motion.

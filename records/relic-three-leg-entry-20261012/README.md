# Smooth entry into selected-leg ReLIC, October12,2026

Current implementation: Method005, ONE native actor with1rad/s command-target
limiter; raw ACK history/weights/gains unchanged. Both60s guarded MuJoCo cases
passed. Earlier methods below are preserved failed experiments, not active live
behavior. No physical trial was run for this fix.

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

Revision: the initial held-foot entry trial stopped6.585s on angular speed1.2357
rad/s (linear0.2097m/s), before fullmode/lift. Numerical native parity passed; first
command continuity held. Baseline skipped; failed recording preserved. The fixed
foot can constrain body weight transfer. Test an overlapping2s modeblend during
the first2s of the existing6s Cartesian lift, not a separate foot-down hold. The
initial Cartesian velocity/acceleration remainzero. The foot can unload as the
policy shifts support. This restores46s plan/47s margin but does NOT restore the
consumed physical authorization; changed source/history mixture still requires
new evidence and finalapproval. No limit changes or automatic retries.

Overlapping2s blend still crossed angular-speed1rad/s at6.49s (002), before full
selected mode. Preserve this outcome. Next bounded revision slows mode transfer
to4s, still overlapping the same6s lift and keeping total46s, to allow more time
for weight redistribution. No torque/speed bound changes, no new random seeds,
no alternate history method or hidden branch rescaling. Native parity remains
independent; physical-stance gate must pass before baseline case.

004 support-position hypothesis:4s overlap still crossed angular-speed cap7.29s
(003). Time alone did not resolve support redistribution. Add a4s four-foot
support-transfer phase before mode entry: existing native torso roll+0.08rad,
pitch−0.08rad requested smoothly, toward right/rear support in the leg/body model.
Keep that orientation request through tripod/lowering, then smoothly restorelevel
in the first4s of final12s hold. No direct body translation/COM claim; actualmodel
pose/contact response must be measured. Singleactor duringfourfootlean; existing
4s target/raw mix overlaps6slift afterward. Total50s/future51s margin. No hardware
motion/approval, newbounds or retraining; notqualified by this geometric hypothesis.

005 final method revision: support lean also crossed body angular cap(00411.345s).
Trace analysis found feedback predictions changing knees~0.35rad/20ms(~17.5rad/s),
even though blend coefficients were smooth. Replace that experimental method:
use ONE unchanged native actor, no entry mixture or support lean. Rate-limit all
12leg virtual targets to1rad/s AFTER mode entry through finalhold. Cache preceding
four-foot output; first exact mode-entry increment iszero; later increments<=0.02rad
at50Hz. No catch-up after late inference. Raw history returns to the native actor's
unmodified ACKed outputs, initialzeros unchanged. Plannedfoot IK/lift/return smooth,
original46s/47s margin. Raw proposals AND limitedcommands recorded separately.
This is explicit target postprocessing, not a claim the raw actor predictions stop
changing. Native raw/obs parity and independent limitedtarget parity checked.
Existing manufacturer/body/timing/speed/torque guards still inspect limited commands
and measuredstate. No physicalexecution, newbounds or qualification assumed.

Method005 passed BOTH60s guarded MuJoCo cases fromfrozen96501f8. Entire46s sequence,
IK motion tracking/clearance/return/final-standing passed. Post-entry target rates
max1rad/s, stepsmax0.02rad. Body angular max0.8783rad/s(recordedpose) and0.9184rad/s
(baseline); IK tracking p95~0.01915/0.01910m. Raw/obs/native reference limitedtarget
parity passed. Separate live adapter exactly matched3,001 recorded predictions.
Both recorded videos60s/50fps. Nominal5ms timing outliers retained; no physical
qualification. Public case/result files and exact-state continuity retain these
observations; superseded methods/failed recordings remain for audit. Final live
contract excludes entry mixtures and support lean. This fix requires NEW exact
operator approval before any hardware run; no motors/robot IO occurred here.

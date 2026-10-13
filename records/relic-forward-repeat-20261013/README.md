# Forward walking repeat preparation

Adam requested the forward walking script again, without offline tests. Reuse
the last operator-accepted walking candidate and embedded plan:0.75m goal,
tapered request up to0.30m/s, acceleration0.10m/s²,3s initial zero-speed hold,
1s final hold and20s policy-active limit. Weights, gains, direct startup and
zero initial ACK-owned raw actions remain unchanged. The goal is latched from
the current robot heading after native standing, not the boot-time heading.

Use the current reviewed physical/timing settings:75Nm requested leg demand,
manufacturer measured-torque bounds, greater-than500ms physical persistence,
body0.8m/s/angular2rad/s, hips12/knees-and-SDK24rad/s, tracking2rad, existing
leg-position and harness arm-motion observation modes,50ms expiry and45ms gap/ACK,
500ms computation/state-gap/health and505ms state/policy age. The50ms expiry does
not permit500ms command-producer stalls. Tablet remains sole E-stop; the exact
known payload INFO exception remains, with all other faults blocking.

Walking retains its enforced target-slew guard (legs200rad/s, arm1rad/s);
the selected-leg-only target-rate observation exception cannot be used for this
task. There is no added target limiter. Walking lateral/heading limits are0.10m/
0.15rad, pose/joint age20ms, pose step0.025m and progress-stall window5s, as in the
embedded plan. The trial stops on a guard, tablet/health issue, completion or20s
timeout and attempts native safe power-off with10s allowance. Short travel was
accepted for the previous proof of concept; do not report goal reached when short.

Previous physical travel was0.692539m and stopped on health; see
`records/relic-firmware-leg-angles-20261012`. The recently completed three-leg
commissioning run is historical evidence of the current runtime/timing profile,
not evidence that this walking configuration passed a new rehearsal. No offline
tests, simulation, replay or benchmark are run for this repeat.

Private packet: `local/relic-forward-repeat-20261013-001`. Prepare a frozen bundle
and motors-off preflight, then obtain the requested exact one-trial confirmation
with Adam operating, Minghao on the functioning connected tablet, and the attached
fall-arrest rig able to follow the full clear0.75m floor/arm path. No retries or
automatic physical activation are authorized by preparation.

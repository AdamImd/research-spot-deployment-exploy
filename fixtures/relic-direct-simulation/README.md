# Direct-start diagnostic, simulation only

This fixture removes pose interpolation: `kind: direct`, preparation duration and
envelope transition both zero. At activation ReLIC evaluates the captured standing
state with zero previous actions. Before activation, the runtime retains its
supported measured-pose hold; no policy warm-up occurs. Direct handover checks
pose drift against that initial pose, not the unvisited nominal reset pose.

The same weights, gains, arm, height request and 50/200 Hz rates apply. For this
observational diagnostic the per-joint handover torque-step allowance is 200 Nm,
matching the existing broad simulation load envelope, rather than the earlier
30 Nm prepared-pose allowance. This lets the requested direct startup be observed;
it is not a new approved physical limit or a transition-qualification pass.
Other diagnostic state/load/fall bounds remain unchanged. The envelope's
`scope: simulation` prevents physical activation with this fixture.

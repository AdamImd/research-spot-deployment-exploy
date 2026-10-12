# ReLIC MuJoCo / Isaac physics attribution — 2026-10-12

## Question and scope

Adam asked why the MuJoCo and Isaac Sim rollouts differ. The active request remains
simulation-only; no robot connection, authentication, lease, motors or E-stop work.
Hypothesis: the engines' contact handling changes measured state before the actor
can produce a different prediction. Compare contact/friction and integration,
check model/actuator equivalence, and repair any concrete conversion defects.
Do not calibrate hardware limits or qualify a policy from these diagnostics.

## Findings supported by controlled comparisons

The initial q, gravity, zero velocities, previous actions and first prediction are
identical. Dynamics diverge in the first 5 ms, before the next 50 Hz policy update.
On the floor, the largest first-step leg velocity difference is **7.83941 rad/s**:
rear-left knee +7.71692 in MuJoCo versus −0.122486 in Isaac. In the 20 ms air probe,
placing the same pose two metres above the floor, the largest first-step leg
velocity difference is **0.00599957 rad/s**. Both have zero foot normal force in
the air. This isolates the ground-contact response as the dominant initial leg
mismatch; it does not assert the simulators are identical in free flight.

The captured pose has foot-sphere clearances −2.301, −5.283, +1.685 and −1.608 mm
(FL/FR/HL/HR) relative to the simulated floor. Thus three feet initially penetrate,
while the hind-left foot is initially outside MuJoCo's contact set. Initial foot
centres agree between engines within about 0.2 micrometres. The capture's reported
physical contacts do not guarantee a penetration-free pose under this URDF.
Different contact detection/penetration handling immediately changes load sharing.
Do not attribute every discrepancy solely to this initial penetration: friction
and gripper self-contact also differ, and no penetration-free standing rollout
was run in this protocol.

| Controlled 60 s cell | Fall proxy | Final forward travel | Peak leg speed |
|---|---:|---:|---:|
| MuJoCo default, 5 ms / no-slip0 | 8.995 s | 0.2461 m | 26.8559 rad/s |
| MuJoCo same inputs, only no-slip10 | none | 0.6696 m | 16.8958 rad/s |
| MuJoCo no-slip0, only 1.25 ms physics | 13.505 s | 0.4696 m | 20.6269 rad/s |
| Isaac original 5 ms physics | none | 0.6768 m | 10.0758 rad/s |

All use prewarm100, seed101, original weights/gains and operating stops disabled.
The finer-step cell holds the same PD torque for four substeps; PD/target-delay
updates remain 200 Hz, actor 50 Hz. Thus a change in controller update rate is not
confounded with integration. Smaller steps delay but do not resolve the fall.
No-slip10 alone changes the outcome. This setting addresses friction drift from
MuJoCo's soft constraints; it is not a general equivalence or physical calibration
claim. [MuJoCo no-slip documentation](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#option-noslip-iterations)
explains the post-solve drift suppression. PhysX instead applies Coulomb friction
through contact patches/anchors and TGS iterations; the same coefficient 0.8 does
not specify the same contact dynamics. [PhysX friction documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html#friction-and-restitution).

MuJoCo uses implicitfast/Newton100, pyramidal condim3 contacts, solref (0.02 s, 1),
solimp (0.9,0.95,0.001,0.5,2). Isaac uses PhysX TGS with articulation position
iterations4/velocity0 and max depenetration velocity1 m/s. Contact compliance,
margin and friction algorithms have not been made equivalent. First-step foot
normal loads differ, but Isaac's recorded forces are preceding-step impulse
averages and MuJoCo's are instantaneous solved forces: they cannot be equated
pointwise. Both upright runs still stall before the 0.75 m goal.

## Actuator, model and policy checks

An independent 400-case native/NumPy audit covers all19 joints, two delay values
0/3, PD, knee angle limits and torque-speed saturation. Maximum effort errors are
**1.69277e-5 / 1.88351e-5 Nm**. It passes. Total mass is 39.067433 / 39.067410 kg;
maximum link-mass error4.89e-5 kg, inertia-eigenvalue error3.72e-7 kg·m², armature
error2.24e-10 kg·m². The largest COM offset mismatch is 36.6 micrometres on the
negligible-mass fixed jaw; leg COM errors are below0.3 micrometres. These checks
provide no support for a major mass/gain/order mismatch as the cause.

At20 ms, the different states produce a maximum raw-action difference0.309495
(dimensionless, target difference0.061899 rad under scale0.2). Offline input
substitution into the same released actor gives identical actions when the full
observation is identical. Partial substitutions change the difference but are
nonlinear counterfactuals, not additive percentages of causal contribution.

Instrumentation itself is read-only: q and issued targets over all12,001 samples
match each previous verified backend recording exactly. The no-slip10 trajectory
also exactly matches the previous prewarm0 version, confirming that discarded
warmup outputs do not alter history or explain its survival. These exact comparisons
were checked before interpreting the interventions.

## Separate gripper collision/filter difference

The arm is held, but all arm joint states still enter the native 84-value policy
observation. In the air, first-step gripper velocity is −0.155047 rad/s in MuJoCo
versus −9.731568 rad/s in Isaac. This difference survives removing the floor.

The fixed jaw is welded to the wrist in MuJoCo. Its parent filtering treats that
weld group as the parent of the movable finger, suppressing finger/jaw contacts.
The actual collision geoms overlap by up to about5.15 mm at the captured closed
pose, despite the normal MuJoCo contact list containing no such pair.
[MuJoCo collision filtering](https://mujoco.readthedocs.io/en/3.3.7/computation/index.html#selection)
documents welded-body parent filtering and explicit geom-pair overrides.

A separate20 ms **fixed first-target** pulse, above the floor, measures native
finger→jaw force using a filtered ContactSensor. Isaac reports **42.7264 N**
normal-force norm during the first5 ms, then9.3835/5.6980/5.8930 N at subsequent
ticks. Add only the12 finger/jaw geom pairs to a separate generated MuJoCo model,
preserving friction and other parameters: contact counts become4/4/4/3/2, and
first-step gripper speed changes from−0.1550 to−1.1251 rad/s. Different contact
filtering is demonstrated; the remaining magnitude difference is consistent
with different contact resolution/compliance and is not independently isolated.
No default collision-filter change or closed-loop gripper-pair promotion is made.

## Concrete converter defect fixed, with outcome check

MuJoCo's URDF import keyed visual/collision assets sharing `arm_link_wr1.obj` by
basename. The compiled wrist collider used the **visual** file: its maximum link-x
was0.235654 m versus the intended/native0.146100 m, an extra **0.089554 m**.
Generated collision copies now have `collision__` names. Source visual/collision
assets and explicit inertials are preserved. A regression checks asset separation
and every collision mesh's bounds against referenced source vertices transformed
by its URDF origin/scale, including mirrored upper-leg origins.

Repeat both60 s MuJoCo cells and the20 ms air probe with only this repair: raw
rollout hashes are exactly identical to the corresponding pre-repair recordings.
The conversion bug was real but did **not** cause this experiment's walking
failure. It is repaired without using an improved score as justification.

## Protocol, source, environment and failures

- Initial instrumentation source: clean4bfc6b02824e0a860c76fa63f6fff28f03e71852.
- Auxiliary-audit recovery source: clean250ce533dc222b4f54d91545077a263af0949fc1.
- Converter repair and final453-test acceptance: clean09426131f6c196cd084255fbd1909ebb36ecf30d.
- Original candidate/capture/reference envelope:
  `local/relic-unguarded-walk-20261012-001` and
  `local/relic-walk-inputs-20261009-001/capture`.
- Base commands, seeds, hypotheses, caps and resources were registered in
  `local/relic-physics-attribution-20261012-001/contract.json` before execution.
  Policy/graph SHA and gains are the same as the previous unguarded diagnostic.
- Main environment: locked UV Python3.12.3/NumPy2.2.6/MuJoCo3.3.7/ORT1.23.2.
  Native Python3.11.15/NumPy1.26.4/IsaacLab0.54.3/IsaacSim5.1.0.0,
  Torch2.7.0+cu128 (CPU policy/physics). Native audit extras are pinned in
  `configs/native-physics-audit-requirements.lock`; installing them leaves NumPy
  unchanged. No host/global driver or system package changes.
- Hostcs-u-rpm-dt-03; CPU2threads, GPU0 only for native Kit; noGPU1 or paid compute.
  Durable tmux, per-cell120/240 s caps, bounded campaign supervisors, no automatic
  retries. No network except the authorized locked project-scoped PyPI install.

The first native60 s physics recording finished, then its auxiliary audit failed
on missing MuJoCo in the Isaac overlay. Kit returned process exit0 while an ERROR
and no COMPLETE showed failure. This is preserved as an **auxiliary-stage failure**;
it must not be promoted as a fully passed run. Its complete12,001-row physics trace
matches the previously verified Isaac trajectory exactly and is used explicitly
as raw data with that provenance limitation. A separate audit-only tool repairs
this dependency issue without repeating the60 s physics. The runner now saves
result.json before auxiliary audits; consumers still require COMPLETE/noERROR.

The executor also incorrectly reused the native60 s command for its first “air”
cell, retaining the audit and omitting height offset2. That cell is invalid for an
air comparison and remains archived. A primary-authored array supervisor executes
the corrected exact air command in a new run. This is a contract correction,
not a scientific retry or hidden replacement of failed artifacts.

Final453 tests, Ruff and offline Exploy demo pass; independent actuator400 cases
pass. Recorded physics data are finite, parity checks retain their original
thresholds. Raw recordings/captures remain ignored; public summary contains
sanitized metrics/hash references, not hardware identity or authentication data.

## Artifact locations and reproduction

- `runs/relic-physics-attribution-20261012-001`: original four60 s/air cells,
  metadata/contact forces, failed audit and invalid original native air attempt.
- `runs/relic-physics-attribution-recovery-20261012-002`: locked extras,
  independent native actuator audit and correct20 ms air probe.
- `runs/relic-physics-mesh-fix-20261012-003`: converter repair outcome comparisons.
- `runs/relic-gripper-contact-probe-20261012-004`: three fixed-target pulses,
  generated explicit-pair XML, native filtered force and five states per pulse.
- `runs/relic-physics-attribution-analysis-20261012-002`: summary, comparison PNG/PDF
  and integrity-bound analysis completion. No dynamics are rerun for the plot.

Rebuild the public summary/plot from preserved private raw recordings:

```bash
uv run --no-sync python records/relic-physics-attribution-20261012/analyze.py \
  --output runs/NEW-physics-analysis
```

`gripper_probe.py --backend {mujoco,isaaclab} --output runs/NEW-probe` reproduces the
short pulse; add `--pairs` for the explicit-pair MuJoCo intervention. Use the
existing native overlay/PYTHONPATH for Isaac as documented in the stored contract.
The public copy replaces the original host-specific repo path with a relative
repo root and formats the script; the executed original script/hash remains in
local/run metadata. Exact reproduction requires the private captured standing
input and bound first target; a fresh generic pose is a different experiment.

Decision: contact/friction and gripper self-collision behavior explain the major
observed divergence. Preserve the simulation-only interventions, fix the mesh
asset alias, and keep investigating contact calibration with common target pulses.
No-slip10 is a useful comparison profile but not proof of physical accuracy or
safe hardware limits. None of the trials qualifies or authorizes deployment.

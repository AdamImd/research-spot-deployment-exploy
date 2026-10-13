# First joystick ReLIC hardware commissioning session

## Question and protocol

Can the unchanged ReLIC policy run with the USB joystick interface, the bound
moving-arm profile, and existing physical/timing guards on this Spot?

Adam approved one recorded 60-second session immediately before execution,
confirming Adam operating, Minghao holding the functioning tablet E-stop, verified
input directions/buttons, following rig and full arm/harness/shutdown clearance.
Native power-on/stand preceded a three-second zero-command hold and 50 Hz policy /
200 Hz joint commands. Body-frame input maxima were X/Y/yaw 0.30/0.20 m/s and
0.50 rad/s; acceleration 0.10/0.10/0.40. Button 2 enables, trigger/index0 toggles the
compact arm path, and printed button 4 finishes. No retry.

Source: clean `9ab5d0d8e8ba1e8655704e7352d6d819c16fbd1e`, existing locked UV CPU
environment. Policy/graph/weights/gains unchanged, zero initial raw actions and
ACK-owned action history. Private frozen bundle/approval/runner:
`local/relic-joystick-hardware-20261013-002`. Raw physical data remain ignored.

## Envelope review

Moving-arm guards enforce the path-derived measured ranges and 0.15 rad six-arm
tracking limit; the gripper retains 0.05 rad. Adam allowed slight penetration
beyond model bounds: 0.01 rad at SH1 lower and EL0 upper bounds only. Commands,
torque and tracking limits remain unchanged. Initial zero-tolerance saved-data
audit failed at 0.53 s on simulated elbow position; all artifacts are retained.
The revised saved-data audit passed 12,001 rows/60 seconds, four transient crossings
recovered within 100 ms, none pending. It uses ideal clock and simulated states;
it is not physical or transport qualification.

Retained requested leg torque 75 N·m, measured manufacturer limits, >500 ms physical
limit persistence, body 0.8 m/s/angular 2 rad/s, hip/knee 12/24 rad/s, leg tracking
2 rad, 50 ms expiry and 45 ms gap/ACK. Tablet sole E-stop; native shutdown allowance
10 seconds. Fresh motors-off preflight and bound deployment check passed.

## Observed result

The session completed **60.000 s**, **3,000 policy predictions/ACKs**, and
**11,995 commands**. Command gap p99/max was **5.60/13.74 ms**, ACK latency
p99/max **9.41/12.44 ms**, and core inference p99/max **0.399/0.544 ms**.
No physical-limit persistence crossings or guard stops were recorded.

Policy input X ranged **−0.1273 to +0.2283 m/s**, Y remained zero, and yaw ranged
zero to **+0.00798 rad/s**. Enable was active in 656 joystick samples. These are
requests consumed by the policy, not measured base speeds. The run does not
establish full-speed lateral/yaw performance.

**No arm-toggle request occurred.** Arm path progress stayed zero, so this run
provides no physical deployment/reversal/stow evidence for the moving-arm path.
The arm remained at its stowed reference. The finish button was not activated;
the registered duration ended the session.

Native shutdown confirmed motors off and lease return in **6.018 s**. Postcheck
passed: motors off, zero active faults, tablet stop state clear. Raw completion
and compact recording integrity verified; compact recording is about 5.8 MB and
the complete raw joystick/status events remain in the original recording.

See `result-summary.json` for exact values and `artifacts.json` for raw hashes.
Raw supervisor/trial/postcheck/compact files:
`runs/relic-joystick-hardware-trial-20261013-002`. Approval consumed.

## Decision

One locomotion-input commissioning session completed cleanly. Moving-arm hardware
behavior and five-minute endurance remain unverified. Adam subsequently requested
a five-minute repeat. The plan and envelope caps increase to 300 seconds with a separate
five-minute example; the default stays 60 seconds. Only duration validation and
documentation change; controls, policy and arm path are unchanged. The targeted joystick, contract and deployment checks passed after the duration-cap changes. Prepare a new
bound 300-second envelope/manifest and obtain the requested immediately-before-run
confirmation. No automatic repeat or blanket hardware qualification follows.

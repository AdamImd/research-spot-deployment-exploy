# Three-leg deployment preparation — awaiting approval

Adam requested offline deployment preparation and an explicit pause before code
runs on Spot. No robot connection, preflight, lease, motor action or command was
performed during this preparation. Earlier physical approvals remain consumed.

The private bundle is at
`local/relic-three-leg-persistence500-deployment-20261013-001/bundle`.
It was prepared from clean commit `217be28`; this record changes documentation
only. The bundle records the complete runtime source hash, frozen policy hashes,
Python version and installed packages. Robot-specific inputs remain ignored.

## Proposed trial

One recorded 47-second front-left demonstration: native power-on/stand, capture
standing body height, then 5 seconds four-foot standing, 6 seconds lift, 3 seconds
hold, 12 seconds body-frame IK square, 6 seconds lower, 2 seconds return blend,
12 seconds four feet and 1 second margin. The square spans x=0.53–0.61 m,
y=0.22 m, z=−0.28–−0.20 m. Policy evaluation is 50 Hz; joint commands are 200 Hz.
Native shutdown allowance is 10 seconds. No automatic retries.

Weights, gains, joint mapping and ACK-owned raw action history are unchanged.
Initial actions are zero. No entry fade or target-rate limiter is present. The
two-second return blend remains. This deliberately retains the native entry jump.

Requested leg PD-plus-feedforward demand has a 75 N·m stop threshold. Measured
manufacturer torque thresholds retain their existing values. Enforced physical
operating limits require a continuous violation **strictly longer than 500 ms**;
the first in-bounds sample resets each independent joint/limit timer. This is
delayed host stopping, not torque clipping or a change to firmware limits.

Other retained limits: hip speed 12 rad/s, knee/shared SDK speed 24 rad/s, body
speed 0.8 m/s, angular speed 2 rad/s, roll/pitch 10°, leg tracking error 2 rad,
virtual hip targets [−3,3] rad and knee targets [−3,0] rad. Measured leg-position,
arm-motion and target-slew observation modes remain explicit exceptions.
Computation/inference 10 ms, state/command gap and ACK 15 ms, command expiry
30 ms, tablet E-stop, data validity, health and startup qualification remain
immediate. Only the previously selected exact payload.fault 9 INFO exception
remains. The tablet is the sole E-stop authority.

## Evidence and unresolved items

The frozen live adapter reproduced all **3,001** saved simulation policy samples
with **zero** difference in observations, raw actions and targets. Discarded
warmup retained zero history. The simulator recording's completion manifest and
all 55 recorded source hashes were verified. This was saved-data replay, with
no new dynamics or robot access.

The prior [persistence validation](../physical-limit-persistence-20261013/README.md)
completed 60 seconds without a fall and passed final standing. Its unlimited
actuator model does not establish hardware saturation behavior. Support evidence
uses foot geometry, not measured contact loads.

A **119.366 ms** computation outlier in that simulation exceeds the unchanged
live **10 ms** deadline. It remains unresolved; no live timing pass is claimed.
The reviewed software parity gate is bound to the new bundle. Other physical
and commissioning gates remain unverified rather than copying old approvals or
relabeling the simulator as hardware qualification.

The local packet contains `pending-final-review.json`, `review-queue.json`,
`adapter-replay.json`, `pending-commands.json` and `HANDOFF.md`. Command arrays
are review templates, with `--execute` intentionally absent. No process is
waiting to automatically launch a trial when a message arrives.

## Resume boundary

Wait for Adam's approval before any robot code, including fresh read-only
preflight. That preflight must establish current identity/model/payload, health,
telemetry, motors-off state and tablet stop state. Review timing and remaining
bound evidence, then obtain the requested exact one-trial confirmation immediately
before physical execution, with Adam operating and Minghao on the connected
tablet, rig attached and three-foot/foot/arm workspace clear. Approval to inspect
is not approval to move. Preserve any failed gate; do not weaken it or retry
automatically.

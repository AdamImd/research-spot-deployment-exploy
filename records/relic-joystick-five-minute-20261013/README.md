# Five-minute joystick session preparation and startup correction

Adam requested “Run again for 5 minutes,” then “Just do it.” This directly
authorizes one five-minute repeat using the previously confirmed operator/tablet/
rig/clearance/control setup. It supersedes asking again for this repeat.
Only duration changes from 60 to 300 seconds; arm path, speeds, weights/gains,
physical limits, tablet authority and 10-second native shutdown stay unchanged.

Plan/envelope schemas were extended to 300 seconds. The first launch from
`918fd3c` rejected the duration at the CLI's remaining 60-second gate before USB
input acquisition, SDK control construction, lease, power or motion. Postcheck
confirmed motors off, tablet stop state clear and zero faults. No policy samples
or motion commands were emitted. Preserve all artifacts at
`runs/relic-joystick-five-minute-hardware-20261013-001`; no physical session occurred.

The CLI now permits 300 seconds for joystick only; other command modes retain
their existing duration limits. A regression test verifies 300-second joystick
requests reach readiness while 301-second joystick and 300-second shadow requests
reject before robot I/O. The corrected continuation uses a new frozen bundle,
source/evidence binding, fresh preflight and new one-invocation runner. The primary
authorized correction of the startup validation error within Adam's direct
five-minute request; no automatic recovery or repeated physical motion is enabled.

The first 60-second physical session exercised forward/backward input with no
arm toggle. Arm deployment and five-minute endurance remain commissioning tasks.
Keep raw results and shutdown evidence; timer completion alone is not full input,
arm-path or hardware robustness qualification.

## Corrected physical result

The corrected session ran once from clean `1ef834bfc496ec81c9206e4dcb9899777532f291`.
The CLI regression and joystick checks passed (12 tests), plus targeted Ruff.
Fresh read-only preflight and the newly bound deployment check passed before
native power-on/stand. The original startup-only rejection remains separate.

The requested **300-second session stopped after about 229.03 seconds (3m49s)**.
It produced **11,451 policy predictions/ACKs** and **45,782 commands**. Policy X
requests reached both ±0.30 m/s, Y ranged −0.09197 to +0.12518 m/s and yaw reached
both ±0.50 rad/s. Enable was active in 3,986 joystick samples. These are policy
commands, not measured base speeds. Core inference p99/max was 0.399/1.649 ms.

Arm deployment was requested and the full **10.072-second** command path reached
the front pose. The stopping joint was **EL0, SDK index14, arm-local index2**,
not SH1. The recorded stop was `arm is outside commanded pose`: absolute target
tracking error **0.155550 rad** remained over the **0.15 rad** threshold for
**0.502005 seconds**. The last issued command held EL0 at **2.4 rad** while its
measured position was **2.555798 rad**, with measured load **−18.294 N·m**.
This is a tracking stop, separate from the reviewed model-bound penetration
tolerance. Do not infer a collision or physical joint-limit fault from it.

Thirteen per-limit timer entries started and ten recovered. Several entries are
redundant checks of the same elbow error; counts are not distinct physical events.
Earlier wrist-speed and requested-knee-torque crossings recovered within 5/10/20ms;
an earlier elbow tracking episode recovered within300ms. The final elbow episode
exceeded500ms and stopped the controller. No retry or limit change followed.

Native shutdown confirmed motors off and lease return in **6.023 seconds**.
Postcheck passed with zero active faults and clear tablet stop state. The failed
trial completion and compact recording hashes verify; compact capture is about
**23.24 MB**. Raw events preserve all joystick/status and guard details.

Raw supervisor/trial/postcheck/compact:
`runs/relic-joystick-five-minute-hardware-20261013-002`. Private frozen packet:
`local/relic-joystick-five-minute-20261013-002`. See `result-summary.json` and
`artifacts.json`. The session exercised all three velocity axes and arm deployment,
but did not complete five-minute endurance. Hardware qualification remains false;
another physical attempt needs a new explicit instruction/authorization.

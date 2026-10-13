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

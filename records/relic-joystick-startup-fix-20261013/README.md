# Joystick startup queue and failure reporting correction

The operator's154632 session passed preflight and native stand, then failed with
`joystick event loss/resynchronization; restart required` before policy prediction.
The earlier145858 session instead raised robot CPU over-temperature during power
activation. Both attempts confirmed motors off, lease return, fault-free postcheck
and complete compact capture; preserve each distinct cause.

The Linux joydev link was opened/polled before robot power/stand, then not read
through blocking native startup. An overflowing joydev queue resends INIT state;
the strict active-session decoder correctly rejected that snapshot. Fix the
startup lifecycle: after native stand, reopen the same physical device, discard
the old queue, reset cached button/latch/timing state and read a fresh INIT snapshot.
Held enable/trigger cannot activate or toggle; the operator must release/center
and press enable again. Replacement and active-session event loss still stop.
No weights/gains, arm path, tracking observation or physical/timing limits change.

Launcher output now includes the recorded failure and blocking robot fault rather
than only `finished_with_failure`. Native power cancellation rethrows the original
health-worker failure when one exists; shutdown still runs. Regression tests cover
discarded startup events, held INIT buttons, fresh enable, device replacement,
fault display and health-cancellation cleanup. Validation is offline; no physical
activation is part of preparing this fix. Refresh the local reviewed bundle and
keep the same operator command path after source validation.

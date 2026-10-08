# Simulation-only rollout settings

These deliberately broad numerical bounds exercise the released controller and
its stop conditions in simulation. They are **not physical operating limits**.
The `scope: simulation` envelope is rejected by the physical standing gate.
The startup settings are a tested candidate for review, not an inspected rig or
approved hardware transition. Do not copy them into a hardware envelope to bypass
qualification. The candidate importer does not create passed evidence.

The 5 s preparation uses unchanged released gains, model-based four-foot gravity
support and quintic interpolation to the released nominal legs, with captured
stowed arm targets. The simulator trial lasts at most 60 s total, including the
5 s preparation. It starts from the existing real standing capture; it does not
simulate Boston Dynamics' proprietary native stand or physical safe-power-off.

The remaining 55 s uses unchanged ReLIC outputs, zero initial previous actions,
50 Hz inference and 200 Hz commands. Acknowledgements are ideal one-tick receipt
events. Every runtime observation/action is independently compared with the
simulation contract. Handover pose, velocity and commanded torque-step checks
are active, along with the envelope guards.

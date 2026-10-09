# First physical joint-API trial and blocked retry

One physical trial was attempted at runtime efef691. It stopped on the body-attitude
limit at approximately 4.75 seconds, after 953 recorded commands. The 10-second
sequence and return-to-start were not completed. The initial two-second hold was
near level; tilt developed during the crouch. Last recorded roll was -9.995 degrees
and pitch -8.988 degrees; the next guard-rejected state was not saved in samples.
The largest sampled leg tracking error was 22.403 degrees at the left-rear knee.

Command gap max 6.756 ms, host-observed ACK delay max 9.738 ms, sampled state age
max 9.081 ms. These were below their configured thresholds before the attitude
stop; this does not qualify a full-duration trial or explain the instability.
ACK delay is reconstructed as (recorded host time minus recorded production time)
minus the acknowledged key's send timestamp, using increasing ACK keys only.

The automatic safe-power-off attempt did not confirm shutdown after 10.009 s.
The operator was immediately asked to stop Spot using the tablet and confirmed
motors off. Subsequent read-only SDK checks confirmed exact STATE_OFF and no
lease owners. Those later observations do not turn failed automatic cleanup into
a successful shutdown test. The informational payload.fault code 9 (COM estimated
left of expected) was present afterward. No faults were cleared and no E-stop
configuration/check-in requests were issued. No ReLIC policy was used.

The user subsequently requested another run. A fresh read-only preflight still
failed its no-active-faults check; motors were off and tablet stop state ready.
No second physical trial, power-on or motion command was issued.

Raw run: runs/joint-api-crouch-20261009-001 (completion hashes verified).
Post-stop: runs/crouch-poststop-inspection-20261009-001 and
runs/crouch-poststop-health-20261009-002. Read-only health probe 001 had a local
Path/string error before connection and is preserved as FAILED.json.
Blocked retry: runs/crouch-retry-inspection-20261009-001.

The trajectory used fixed native-load feedforward and joint-space reference without
body-attitude feedback. This is a plausible contributor to crouch imbalance, not
a demonstrated root cause. Review tracking, load convention and shutdown behavior
before another physical attempt. Preserve limits and do not bypass the fault gate.

# Firmware compatibility

The pinned SDK is 5.1.1. Profiles accept the reviewed 5.0.1 and 5.1.x wire contracts.
Joint-control entitlement must be verified on each robot; a firmware match does
not establish access or physical qualification. Unary polling and streaming are
separate capabilities. Full-state polling is available to the read-only viewer
when streaming access is unavailable.

Use an operator-supplied `local/robot.json` and the read-only `preflight` or
`tools/probe_robot_compatibility.py` procedure in [OPERATIONS](OPERATIONS.md).
Do not publish the resulting identity, licensing, endpoint or payload records.
Static compatibility and SDK enum order have regression tests. Revalidate private
streaming stub compatibility before changing SDK versions. See [architecture](ARCHITECTURE.md).

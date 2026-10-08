# Bundled simulator source and modifications

ReLIC source/assets: https://github.com/rai-opensource/relic at
`27f8033c5064d32f049a17accb71cd1091422878`, under the retained RAI Institute Research
License. This is a **modified research distribution**, not an unmodified upstream
checkout. `source/relic/` was imported from simulator revision
`74cb498719164d4c6e698a1f056db5ca348f60d2`.

Two upstream Python files carry compatibility modifications: the Spot actuator
constructor forwards Isaac Lab's current friction parameters; the termination
height calculation uses its current tensor shape. The runtime helpers disable
implicit importer PD drives and preserve explicit released actuator behavior.
The host-independent adapters in `spot_relic_sim/` provide MuJoCo conversion,
collision processing, native Isaac articulation, settling metrics and provenance.
`tools/debug_shadow.py` supplies captured-pose initialization used by the shared core.

This directory is simulation-only. Never connect it to a physical robot. See the
root agent instructions and docs/SIMULATION.md for the registered protocol.

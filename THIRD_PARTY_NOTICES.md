# Licensing and upstream attribution

This is a modified research integration, independently maintained by Adam Imdieke.
It is not an official RAI Institute or Boston Dynamics deployment release.

| Component | Origin and license |
| --- | --- |
| Deployment integration and tools | Adam Imdieke; MIT, subject to the exceptions below |
| `simulation/source/relic/` | RAI Institute ReLIC, revision `27f8033c5064d32f049a17accb71cd1091422878`; RAI Institute Research License |
| Original ReLIC checkpoints and meshes | Same upstream and research license |
| `policies/relic-exploy-standing/` | Modified derived ReLIC export; retained research license in that directory |
| Embedded ReLIC source in `training-config.json` | Same retained upstream research license |
| Exploy exporter dependency | Pinned upstream revision in `requirements-exploy-export.txt`; upstream license applies |
| Vendored Three.js | MIT; retained in `src/spot_deploy/web/vendor/three/LICENSE` |
| Spot SDK, Isaac Lab/Sim, MuJoCo and other dependencies | Their respective upstream licenses; not relicensed here |

ReLIC's research license limits use to non-commercial research and requires
retaining its notices and marking modifications. The modified graph combines
native observations, the actor and target conversion while preserving released
weights. The bundled source's Isaac compatibility changes are documented in
`simulation/UPSTREAM.md`. These notices and the adjacent research licenses must
accompany redistribution. The integration's MIT license does not remove those
restrictions or grant rights to third-party dependencies.

Sources:
[ReLIC](https://github.com/rai-opensource/relic/tree/27f8033c5064d32f049a17accb71cd1091422878),
[Exploy](https://github.com/rai-opensource/exploy/tree/05f9dc3b2e589abdae8de942e8c6749e7a123c88).

The optional chair-perception environment uses Meta SAM 2 source and SAM2.1 tiny
weights under Apache-2.0, downloaded separately (not bundled). Pinned revision,
checkpoint URL and SHA256 are documented in [CHAIR_PERCEPTION](docs/CHAIR_PERCEPTION.md).
[Upstream license](https://github.com/facebookresearch/sam2/blob/2b90b9f5ceec907a1c18123530e92e794ad901a4/LICENSE).

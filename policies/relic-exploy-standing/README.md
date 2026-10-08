# Modified ReLIC / Exploy standing export

This is a modified research export of ReLIC: native observation construction,
released actor and native joint-target conversion are combined in one ONNX graph
using Exploy. The trained weights are unchanged. Upstream is RAI Institute's
ReLIC at `27f8033c5064d32f049a17accb71cd1091422878`; the Exploy pin and original
weight hashes are in `manifest.json` and `graph-contract.json`.

Retain RELIC-LICENSE with redistribution. Use is limited to non-commercial
research under that license. This integration is not an upstream hardware-ready
release and carries no physical qualification. The manifest's captured stowed arm
and model/payload digests describe the historical candidate. Rebinding another
robot requires a new import and qualification, not an edit to bypass a hash.

The included export configuration preserves its original hash-bound provenance.
Historical build paths inside it are informational; runtime opens only relative
manifest artifacts. No robot endpoint, credential or live health record is included.

"""Package a native Exploy graph with an existing unqualified ReLIC standing candidate."""

import argparse
import json
from pathlib import Path
import shutil

from spot_deploy.contracts import ContractError, artifact_path, check_artifacts, sha256
from spot_deploy.exploy_policy import EXPLOY_COMMIT, SCRIPT_SHA256, TARGET, ExployReLICManifest
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_policy import CHECKPOINT_SHA256


def import_candidate(reference, exported, output):
    base = load_manifest(reference)
    if base.schema_version != 2 or base.purpose != "candidate":
        raise ContractError("Import requires an existing v2 ReLIC candidate")
    check_artifacts(reference, base)
    if not verify_run(exported, status="completed"):
        raise ContractError("Export completion record failed integrity verification")
    record = json.loads((exported / "export.json").read_text())
    if record["source"]["dirty"] or record["simulation_dirty"]:
        raise ContractError("Export must come from recorded clean source snapshots")
    if sha256(exported / "policy.onnx") != record["policy_sha256"]:
        raise ContractError("Exported graph hash mismatch")
    output.mkdir(parents=True, exist_ok=False)
    copies = {
        "policy.onnx": exported / "policy.onnx",
        "export.json": exported / "export.json",
        "export-configuration.json": exported / "configuration.json",
        "graph-contract.json": exported / "graph-contract.json",
        "actor-parity.json": exported / "actor-parity.json",
        "training-config.json": artifact_path(reference, base.training_config_file),
        "support.urdf": artifact_path(reference, base.relic.preparation.model_file),
        "RELIC-LICENSE": reference.parent / "RELIC-LICENSE",
    }
    for name, src in copies.items():
        shutil.copy2(src, output / name)
    data = base.model_dump()
    data.update(schema_version=3, adapter="relic-exploy", policy_file="policy.onnx",
                policy_sha256=record["policy_sha256"], input_name="named-state-v1",
                output_name=TARGET, training_config_file="training-config.json",
                exploy=dict(contract="relic-exploy-v1", exporter_commit=EXPLOY_COMMIT,
                    source_policy_sha256=CHECKPOINT_SHA256, source_script_sha256=SCRIPT_SHA256,
                    export_record_file="export.json", export_record_sha256=sha256(output / "export.json"),
                    configuration_file="export-configuration.json",
                    configuration_sha256=sha256(output / "export-configuration.json")))
    data["relic"]["preparation"]["model_file"] = "support.urdf"
    manifest = ExployReLICManifest.model_validate(data)
    path = output / "manifest.json"
    atomic_json(path, manifest.model_dump())
    # Load the actual graph and metadata before declaring an import complete.
    make_policy(check_artifacts(path, manifest), manifest, path)
    atomic_json(output / "import.json", dict(
        reference_manifest_sha256=sha256(reference), export_complete_sha256=sha256(exported / "COMPLETE.json"),
        adapter="relic-exploy", hardware_qualified=False, hardware_access=False,
        note="New graph/hash requires new evidence. No physical qualification is inherited."))
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-manifest", required=True, type=Path)
    parser.add_argument("--export", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(import_candidate(args.reference_manifest, args.export, args.output))

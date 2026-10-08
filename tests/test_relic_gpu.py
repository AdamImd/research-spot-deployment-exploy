from types import SimpleNamespace

import onnxruntime as ort
import pytest

from spot_deploy import relic_policy
from spot_deploy.contracts import ContractError
from spot_deploy.records import runtime_packages


def test_explicit_cuda_never_silently_uses_cpu(monkeypatch, tmp_path):
    monkeypatch.setattr(relic_policy, "sha256", lambda _: relic_policy.CHECKPOINT_SHA256)
    monkeypatch.setattr(ort, "get_available_providers", lambda: ["CPUExecutionProvider"])
    with pytest.raises(ContractError, match="unavailable"):
        relic_policy.ReLICPolicy(tmp_path / "model", device="cuda")
    monkeypatch.setattr(ort, "get_available_providers", lambda: ["CUDAExecutionProvider"])
    fallback = SimpleNamespace(disable_fallback=lambda: None,
                               get_providers=lambda: ["CPUExecutionProvider"])
    monkeypatch.setattr(ort, "InferenceSession", lambda *a, **k: fallback)
    with pytest.raises(ContractError, match="did not initialize"):
        relic_policy.ReLICPolicy(tmp_path / "model", device="cuda")


def test_source_only_gpu_environment_records_missing_distributions(monkeypatch):
    import importlib.metadata

    def version(name):
        if name in ("spot-deployment", "onnxruntime"):
            raise importlib.metadata.PackageNotFoundError(name)
        return "1.23.2" if name == "onnxruntime-gpu" else "test"
    monkeypatch.setattr(importlib.metadata, "version", version)
    packages = runtime_packages()
    assert packages["spot-deployment"] is None
    assert packages["onnxruntime"] is None
    assert packages["onnxruntime-gpu"] == "1.23.2"


def test_simulation_completion_keeps_hash_checks_and_does_not_pass_default_gate(tmp_path):
    import json
    from spot_deploy.contracts import sha256
    from spot_deploy.records import verify_run

    artifact = tmp_path / "data.json"
    artifact.write_text('{"value":1}')
    (tmp_path / "COMPLETE.json").write_text(json.dumps(
        {"status": "completed", "artifacts": {"data.json": sha256(artifact)}}))
    assert not verify_run(tmp_path)
    assert verify_run(tmp_path, status="completed")
    artifact.write_text('{"value":2}')
    assert not verify_run(tmp_path, status="completed")

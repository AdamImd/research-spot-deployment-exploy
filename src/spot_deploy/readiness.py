"""Human-reviewed evidence is required; missing facts are explicitly unverified."""

from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

from pydantic import Field

from .contracts import ContractError, Hash, RobotConfig, StrictModel, canonical_hash, load, sha256
from .records import source_identity, utcnow

GATES = {
    "post_repair_stock_check": "Robot identity, repair return and stock operation inspected",
    "rig_inspection": "Installed fall support and operating clearance inspected",
    "arm_hold": "Stowed-arm targets, gains and morphology validated",
    "operating_envelope": "Xun's per-joint and runtime limits reviewed",
    "simulator_standing": "Minghao's arm-attached standing qualification passes",
    "transition_and_shutdown": "Controller transition and stop behavior qualified in simulation",
    "wired_timing": "RPM wired state/command timing meets the operating envelope",
    "shadow_parity": "Exact candidate agrees with training observation/action reference",
}
HARDWARE_GATE = "hardware_estop"
HARDWARE_DESCRIPTION = "Physical stop input, latch, link loss and Spot stop latency qualified"


class Evidence(StrictModel):
    gate: str
    status: Literal["passed", "failed", "unverified"]
    reviewed_by: str = Field(min_length=1)
    recorded_at: datetime
    expires_at: datetime
    binding_sha256: Hash
    artifact: str
    artifact_sha256: Hash
    note: str


class EvidenceIndex(StrictModel):
    schema_version: Literal[1]
    evidence: list[Evidence]


def binding(manifest_path, envelope_path, robot_path):
    return {
        "manifest_sha256": sha256(manifest_path),
        "envelope_sha256": sha256(envelope_path),
        "robot_config_sha256": sha256(robot_path),
        "source_sha256": source_identity()["source_sha256"],
    }


def report(manifest_path=None, envelope_path=None, robot_path=None, index_path=None):
    checks = []
    digest = None
    values = None
    if all(p and Path(p).is_file() for p in (manifest_path, envelope_path, robot_path)):
        values = binding(manifest_path, envelope_path, robot_path)
        digest = canonical_hash(values)
    records = {}
    gates = GATES.copy()
    if (
        robot_path
        and Path(robot_path).is_file()
        and load(Path(robot_path), RobotConfig).hardware_estop
    ):
        gates[HARDWARE_GATE] = HARDWARE_DESCRIPTION
    if index_path and Path(index_path).is_file():
        index = EvidenceIndex.model_validate_json(Path(index_path).read_text())
        for entry in index.evidence:
            if entry.gate not in {*GATES, HARDWARE_GATE} or entry.gate in records:
                raise ContractError("unknown or duplicate readiness evidence gate")
            records[entry.gate] = entry
    now = datetime.now(timezone.utc)
    for gate, description in gates.items():
        status, reason, artifact = "unverified", "No evidence supplied", None
        entry = records.get(gate)
        if entry:
            status, reason = entry.status, entry.note
            if entry.recorded_at.tzinfo is None or entry.expires_at.tzinfo is None:
                status, reason = "failed", "Evidence timestamps require a timezone"
            elif entry.recorded_at > now or entry.expires_at <= now:
                status, reason = "failed", "Evidence is future-dated or expired"
            elif not digest or entry.binding_sha256 != digest:
                status, reason = "failed", "Evidence belongs to different source or configuration"
            else:
                base = Path(index_path).resolve().parent
                artifact = (base / entry.artifact).resolve()
                if (
                    not artifact.is_relative_to(base)
                    or not artifact.is_file()
                    or sha256(artifact) != entry.artifact_sha256
                ):
                    status, reason = "failed", "Evidence artifact is missing or hash-mismatched"
        checks.append(
            {
                "gate": gate,
                "description": description,
                "status": status,
                "reason": reason,
                "artifact": str(artifact) if artifact else None,
            }
        )
    return {
        "schema_version": 1,
        "generated_at": utcnow(),
        "binding": values,
        "binding_sha256": digest,
        "evidence_ready": all(c["status"] == "passed" for c in checks),
        "checks": checks,
    }


def require_live(
    manifest,
    envelope,
    robot_config,
    manifest_path,
    envelope_path,
    robot_path,
    index_path,
    operator,
    safety_operator,
    execute,
    duration,
):
    if not execute or not operator.strip() or not safety_operator.strip():
        raise ContractError("live control requires explicit execution and two named operators")
    if operator.strip().casefold() == safety_operator.strip().casefold():
        raise ContractError("robot operator and E-stop operator must be distinct")
    if manifest.purpose != "candidate":
        raise ContractError("fixture policies cannot control hardware")
    if envelope.scope != "hardware":
        raise ContractError("simulation envelopes cannot control hardware")
    if getattr(manifest, "adapter", None) in ("relic84", "relic-exploy"):
        if envelope.transition_s != manifest.relic.preparation.duration_s:
            raise ContractError("ReLIC preparation duration must match the registered transition")
    if duration <= envelope.transition_s or duration > envelope.max_duration_s:
        raise ContractError("duration exceeds registered standing limit")
    if source_identity()["dirty"]:
        raise ContractError("live control requires a clean committed source tree")
    result = report(manifest_path, envelope_path, robot_path, index_path)
    if not result["evidence_ready"]:
        raise ContractError("live readiness evidence is incomplete, failed or expired")
    return result


def markdown_report(result):
    lines = [
        "# Spot readiness",
        "",
        f"Generated: {result['generated_at']}",
        "",
        "These evidence gates do not assert hardware readiness without a current preflight.",
        "",
        "| Gate | Status | Evidence / next action |",
        "| --- | --- | --- |",
    ]
    for check in result["checks"]:
        reason = check["reason"].replace("|", "\\|").replace("\n", " ")
        lines.append(f"| {check['gate']} | {check['status']} | {reason} |")
    return "\n".join(lines) + "\n"

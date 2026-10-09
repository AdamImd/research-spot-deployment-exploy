"""Offline deployment packaging and checks; never construct an SDK client."""

import json
import math
import platform
import shutil
from datetime import datetime, timezone
from typing import Literal

import numpy as np
from pydantic import Field

from .contracts import (
    JOINTS, ContractError, Envelope, Hash, RobotConfig, StrictModel,
    artifact_path, check_artifacts, load, sha256,
)
from .exploy_policy import ExployReLICManifest
from .readiness import GATES, HARDWARE_DESCRIPTION, HARDWARE_GATE, report
from .records import atomic_json, runtime_packages, source_identity, verify_run
from .relic_contract import load_manifest, make_policy
from .safety import Guard


class DeploymentBundle(StrictModel):
    schema_version: Literal[1]
    manifest_file: Literal["policy/manifest.json"]
    source_commit: str = Field(min_length=40, max_length=40)
    source_sha256: Hash
    python_version: str
    packages: dict[str, str | None]
    frozen_files: dict[str, Hash]
    hardware_qualified: Literal[False]


def candidate(path):
    manifest = load_manifest(path)
    if not isinstance(manifest, ExployReLICManifest) or manifest.purpose != "candidate":
        raise ContractError("deployment preparation requires a real Exploy ReLIC candidate")
    policy = make_policy(check_artifacts(path, manifest), manifest, path)
    policy.warmup()
    if np.any(policy.previous_action != 0) or policy.body_height != manifest.relic.body_height_m:
        raise ContractError("discarded warmup changed action history or initial height")
    return manifest


def policy_files(manifest):
    return {
        manifest.policy_file, manifest.training_config_file,
        manifest.relic.preparation.model_file, manifest.exploy.export_record_file,
        manifest.exploy.configuration_file,
    }


def hardware_envelope(path, manifest):
    envelope = load(path, Envelope)
    if envelope.scope != "hardware":
        raise ContractError("supply reviewed hardware limits; simulation envelopes cannot be promoted")
    Guard(manifest, envelope)
    if envelope.transition_s != manifest.relic.preparation.duration_s:
        raise ContractError("hardware transition must match the policy preparation contract")
    return envelope


def review_template(manifest):
    data = {name: None for name in Envelope.model_fields}
    for name in ("position_min", "position_max", "velocity_max", "load_max",
                 "tracking_error_max", "command_rate_max"):
        data[name] = [None] * 19
    data.update(schema_version=1, scope="hardware", joint_order=list(JOINTS),
                transition_s=manifest.relic.preparation.duration_s)
    return data


def review_text(manifest):
    lines = [
        "# Deployment review", "",
        "This bundle is prepared for review, not qualified for activation.",
        "Fill envelope.review.json with reviewed hardware limits, then save the completed",
        "file as envelope.json. Null entries intentionally fail validation. Do not copy",
        "simulation limits or treat target-minus-measured offsets as physical limits.", "",
        "## Joint-limit worksheet", "",
        "Order is canonical SDK order. Positions/tracking are rad; rates are rad/s;",
        "load_max bounds measured and predicted PD-plus-feedforward loads in SDK joint",
        "units. Verify the gripper's coordinate/load convention separately. Gains below",
        "are policy contract values, not approved hardware limits.",
        "actuator_limit_profile=spot-sdk-5.0.1 adds measured-position knee and coupled-arm limits.",
        "sdk_velocity_safety_limit is the shared robot backstop; host speed limits remain per-joint.",
        "",
        "| Joint | Policy kp | Policy kd | Handover torque-step cap | Min/max position | Speed | Load | Tracking | Target rate |",
        "| --- | ---: | ---: | ---: | --- | --- | --- | --- | --- |",
    ]
    for i, name in enumerate(JOINTS):
        lines.append(f"| {name} | {manifest.gains.kp[i]} | {manifest.gains.kd[i]} | "
                     f"{manifest.relic.preparation.handover_torque_step_max[i]} | "
                     "REVIEW | REVIEW | REVIEW | REVIEW | REVIEW |")
    lines += [
        "", "## Timing and body limits", "",
        "Review every remaining null in the JSON, including initial/final body and arm",
        "bounds, inference/state/ACK deadlines, command expiry, battery, duration and",
        "shutdown. Policy is 50 Hz; commands are 200 Hz. A previous host benchmark had",
        "a 5.920 ms response and one missed nominal release; physical wired timing is",
        "unqualified. Raw timing, command latency and stop evidence are still required.",
        "Review the candidate's handover torque-step caps and pose/speed gates as well;",
        "they came from the simulated candidate and are not physical approvals. Changes",
        "to these manifest values require a new candidate bundle and renewed evidence.",
        "", "The registered relationships are command gap < TTL <= policy age; ACK < TTL;",
        "policy period < policy age; command period < command gap; transition < duration.",
        f"Preparation: {manifest.relic.preparation.kind}, "
        f"{manifest.relic.preparation.duration_s:g} s. Keep zero initial raw actions.",
        "Initial height is captured after native standing. Do not substitute a guessed height.",
        "", "## Evidence review", "",
        "Run readiness after the final manifest/envelope/robot files are in place to get",
        "their source/configuration binding. evidence.json starts empty. Supply reviewed",
        "artifacts with matching hashes, named reviewers and timezone-aware validity dates",
        "using schemas/EvidenceIndex.json. No gate is passed by creating this bundle.", "",
    ]
    for name, description in {**GATES, HARDWARE_GATE: HARDWARE_DESCRIPTION}.items():
        lines.append(f"- `{name}`: {description}. Status: UNVERIFIED; reviewer/artifact: pending.")
    lines += [
        "", "For estop_authority=tablet, the hardware_estop evidence gate covers the",
        "manufacturer tablet's stop control, connection loss and stop response. No local",
        "joystick/ESP32 bridge is required. A clear robot stop status does not verify",
        "the tablet connection or button. See docs/TABLET_ESTOP.md.",
        "", "## Final operator entries", "",
        "Robot operator: ______; independent E-stop operator: ______.",
        "Rig inspection/artifact: ______; registered trial duration: ______.",
        "Use commands.json and docs/DEPLOYMENT.md. The stand command requires both",
        "operator names, a duration, complete evidence and a separate explicit --execute.",
        "All preparation/check commands are offline. Preflight and shadow are separate",
        "read-only commands. None of these configure or rearm an E-stop.",
    ]
    return "\n".join(lines) + "\n"


def command_templates(directory):
    base = directory.resolve()
    inputs = ["--manifest", str(base / "policy/manifest.json"),
              "--envelope", str(base / "envelope.json"), "--robot", str(base / "robot.json")]
    prefix = ["uv", "run", "--no-sync", "spot-deploy"]
    return {
        "note": "Argument templates only. Replace uppercase placeholders; run from the repository root.",
        "preflight": prefix + ["preflight", "--manifest", str(base / "policy/manifest.json"),
                               "--robot", str(base / "robot.json"), "--output", "NEW_PREFLIGHT_RUN"],
        "shadow": prefix + ["shadow", *inputs, "--duration", "REVIEWED_DURATION_SECONDS",
                            "--output", "NEW_SHADOW_RUN"],
        "readiness": prefix + ["readiness", *inputs, "--evidence", str(base / "evidence.json"),
                               "--output", "NEW_READINESS_RUN"],
        "stand_requires_explicit_execution": prefix + ["stand", *inputs,
            "--evidence", str(base / "evidence.json"), "--operator", "ROBOT_OPERATOR",
            "--safety-operator", "INDEPENDENT_ESTOP_OPERATOR", "--duration",
            "REVIEWED_DURATION_SECONDS", "--output", "NEW_STAND_RUN"],
    }


def prepare(manifest_path, robot_path, envelope_path, record):
    source = source_identity()
    if source["dirty"] or not source["commit"]:
        raise ContractError("prepare deployment from a clean committed checkout")
    manifest = candidate(manifest_path)
    if robot_path:
        load(robot_path, RobotConfig)
    if envelope_path:
        hardware_envelope(envelope_path, manifest)
    base = record.directory
    target = base / "policy"
    target.mkdir()
    shutil.copy2(manifest_path, target / "manifest.json")
    for name in policy_files(manifest):
        src = artifact_path(manifest_path, name)
        dst = artifact_path(target / "manifest.json", name)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
    for name in ("RELIC-LICENSE", "README.md", "graph-contract.json", "actor-parity.json"):
        src = artifact_path(manifest_path, name)
        if src.is_file():
            shutil.copy2(src, target / name)
    if robot_path:
        shutil.copy2(robot_path, base / "robot.json")
    if envelope_path:
        shutil.copy2(envelope_path, base / "envelope.json")
    atomic_json(base / "envelope.review.json", review_template(manifest))
    atomic_json(base / "evidence.json", {"schema_version": 1, "evidence": []})
    atomic_json(base / "commands.json", command_templates(base))
    (base / "REVIEW.md").write_text(review_text(manifest))
    bundle = DeploymentBundle(schema_version=1, manifest_file="policy/manifest.json",
        source_commit=source["commit"], source_sha256=source["source_sha256"],
        python_version=platform.python_version(),
        packages=runtime_packages(), hardware_qualified=False,
        frozen_files={str(p.relative_to(base)): sha256(p) for p in sorted(target.rglob("*"))
                      if p.is_file()})
    atomic_json(base / "bundle.json", bundle.model_dump())
    # Validate the relocated candidate, not just its original path.
    candidate(target / "manifest.json")
    return {"bundle_created": True, "hardware_qualified": False, "live_ready": False,
            "robot_profile_supplied": bool(robot_path),
            "hardware_envelope_supplied": bool(envelope_path),
            "policy_hz": manifest.policy_hz, "stream_hz": manifest.stream_hz,
            "next": "Complete REVIEW.md and run deployment-check; no hardware action was taken."}


def preflight_check(path, robot_path, manifest_path, max_age_s):
    if not math.isfinite(max_age_s) or not 0 < max_age_s <= 300:
        raise ContractError("preflight age limit must be finite and in (0, 300] seconds")
    if not verify_run(path):
        raise ContractError("preflight did not pass or its artifacts changed")
    meta = json.loads((path / "run.json").read_text())
    result = json.loads((path / "result.json").read_text())
    complete = json.loads((path / "COMPLETE.json").read_text())
    if meta["mode"] != "preflight" or result.get("passed") is not True:
        raise ContractError("expected a passed read-only preflight")
    age = (datetime.now(timezone.utc) - datetime.fromisoformat(complete["finished_at"])).total_seconds()
    if not 0 <= age <= max_age_s:
        raise ContractError("preflight is stale or future-dated")
    if not {sha256(robot_path), sha256(manifest_path)} <= set(meta["inputs"].values()):
        raise ContractError("preflight does not bind this robot profile and candidate")
    if not result.get("checks") or not all(result["checks"].values()):
        raise ContractError("preflight reported failed checks")


def check(directory, preflight_path=None, max_age_s=300, check_estop=False):
    if not math.isfinite(max_age_s) or not 0 < max_age_s <= 300:
        raise ContractError("preflight age limit must be finite and in (0, 300] seconds")
    base = directory.resolve()
    bundle_path = base / "bundle.json"
    bundle = load(bundle_path, DeploymentBundle)
    checks = []

    def item(name, fn):
        try:
            value = fn()
            checks.append({"check": name, "status": "passed", "reason": "Verified"})
            return value
        except Exception as exc:
            # Do not include raw validation inputs, SDK strings, identities or credentials.
            reason = str(exc) if isinstance(exc, ContractError) else f"{type(exc).__name__}: invalid or missing input"
            checks.append({"check": name, "status": "blocked", "reason": reason})
            return None

    def frozen():
        manifest_path = artifact_path(bundle_path, bundle.manifest_file)
        manifest = candidate(manifest_path)
        required = {bundle.manifest_file} | {"policy/" + p for p in policy_files(manifest)}
        if not required <= bundle.frozen_files.keys():
            raise ContractError("bundle omits required frozen policy artifacts")
        for name, digest in bundle.frozen_files.items():
            if sha256(artifact_path(bundle_path, name)) != digest:
                raise ContractError("frozen deployment artifact changed; prepare a new bundle")
        return manifest

    manifest = item("policy_bundle", frozen)

    def runtime():
        source = source_identity()
        if source["dirty"] or source["source_sha256"] != bundle.source_sha256:
            raise ContractError("source is dirty or differs from bundle; prepare again from reviewed code")
        if (runtime_packages() != bundle.packages
                or platform.python_version() != bundle.python_version):
            raise ContractError("installed runtime packages differ from prepared bundle")
    item("runtime", runtime)
    robot_path = artifact_path(bundle_path, "robot.json")
    envelope_path = artifact_path(bundle_path, "envelope.json")
    manifest_path = artifact_path(bundle_path, bundle.manifest_file)
    robot = item("robot_profile", lambda: load(robot_path, RobotConfig))
    def envelope_input():
        if manifest is None:
            raise ContractError("candidate is invalid")
        return hardware_envelope(envelope_path, manifest)

    envelope = item("hardware_envelope", envelope_input)
    evidence = None
    if robot and envelope and manifest:
        evidence = item("evidence_records", lambda: report(manifest_path, envelope_path, robot_path,
                                                          base / "evidence.json"))
    for gate in (evidence or report(robot_path=robot_path if robot else None))["checks"]:
        checks.append({"check": gate["gate"], "status": gate["status"], "reason": gate["reason"]})
    if preflight_path and robot and manifest:
        item("fresh_preflight", lambda: preflight_check(preflight_path, robot_path, manifest_path,
                                                       max_age_s))
    else:
        checks.append({"check": "fresh_preflight", "status": "unverified",
                       "reason": "Supply a fresh passed preflight for this candidate and robot"})
    if robot and robot.estop_authority == "tablet":
        checks.append({"check": "local_estop_status", "status": "passed",
                       "reason": "Not applicable: tablet authority; fresh preflight and tablet stop evidence remain required"})
    elif robot and robot.hardware_estop:
        if check_estop:
            from .estop_interlock import HardwareInterlock

            item("local_estop_status", lambda: HardwareInterlock(robot).check())
        else:
            checks.append({"check": "local_estop_status", "status": "unverified",
                           "reason": "Use --check-estop for a read-only local bridge-status check"})
    else:
        checks.append({"check": "local_estop_status", "status": "unverified",
                       "reason": "No physical-input interlock is bound to the supplied robot profile"})
    return {"passed": all(c["status"] == "passed" for c in checks), "checks": checks,
            "live_ready": False, "hardware_access": False, "motion_commands": 0,
            "binding_sha256": evidence["binding_sha256"] if evidence else None,
            "note": "Offline preparation checks only. Stand rechecks evidence, operators, preflight and interlocks."}

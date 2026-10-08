"""Durable observations, provenance, and preregistered acceptance metrics."""
from pathlib import Path
import importlib.metadata
import json
import platform
import os
import subprocess
import time
import numpy as np
from .contract import ROOT, ASSET, POLICY_HASH, DT, DECIMATION, sha256


def write_json(path, obj):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def direction_response(rows, smoke=False):
    """Require actual locomotion; a stationary policy must fail walking gates."""
    if smoke:
        return True, {}
    means = {}
    passed = True
    for name, start, axis, target in (("forward", 6, 0, .3), ("lateral", 10, 1, .2),
                                      ("yaw", 14, 2, .4), ("backward", 18, 0, -.2)):
        values = [r["velocity"][axis] for r in rows if start <= r["time"] < start + 2]
        value = float(np.mean(values)) if len(values) >= 90 else None
        means[name] = value
        passed = passed and value is not None and value / target >= .5
    return bool(passed), means


def provenance(backend, contract):
    versions = {}
    for package in ("numpy", "onnxruntime", "mujoco", "pygame", "torch", "isaacsim", "isaaclab"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            pass
    def git(*args):
        return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()
    assets = {str(p.relative_to(ASSET)): sha256(p) for p in ASSET.rglob("*")
              if p.is_file() and p.suffix in (".urdf", ".obj", ".stl", ".py", ".onnx", ".pt")}
    runtime = {}
    lab = Path(os.environ.get("ISAACLAB_PATH", "__isaaclab_path_not_set__"))
    if backend == "isaaclab" and (lab / ".git").exists():
        import hashlib
        def lab_git(*args):
            return subprocess.check_output(["git", "-C", str(lab), *args])
        runtime = dict(path=str(lab), commit=lab_git("rev-parse", "HEAD").decode().strip(),
                       dirty_state=lab_git("status", "--porcelain").decode(),
                       tracked_diff_sha256=hashlib.sha256(lab_git("diff", "HEAD", "--binary")).hexdigest())
    frozen = ROOT / ".source-provenance.json"
    source = json.loads(frozen.read_text()) if frozen.exists() else dict(
        code_commit=git("rev-parse", "HEAD"), dirty_state=git("status", "--porcelain"))
    return dict(backend=backend, **source, host=platform.node(),
                python=platform.python_version(), packages=versions,
                checkpoint_sha256=POLICY_HASH, asset_sha256=assets,
                isaaclab_runtime=runtime,
                dt=DT, decimation=DECIMATION, joint_names=contract.names,
                created_unix=time.time())


class Record:
    def __init__(self, path, backend, contract, scenario, metadata=None):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        self.scenario = scenario
        self.rows = []
        write_json(self.path / "provenance.json", provenance(backend, contract) | (metadata or {}))
        write_json(self.path / "scenario.json", scenario.to_dict())
        self.stream = (self.path / "trace.jsonl").open("w")

    def add(self, t, state, command, observation, action, target, physics_time=None,
            requested_command=None, applied_target=None, communications=None):
        tilt = float(np.arccos(np.clip(-state["gravity"][2], -1, 1)))
        row = dict(time=t, height=float(state["position"][2]), tilt=tilt,
                   physics_time=physics_time,
                   velocity=np.r_[state["linear_velocity"][:2], state["angular_velocity"][2]].tolist(),
                   command=command.velocity.tolist(), q=state["q"].tolist(), dq=state["dq"].tolist(),
                   gravity=state["gravity"].tolist(), position=state["position"].tolist(),
                   feet=state["feet"].tolist(), observation=None if observation is None else observation.tolist(),
                   action=action.tolist(), target=target.tolist())
        if communications is not None:
            row.update(communications=communications, requested_command=requested_command.tolist(),
                       applied_target=applied_target.tolist())
        self.rows.append(row)
        self.stream.write(json.dumps(row, allow_nan=False) + "\n")
        if len(self.rows) % 50 == 0:
            self.stream.flush()
        if t > .5 and (row["height"] < .25 or tilt > .9):
            return "fall"
        return None

    def finish(self, reason="completed"):
        self.stream.close()
        rows = self.rows
        selected = [r for r in rows if r["time"] >= 1]
        if not selected:
            selected = rows
        error = np.array([np.subtract(r["velocity"], r["command"]) for r in selected])
        rmse = np.sqrt(np.mean(error ** 2, axis=0)).tolist() if selected else [None] * 3
        clearance = None
        if self.scenario.mode != "four":
            idx = ("fl", "fr", "hl", "hr").index(self.scenario.mode)
            airborne = [r["feet"][idx][2] - .036 for r in rows if r["time"] >= 4]
            clearance = float(np.mean(np.array(airborne) > .05)) if airborne else 0.0
        survived = reason == "completed"
        tracking = bool(selected and rmse[0] <= .25 and rmse[1] <= .25 and rmse[2] <= .35)
        lifted = clearance is None or clearance >= .9
        response, segment_means = direction_response(rows, self.scenario.smoke)
        result = dict(reason=reason, survived=survived,
                      passed=survived and tracking and lifted and response,
                      gate_revision=2, direction_response_passed=response, segment_means=segment_means,
                      samples=len(rows), simulated_seconds=rows[-1]["time"] if rows else 0,
                      rmse_vx_vy_wz=rmse, min_height=min((r["height"] for r in rows), default=None),
                      max_tilt=max((r["tilt"] for r in rows), default=None),
                      commanded_foot_airborne_fraction=clearance,
                      max_abs_action=max((max(abs(x) for x in r["action"]) for r in rows), default=None))
        write_json(self.path / "result.json", result)
        return result

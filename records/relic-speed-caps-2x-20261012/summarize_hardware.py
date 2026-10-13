"""Summarize recorded physical attempt; no SDK or robot connection."""

import json
from pathlib import Path
import numpy as np
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.contracts import JOINTS, sha256

base = Path("runs/relic-speed-caps-2x-hardware-20261012-001")
complete = json.loads((base / "COMPLETE.json").read_text())
assert verify_run(base, status=complete["status"])
events = [json.loads(line) for line in (base / "events.jsonl").read_text().splitlines()]
policies = [e for e in events if e["event"] == "policy"]
commands = [e for e in events if e["event"] == "command"]
origin = next((e for e in events if e["event"] == "walk_origin"), None)
shutdown = next((e for e in reversed(events) if e["event"] == "shutdown_result"), None)
failure = next((e for e in events if e["event"] == "first_failure"), None)
guard = next((e for e in events if e["event"] == "guard_failure"), None)
result = {
    "raw_status": complete["status"],
    "policy_samples": len(policies),
    "commands": len(commands),
    "history_acknowledgements": sum(e["event"] == "policy_ack" for e in events),
    "first_failure": failure,
    "shutdown_result": shutdown,
    "nominal_goal_reached": False,
    "hardware_qualified": False,
    "reference": "Measured robot odom/body displacement along current post-native-stand heading; no independent metrology",
}
if origin:
    axis = np.array(
        [
            [np.cos(origin["yaw_rad"]), np.sin(origin["yaw_rad"])],
            [-np.sin(origin["yaw_rad"]), np.cos(origin["yaw_rad"])],
        ]
    )
    observed = [e for e in commands if e.get("state", {}).get("body_position_odom") is not None]
    if observed:
        travel = np.array(
            [
                axis
                @ (np.array(e["state"]["body_position_odom"][:2]) - origin["odom_position_m"][:2])
                for e in observed
            ]
        )
        result.update(
            final_forward_m=float(travel[-1, 0]),
            maximum_forward_m=float(travel[:, 0].max()),
            final_lateral_m=float(travel[-1, 1]),
            maximum_abs_lateral_m=float(abs(travel[:, 1]).max()),
        )
        progress = [e for e in events if e["event"] == "walk_progress"]
        result["nominal_goal_reached"] = any(e.get("phase") == "completed" for e in progress)
        if progress:
            result["last_walking_phase"] = progress[-1]["phase"]
        if guard and guard.get("state", {}).get("body_position_odom") is not None:
            rejected = axis @ (
                np.array(guard["state"]["body_position_odom"][:2]) - origin["odom_position_m"][:2]
            )
            result["rejected_state_forward_m"] = float(rejected[0])
if policies:
    result["maximum_forward_request_m_s"] = max(e.get("velocity_command", [0])[0] for e in policies)
    result["maximum_inference_s"] = max(e["inference_s"] for e in policies)
if commands:
    times = np.array([e["monotonic_s"] for e in commands])
    result["command_span_s"] = float(times[-1] - times[0])
    if len(times) > 1:
        result["maximum_command_gap_s"] = float(np.diff(times).max())
    result["peak_issued_state_leg_speed_rad_s"] = float(
        np.max(np.abs([e["state"]["velocities"][:12] for e in commands]))
    )
    manifest = json.loads(
        Path("local/relic-speed-caps-2x-20261012-001/bundle/policy/manifest.json").read_text()
    )
    offsets = np.array([np.array(e["positions"]) - e["state"]["positions"] for e in commands])
    velocity = np.array([e["state"]["velocities"] for e in commands])
    torque = (
        np.array(manifest["gains"]["kp"]) * offsets
        - np.array(manifest["gains"]["kd"]) * velocity
        + np.array([e["feedforward"] for e in commands])
    )
    result["per_joint_max_abs_issued_offset_rad"] = np.max(abs(offsets), axis=0).tolist()
    result["per_joint_max_abs_estimated_PD_torque_Nm"] = np.max(abs(torque), axis=0).tolist()
    result["estimated_torque_note"] = (
        "PD-plus-feedforward estimate; not independently measured torque"
    )
# The stop is a measured-state guard, not a rejected angle-target diagnosis.
guard_event = next(e for e in events if e['event']=='guard_failure')
env_path = Path('local/relic-speed-caps-2x-20261012-001/bundle/envelope.json')
env = json.loads(env_path.read_text())
state = guard_event['state']
result['measured_guard_violations'] = []
for i,name in enumerate(JOINTS):
    if i<12 and not env['position_min'][i]<=state['positions'][i]<=env['position_max'][i]:
        result['measured_guard_violations'].append(dict(joint=name, quantity='position',
            actual_rad=state['positions'][i], min_rad=env['position_min'][i],
            max_rad=env['position_max'][i]))
    if i<12 and abs(state['velocities'][i])>env['velocity_max'][i]:
        result['measured_guard_violations'].append(dict(joint=name, quantity='velocity',
            actual_rad_s=state['velocities'][i], bound_rad_s=env['velocity_max'][i]))
    if abs(state['loads'][i])>env['load_max'][i]:
        result['measured_guard_violations'].append(dict(joint=name, quantity='load',
            actual_Nm=state['loads'][i], bound_Nm=env['load_max'][i]))
active_at = next(e['monotonic_s'] for e in events
    if e['event']=='relic_phase' and e['phase']=='policy')
result['recorded_policy_sample_span_s'] = policies[-1]['monotonic_s']-active_at
result['issued_policy_command_span_s'] = commands[-1]['guard_monotonic_s']-active_at
result['max_recorded_core_computation_s'] = max(
    e['monotonic_s']-e['guard_monotonic_s'] for e in commands)
result['selected_core_limit_s'] = env['max_inference_s']
result['envelope_sha256'] = sha256(env_path)
result["initial_body_height_m"] = next(
    e["requested_height_m"] for e in events if e["event"] == "relic_initialized"
)
result["raw_complete_sha256"] = sha256(base / "COMPLETE.json")
p = Path("runs/relic-speed-caps-2x-postcheck-20261012-001")
assert verify_run(p)
post = json.loads((p / "result.json").read_text())
result["postcheck"] = dict(
    passed=post["passed"],
    motors_off=post["snapshot"]["motors_off"],
    active_fault_count=post["snapshot"]["active_fault_count"],
)
p = Path("runs/relic-speed-caps-2x-compact-20261012-001")
assert verify_run(p, status="completed")
compact = json.loads((p / "summary.json").read_text())
result["compact"] = {
    k: compact[k] for k in ["source_status", "capture_complete", "status", "compressed_bytes"]
}
result["compact_complete_sha256"] = sha256(p / "COMPLETE.json")
result["physical_execution_commit"] = "90a4edbf0b1c593f30bdd67fc4b7ea6c2bfacb5a"
result["retry_performed"] = False
# Public record contains derived measurements, never endpoint/identity/raw pose.
result["shutdown_result"] = {
    k: v
    for k, v in result["shutdown_result"].items()
    if k
    in ["motors_off_confirmed", "lease_returned", "elapsed_s", "errors", "within_shutdown_budget"]
}
result["first_failure"] = {
    k: v for k, v in result["first_failure"].items() if k in ["error_type", "reason"]
}
atomic_json(Path("records/relic-speed-caps-2x-20261012/hardware-result-summary.json"), result)
print(json.dumps({k:result[k] for k in ['final_forward_m','rejected_state_forward_m','measured_guard_violations','issued_policy_command_span_s','max_recorded_core_computation_s','shutdown_result','compact']}))

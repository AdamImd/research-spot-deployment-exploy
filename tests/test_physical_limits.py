import math

import numpy as np
import pytest
from pydantic import ValidationError

from spot_deploy.contracts import ContractError, Envelope
from spot_deploy.physical_limits import PhysicalLimits
from spot_deploy.safety import Guard


def persistent(envelope):
    return Envelope.model_validate(
        envelope.model_dump()
        | dict(
            physical_limit_persistence_s=0.5,
            physical_limit_persistence_reason="Operator selected all physical limits",
        )
    )


def fresh(state, t, **updates):
    return state.model_copy(
        update=dict(robot_time_s=1000 + t, received_monotonic_s=100 + t, **updates)
    )


def test_strict_boundary_recovery_separate_joints_and_duplicate_checks():
    events = []
    limits = PhysicalLimits(0.5, lambda name, **data: events.append((name, data)))
    limits.check([True, False], 1.0, "speed")
    limits.check([True, False], 1.0, "speed")  # Same control tick is not another duration.
    limits.check([True, False], 1.5, "speed")
    limits.check([False, True], 1.5, "speed")  # New joint gets its own clock.
    limits.check([True, True], 1.8, "speed")  # Recovered joint starts from zero.
    limits.check([True, True], 2.0, "speed")
    with pytest.raises(ContractError, match="persisted 0.501000s"):
        limits.check([True, True], 2.001, "speed")
    assert [e[0] for e in events] == [
        "physical_limit_started",
        "physical_limit_recovered",
        "physical_limit_started",
        "physical_limit_started",
        "physical_limit_stop",
    ]
    assert events[-1][1]["index"] == 1


def test_repeated_short_impulses_do_not_accumulate_and_default_stops_immediately():
    limits = PhysicalLimits(0.5)
    for t in [0.0, 1.0, 2.0, 3.0]:
        limits.check(True, t, "torque")
        limits.check(False, t + 0.49, "torque")
    assert not limits.pending
    with pytest.raises(ContractError, match="torque"):
        PhysicalLimits().check(True, 0.0, "torque")


@pytest.mark.parametrize(
    "updates,pattern",
    [
        ({"positions": [3.0] + [0.0] * 18}, "measured joint position"),
        ({"velocities": [3.0] + [0.0] * 18}, "measured joint velocity"),
        ({"loads": [3.0] + [0.0] * 18}, "measured joint velocity or load"),
        ({"linear_velocity_odom": [1.0, 0.0, 0.0]}, "body speed"),
        ({"angular_velocity_odom": [1.0, 0.0, 0.0]}, "body speed"),
        ({"odom_quaternion_wxyz": [math.cos(0.2), math.sin(0.2), 0.0, 0.0]}, "body attitude"),
        ({"odom_quaternion_wxyz": [math.cos(0.2), 0.0, math.sin(0.2), 0.0]}, "body attitude"),
        ({"positions": [0.0] * 12 + [0.03] + [0.0] * 6}, "stowed pose"),
    ],
)
def test_state_limits_persist_only_after_half_second(manifest, envelope, state, updates, pattern):
    guard = Guard(manifest, persistent(envelope))
    for tick in range(101):
        t = tick * 0.005
        guard.check_state(fresh(state, t, **updates), 100 + t, 1000 + t)
    with pytest.raises(ContractError, match=pattern):
        guard.check_state(fresh(state, 0.505, **updates), 100.505, 1000.505)


@pytest.mark.parametrize(
    "target,ff,pattern",
    [
        ([3.0] + [0.0] * 18, None, "command position"),
        ([0.6] + [0.0] * 18, None, "command tracking"),
        ([0.0] * 19, [3.0] + [0.0] * 18, "feedforward exceeds"),
    ],
)
def test_command_physical_limits_persist(manifest, envelope, state, target, ff, pattern):
    guard = Guard(manifest, persistent(envelope))
    for tick in range(101):
        t = tick * 0.005
        guard.command(target, fresh(state, t), 100 + t, 1000 + t, 100 + t, 0.001, ff)
    with pytest.raises(ContractError, match=pattern):
        guard.command(target, fresh(state, 0.505), 100.505, 1000.505, 100.505, 0.001, ff)


def test_requested_torque_persists_without_changing_target(manifest, envelope, state):
    manifest.gains.kp = [100.0] * 19
    guard = Guard(manifest, persistent(envelope))
    target = [0.03] + [0.0] * 18
    for tick in range(101):
        t = tick * 0.005
        cmd = guard.command(target, fresh(state, t), 100 + t, 1000 + t, 100 + t, 0.001)
        assert list(cmd.positions) == target
    with pytest.raises(ContractError, match="predicted PD-plus-feedforward"):
        guard.command(target, fresh(state, 0.505), 100.505, 1000.505, 100.505, 0.001)


def test_measured_tracking_and_command_slew_persist(manifest, envelope, state):
    guard = Guard(manifest, persistent(envelope))
    guard.last_command = np.array([0.6] + [0.0] * 18)
    for tick in range(101):
        t = tick * 0.005
        guard.check_state(fresh(state, t), 100 + t, 1000 + t)
    with pytest.raises(ContractError, match="measured joint tracking"):
        guard.check_state(fresh(state, 0.505), 100.505, 1000.505)
    guard = Guard(manifest, persistent(envelope))
    for tick in range(102):
        t = tick * 0.005
        target = [0.01 * (-1) ** tick] + [0.0] * 18
        guard.command(target, fresh(state, t), 100 + t, 1000 + t, 100 + t, 0.001)
    with pytest.raises(ContractError, match="slew"):
        guard.command([0.01] + [0.0] * 18, fresh(state, 0.51), 100.51, 1000.51, 100.51, 0.001)


@pytest.mark.parametrize("case", ["stale", "nan", "inf", "inference", "gap", "clock"])
def test_integrity_and_timing_remain_immediate(manifest, envelope, state, case):
    guard = Guard(manifest, persistent(envelope))
    guard.command([0.0] * 19, state, 100.0, 1000.0, 100.0, 0.001)
    sample = fresh(state, 0.005)
    now, robot_now, policy_at, inference = 100.005, 1000.005, 100.005, 0.001
    if case == "stale":
        sample = state
        now, robot_now = 100.2, 1000.2
    elif case == "nan":
        sample.positions[0] = float("nan")
    elif case == "inf":
        sample.loads[0] = float("inf")
    elif case == "inference":
        inference = 0.1
    elif case == "gap":
        now, robot_now, policy_at = 100.06, 1000.06, 100.06
        sample = fresh(state, 0.06)
    else:
        now = 99.99
    with pytest.raises(ContractError):
        guard.command([0.0] * 19, sample, now, robot_now, policy_at, inference)


@pytest.mark.parametrize(
    "change",
    [
        {"physical_limit_persistence_s": 0.5},
        {"physical_limit_persistence_s": 0.51, "physical_limit_persistence_reason": "x"},
        {"physical_limit_persistence_s": float("nan"), "physical_limit_persistence_reason": "x"},
        {"physical_limit_persistence_reason": "x"},
    ],
)
def test_duration_requires_bounded_finite_value_and_reason(envelope, change):
    with pytest.raises(ValidationError):
        Envelope.model_validate(envelope.model_dump() | change)


def test_manufacturer_torque_coupling_and_knee_domain_use_same_persistence(manifest, envelope):
    from test_actuator_limits import manufacturer_envelope, pose

    env = persistent(manufacturer_envelope(envelope))
    for q, loads, pattern in [
        (pose(), [50.0] + [0.0] * 18, "manufacturer joint torque"),
        (pose(-2.78), [0.0, 0.0, 42.0] + [0.0] * 16, "manufacturer joint torque"),
        (pose(), [0.0] * 13 + [160.0, -80.0] + [0.0] * 4, "coupled"),
        (pose(-2.8), [0.0] * 19, "outside manufacturer torque table"),
    ]:
        guard = Guard(manifest, env)
        for tick in range(101):
            guard.check_actuator_load(q, loads, "measured load", 100 + tick * 0.005)
        with pytest.raises(ContractError, match=pattern):
            guard.check_actuator_load(q, loads, "measured load", 100.505)


def test_knee_outside_domain_uses_endpoint_cap_during_grace(manifest, envelope):
    from test_actuator_limits import manufacturer_envelope, pose

    guard = Guard(manifest, persistent(manufacturer_envelope(envelope)))
    loads = np.zeros(19)
    loads[2] = 38.0
    guard.check_actuator_load(pose(-3.0), loads, "measured load", 100.0)
    assert ("measured knee table domain", 0) in guard.physical_limits.pending
    assert ("measured load: manufacturer joint torque limit", 2) in guard.physical_limits.pending


def test_walking_deviation_persists_but_stale_pose_stops_immediately(state):
    import json
    from pathlib import Path
    from test_walking import posed
    from test_control_loop import Log
    from spot_deploy.walking import ForwardTravel, WalkingPlan

    plan = WalkingPlan.model_validate(
        json.loads((Path(__file__).parents[1] / "configs/relic-walk-075.json").read_text())
    )
    travel = ForwardTravel(plan, Log(), physical_limits=PhysicalLimits(0.5))
    travel.latch(posed(state, 0))
    travel.activate(10.0)
    # Gradually cross the path bound without creating an odometry jump.
    for tick in range(21):
        travel.update(posed(state, tick * 0.005, y=tick * 0.01), 10 + tick * 0.005)
    with pytest.raises(ContractError, match="lateral or heading.*persisted"):
        for tick in range(21, 160):
            travel.update(posed(state, tick * 0.005, y=0.2), 10 + tick * 0.005)
    stale = ForwardTravel(plan, Log(), physical_limits=PhysicalLimits(0.5))
    stale.latch(posed(state, 0))
    stale.activate(10.0)
    bad = posed(state, 0.005)
    bad.body_pose_robot_time_s = 999.0
    with pytest.raises(ContractError, match="stale"):
        stale.update(bad, 10.005)


def test_saved_simulator_audit_accepts_completed_marker_and_rejects_modified_data(
    tmp_path, envelope, fixture_path
):
    import hashlib
    import json
    import runpy
    from pathlib import Path
    from spot_deploy.contracts import JOINTS

    audit = runpy.run_path(str(Path(__file__).parents[1] / "tools/audit_physical_limits.py"))["run"]
    source = tmp_path / "source"
    source.mkdir()
    (source / "configuration.json").write_text(json.dumps({"simulation_joint_names": list(JOINTS)}))
    rows = [
        dict(
            trial_time=t,
            q=[0.0] * 19,
            dq=[0.0] * 19,
            positions=[0.0] * 19,
            feedforward=[0.0] * 19,
            preceding_applied_torque_Nm=([3.0] + [0.0] * 18 if t < 0.01 else [0.0] * 19),
            root_quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
            position=[0.0, 0.0, 0.5],
            linear_velocity=[0.0] * 3,
            angular_velocity=[0.0] * 3,
        )
        for t in [0.0, 0.005, 0.01]
    ]
    data = source / "rollout.jsonl"
    data.write_text("".join(json.dumps(r) + "\n" for r in rows))
    (source / "COMPLETE.json").write_text(
        json.dumps(
            dict(
                status="completed",
                artifacts={
                    p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in source.glob("*.json*")
                },
            )
        )
    )
    env = persistent(envelope).model_copy(update={"scope": "simulation"})
    env_path = tmp_path / "envelope.json"
    env_path.write_text(env.model_dump_json())
    result = audit(source, fixture_path / "manifest.json", env_path, tmp_path / "audit")
    assert result["passed"] and result["samples_checked"] == 3
    assert result["crossings_started"] == 1 and result["crossings_recovered"] == 1
    assert result["max_recovered_duration_s"] == pytest.approx(0.01)
    data.write_text(data.read_text() + "\n")
    with pytest.raises(ValueError, match="integrity"):
        audit(source, fixture_path / "manifest.json", env_path, tmp_path / "bad-audit")

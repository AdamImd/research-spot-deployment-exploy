"""Offline zero-history ReLIC startup audit on a recorded native standing baseline.

Every sample is an independent hypothetical startup, not a closed-loop rollout.
No robot client is constructed and no commands are sent.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import ContractError, Envelope, State, check_artifacts, load
from spot_deploy.records import RunRecord, atomic_json, verify_run
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_rollout import ReLICRollout


class MemoryRecord:
    def __init__(self):
        self.events = []

    def event(self, event, **values):
        self.events.append(dict(event=event, **values))


def audit_sample(policy, envelope, state, height):
    """Use historical logical times, then the real supported-hold/handover core."""
    policy.reset()
    policy.initialize_height(height)
    previous = np.zeros(12, dtype=np.float32)
    target, inference_s, observation, action = policy.evaluate(state, previous)
    kp, kd = map(np.asarray, (policy.manifest.gains.kp, policy.manifest.gains.kd))
    torque = kp * (target - state.positions) - kd * state.velocities
    events = MemoryRecord()
    core = ReLICRollout(policy, envelope, events)
    now, robot_now = state.received_monotonic_s, state.robot_time_s
    result = dict(target=target.tolist(), raw_action=action.tolist(),
                  observation=observation.tolist(), previous_actions=previous.tolist(),
                  policy_torque_Nm=torque.tolist(), inference_s=inference_s,
                  measured_load_Nm=state.loads, guard_passed=False)
    try:
        core.initialize(state, height, now, robot_now)
        hold = core.command(state, now, robot_now)
        hold_torque = kp * (np.asarray(hold.positions) - state.positions)
        hold_torque += -kd * state.velocities + np.asarray(hold.feedforward)
        result.update(supported_hold_torque_Nm=hold_torque.tolist(),
                      support_feedforward_Nm=list(hold.feedforward),
                      torque_step_Nm=(torque-hold_torque).tolist())
        # Explicit ideal next-tick receipt; state physics are unchanged. This only
        # checks the startup command, not later history or robot dynamics.
        dt = 1 / policy.manifest.stream_hz
        next_state = state.model_copy(update=dict(
            robot_time_s=robot_now+dt, received_monotonic_s=now+dt,
            last_command_key=hold.key, last_command_received_robot_s=robot_now+dt))
        core.activate(now+dt)
        core.command(next_state, now+dt, robot_now+dt)
        result['guard_passed'] = True
    except ContractError as exc:
        result['first_guard_failure'] = str(exc)
    result['events'] = events.events
    return result


def run(args):
    # A failed motion trial is still valid recorded input if all artifacts match.
    completion = json.loads((args.source_run/'COMPLETE.json').read_text())
    if not verify_run(args.source_run, status=completion['status']):
        raise ContractError('source recording integrity failed')
    baseline = json.loads((args.source_run/'baseline.json').read_text())
    manifest = load_manifest(args.manifest)
    if manifest.relic.preparation.kind != 'direct':
        raise ContractError('startup audit requires the direct-start candidate')
    envelope = load(args.envelope, Envelope)
    checkpoint = check_artifacts(args.manifest, manifest)
    record = RunRecord(args.output, 'offline-relic-startup-audit', [
        Path(__file__), args.manifest, args.envelope, args.source_run/'COMPLETE.json',
        args.source_run/'baseline.json'])
    try:
        policy = make_policy(checkpoint, manifest, args.manifest)
        states = [State.model_validate(s) for s in baseline['samples']]
        if not states:
            raise ContractError('empty native standing baseline')
        rows = []
        for i, state in enumerate(states):
            row = audit_sample(policy, envelope, state, baseline['height'])
            row['sample_index'] = i
            rows.append(row)
            record.event('independent_startup', **row)
        torque = np.array([r['policy_torque_Nm'] for r in rows])
        steps = np.array([r['torque_step_Nm'] for r in rows if 'torque_step_Nm' in r])
        result = dict(hardware_access=False, hardware_qualified=False,
                      joint_order=list(envelope.joint_order),
                      closed_loop=False, source_status=completion['status'],
                      history='zero independently at every sample',
                      clocks='recorded sample rebased to ideal next-tick receipt for handover only',
                      sample_count=len(rows), passed_samples=sum(r['guard_passed'] for r in rows),
                      all_startup_guards_passed=all(r['guard_passed'] for r in rows),
                      body_height_m=baseline['height']['height_m'],
                      policy_torque_min_Nm=torque.min(axis=0).tolist(),
                      policy_torque_max_Nm=torque.max(axis=0).tolist(),
                      max_abs_torque_step_Nm=(abs(steps).max(axis=0).tolist()
                                              if len(steps) else None),
                      final_sample=rows[-1])
        atomic_json(args.output/'audit.json', result)
        record.finish('passed' if result['all_startup_guards_passed'] else 'failed', result)
        print(json.dumps({k: result[k] for k in ('sample_count', 'passed_samples',
                         'all_startup_guards_passed', 'hardware_qualified')}))
        return 0 if result['all_startup_guards_passed'] else 1
    except BaseException as exc:
        record.finish('failed', dict(error_type=type(exc).__name__, hardware_access=False))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source-run', 'manifest', 'envelope', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    return run(parser.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())

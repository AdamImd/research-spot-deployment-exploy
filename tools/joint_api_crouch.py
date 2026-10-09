# /// script
# requires-python = ">=3.12,<3.13"
# dependencies = ["bosdyn-client==5.1.1", "numpy==2.2.6", "pydantic==2.12.3"]
# ///
"""Standalone diagnostic CLI: offline plan, read-only inspect, or explicit joint-API trial.

Uses the checkout's low-level SDK adapters and limits; no policy, ONNX or simulator.
The manufacturer tablet is the sole E-stop authority. No E-stop writes are made.
"""

import argparse
import json
from pathlib import Path
import signal
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from spot_deploy.contracts import ContractError, Envelope, JOINTS, RobotConfig, load, sha256
from spot_deploy.network import wired_route
from spot_deploy.records import atomic_json, source_identity, utcnow
from spot_deploy.safety import Guard, Schedule, percentiles
from spot_deploy.sdk_read import ReadOnlySpot, validate_snapshot


def settings(path):
    value = json.loads(Path(path).read_text())
    if value['duration_s'] != 10 or value['stream_hz'] != 200:
        raise ValueError('this registered test is 10 seconds at 200 Hz')
    for key in ('kp', 'kd'):
        a = np.asarray(value[key], dtype=float)
        if a.shape != (19,) or not np.isfinite(a).all() or (a <= 0).any():
            raise ValueError('invalid gains')
    if not (0 <= value['hip_pitch_delta_deg'] <= 5 and -10 <= value['knee_delta_deg'] <= 0):
        raise ValueError('draft motion may not exceed +5 hip / -10 knee degrees')
    if (value['feedforward'] != 'median_native_load' or value['native_baseline_s'] != 1
            or not 0 < value['max_baseline_speed_rad_s'] <= .1
            or not 2 <= value['shutdown_measurement_timeout_s'] <= 15):
        raise ValueError('invalid baseline or shutdown protocol')
    return value


def profile(t):
    """C2 smooth reference; 2 hold, 3 down, 1 crouch, 3 up, 1 hold seconds."""
    if t < 2 or t >= 9:
        return 0., 0.
    if 5 <= t < 6:
        return 1., 0.
    returning = t >= 6
    x = (t - (6 if returning else 2)) / 3
    value = x**3 * (10 - 15*x + 6*x*x)
    derivative = 30*x*x*(1-x)**2 / 3
    return (1-value, -derivative) if returning else (value, derivative)


def reference(initial, t, config):
    delta = np.zeros(19)
    delta[[1, 4, 7, 10]] = np.radians(config['hip_pitch_delta_deg'])
    delta[[2, 5, 8, 11]] = np.radians(config['knee_delta_deg'])
    blend, derivative = profile(t)
    return np.asarray(initial) + blend*delta, derivative*delta


class ReferenceGuard(Guard):
    """Reuse numeric state/command guards without a fictitious RL manifest/handover."""

    def __init__(self, initial, config, envelope, feedforward):
        self.envelope = envelope
        self.manifest = SimpleNamespace(
            gains=SimpleNamespace(kp=config['kp'], kd=config['kd'], feedforward=feedforward),
            arm_stowed_positions=np.asarray(initial)[12:])
        self.last_state_time = self.last_received = self.last_command = self.last_sent = None
        self.key = 0
        if 1/config['stream_hz'] >= envelope.max_command_gap_s:
            raise ContractError('reference period exceeds command gap limit')
        if np.any(np.abs(feedforward) > envelope.load_max):
            raise ContractError('baseline feedforward exceeds limits')
        self.check_actuator_load(initial, feedforward, 'baseline feedforward')


class Samples:
    """Fixed-size in-memory numeric recording; compression happens after cleanup."""

    TIMING = ('host_monotonic_s', 'trial_time_s', 'command_key', 'state_robot_s',
              'state_receive_monotonic_s', 'state_age_s', 'end_robot_s', 'ack_key',
              'ack_received_robot_s', 'reference_compute_s', 'schedule_lateness_s')
    VECTORS = ('target', 'measured_q', 'measured_dq', 'measured_load', 'feedforward',
               'position_error', 'p_term', 'd_term', 'estimated_total_torque', 'reference_dq')

    def __init__(self, capacity=6000):
        self.count = 0
        self.timing = np.empty((capacity, len(self.TIMING)))
        self.vectors = np.empty((capacity, len(self.VECTORS), 19))
        self.body = np.empty((capacity, 10))

    def add(self, timing, state, target, derivative, ff, kp, kd):
        i = self.count
        if i == len(self.timing):
            raise ContractError('measurement buffer full')
        error = target - state.positions
        p, d = kp*error, -kd*np.asarray(state.velocities)
        self.timing[i] = timing
        self.vectors[i] = [target, state.positions, state.velocities, state.loads,
                           ff, error, p, d, p+d+ff, derivative]
        self.body[i] = [*state.odom_quaternion_wxyz, *state.linear_velocity_odom,
                        *state.angular_velocity_odom]
        self.count += 1

    def save(self, output):
        np.savez_compressed(output / 'samples.npz', timing=self.timing[:self.count],
                            body=self.body[:self.count], **{
                                name: self.vectors[:self.count, i] for i, name in enumerate(self.VECTORS)})


def require_off(reader):
    from bosdyn.api.robot_state_pb2 import PowerState
    state = reader.state_client.get_robot_state(timeout=reader.config.rpc_timeout_s)
    if state.power_state.motor_power_state != PowerState.STATE_OFF:
        raise ContractError('motor-off not confirmed')


def live(args, config, samples, events):
    from spot_deploy.sdk_control import ControlSpot, command_proto

    robot = load(args.robot, RobotConfig)
    envelope = load(args.envelope, Envelope)
    binding = SimpleNamespace(**json.loads(args.binding.read_text()))
    if envelope.scope != 'hardware' or robot.estop_authority != 'tablet' or robot.hardware_estop:
        raise ContractError('hardware envelope and tablet authority required')
    if config['duration_s'] > envelope.max_duration_s:
        raise ContractError('trial exceeds reviewed duration')
    if args.mode == 'execute' and (not args.operator or not args.safety_operator
            or not args.operator.strip() or not args.safety_operator.strip()
            or args.operator.strip().casefold() == args.safety_operator.strip().casefold()):
        raise ContractError('two distinct named operators required')
    events.append({'event': 'route', **wired_route(robot)})
    reader, control, worker = ReadOnlySpot(robot), None, None
    stop, emitted, active = threading.Event(), threading.Event(), threading.Event()
    shared = {'error': None, 'health_at': 0., 'active_at': None, 'last_emit': None}
    result = {'completed': False, 'policy_used': False, 'estop_writes': 0}
    handlers = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}

    def check():
        if shared['error']:
            raise ContractError(shared['error'])
        if stop.is_set():
            raise ContractError('trial cancelled')

    def health():
        try:
            while not stop.is_set():
                control.heartbeat()
                h = reader.health()
                if not h['estop_ready'] or h['fault_count'] or h['battery_percent'] < envelope.min_battery_percent:
                    raise ContractError('robot health/stop check failed')
                shared['health_at'] = h['time']
                events.append({'event': 'health', **h})
                if active.is_set():
                    control.check_stream()
                    if not control.active():
                        raise ContractError('joint control inactive')
                stop.wait(.08)
        except Exception as exc:
            if not stop.is_set():
                shared['error'] = type(exc).__name__ + ': health monitor stopped'
                stop.set()

    try:
        reader.connect()
        snapshot = reader.snapshot()
        atomic_json(args.output / 'robot-snapshot.json', snapshot)
        checks = validate_snapshot(snapshot, robot, binding, envelope, for_control=True)
        result['preflight'] = checks
        if not all(checks.values()):
            raise ContractError('fresh preflight failed')
        require_off(reader)
        if args.mode == 'inspect':
            result['inspection_passed'] = True
            return result
        # A native shutdown measurement has its own timeout; retain the RL budget for comparison.
        control = ControlSpot(reader, envelope.model_copy(update={
            'shutdown_timeout_s': config['shutdown_measurement_timeout_s']}))
        control.acquire()  # Never take a tablet/client lease.
        worker = threading.Thread(target=health, daemon=True)
        worker.start()
        control.native_stand(stop)
        check()
        height = reader.read_body_height()
        reader.start_stream(35)
        baseline, last_stamp = [], None
        until = time.monotonic()+config['native_baseline_s']
        while time.monotonic() < until:
            check()
            state = reader.mailbox.get()
            if time.monotonic()-state.received_monotonic_s > envelope.max_state_age_s:
                raise ContractError('stale baseline state')
            if state.robot_time_s != last_stamp:
                baseline.append(state.model_dump())
                last_stamp = state.robot_time_s
            stop.wait(.005)
        if len(baseline) < 100 or max(np.max(np.abs(s['velocities'])) for s in baseline) > config['max_baseline_speed_rad_s']:
            raise ContractError('native baseline not settled')
        initial = np.asarray(state.positions)
        ff = np.median([s['loads'] for s in baseline], axis=0)
        atomic_json(args.output / 'baseline.json', {'samples': baseline, 'initial_q': initial.tolist(),
                    'feedforward': ff.tolist(), 'height': height,
                    'native_load_equivalent_position_offset_rad': (ff/np.asarray(config['kp'])).tolist()})
        guard = ReferenceGuard(initial, config, envelope, ff)
        guard.key = state.last_command_key
        # Check the entire geometric path against reviewed positions before joint activation.
        trajectory = np.array([reference(initial, t, config)[0] for t in np.linspace(0, 10, 2001)])
        if np.any(trajectory < envelope.position_min) or np.any(trajectory > envelope.position_max):
            raise ContractError('planned trajectory exceeds reviewed position limits')
        kp, kd = np.asarray(config['kp']), np.asarray(config['kd'])
        ack_delays, gaps, sent = [], [], {}
        last_ack, last_ack_at = state.last_command_key, time.monotonic()
        producer_done = threading.Event()

        def commands():
            nonlocal last_ack, last_ack_at
            schedule = Schedule(config['stream_hz'])
            try:
                while not stop.is_set():
                    check()
                    now = time.monotonic()
                    if now-shared['health_at'] > envelope.max_full_state_age_s:
                        raise ContractError('health watchdog expired')
                    state = reader.mailbox.get()
                    now = time.monotonic()  # A receiver can publish while the mailbox is read.
                    if state.last_command_key > last_ack:
                        if state.last_command_key not in sent:
                            raise ContractError('unknown acknowledgement')
                        ack_delays.append(now-sent[state.last_command_key])
                        if ack_delays[-1] > envelope.max_command_ack_s:
                            raise ContractError('acknowledgement latency exceeded')
                        last_ack, last_ack_at = state.last_command_key, now
                        for key in list(sent):
                            if key <= last_ack:
                                del sent[key]
                    elapsed = max(0., now-shared['active_at']) if active.is_set() else 0.
                    if active.is_set() and now-max(last_ack_at, shared['active_at']) > envelope.max_command_ack_s:
                        raise ContractError('acknowledgement watchdog expired')
                    target, derivative = reference(initial, elapsed, config)
                    robot_now = reader.robot_now()
                    command = guard.command(target, state, now, robot_now, now, 0., ff)
                    wire = command_proto(command, guard.manifest, envelope)
                    finished = time.monotonic()
                    if finished-now > envelope.max_inference_s:
                        raise ContractError('reference production deadline exceeded')
                    if shared['last_emit'] is not None:
                        gaps.append(finished-shared['last_emit'])
                        if gaps[-1] > envelope.max_command_gap_s:
                            raise ContractError('command gap exceeded')
                    samples.add([finished, elapsed, command.key, state.robot_time_s,
                                 state.received_monotonic_s, robot_now-state.robot_time_s,
                                 command.end_robot_time_s, state.last_command_key,
                                 state.last_command_received_robot_s, finished-now,
                                 max(0., now-schedule.next)],
                                state, target, derivative, ff, kp, kd)
                    sent[command.key] = finished
                    shared['last_emit'] = finished
                    emitted.set()
                    yield wire
                    stop.wait(schedule.advance(time.monotonic()))
            except Exception as exc:
                shared['error'] = str(exc) if isinstance(exc, ContractError) else type(exc).__name__
                stop.set()
            finally:
                producer_done.set()

        control.start(commands(), 20)
        if not emitted.wait(robot.rpc_timeout_s):
            check()
            raise ContractError('command producer did not start')
        check()
        control.activate()
        deadline = time.monotonic()+robot.rpc_timeout_s
        while not control.active():
            check()
            if time.monotonic() > deadline:
                raise ContractError('joint activation timeout')
            stop.wait(.005)
        shared['active_at'] = time.monotonic()
        active.set()
        events.append({'event': 'active', 'monotonic_s': shared['active_at']})
        while time.monotonic()-shared['active_at'] < 10:
            check()
            if time.monotonic()-shared['last_emit'] > envelope.max_command_gap_s:
                raise ContractError('command producer stalled')
            stop.wait(.003)
        check()
        result.update(completed=True, command_gap_s=percentiles(gaps), ack_delay_s=percentiles(ack_delays))
    except Exception as exc:
        result['error'] = str(exc) if isinstance(exc, ContractError) else type(exc).__name__
    finally:
        stop.set()
        if control:
            start = time.monotonic()
            try:
                control.close()
                require_off(reader)
                result['motors_off_confirmed'] = True
                result['lease_returned'] = control.lease is not None
            except Exception as exc:
                result.update(completed=False, cleanup_error=type(exc).__name__,
                              motors_off_confirmed=False)
            result['shutdown_s'] = time.monotonic()-start
            result['within_rl_shutdown_budget'] = result['shutdown_s'] <= envelope.shutdown_timeout_s
        if worker:
            worker.join(robot.rpc_timeout_s+1)
            if worker.is_alive():
                result.update(completed=False, health_thread_unconfirmed=True)
        if 'producer_done' in locals():
            if not producer_done.wait(robot.rpc_timeout_s+1):
                result.update(completed=False, producer_shutdown_unconfirmed=True)
        if shared['error']:
            result.update(completed=False, runtime_error=shared['error'])
        result['state_stream'] = reader.stream_statistics()
        try:
            reader.close()
        except Exception as exc:
            result.update(completed=False, reader_close_error=type(exc).__name__)
        for sig, previous in handlers.items():
            signal.signal(sig, previous)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('plan', 'inspect', 'execute'))
    parser.add_argument('--settings', type=Path, default=Path(__file__).resolve().parents[1]/'configs/joint-api-crouch.json')
    parser.add_argument('--robot', type=Path)
    parser.add_argument('--envelope', type=Path)
    parser.add_argument('--binding', type=Path, help='model/payload hash binding, no policy weights')
    parser.add_argument('--operator')
    parser.add_argument('--safety-operator')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = settings(args.settings)
    if args.mode != 'plan' and not all((args.robot, args.envelope, args.binding)):
        parser.error('inspect/execute requires robot, envelope and binding paths')
    if args.mode == 'execute' and not args.execute:
        parser.error('physical motion requires --execute')
    args.output.mkdir(parents=True, exist_ok=False)
    samples, events = Samples(), []
    atomic_json(args.output/'run.json', {'mode': args.mode, 'started_at': utcnow(),
                'source': source_identity(), 'script_sha256': sha256(Path(__file__)),
                'settings': config, 'operator': args.operator, 'safety_operator': args.safety_operator,
                'joint_order': list(JOINTS), 'timing_columns': Samples.TIMING,
                'body_columns': ['qw','qx','qy','qz','vx','vy','vz','wx','wy','wz'],
                'input_hashes': {str(p):sha256(p) for p in (args.settings,args.robot,args.envelope,args.binding) if p},
                'desired_joint_velocity': 0, 'estimated_torque_note': 'kp*(target-q)-kd*dq+constant native load; not measured motor torque'})
    if args.mode == 'plan':
        times = np.linspace(0,10,2001)
        vectors = [reference(np.zeros(19), t, config) for t in times]
        np.savez_compressed(args.output/'trajectory.npz', time_s=times,
                            delta_q=[v[0] for v in vectors], reference_dq=[v[1] for v in vectors])
        result = {'completed': True, 'hardware_access': False}
    else:
        try:
            result = live(args, config, samples, events)
        except Exception as exc:
            result = {'completed': False, 'error_type': type(exc).__name__}
        samples.save(args.output)
    atomic_json(args.output/'events.json', events)
    result['recorded_commands'] = samples.count
    result['rl_shutdown_qualified'] = False
    if samples.count:
        for field, column in [('state_age_s', 5), ('reference_compute_s', 9), ('schedule_lateness_s', 10)]:
            result[field] = percentiles(samples.timing[:samples.count,column].tolist())
        result['max_abs_position_error_rad'] = np.max(np.abs(samples.vectors[:samples.count,5]),axis=0).tolist()
        result['max_abs_measured_load_Nm'] = np.max(np.abs(samples.vectors[:samples.count,3]),axis=0).tolist()
    atomic_json(args.output/'result.json', result)
    atomic_json(args.output/'COMPLETE.json', {'status': 'passed' if result.get('completed') or result.get('inspection_passed') else 'failed',
                'artifacts': {p.name:sha256(p) for p in args.output.iterdir() if p.is_file() and p.name != 'COMPLETE.json'}})
    print(json.dumps(result))
    return 0 if result.get('completed') or result.get('inspection_passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())

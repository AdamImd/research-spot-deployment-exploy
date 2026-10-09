import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import numpy as np
import pytest

from spot_deploy.contracts import ContractError, Envelope, State, load

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('crouch', ROOT/'tools/joint_api_crouch.py')
crouch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(crouch)


def config():
    return crouch.settings(ROOT/'configs/joint-api-crouch.json')


def test_small_crouch_is_one_fifth_original():
    small=crouch.settings(ROOT/'configs/joint-api-crouch-small.json')
    for t in np.linspace(0,10,201):
        original=crouch.reference(np.zeros(19),t,config())
        reduced=crouch.reference(np.zeros(19),t,small)
        np.testing.assert_allclose(reduced,np.asarray(original)/5,atol=1e-15)


@pytest.mark.parametrize('change', [{'severity':0},{'severity':2},{'severity':3},
                                  {'code':10},{'kind':'service_fault_state'},{'name':'other'}])
def test_only_exact_explicit_info_exception(change):
    fault={'kind':'system_fault_state','name':'payload.fault','code':9,'severity':1}
    assert crouch.accepted_faults(0,[],False)
    assert crouch.accepted_faults(1,[fault],True)
    assert not crouch.accepted_faults(1,[fault],False)
    assert not crouch.accepted_faults(1,[fault|change],True)
    assert not crouch.accepted_faults(2,[fault],True)
    assert not crouch.accepted_faults(0,[fault],True)


def fixture():
    import json
    m = json.loads((ROOT/'policies/relic-exploy-standing/manifest.json').read_text())
    q = [.1, 1., -1.5]*4 + m['arm_stowed_positions']
    state = State(robot_time_s=100, received_monotonic_s=10, positions=q,
                  velocities=[0]*19, loads=[0]*19, odom_quaternion_wxyz=[1,0,0,0],
                  linear_velocity_odom=[0]*3, angular_velocity_odom=[0]*3,
                  last_command_key=0, last_command_received_robot_s=0)
    return state, load(ROOT/'configs/spot-manufacturer-standing-envelope.json', Envelope)


def test_trajectory_holds_returns_and_only_moves_named_pitch_joints():
    q = np.arange(19)/10
    for t in [0, 1, 2, 9, 10, 11]:
        target, derivative = crouch.reference(q,t,config())
        np.testing.assert_array_equal(target,q)
        np.testing.assert_array_equal(derivative,np.zeros(19))
    expected = np.zeros(19)
    expected[[1,4,7,10]]=np.radians(5)
    expected[[2,5,8,11]]=np.radians(-10)
    for t in [5,5.5,6]:
        np.testing.assert_allclose(crouch.reference(q,t,config())[0]-q,expected,atol=1e-15)


@pytest.mark.parametrize('t', [2,5,6,9])
def test_transition_velocity_is_continuous_and_zero(t):
    assert abs(crouch.profile(t)[1]) < 1e-10
    assert abs(crouch.profile(t-1e-6)[0]-crouch.profile(t+1e-6)[0]) < 1e-10
    assert abs(crouch.profile(t-1e-6)[1]-crouch.profile(t+1e-6)[1]) < 1e-10


def test_torque_and_position_error_recording_with_bounded_buffer(tmp_path):
    state,_=fixture()
    state=state.model_copy(update={'velocities':[.3]*19})
    samples=crouch.Samples(capacity=1)
    samples.add([0]*11,state,np.asarray(state.positions)+.1,np.zeros(19),np.ones(19)*4,
                np.ones(19)*60,np.ones(19)*1.5)
    samples.save(tmp_path)
    with np.load(tmp_path/'samples.npz') as data:
        np.testing.assert_allclose(data['position_error'],.1)
        np.testing.assert_allclose(data['p_term'],6)
        np.testing.assert_allclose(data['d_term'],-.45)
        np.testing.assert_allclose(data['estimated_total_torque'],9.55)
    with pytest.raises(ContractError,match='buffer full'):
        samples.add([0]*11,state,np.asarray(state.positions),np.zeros(19),np.zeros(19),
                    np.ones(19),np.ones(19))


def test_shared_limits_and_wire_commands_still_apply():
    from spot_deploy.sdk_control import command_proto
    state,envelope=fixture()
    guard=crouch.ReferenceGuard(state.positions,config(),envelope,np.zeros(19))
    target,derivative=crouch.reference(state.positions,5,config())
    command=guard.command(target,state,10,100,10,0,np.zeros(19))
    wire=command_proto(command,guard.manifest,envelope)
    np.testing.assert_allclose(wire.joint_command.position,target)
    np.testing.assert_allclose(wire.joint_command.velocity,np.zeros(19))
    np.testing.assert_allclose(wire.joint_command.gains.k_q_p,config()['kp'])
    invalid=target.copy()
    invalid[0]=1
    with pytest.raises(ContractError,match='position limit'):
        guard.command(invalid,state,10.005,100.005,10.005,0,np.zeros(19))


@pytest.mark.parametrize('power', [0,1,2,3,4])
def test_exact_motor_off_confirmation(power):
    from bosdyn.api.robot_state_pb2 import PowerState
    reader=SimpleNamespace(config=SimpleNamespace(rpc_timeout_s=1),
                           state_client=SimpleNamespace(get_robot_state=lambda **_:SimpleNamespace(
                               power_state=SimpleNamespace(motor_power_state=power))))
    if power==PowerState.STATE_OFF:
        crouch.require_off(reader)
    else:
        with pytest.raises(ContractError):
            crouch.require_off(reader)


def test_plan_never_constructs_reader(monkeypatch,tmp_path):
    def forbidden(*args,**kwargs):
        raise AssertionError('plan contacted robot')
    monkeypatch.setattr(crouch,'ReadOnlySpot',forbidden)
    monkeypatch.setattr('sys.argv',['joint_api_crouch.py','plan','--output',str(tmp_path/'plan')])
    assert crouch.main()==0
    assert (tmp_path/'plan/trajectory.npz').is_file()


def test_execute_requires_explicit_flag_before_creating_output(monkeypatch,tmp_path):
    monkeypatch.setattr('sys.argv',['joint_api_crouch.py','execute','--robot','robot.json',
                                  '--envelope','limits.json','--binding','binding.json',
                                  '--output',str(tmp_path/'run')])
    with pytest.raises(SystemExit):
        crouch.main()
    assert not (tmp_path/'run').exists()


@pytest.mark.parametrize('activation_fails', [False, True])
def test_mocked_stream_lifecycle_and_shutdown(monkeypatch,tmp_path,activation_fails):
    """Fake perfect tracking validates plumbing, not physical dynamics or real-time timing."""
    import json
    import spot_deploy.sdk_control as sdk_control
    initial,envelope=fixture()
    # Relax timing only for OS-independent unit testing; physical limits/config are unchanged.
    envelope=envelope.model_copy(update={'max_state_gap_s':.2,'max_command_gap_s':.2,
        'max_command_ack_s':.2,'max_state_age_s':.2,'max_inference_s':.1})
    cfg=SimpleNamespace(estop_authority='tablet',hardware_estop=None,rpc_timeout_s=1)
    robot=SimpleNamespace(config=cfg)
    q=list(initial.positions)
    key=[0]

    def sample(*_):
        now=time.monotonic()
        return initial.model_copy(update={'robot_time_s':now,'received_monotonic_s':now,
            'positions':list(q),'last_command_key':key[0], 'last_command_received_robot_s':now})

    known_fault={'kind':'system_fault_state','name':'payload.fault','code':9,'severity':1}
    reader=SimpleNamespace(config=cfg,robot=robot,connect=lambda:None,
        snapshot=lambda:{'active_fault_count':1,'fault_details':[known_fault]},
        read_body_height=lambda:{'height_m':.515},start_stream=lambda _:sample(),
        mailbox=SimpleNamespace(get=sample),robot_now=time.monotonic,
        health=lambda:{'time':time.monotonic(),'estop_ready':True,'fault_count':1,
                       'fault_details':[known_fault],'battery_percent':96},
        stream_statistics=lambda:{'mock':True},close=lambda:None)
    calls=[]

    class FakeControl:
        def __init__(self,*_):
            self.lease=None
            self.thread=None

        def acquire(self):
            self.lease=object()
            calls.append('acquire')

        def heartbeat(self):
            pass

        def native_stand(self,_):
            calls.append('native_stand')

        def start(self,commands,_):
            def consume():
                for command in commands:
                    q[:]=command.joint_command.position
                    key[0]=command.joint_command.user_command_key
            self.thread=threading.Thread(target=consume)
            self.thread.start()

        def activate(self):
            calls.append('activate')
            if activation_fails:
                raise RuntimeError('synthetic activation failure')

        def active(self):
            return True

        def check_stream(self):
            pass

        def close(self):
            if self.thread:
                self.thread.join(2)
                assert not self.thread.is_alive()
            calls.append('safe_off_and_return')

    monkeypatch.setattr(crouch,'ReadOnlySpot',lambda _:reader)
    monkeypatch.setattr(crouch,'wired_route',lambda _: {})
    monkeypatch.setattr(crouch,'load',lambda _,kind:envelope if kind is Envelope else cfg)
    monkeypatch.setattr(crouch,'validate_snapshot',lambda *_,**__: {'mock_check':True,'no_active_faults':False})
    monkeypatch.setattr(crouch,'require_off',lambda _:calls.append('confirm_off'))
    monkeypatch.setattr(sdk_control,'ControlSpot',FakeControl)
    binding=tmp_path/'binding.json'
    binding.write_text(json.dumps({'robot_model_sha256':'0'*64,'payload_config_sha256':'0'*64}))
    args=SimpleNamespace(robot=Path('robot'),envelope=Path('limits'),binding=binding,
                         mode='execute',operator='Adam',safety_operator='Minghao',output=tmp_path,
                         allow_known_payload_info=True)
    samples=crouch.Samples()
    result=crouch.live(args,config(),samples,[])
    assert result['completed'] is not activation_fails, result
    assert result['motors_off_confirmed'] and result['lease_returned']
    assert result['preflight_original']['no_active_faults'] is False
    assert result['preflight']['no_unaccepted_faults'] is True
    assert calls[-2:]==['safe_off_and_return','confirm_off']
    assert samples.count > 0
    if not activation_fails:
        knee=samples.vectors[:samples.count,0,2]
        assert knee.min()==pytest.approx(initial.positions[2]-np.radians(10),abs=1e-5)
        assert knee[-1]==pytest.approx(initial.positions[2],abs=1e-5)

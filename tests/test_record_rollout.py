import gzip
import importlib.util
import json
from pathlib import Path
import threading
import time

import pytest

from spot_deploy.contracts import sha256
from spot_deploy.records import atomic_json

SPEC = importlib.util.spec_from_file_location(
    "record_rollout", Path(__file__).resolve().parents[1] / "tools/record_rollout.py")
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


def state(t=100.):
    return dict(robot_time_s=t, received_monotonic_s=t + 1, positions=[.1]*19,
                velocities=[.2]*19, loads=[.3]*19, odom_quaternion_wxyz=[1, 0, 0, 0],
                linear_velocity_odom=[0]*3, angular_velocity_odom=[0]*3,
                last_command_key=3, last_command_received_robot_s=t-.01)


def complete(source, status="passed"):
    atomic_json(source / "COMPLETE.json", {"status": status, "artifacts": {
        "events.jsonl": sha256(source / "events.jsonl")}})


def rows(output):
    with gzip.open(output / "rollout.jsonl.gz", "rt") as stream:
        return [json.loads(line) for line in stream]


def test_preserve_all_actions_commands_and_exact_policy_states(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    events = []
    for i in range(20):
        sample = state(100 + i*.005)
        if i % 4 == 0:
            events.append(dict(event="policy", state=sample, observations=[9]*84,
                               raw_actions=[i]*12, targets=[i+.1]*19, inference_s=.001))
        events.append(dict(event="command", state=sample, key=i, positions=[i+.1]*19,
                           feedforward=[0]*19, end_robot_time_s=sample['robot_time_s']+.03))
    events.append(dict(event="route", password="not copied"))
    (source / "events.jsonl").write_text(''.join(json.dumps(e)+'\n' for e in events))
    complete(source)
    out = tmp_path / "compact"
    result = recorder.record(source, out)
    assert result['capture_complete']
    captured = rows(out)
    policies = [e for e in captured if e['event']=='policy']
    commands = [e for e in captured if e['event']=='command']
    assert len(policies)==5 and len(commands)==20
    assert [e['raw_actions'] for e in policies]==[[i]*12 for i in range(0,20,4)]
    assert policies[1]['state']==state(100.02)
    assert commands[-1]['state_last_command_key']==3
    assert not any(e['event']=='route' or 'observations' in e for e in captured)
    assert not any('state' in e for e in commands)


def test_state_rate_and_first_sample_are_retained():
    projector = recorder.Projector(50)
    out = []
    for i in range(200):
        out += projector.project(dict(event="state", state=state(100+i*.005)), i+1)
    assert len(out)==50
    assert out[0]['state']['robot_time_s']==100
    assert len(recorder.Projector(0).project(dict(event="state", state=state()),1))==1


def test_partial_live_writes_are_waited_for_and_final_source_failure_is_distinct(tmp_path):
    source, out = tmp_path/'source', tmp_path/'out'
    source.mkdir()
    event = json.dumps(dict(event="policy", state=state(), raw_actions=[1]*12,
                            targets=[2]*19))+'\n'
    (source/'events.jsonl').write_text(event[:50])

    def writer():
        time.sleep(.08)
        with (source/'events.jsonl').open('a') as stream:
            stream.write(event[50:])
        complete(source, status='failed')

    thread = threading.Thread(target=writer)
    thread.start()
    result = recorder.record(source, out, follow=True, duration=2)
    thread.join()
    assert result['capture_complete'] and result['source_status']=='failed'
    assert len(rows(out))==1
    assert not result['hardware_qualified']


@pytest.mark.parametrize('contents', [b'{"event":', b'bad json\n', b'x'*256001+b'\n'])
def test_bad_or_partial_completed_source_fails_without_discarding_silently(tmp_path, contents):
    source = tmp_path/'source'
    source.mkdir()
    (source/'events.jsonl').write_bytes(contents)
    complete(source)
    result = recorder.record(source,tmp_path/'out')
    assert result['status']=='failed' and not result['capture_complete']


def test_hash_mismatch_and_overwrite_are_rejected(tmp_path):
    source, out = tmp_path/'source', tmp_path/'out'
    source.mkdir()
    (source/'events.jsonl').write_text('{}\n')
    complete(source)
    (source/'events.jsonl').write_text('{"event":"different"}\n')
    assert recorder.record(source,out)['status']=='failed'
    with pytest.raises(FileExistsError):
        recorder.record(source,out)
    with pytest.raises(ValueError):
        recorder.record(source,source/'compact')


def test_live_duration_is_explicitly_partial(tmp_path):
    source, out = tmp_path/'source', tmp_path/'out'
    source.mkdir()
    with pytest.raises(ValueError, match='unfinished'):
        recorder.record(source,out)
    result = recorder.record(source,out,follow=True,duration=.05)
    assert result['status']=='duration_limited' and not result['capture_complete']
    assert rows(out)==[]


def test_shutdown_keeps_valid_gzip(tmp_path):
    source, out = tmp_path/'source', tmp_path/'out'
    source.mkdir()
    stop=threading.Event()
    stop.set()
    result=recorder.record(source,out,follow=True,stop=stop)
    assert result['status']=='interrupted' and rows(out)==[]


@pytest.mark.parametrize('hz', [-1, float('nan'), float('inf'), 1001])
def test_invalid_rates(hz):
    with pytest.raises(ValueError):
        recorder.Projector(hz)

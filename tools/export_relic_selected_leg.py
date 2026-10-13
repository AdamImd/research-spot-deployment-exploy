"""Compose explicit front-left commands with the pinned Exploy observation graph.

No robot access, training, Torch or simulator startup. This graph is a preparation
artifact until native parity and a guarded dynamics rehearsal pass.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import helper, numpy_helper, utils
import onnxruntime as ort

from spot_deploy.contracts import JOINTS, sha256
from spot_deploy.exploy_policy import INPUT_SIZES, META_KEY, TARGET
from spot_deploy.records import atomic_json, source_identity
from spot_deploy.relic_policy import ACTION_IDS, CHECKPOINT_SHA256, DEFAULT_Q

BASE_SHA = '1940df208bf9e30e6134eeb71d1c4e67d3a0d66778726507d4886c62eae662e8'


def export(base, actor, output):
    if sha256(base) != BASE_SHA or sha256(actor) != CHECKPOINT_SHA256:
        raise ValueError('Only the pinned Exploy graph and unchanged released actor are allowed')
    output.mkdir(parents=True, exist_ok=False)
    original = onnx.load(base)
    model = utils.Extractor(original).extract_model(list(INPUT_SIZES), ['obs'])
    for node in model.graph.node:
        for i, value in enumerate(node.output):
            if value == 'obs':
                node.output[i] = 'obs.base'
    del model.graph.output[:]
    # Prefix actor internals to avoid collisions with native observation nodes.
    released = onnx.version_converter.convert_version(onnx.load(actor), 20)
    names = {v.name for v in released.graph.initializer}
    for node in released.graph.node:
        names.update(node.output)
    rename = {name: 'actor.'+name for name in names}
    rename['obs'], rename['actions'] = 'obs', 'actions'
    for node in released.graph.node:
        node.name = 'actor.'+node.name
        for field in (node.input, node.output):
            for i, value in enumerate(field):
                field[i] = rename.get(value, value)
    for value in released.graph.initializer:
        value.name = rename[value.name]

    nodes, constants = [], []
    def constant(name, value):
        constants.append(numpy_helper.from_array(np.asarray(value), name))
        return name
    def slice_(source, begin, end, dest):
        nodes.append(helper.make_node('Slice', [source,
            constant(dest+'.start', np.array([begin], dtype=np.int64)),
            constant(dest+'.end', np.array([end], dtype=np.int64)),
            constant(dest+'.axis', np.array([1], dtype=np.int64))], [dest]))
    slice_('obs.base', 0, 19, 'obs.head')
    slice_('obs.base', 31, 84, 'obs.tail')
    nodes.append(helper.make_node('Concat', ['obs.head', 'leg_command', 'obs.tail'], ['obs'], axis=1))
    nodes.extend(released.graph.node)
    # Actor outputs remain joint-type-major; all public targets use SDK order.
    permutation = [list(ACTION_IDS).index(i) for i in range(12)]
    nodes.append(helper.make_node('Gather', ['actions', constant('sdk.permutation',
        np.asarray(permutation, dtype=np.int64))], ['actions.sdk'], axis=1))
    nodes.append(helper.make_node('Mul', ['actions.sdk', constant('action.scale',
        np.array(.2, dtype=np.float32))], ['action.scaled']))
    nodes.append(helper.make_node('Add', ['action.scaled', constant('leg.defaults',
        DEFAULT_Q[:12][None].astype(np.float32))], ['leg.targets']))
    slice_('leg.targets', 0, 3, 'fl.actor')
    slice_('leg.targets', 3, 12, 'other.actor')
    slice_('leg_command', 0, 3, 'fl.command')
    nodes.append(helper.make_node('Abs', ['fl.command'], ['fl.abs']))
    nodes.append(helper.make_node('ReduceSum', ['fl.abs', constant('fl.axis',
        np.array([1], dtype=np.int64))], ['fl.sum'], keepdims=1))
    nodes.append(helper.make_node('Greater', ['fl.sum', constant('fl.zero',
        np.array(0, dtype=np.float32))], ['fl.active']))
    nodes.append(helper.make_node('Where', ['fl.active', 'fl.command', 'fl.actor'], ['fl.target']))
    nodes.append(helper.make_node('Concat', ['fl.target', 'other.actor', 'arm_command'], [TARGET], axis=1))
    model.graph.node.extend(nodes)
    model.graph.initializer.extend(released.graph.initializer)
    model.graph.initializer.extend(constants)
    model.graph.input.append(helper.make_tensor_value_info('leg_command', onnx.TensorProto.FLOAT, [1, 12]))
    model.graph.output.extend(helper.make_tensor_value_info(n, onnx.TensorProto.FLOAT, [1, size])
                              for n, size in [('actions',12), ('obs',84), (TARGET,19)])
    metadata = json.loads(next(p.value for p in original.metadata_props if p.key == META_KEY))
    metadata.update(contract='relic-exploy-leg-v1', mode='front-left-selected-leg',
        joint_names=list(JOINTS), base_graph_sha256=BASE_SHA,
        leg_command_joint_names=list(JOINTS[:12]), selected_leg='fl',
        inactive_command='all zeros; other legs must always be zero')
    del model.metadata_props[:]
    prop = model.metadata_props.add()
    prop.key, prop.value = META_KEY, json.dumps(metadata, sort_keys=True)
    model.ir_version = 10
    onnx.checker.check_model(model, full_check=True)
    onnx.save(model, output/'policy.onnx')
    session = ort.InferenceSession(str(output/'policy.onnx'), providers=['CPUExecutionProvider'])
    baseline = ort.InferenceSession(str(base), providers=['CPUExecutionProvider'])
    raw = ort.InferenceSession(str(actor), providers=['CPUExecutionProvider'])
    rng = np.random.default_rng(101)
    errors = np.zeros(3)
    graph_order = json.loads(next(p.value for p in original.metadata_props if p.key == META_KEY))['joint_names']
    for i in range(64):
        values = {k: rng.normal(0,.1,(1,n)).astype(np.float32) for k,n in INPUT_SIZES.items()}
        values['base_quaternion'] = np.array([[1,0,0,0]],dtype=np.float32)
        values['joint_positions'] += DEFAULT_Q[[JOINTS.index(n) for n in graph_order]][None]
        values['leg_command'] = np.zeros((1,12),dtype=np.float32)
        if i % 2:
            values['leg_command'][0,:3] = [.1,.6,-1.1]
        expected_obs = baseline.run(['obs'], {k:v for k,v in values.items() if k!='leg_command'})[0]
        expected_obs[:,19:31] = values['leg_command']
        expected_action = raw.run(['actions'], {'obs':expected_obs})[0]
        expected_leg = DEFAULT_Q[:12][None]+.2*expected_action[:,permutation]
        if i % 2:
            expected_leg[:,:3] = values['leg_command'][:,:3]
        expected_target = np.concatenate([expected_leg,values['arm_command']],axis=1)
        actual = session.run(['obs','actions',TARGET], values)
        for j,(a,b,tol) in enumerate(zip(actual,[expected_obs,expected_action,expected_target],[1e-5,1e-4,3e-5])):
            np.testing.assert_allclose(a,b,rtol=0,atol=tol)
            errors[j] = max(errors[j],float(np.max(np.abs(a-b))))
    atomic_json(output/'graph-contract.json', metadata)
    atomic_json(output/'export.json', dict(policy_sha256=sha256(output/'policy.onnx'),
        base_graph_sha256=BASE_SHA, source_policy_sha256=CHECKPOINT_SHA256,
        source=source_identity(), seed=101, parity_samples=64,
        parity_max_abs_error=dict(zip(['obs','actions','targets'],errors.tolist())),
        weights_changed=False, hardware_access=False, physics_executed=False,
        hardware_qualified=False))
    print(json.dumps({'exported':str(output), 'parity_max_abs_error':errors.tolist()}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--actor', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    export(args.base, args.actor, args.output)

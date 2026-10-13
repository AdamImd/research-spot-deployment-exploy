"""Package a selected-leg review candidate offline; never authorize or run motion."""
import argparse
import json
from pathlib import Path
import shutil

from spot_deploy.contracts import artifact_path, sha256
from spot_deploy.exploy_policy import SCRIPT_SHA256
from spot_deploy.records import atomic_json
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.selected_leg_policy import BASE_GRAPH_SHA, SelectedLegManifest
from spot_deploy.simulation_demo import ThreeLegPlan


def prepare(base_manifest, graph, plan_path, output):
    base = load_manifest(base_manifest)
    if base.task != 'standing' or base.purpose != 'candidate' or base.adapter != 'relic-exploy':
        raise ValueError('requires an existing real standing Exploy candidate')
    base_graph = artifact_path(base_manifest, base.policy_file)
    if sha256(base_graph) != BASE_GRAPH_SHA:
        raise ValueError('base Exploy graph mismatch')
    composition = json.loads((graph/'export.json').read_text())
    if (sha256(graph/'policy.onnx') != composition['policy_sha256']
            or composition['base_graph_sha256'] != BASE_GRAPH_SHA):
        raise ValueError('graph composition record mismatch')
    plan = ThreeLegPlan.model_validate_json(plan_path.read_text())
    output.mkdir(parents=True, exist_ok=False)
    for name in [base.training_config_file, base.relic.preparation.model_file]:
        destination = output/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(artifact_path(base_manifest,name),destination)
    shutil.copy2(base_graph,output/'base.onnx')
    shutil.copy2(graph/'policy.onnx',output/'policy.onnx')
    configuration = dict(demo=plan.model_dump(), composition=composition,
        base_export=json.loads(artifact_path(base_manifest,base.exploy.export_record_file).read_text()),
        input_order='pinned base graph native names', target_order='SDK',
        entry_controller=('single native actor; unfiltered targets; native raw ACK history'
            if plan.target_slew_rad_s is None else
            f'single native actor; target slew{plan.target_slew_rad_s}rad/s; native raw ACK history; no catch-up'),
        return_blend='two-second quintic on selected-leg output; zero native leg command; raw ACK history unchanged')
    atomic_json(output/'configuration.json',configuration)
    record = dict(composition, configuration_sha256=sha256(output/'configuration.json'),
        source_script_sha256=SCRIPT_SHA256,
        source_script_role='Pinned original Exploy export provenance; composed actor is published ONNX')
    atomic_json(output/'export.json',record)
    data=base.model_dump()
    data.update(schema_version=4,adapter='relic-exploy-leg',task='three-leg-demo',
        demo=plan.model_dump(),policy_file='policy.onnx',policy_sha256=sha256(output/'policy.onnx'))
    data['exploy'].update(contract='relic-exploy-leg-v1',base_graph_file='base.onnx',
        base_graph_sha256=BASE_GRAPH_SHA,configuration_file='configuration.json',
        configuration_sha256=sha256(output/'configuration.json'),export_record_file='export.json',
        export_record_sha256=sha256(output/'export.json'))
    manifest=SelectedLegManifest.model_validate(data)
    atomic_json(output/'manifest.json',manifest.model_dump())
    policy=make_policy(output/'policy.onnx',manifest,output/'manifest.json')
    policy.warmup()
    assert not policy.previous_action.any()
    print(json.dumps(dict(candidate=str(output),sequence_duration_s=plan.duration_s,
                         physical_trial_authorized=False,hardware_qualified=False)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-manifest',type=Path,required=True)
    parser.add_argument('--graph',type=Path,required=True)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    prepare(args.base_manifest,args.graph,args.plan,args.output)

"""Analyze existing recordings only. No inference, simulation or robot IO."""
import hashlib
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import State
from spot_deploy.policy import rotation
from spot_deploy.static_support import StaticSupport, axis_rotation

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = Path(__file__).parent
MODEL = ROOT / 'local/relic-three-leg-angular2-hardware-20261012-001/bundle/policy/support.urdf'
MANIFEST = json.loads(MODEL.with_name('manifest.json').read_text())
SUPPORT = StaticSupport(MODEL, .025, .8)


def geometry(state):
    frames = {SUPPORT.root: (np.zeros(3), rotation(state))}
    for parent, child, idx, xyz, rot, axis in SUPPORT.joints:
        point, basis = frames[parent]
        point = point + basis @ xyz
        basis = basis @ rot
        if idx is not None:
            basis = basis @ axis_rotation(axis, state.positions[idx])
        frames[child] = point, basis
    mass = sum(m for m, _ in SUPPORT.inertials.values())
    com = sum(m * (frames[n][0] + frames[n][1] @ c)
              for n, (m, c) in SUPPORT.inertials.items()) / mass
    feet = np.array([frames[n][0] for n in SUPPORT.feet])
    tri = feet[[1, 2, 3], :2]
    bary = np.linalg.solve(np.vstack([tri.T, np.ones(3)]), np.r_[com[:2], 1.])
    edge, offset, inside = tri[1]-tri[0], com[:2]-tri[0], tri[2]-tri[0]
    def cross(a, b):
        return a[0]*b[1]-a[1]*b[0]
    margin = cross(edge, offset)*np.sign(cross(edge, inside))/np.linalg.norm(edge)
    return dict(model_mass_kg=mass, tripod_barycentric_fr_hl_hr=bary.tolist(),
                tripod_diagonal_margin_m=float(margin))


def analyze(relative):
    path = ROOT / relative / 'events.jsonl'
    events = [json.loads(line) for line in path.read_text().splitlines()]
    policies = [x for x in events if x['event'] == 'policy']
    begin = next(x['monotonic_s'] for x in events
                 if x['event'] == 'relic_phase' and x['phase'] == 'direct_handover')
    demos = [x for x in events if x['event'] == 'demo_prediction']
    assert len(demos) == len(policies)
    samples = []
    for requested in [4.98, 5.001, 5.1, 5.2, 5.3, 5.4, 5.48]:
        idx = min(range(len(policies)),
                  key=lambda i: abs(policies[i]['monotonic_s']-begin-requested))
        x, d = policies[idx], demos[idx]
        assert x['state'] == d['state']
        s = State.model_validate(x['state'])
        r = rotation(s)
        q, target = np.array(s.positions), np.array(x['targets'])
        proposed = np.array(d['proposed_targets'])
        pd = (np.array(MANIFEST['gains']['kp'])*(target-q)
              - np.array(MANIFEST['gains']['kd'])*np.array(s.velocities))
        sample = dict(elapsed_s=x['monotonic_s']-begin, phase=d['phase'],
                      pitch_deg=float(np.degrees(np.arcsin(np.clip(-r[2, 0], -1, 1)))),
                      angular_speed_rad_s=float(np.linalg.norm(s.angular_velocity_odom)),
                      fl_actual_rad=q[:3].tolist(), fl_target_rad=target[:3].tolist(),
                      fl_proposed_rad=proposed[:3].tolist(),
                      fl_reported_load_Nm=s.loads[:3], fl_estimated_pd_Nm=pd[:3].tolist(),
                      support_leg_target_filter_gap_max_rad=float(abs(target[3:12]-proposed[3:12]).max()),
                      max_abs_raw_action=float(abs(np.array(x['raw_actions'])).max()),
                      ik_error_m=d.get('ik_tracking_error_m'), **geometry(s))
        if d.get('ik_target_body_m'):
            sample['ik_target_body_m'] = d['ik_target_body_m']
        samples.append(sample)
    return dict(recording=relative, events_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                samples=samples)


if __name__ == '__main__':
    result = dict(model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
                  hardware=analyze('runs/relic-three-leg-angular2-hardware-20261012-001'),
                  prior_simulation=analyze('runs/relic-three-leg-entry-20261012-005/physical-stance'))
    (OUTPUT/'analysis.json').write_text(json.dumps(result, indent=2)+'\n')
    for s in result['hardware']['samples']:
        print(f"{s['elapsed_s']:.3f}s pitch={s['pitch_deg']:.2f}deg "
              f"FL knee PD={s['fl_estimated_pd_Nm'][2]:.2f}Nm "
              f"support filter gap={s['support_leg_target_filter_gap_max_rad']:.3f}rad "
              f"tripod margin={s['tripod_diagonal_margin_m']:.3f}m")

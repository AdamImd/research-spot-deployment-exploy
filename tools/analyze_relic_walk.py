"""Measure stepping versus body translation and plot the exact walking guard stop."""

import argparse
import json
from pathlib import Path

import numpy as np

from spot_deploy.contracts import JOINTS, sha256
from spot_deploy.records import atomic_json, verify_run
from spot_deploy.walk_diagnostics import summarize


def plots(report, data, events, origin, failure, config, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = data['t']
    progress = [e for e in events if e.get('event') == 'walk_progress']
    # Log clock origin is explicit in the policy's state and local timestamp.
    p = next(e for e in events if e.get('event') == 'policy')
    clock_origin = p['monotonic_s']-(p['state']['body_pose_robot_time_s']-origin['robot_time_s'])
    figure, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True, layout='constrained')
    axes[0, 0].plot(t, data['body_forward'], label='Body odometry')
    axes[0, 0].plot(t, data['foot_forward'].mean(axis=1), label='Mean foot position')
    axes[0, 0].set_ylabel('Forward displacement (m)')
    axes[0, 0].legend()
    pt = [e['monotonic_s']-clock_origin for e in progress]
    axes[0, 1].step(pt, [e['requested_forward_m_s'] for e in progress], where='post', label='Requested vx')
    axes[0, 1].plot(pt, [e['speed_m_s'] for e in progress], label='Measured planar speed')
    axes[0, 1].set_ylabel('Velocity (m/s)')
    axes[0, 1].legend()
    for i, leg in enumerate(('fl', 'fr', 'hl', 'hr')):
        axes[1, 0].plot(t, data['foot_forward'][:, i], label=leg)
        axes[1, 1].plot(t, data['clearance'][:, i], label=leg)
    axes[1, 0].set_ylabel('Foot forward displacement (m)')
    axes[1, 1].set_ylabel('Foot clearance geometry (m)')
    axes[1, 1].axhline(.015, color='gray', linestyle='--', label='15 mm clearance')
    for ax in axes.flat:
        ax.grid(alpha=.25)
        if report['first_nonzero_velocity_request_s'] is not None:
            ax.axvline(report['first_nonzero_velocity_request_s'], color='gray', linestyle=':')
    axes[1, 0].legend(ncol=4)
    axes[1, 1].legend(ncol=3, loc='lower left', bbox_to_anchor=(0, 1.02), fontsize=8)
    for ax in axes[1]:
        ax.set_xlabel('Recorded simulation time (s)')
    figure.suptitle(f"Walking diagnostics · {config['backend']}\n"
                   'Foot geometry does not measure loaded contact or independently verify odometry')
    figure.savefig(output/'walking-overview.png', dpi=160)
    figure.savefig(output/'walking-overview.pdf')
    plt.close(figure)
    j = JOINTS.index(report['selected_joint'])
    figure, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True, layout='constrained')
    axes[0, 0].plot(t, data['q'][:, j], label='Measured joint')
    axes[0, 0].step(t, data['target'][:, j], where='post', label='Issued position target')
    axes[0, 0].set_ylabel('Joint angle (rad)')
    axes[0, 1].plot(t, data['dq'][:, j], label='Simulator velocity')
    axes[0, 1].plot((t[1:]+t[:-1])/2, np.diff(data['q'][:, j])/np.diff(t), label='Position-derived velocity')
    limit = report['selected_joint_speed_limit_rad_s']
    for value in (-limit, limit):
        axes[0, 1].axhline(value, color='red', linestyle='--')
    axes[0, 1].set_ylabel('Joint speed (rad/s)')
    axes[1, 0].plot(t, data['torque'][:, j], label='Requested PD torque')
    axes[1, 0].plot(t, data['applied'][:, j], label='Preceding applied torque')
    axes[1, 0].set_ylabel('Joint torque (Nm)')
    leg = ('fl', 'fr', 'hl', 'hr').index(report['selected_joint'].split('_')[0])
    axes[1, 1].plot(t, data['clearance'][:, leg])
    axes[1, 1].axhline(0, color='gray')
    axes[1, 1].set_ylabel('Foot clearance geometry (m)')
    end = failure['simulation_time_s'] if failure else t[-1]
    for ax in axes.flat:
        ax.set_xlim(max(0, end-.5), end+.015)
        ax.grid(alpha=.25)
        if failure:
            ax.axvline(end, color='red', linestyle=':', label='Guard stop')
    if failure:
        state = failure['state']
        axes[0, 0].scatter([end], [state['positions'][j]], color='red', marker='x')
        axes[0, 1].scatter([end], [state['velocities'][j]], color='red', marker='x', label='Exact rejected state')
        rejected_speed = report['selected_joint_rejected_step_position_derived_speed_rad_s']
        if rejected_speed is not None:
            axes[0, 1].scatter([(end+t[-1])/2], [rejected_speed], color='orange', marker='+',
                               label='Rejected-step position derivative')
        for violation in report['first_failure']['tracking_violations']:
            if violation['joint'] == report['selected_joint']:
                axes[0, 0].scatter([end], [violation['target_rad']], color='orange', marker='x',
                                   label=report['first_failure']['target_source'])
    for ax in axes.flat[:3]:
        ax.legend(fontsize=8)
    for ax in axes[1]:
        ax.set_xlabel('Recorded simulation time (s)')
    figure.suptitle(f"{report['selected_joint']} onset · {config['backend']}\n"
                   'Rejected state marked separately; no targets or foot states fabricated after stop')
    figure.savefig(output/'walking-onset.png', dpi=160)
    figure.savefig(output/'walking-onset.pdf')
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not verify_run(args.run, status='completed'):
        parser.error('walking source integrity verification failed')
    config = json.loads((args.run/'configuration.json').read_text())
    rows = [json.loads(line) for line in (args.run/'rollout.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (args.run/'events.jsonl').read_text().splitlines()]
    report, data, origin, failure = summarize(rows, config, events)
    args.output.mkdir(parents=True, exist_ok=False)
    atomic_json(args.output/'diagnostics.json', report)
    plots(report, data, events, origin, failure, config, args.output)
    atomic_json(args.output/'COMPLETE.json', dict(status='completed', hardware_access=False,
        source_complete_sha256=sha256(args.run/'COMPLETE.json'),
        tool_sha256=sha256(Path(__file__)),
        analysis_module_sha256=sha256(Path(__file__).resolve().parents[1]
                                    /'src/spot_deploy/walk_diagnostics.py'),
        artifacts={p.name: sha256(p) for p in args.output.iterdir() if p.is_file()}))
    print(json.dumps(report, allow_nan=False))


if __name__ == '__main__':
    main()

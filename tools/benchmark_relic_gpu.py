"""Offline CUDA/CPU audit and paced command-production benchmark; no robot client.

Replays recorded states through the actual rollout, async recorder and command
protobuf encoder. Ideal logical state clocks/ACKs are explicit; wall deadlines
are measured independently and misses are retained without catch-up bursts.
"""

import argparse
from collections import Counter
import gc
import json
from pathlib import Path
import time

import numpy as np

from spot_deploy.contracts import Envelope, State, check_artifacts, load, sha256
from spot_deploy.records import RunRecord, atomic_json, verify_run
from spot_deploy.relic_contract import load_manifest, make_policy
from spot_deploy.relic_policy import ReLICPolicy
from spot_deploy.relic_rollout import ReLICRollout
from spot_deploy.safety import Schedule
from spot_deploy.sdk_control import command_proto


def stats(values, budget=None):
    values = np.asarray(values, dtype=float)
    if not values.size:
        return None
    result = dict(samples=int(values.size), p50_s=float(np.quantile(values, .5)),
                  p95_s=float(np.quantile(values, .95)), p99_s=float(np.quantile(values, .99)),
                  p999_s=float(np.quantile(values, .999)), max_s=float(values.max()))
    if budget is not None:
        result.update(budget_s=budget, over_budget=int(np.sum(values > budget)))
    return result


def audit(policy, observations, checkpoint, device, directory):
    # Timing includes numpy input -> device -> returned numpy output. The first
    # call is retained separately; discarded calls do not touch policy history.
    cold_start = time.monotonic()
    policy.runner.session.run(["actions"], {"obs": observations[0:1]})
    first_inference = time.monotonic() - cold_start
    warm = []
    for i in range(100):
        start = time.monotonic()
        policy.runner.session.run(["actions"], {"obs": observations[i % len(observations)][None]})
        warm.append(time.monotonic() - start)
    reference = ReLICPolicy(checkpoint)
    largest = 0.
    for obs in observations:
        expected = reference.session.run(["actions"], {"obs": obs[None]})[0]
        actual = policy.runner.session.run(["actions"], {"obs": obs[None]})[0]
        largest = max(largest, float(np.max(np.abs(expected - actual))))
    if largest > 1e-4:
        raise AssertionError(f"CPU/CUDA raw action parity failed: {largest}")
    # Separate session so profiling overhead is absent from the timed rollout.
    profiled = ReLICPolicy(checkpoint, device=device, profile_prefix=directory / "ort-profile")
    profiled.session.run(["actions"], {"obs": observations[0:1]})
    profile = Path(profiled.session.end_profiling())
    nodes = [e for e in json.loads(profile.read_text())
             if e.get("cat") == "Node" and e.get("args", {}).get("provider")]
    compute = [e for e in nodes if not e["args"].get("op_name", "").startswith("Memcpy")]
    providers = Counter(e["args"]["provider"] for e in compute)
    expected_provider = "CUDAExecutionProvider" if device == "cuda" else "CPUExecutionProvider"
    if not providers or set(providers) != {expected_provider}:
        raise AssertionError(f"actual compute node placement differs: {providers}")
    result = dict(first_inference_s=first_inference, discarded_pre_activation_calls=101,
                  discarded_parity_calls=len(observations), warm_inference=stats(warm, .005),
                  max_raw_action_error=largest, max_target_error_rad=.2 * largest,
                  parity_samples=len(observations), compute_node_providers=dict(providers),
                  compute_ops=dict(Counter(e["args"].get("op_name") for e in compute)),
                  profile_file=profile.name, profile_sha256=sha256(profile),
                  providers=policy.runner.session.get_providers(),
                  provider_options=policy.runner.session.get_provider_options(),
                  history_modified=False)
    atomic_json(directory / "provider-audit.json", result)
    return result


def run(args):
    if not verify_run(args.source_run, status="completed") or not verify_run(args.capture):
        raise ValueError("source or capture integrity check failed")
    manifest = load_manifest(args.manifest)
    envelope = load(args.envelope, Envelope)
    if (envelope.scope != "simulation" or manifest.relic.preparation.kind != "direct"
            or not 0 < args.duration <= 60):
        raise ValueError("benchmark requires direct startup and simulation limits, at most 60 seconds")
    states, observations, frames = [], [], []
    for line in (args.source_run / "events.jsonl").open():
        event = json.loads(line)
        if event["event"] == "command":
            states.append((event["simulation_time_s"], event["state"]))
        elif event["event"] == "policy":
            observations.append(event["observations"])
            frames.append(event)
    np.testing.assert_allclose([x[0] for x in states], np.arange(len(states)) * .005,
                               rtol=0, atol=1e-8)
    if states[-1][0] + .005 < args.duration:
        raise ValueError("source trace is shorter than benchmark")
    observations = np.asarray(observations, dtype=np.float32)
    checkpoint = check_artifacts(args.manifest, manifest)
    record = RunRecord(args.output, "offline-gpu-timing", [args.manifest, args.envelope,
                       args.source_run / "COMPLETE.json", args.capture / "COMPLETE.json"])
    began = time.monotonic()
    policy = make_policy(checkpoint, manifest, args.manifest, device=args.device)
    setup_s = time.monotonic() - began
    height = json.loads((args.capture / "initial-height.json").read_text())
    if getattr(manifest, "adapter", None) == "relic-exploy":
        if not args.reference_checkpoint:
            raise ValueError("Exploy timing requires --reference-checkpoint for independent parity")
        policy.initialize_height(height)
        provider_audit = audit_exploy(policy, frames, args.reference_checkpoint, args.output,
                                      args.manifest, height)
    else:
        provider_audit = audit(policy, observations, checkpoint, args.device, args.output)
    core = ReLICRollout(policy, envelope, record)
    initial = State.model_validate(states[0][1])
    initial.received_monotonic_s, initial.robot_time_s = 9.995, 999.995
    initial.last_command_key = 0
    core.initialize(initial, height, 9.995, 999.995)
    began = time.monotonic()
    command = core.command(initial, 9.995, 999.995)
    hold_s = time.monotonic() - began
    command_proto(command, manifest, envelope).SerializeToString()
    core.activate(10.)
    np.testing.assert_array_equal(core.previous, 0)
    config = dict(device=args.device, duration_s=args.duration, source_run=str(args.source_run),
                  capture=str(args.capture), hardware_access=False, sockets_created=False,
                  physics=False, source_state_hz=200, policy_hz=50, command_hz=200,
                  state_clocks="ideal logical clocks, wall deadlines measured separately",
                  acknowledgements="ideal next-command receipt, no network transport",
                  logging="actual RunRecord async writer including synchronous JSON serialization",
                  encoding="actual JointControlStreamRequest construction and SerializeToString",
                  pre_activation_inferences="cold + 100 discarded + recorded-input parity; history remains zero",
                  session_setup_s=setup_s, supported_hold_core_s=hold_s,
                  manifest=manifest.model_dump(), envelope=envelope.model_dump())
    atomic_json(args.output / "configuration.json", config)
    events, samples = [], []
    def gc_event(phase, info):
        events.append(dict(phase=phase, generation=info["generation"], wall_s=time.monotonic()))
    gc.collect()
    gc.callbacks.append(gc_event)
    epoch = time.monotonic() + .02
    schedule = Schedule(200, start=epoch)
    reason, failure, last_finish = "completed", None, None
    try:
        while schedule.next < epoch + args.duration - 1e-7:
            release = schedule.next
            time.sleep(max(0., release - time.monotonic()))
            began = time.monotonic()
            logical_t = release - epoch
            source_index = min(round(logical_t / .005), len(states) - 1)
            state = State.model_validate(states[source_index][1])
            pose_offset = (state.body_pose_robot_time_s - state.robot_time_s
                           if state.body_pose_robot_time_s is not None else None)
            state.robot_time_s, state.received_monotonic_s = 1000 + logical_t, 10 + logical_t
            if pose_offset is not None:
                state.body_pose_robot_time_s = state.robot_time_s + pose_offset
            state.last_command_key = command.key
            state.last_command_received_robot_s = state.robot_time_s
            state_at = time.monotonic()
            previous_count = core.policy_samples
            command = core.command(state, 10 + logical_t, 1000 + logical_t)
            computed = time.monotonic()
            record.event("command", key=command.key, positions=list(command.positions),
                         feedforward=list(command.feedforward), phase=core.phase,
                         end_robot_time_s=command.end_robot_time_s, monotonic_s=computed,
                         state=state.model_dump())
            payload = command_proto(command, manifest, envelope).SerializeToString()
            finished = time.monotonic()
            samples.append(dict(release_wall_s=release, elapsed_s=logical_t,
                                begin_wall_s=began, finish_wall_s=finished,
                                release_lateness_s=began-release,
                                state_decode_s=state_at-began, core_s=computed-state_at,
                                log_encode_s=finished-computed, production_s=finished-began,
                                deadline_response_s=finished-release,
                                gap_s=None if last_finish is None else finished-last_finish,
                                new_policy=core.policy_samples != previous_count,
                                inference_s=core.inference_s if core.policy_samples != previous_count else None,
                                source_index=source_index, bytes=len(payload), key=command.key))
            last_finish = finished
            schedule.advance(finished)
    except Exception as exc:
        reason, failure = "guard_or_runtime_stop", f"{type(exc).__name__}: {exc}"
    finally:
        gc.callbacks.remove(gc_event)
    wall_s = time.monotonic() - epoch
    with (args.output / "timing.jsonl").open("x") as stream:
        for sample in samples:
            stream.write(json.dumps(sample) + "\n")
    atomic_json(args.output / "gc-events.json", events)
    timing = {name: stats([r[name] for r in samples if r[name] is not None], budget)
              for name, budget in (("core_s", .005), ("production_s", .005),
                                   ("deadline_response_s", .005), ("gap_s", .02),
                                   ("inference_s", .02), ("state_decode_s", None),
                                   ("log_encode_s", None), ("release_lateness_s", None))}
    gc_pairs, start_gc = [], None
    for event in events:
        if event["phase"] == "start":
            start_gc = event["wall_s"]
        elif start_gc is not None:
            gc_pairs.append((start_gc, event["wall_s"]))
            start_gc = None
    over = [r for r in samples if r["deadline_response_s"] > .005]
    result = dict(reason=reason, failure=failure, hardware_access=False, hardware_qualified=False,
                  command_samples=len(samples), policy_samples=core.policy_samples,
                  wall_s=wall_s, command_hz=len(samples)/wall_s if wall_s > 0 else None,
                  timing=timing, cold_inference_s=provider_audit["first_inference_s"],
                  supported_hold_core_s=hold_s, session_setup_s=setup_s,
                  max_raw_action_error=provider_audit["max_raw_action_error"],
                  observed_5ms_gate=bool(reason == "completed" and samples and not over
                                        and len(samples) >= round(args.duration * 200)),
                  missed_releases=max(0, round(args.duration * 200) - len(samples)),
                  gc=stats([b-a for a,b in gc_pairs]),
                  late_commands_overlapping_gc=sum(any(a < r["finish_wall_s"] and b > r["release_wall_s"]
                                                        for a,b in gc_pairs) for r in over),
                  largest_deadline_responses=sorted(samples, key=lambda r:r["deadline_response_s"],
                                                    reverse=True)[:10])
    record.finish("completed" if reason == "completed" else "failed", result)
    print(json.dumps({k: result[k] for k in ("reason", "command_samples", "policy_samples",
                     "observed_5ms_gate", "missed_releases", "timing")}), flush=True)


def audit_exploy(policy, frames, checkpoint, directory, manifest_path, height):
    state = State.model_validate(frames[0]["state"])
    previous = np.zeros(12, dtype=np.float32)
    began = time.monotonic()
    policy.evaluate(state, previous)
    cold = time.monotonic() - began
    warm = []
    for _ in range(100):
        began = time.monotonic()
        policy.warmup()
        warm.append(time.monotonic() - began)
    reference = ReLICPolicy(checkpoint)
    largest = 0.
    for frame in frames:
        obs = np.asarray(frame["observations"], dtype=np.float32)
        expected = reference.session.run(["actions"], {"obs": obs[None]})[0][0]
        _, _, actual_obs, action = policy.evaluate(State.model_validate(frame["state"]), obs[-12:],
                                                  velocity_command=frame.get('velocity_command'))
        np.testing.assert_allclose(actual_obs[0], obs, rtol=0, atol=1e-5)
        largest = max(largest, float(np.max(np.abs(expected - action))))
    if largest > 1e-4:
        raise AssertionError("Exploy recorded-frame action parity failed")
    profiled = make_policy(check_artifacts(manifest_path, policy.manifest), policy.manifest,
                           manifest_path, profile_prefix=directory / "ort-profile")
    profiled.initialize_height(height)
    profiled.evaluate(state, previous)
    profile = Path(profiled.runner.session.end_profiling())
    nodes = [e for e in json.loads(profile.read_text())
             if e.get("cat") == "Node" and e.get("args", {}).get("provider")]
    providers = Counter(e["args"]["provider"] for e in nodes)
    if not providers or set(providers) != {"CPUExecutionProvider"}:
        raise AssertionError("Unexpected Exploy compute provider")
    result = dict(first_inference_s=cold, warmup_s=stats(warm),
                  max_raw_action_error=largest, reference_checkpoint_sha256=sha256(checkpoint),
                  provider_compute_nodes=dict(providers), history_modified=False,
                  providers=policy.runner.session.get_providers(),
                  provider_options=policy.runner.session.get_provider_options())
    atomic_json(directory / "provider-audit.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-checkpoint", type=Path,
                        help="Original released ONNX actor; required for Exploy parity")
    for arg in ("manifest", "envelope", "source-run", "capture", "output"):
        parser.add_argument("--" + arg, type=Path, required=True)
    parser.add_argument("--device", choices=["cpu", "cuda"], required=True)
    parser.add_argument("--duration", type=float, default=60.)
    args = parser.parse_args()
    # Preserve matched CPU/CUDA imports for the original GPU comparison. The
    # Exploy CPU deployment benchmark needs only the locked runtime environment.
    if getattr(load_manifest(args.manifest), "adapter", None) != "relic-exploy":
        import torch
        torch.set_num_threads(2)
    run(args)


if __name__ == "__main__":
    main()

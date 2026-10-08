"""Shared evaluation loop. Falls finish the episode and are never auto-reset."""
import time
import numpy as np
from .contract import Policy, POLICY_DT, DECIMATION
from .record import Record
from .teleop import Receiver
from .comms import Communications, ControllerLink
from .record import write_json


def run(backend, output, engine, scenario, interactive=False, port=18765, comms=None):
    communications = Communications(comms)
    record = Record(output, engine, backend.contract, scenario,
                    metadata={"model": backend.metadata(), "interactive": interactive,
                              "communications": communications.config.metadata(),
                              "command_source": "pygame_udp" if interactive else "registered_sequence"})
    policy = Policy()
    controller = ControllerLink(backend.contract, policy, communications)
    receiver = Receiver(port, communications) if interactive else None
    start = time.monotonic()
    reason = "completed"
    try:
        for step in range(int(scenario.duration / POLICY_DT)):
            t = step * POLICY_DT
            if scenario.push_time >= 0 and abs(t - scenario.push_time) < POLICY_DT / 2:
                backend.push(scenario.push_dvy)
            state = backend.state()
            command = scenario.command(t)
            requested = command.velocity.copy()
            if receiver:
                command.velocity, quit_requested = receiver.poll(t)
                if quit_requested:
                    reason = "operator_quit"
                    break
                requested = receiver.requested_velocity.copy()
                status = receiver.status.copy()
            else:
                link = communications.links["command"]
                link.send(requested, t)
                packet = link.poll(t)
                command.velocity = np.zeros(3, dtype=np.float32) if packet is None else packet.payload
                status = dict(command_age_s=link.age(t), command_stale=packet is None,
                              command_sequence=None if packet is None else packet.sequence)
            applied, link_failure, controller_status = controller.step(state, command, t)
            status.update(controller_status)
            failure = record.add(t, state, command, controller.observation, controller.action, controller.target,
                                 physics_time=backend.physics_time, requested_command=requested,
                                 applied_target=applied, communications=status)
            failure = failure or link_failure
            if receiver:
                receiver.telemetry(state, controller.action, command, scenario.mode, t,
                                   status="stopped" if failure else "running", reason=failure,
                                   communications_status=status)
            if failure:
                reason = failure
                break
            for _ in range(DECIMATION):
                backend.step(applied)
            backend.render()
            if interactive:
                time.sleep(max(0, start + (step + 1) * POLICY_DT - time.monotonic()))
    except BaseException as error:
        record.finish(f"error:{type(error).__name__}:{error}")
        raise
    finally:
        if receiver:
            receiver.close()
        write_json(record.path / "communications.json", communications.summary())
    return record.finish(reason)

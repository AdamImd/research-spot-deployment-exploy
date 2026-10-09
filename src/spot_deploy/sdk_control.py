"""Control capability, imported only after offline standing gates pass.

No automatic E-stop registration, lease takeover, fault clearing or retries.
All robot requests use the SDK 5.1.1 protocol with bounded deadlines.
"""

import time

from .contracts import ContractError


def command_proto(command, manifest, envelope):
    from bosdyn.api.robot_command_pb2 import JointControlStreamRequest
    from bosdyn.util import seconds_to_timestamp, set_timestamp_from_now

    result = JointControlStreamRequest()
    set_timestamp_from_now(result.header.request_timestamp)
    result.header.client_name = "spot-deployment-standing"
    joint = result.joint_command
    joint.position.extend(command.positions)
    joint.velocity.extend([0.0] * 19)
    joint.load.extend(manifest.gains.feedforward if command.feedforward is None
                      else command.feedforward)
    # Repeat gains so restarting a stream never depends on stale firmware state.
    joint.gains.k_q_p.extend(manifest.gains.kp)
    joint.gains.k_qd_p.extend(manifest.gains.kd)
    joint.end_time.CopyFrom(seconds_to_timestamp(command.end_robot_time_s))
    joint.extrapolation_duration.nanos = 0
    joint.user_command_key = command.key
    # The server supports one shared velocity threshold; the host enforces per-joint bounds.
    joint.velocity_safety_limit.value = (envelope.sdk_velocity_safety_limit
                                        if envelope.sdk_velocity_safety_limit is not None
                                        else min(envelope.velocity_max))
    return result


class ControlSpot:
    def __init__(self, reader, envelope):
        from bosdyn.client.lease import LeaseClient
        from bosdyn.client.robot_command import RobotCommandClient, RobotCommandStreamingClient

        self.reader, self.robot, self.envelope = reader, reader.robot, envelope
        self.timeout = min(reader.config.rpc_timeout_s, envelope.shutdown_timeout_s)
        self.command = self.robot.ensure_client(RobotCommandClient.default_service_name)
        # Registering a type on an SDK after Robot creation does not update Robot's factory map.
        self.streaming = RobotCommandStreamingClient()
        self.streaming.channel = self.robot.ensure_channel(
            RobotCommandStreamingClient.default_service_name
        )
        self.streaming.update_from(self.robot)
        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.lease = None
        self.future = None
        self.command_id = None
        self.power_owned = False
        self.shutdown_result = None

    def acquire(self):
        self.lease = self.lease_client.acquire(timeout=self.timeout)

    def heartbeat(self):
        self.lease_client.retain_lease(self.lease, timeout=self.timeout)

    def native_stand(self, cancelled):
        from bosdyn.api import basic_command_pb2
        from bosdyn.client.robot_command import RobotCommandBuilder

        # Set before calling: a timed-out response can still have powered the motors.
        self.power_owned = True
        self.robot.power_on(timeout_sec=20, timeout=self.timeout)
        if cancelled.is_set():
            raise ContractError("health failure during power-on")
        command_id = self.command.robot_command(
            RobotCommandBuilder.synchro_stand_command(), timeout=self.timeout
        )
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if cancelled.is_set():
                raise ContractError("health failure during native standing")
            self.heartbeat()
            feedback = self.command.robot_command_feedback(command_id, timeout=self.timeout)
            mobility = feedback.feedback.synchronized_feedback.mobility_command_feedback
            if mobility.status != basic_command_pb2.RobotCommandFeedbackStatus.STATUS_PROCESSING:
                raise ContractError("native standing command rejected or interrupted")
            if (
                mobility.stand_feedback.status
                == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING
            ):
                return
            time.sleep(0.1)
        raise ContractError("native standing timeout")

    def start(self, commands, duration):
        self.future = self.streaming._stub.JointControlStream.future(
            commands, timeout=duration + self.timeout
        )

    def activate(self):
        from bosdyn.client.robot_command import RobotCommandBuilder

        self.command_id = self.command.robot_command(
            RobotCommandBuilder.joint_command(), timeout=self.timeout
        )

    def check_stream(self):
        if self.future.done():
            # Even STATUS_OK is unexpected while the bounded controller is still running.
            self.future.result()
            raise ContractError("command stream ended unexpectedly")

    def active(self):
        from bosdyn.api import basic_command_pb2

        response = self.command.robot_command_feedback(self.command_id, timeout=self.timeout)
        feedback = response.feedback.full_body_feedback
        if feedback.status != basic_command_pb2.RobotCommandFeedbackStatus.STATUS_PROCESSING:
            raise ContractError("joint control no longer processing")
        if feedback.joint_feedback.status == basic_command_pb2.JointCommand.Feedback.STATUS_ERROR:
            raise ContractError("joint control feedback error")
        return (
            feedback.joint_feedback.status == basic_command_pb2.JointCommand.Feedback.STATUS_ACTIVE
        )

    def close(self):
        from bosdyn.api.robot_state_pb2 import PowerState

        started = time.monotonic()
        self.shutdown_result = dict(motors_off_confirmed=False, lease_returned=False,
                                    power_owned=self.power_owned, errors=[])
        status = self.shutdown_result
        failures = []
        if self.future:
            try:
                self.future.cancel()
            except BaseException as exc:
                status['errors'].append(dict(stage='cancel_stream', error_type=type(exc).__name__))
                failures.append('Command stream cancellation unconfirmed')
        status['stream_cancel_elapsed_s'] = time.monotonic() - started
        if self.power_owned:
            power_started = time.monotonic()
            try:
                self.robot.power_off(
                    cut_immediately=False,
                    timeout_sec=self.envelope.shutdown_timeout_s,
                    timeout=self.timeout,
                )
                state = self.reader.state_client.get_robot_state(timeout=self.timeout)
                status['motor_power_state'] = int(state.power_state.motor_power_state)
                # is_powered_on() is also false for UNKNOWN/transition states.
                # Only an explicit OFF readback is affirmative shutdown evidence.
                if state.power_state.motor_power_state != PowerState.STATE_OFF:
                    raise ContractError("motor-OFF state not confirmed after shutdown")
                status['motors_off_confirmed'] = True
            except BaseException as exc:
                status['errors'].append(dict(stage='safe_power_off', error_type=type(exc).__name__))
                failures.append("Safe power-off unconfirmed; independent E-stop operator must act")
            status['safe_power_off_elapsed_s'] = time.monotonic() - power_started
        if self.lease:
            lease_started = time.monotonic()
            try:
                self.lease_client.return_lease(self.lease, timeout=self.timeout)
                status['lease_returned'] = True
            except BaseException as exc:
                status['errors'].append(dict(stage='return_lease', error_type=type(exc).__name__))
                failures.append("Lease return unconfirmed")
            status['lease_return_elapsed_s'] = time.monotonic() - lease_started
        status['elapsed_s'] = time.monotonic() - started
        status['within_shutdown_budget'] = bool(
            status['motors_off_confirmed'] and status['lease_returned']
            and status['elapsed_s'] <= self.envelope.shutdown_timeout_s)
        if failures:
            raise ContractError("; ".join(failures))
        return status

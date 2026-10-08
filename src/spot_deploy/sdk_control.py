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
    joint.velocity_safety_limit.value = min(envelope.velocity_max)
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
        failures = []
        if self.future:
            self.future.cancel()
        if self.power_owned:
            try:
                self.robot.power_off(
                    cut_immediately=False,
                    timeout_sec=self.envelope.shutdown_timeout_s,
                    timeout=self.timeout,
                )
                if self.robot.is_powered_on(timeout=self.timeout):
                    raise ContractError("motors remain powered after shutdown")
            except BaseException:
                failures.append("Safe power-off unconfirmed; independent E-stop operator must act")
        if self.lease:
            try:
                self.lease_client.return_lease(self.lease, timeout=self.timeout)
            except BaseException:
                failures.append("Lease return unconfirmed")
        if failures:
            raise ContractError("; ".join(failures))

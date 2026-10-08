"""Explicit E-stop configuration/registration. No power-on or motion commands.

Never calls force_simple_setup or uses an automatically-allowing keepalive.
All state-changing RPCs require an explicit CLI mode, motors-off identity check,
and the current config ID. The bridge preserves independent E-stop endpoints.
"""

from .contracts import ContractError


def primary_joystick(profile):
    return (getattr(profile, "input_type", None) == "linux_joystick"
            and profile.endpoint_role == "PDB_rooted")


def endpoint_for(client, profile):
    from bosdyn.client.estop import EstopEndpoint

    return EstopEndpoint(
        client,
        profile.endpoint_name,
        profile.endpoint_timeout_s,
        role=profile.endpoint_role,
        estop_cut_power_timeout=profile.cut_power_timeout_s,
    )


def assert_slot(config, profile, config_id, endpoint_id=None):
    if config.unique_id != config_id:
        raise ContractError("E-stop configuration changed; inspect it again")
    matches = [e for e in config.endpoints if e.role == profile.endpoint_role]
    if len(matches) != 1:
        raise ContractError("exactly one dedicated E-stop endpoint slot is required")
    slot = matches[0]
    expected = endpoint_for(None, profile).to_proto()
    if (
        slot.name != expected.name
        or slot.timeout != expected.timeout
        or slot.cut_power_timeout != expected.cut_power_timeout
        or (endpoint_id is not None and slot.unique_id != endpoint_id)
    ):
        raise ContractError("E-stop endpoint identity or watchdogs differ from the profile")
    if not primary_joystick(profile) and not any(
        e.role == "PDB_rooted" and e.role != profile.endpoint_role for e in config.endpoints
    ):
        raise ContractError("retain the independent PDB_rooted E-stop endpoint")
    return slot


class SpotEstop:
    def __init__(self, reader, profile):
        from bosdyn.client.estop import EstopClient

        self.reader, self.profile = reader, profile
        self.client = reader.robot.ensure_client(EstopClient.default_service_name)
        self.endpoint = None
        self.config_id = None

    def motors_off(self):
        from bosdyn.api import robot_state_pb2
        from bosdyn.client.robot_id import RobotIdClient

        identity = self.reader.robot.ensure_client(RobotIdClient.default_service_name).get_id(
            timeout=self.profile.rpc_timeout_s
        )
        version = identity.software_release.version
        config = self.reader.config
        if identity.serial_number != config.expected_serial or (
            f"{version.major_version}.{version.minor_version}.{version.patch_level}"
            != config.expected_firmware
        ):
            raise ContractError("E-stop robot identity or firmware mismatch")
        state = self.reader.state_client.get_robot_state(timeout=self.profile.rpc_timeout_s)
        if state.power_state.motor_power_state != robot_state_pb2.PowerState.STATE_OFF:
            raise ContractError("E-stop configuration and registration require motors off")

    def inspect(self):
        from google.protobuf.json_format import MessageToDict

        config = self.client.get_config(timeout=self.profile.rpc_timeout_s)
        status = self.client.get_status(timeout=self.profile.rpc_timeout_s)
        return {
            "config": MessageToDict(config, preserving_proto_field_name=True),
            "status": MessageToDict(status, preserving_proto_field_name=True),
        }

    def configure(self, expected_config_id):
        from bosdyn.api import estop_pb2
        from google.protobuf.json_format import MessageToDict

        self.motors_off()
        current = self.client.get_config(timeout=self.profile.rpc_timeout_s)
        if current.unique_id != expected_config_id:
            raise ContractError("E-stop configuration changed; inspect it again")
        if any(
            e.role == self.profile.endpoint_role or e.name == self.profile.endpoint_name
            for e in current.endpoints
        ):
            raise ContractError("E-stop endpoint name or role already exists; do not replace it")
        if primary_joystick(self.profile) and current.endpoints:
            raise ContractError("primary joystick setup requires an empty E-stop configuration")
        if not primary_joystick(self.profile) and not any(
            e.role == "PDB_rooted" for e in current.endpoints
        ):
            raise ContractError("configure the independent manufacturer E-stop first")
        proposed = estop_pb2.EstopConfig()
        proposed.endpoints.extend(current.endpoints)
        proposed.endpoints.add().CopyFrom(endpoint_for(self.client, self.profile).to_proto())
        updated = self.client.set_config(
            proposed, expected_config_id, timeout=self.profile.rpc_timeout_s
        )
        slot = assert_slot(updated, self.profile, updated.unique_id)
        return {
            "config_id": updated.unique_id,
            "endpoint_id": slot.unique_id,
            "config_before": MessageToDict(current, preserving_proto_field_name=True),
            "config_after": MessageToDict(updated, preserving_proto_field_name=True),
            "note": "Re-register and verify independent endpoints; all robot motors remain off.",
        }

    def register(self, config_id, endpoint_id):
        from bosdyn.api import estop_pb2
        from bosdyn.client.estop import (
            _new_endpoint_from_register_response,
            _register_endpoint_error_from_response,
        )

        self.motors_off()
        current = self.client.get_config(timeout=self.profile.rpc_timeout_s)
        slot = assert_slot(current, self.profile, config_id, endpoint_id)
        status = self.client.get_status(timeout=self.profile.rpc_timeout_s)
        for active in status.endpoints:
            if active.endpoint.role == self.profile.endpoint_role:
                age = (
                    active.time_since_valid_response.seconds
                    + active.time_since_valid_response.nanos / 1e9
                )
                # stop_level is the last requested level, even after a timeout.
                # An expired NONE does not mean the previous process is still alive.
                if age <= self.profile.endpoint_timeout_s:
                    raise ContractError(
                        "E-stop endpoint has an active owner; stop it and wait for timeout"
                    )
        self.endpoint = endpoint_for(self.client, self.profile)
        self.endpoint.from_proto(slot)
        # SDK 5.1.1's register helper omits the target ID. Explicitly bind replacement
        # to this verified, expired slot so another endpoint cannot be taken over.
        request = estop_pb2.RegisterEstopEndpointRequest(
            target_config_id=config_id,
            new_endpoint=self.endpoint.to_proto(),
            target_endpoint=estop_pb2.EstopEndpoint(
                role=self.profile.endpoint_role, unique_id=slot.unique_id
            ),
        )
        registered = self.client.call(
            self.client._stub.RegisterEstopEndpoint,
            request,
            value_from_response=_new_endpoint_from_register_response,
            error_from_response=_register_endpoint_error_from_response,
            copy_request=False,
            timeout=self.profile.rpc_timeout_s,
        )
        self.endpoint.from_proto(registered)
        self.config_id = config_id
        self.check_in(False)

    @property
    def endpoint_id(self):
        return self.endpoint.to_proto().unique_id if self.endpoint else None

    def check_in(self, allowed):
        from bosdyn.api import estop_pb2

        if self.endpoint is None:
            raise ContractError("E-stop endpoint is not registered")
        stop = (
            estop_pb2.ESTOP_LEVEL_CUT
            if self.profile.stop_level == "cut"
            else estop_pb2.ESTOP_LEVEL_SETTLE_THEN_CUT
        )
        self.endpoint.check_in_at_level(
            estop_pb2.ESTOP_LEVEL_NONE if allowed else stop, timeout=self.profile.rpc_timeout_s
        )

    def stop(self):
        if self.endpoint is not None:
            self.check_in(False)

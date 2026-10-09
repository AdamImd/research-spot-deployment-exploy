"""Read-only Spot SDK capability. No lease, motor, command or E-stop-write methods.

SDK 5.1.1 streaming wrappers ignore kwargs and hide cancellation. The pinned
adapter intentionally uses the generated state stub to retain a cancellable call.
"""

import hashlib
import os
import threading
import time

import numpy as np

from .contracts import ContractError, JOINTS, State, canonical_hash
from .safety import LatestState, bounded_join, percentiles


def seconds(timestamp):
    return timestamp.seconds + timestamp.nanos / 1e9


def fault_details(state):
    """Structured severity data; callers retain their own unchanged fault policy."""
    return [dict(kind=kind, name=getattr(fault, 'name', ''), code=getattr(fault, 'code', 0),
                 severity=int(getattr(fault, 'severity', 0)))
            for kind in ('system_fault_state', 'behavior_fault_state', 'service_fault_state')
            for fault in getattr(state, kind).faults]


def decode_state(message, received=None):
    from bosdyn.api.header_pb2 import CommonError

    code = message.header.error.code
    # Spot 5.0.1 omits header.error on successful high-rate state messages.
    # An explicitly populated error, including UNSPECIFIED, must still be rejected.
    omitted_success = code == CommonError.CODE_UNSPECIFIED and not message.header.HasField("error")
    if code != CommonError.CODE_OK and not omitted_success:
        raise ContractError("state stream response header error")
    joints, kin = message.joint_states, message.kinematic_state
    quat = kin.odom_tform_body.rotation
    lin, ang = kin.velocity_of_body_in_odom.linear, kin.velocity_of_body_in_odom.angular
    return State(
        robot_time_s=seconds(joints.acquisition_timestamp),
        received_monotonic_s=time.monotonic() if received is None else received,
        positions=list(joints.position),
        velocities=list(joints.velocity),
        loads=list(joints.load),
        odom_quaternion_wxyz=[quat.w, quat.x, quat.y, quat.z],
        linear_velocity_odom=[lin.x, lin.y, lin.z],
        angular_velocity_odom=[ang.x, ang.y, ang.z],
        last_command_key=message.last_command.user_command_key,
        last_command_received_robot_s=seconds(message.last_command.received_timestamp),
    )


def decode_full_state(message, received=None):
    """Map named unary state joints into SDK order, using the odom/body transform."""
    from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b

    kin = message.kinematic_state
    joints = {j.name.replace(".", "_"): j for j in kin.joint_states}
    if len(kin.joint_states) != 19 or set(joints) != set(JOINTS):
        raise ContractError("full state does not contain the 19 canonical Spot joints")
    pose = get_a_tform_b(kin.transforms_snapshot, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    if pose is None:
        raise ContractError("full state lacks odom/body transform")
    quat = pose.rot
    lin, ang = kin.velocity_of_body_in_odom.linear, kin.velocity_of_body_in_odom.angular
    return State(
        robot_time_s=seconds(kin.acquisition_timestamp),
        received_monotonic_s=time.monotonic() if received is None else received,
        positions=[joints[n].position.value for n in JOINTS],
        velocities=[joints[n].velocity.value for n in JOINTS],
        loads=[joints[n].load.value for n in JOINTS],
        odom_quaternion_wxyz=[quat.w, quat.x, quat.y, quat.z],
        linear_velocity_odom=[lin.x, lin.y, lin.z],
        angular_velocity_odom=[ang.x, ang.y, ang.z],
        # Unary measured-state telemetry does not provide joint-command acknowledgements.
        last_command_key=0,
        last_command_received_robot_s=0,
    )


def body_height_from_state(message):
    """Vertical body-origin height above the SDK's estimated ground plane.

    Odom's origin is arbitrary. Intersect the vertical line through the body
    with GPE's plane instead of treating odom Z as height above the floor.
    """
    from bosdyn.client.frame_helpers import (
        BODY_FRAME_NAME, GROUND_PLANE_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b,
    )

    kin = message.kinematic_state
    body = get_a_tform_b(kin.transforms_snapshot, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    ground = get_a_tform_b(kin.transforms_snapshot, ODOM_FRAME_NAME, GROUND_PLANE_FRAME_NAME)
    if body is None or ground is None:
        raise ContractError("initial body height requires odom/body and odom/gpe transforms")
    p = np.array([body.x, body.y, body.z])
    origin = np.array([ground.x, ground.y, ground.z])
    normal = np.asarray(ground.rot.to_matrix())[:, 2]
    if not np.isfinite([p, origin, normal]).all() or abs(normal[2]) < .5:
        raise ContractError("initial body height has an invalid or steep ground plane")
    height = float(np.dot(normal, p - origin) / normal[2])
    if not 0 < height <= 1.5:
        raise ContractError("initial body height is outside (0, 1.5] metres")
    return {"height_m": height, "source": "initial SDK ground-plane estimate",
            "robot_time_s": seconds(kin.acquisition_timestamp),
            "body_position_odom": p.tolist(), "ground_point_odom": origin.tolist(),
            "ground_normal_odom": normal.tolist(),
            "formula": "dot(ground_normal, body_position - ground_point) / ground_normal.z",
            "foot_contacts": [int(foot.contact) for foot in message.foot_state]}


def estop_observation(config, state, status):
    """Observe stop state; never register, allow or check in to an endpoint.

    Modern Spot permits an empty SDK endpoint configuration. Tablet mode uses
    aggregate service status AND the full robot hardware/software stop states.
    This does not prove tablet connectivity or physical button functionality.
    """
    from bosdyn.api import estop_pb2, robot_state_pb2

    cls = robot_state_pb2.EStopState
    states = list(state.estop_states)
    complete = {cls.TYPE_HARDWARE, cls.TYPE_SOFTWARE} <= {s.type for s in states}
    robot_clear = complete and all(s.state == cls.STATE_NOT_ESTOPPED for s in states)
    required = robot_clear if config.estop_authority == "tablet" else bool(status.endpoints)
    return {
        "estop_ready": bool(required and status.stop_level == estop_pb2.ESTOP_LEVEL_NONE),
        "estop_authority": config.estop_authority,
        "estop_endpoint_count": len(status.endpoints),
        "estop_states": [{"name": s.name, "type": cls.Type.Name(s.type),
                          "state": cls.State.Name(s.state)} for s in states],
    }


class ReadOnlySpot:
    def __init__(self, config):
        self.config = config
        self.mailbox = LatestState()
        self.robot = None
        self.connection_stage = "not_started"
        self._time_sync = None
        self._stream = None
        self._thread = None
        self._stop = threading.Event()
        self._acquisition_gaps = []
        self._receive_gaps = []
        self._received_count = 0

    def connect(self, streaming=True):
        import bosdyn.client
        from bosdyn.client.robot_state import RobotStateClient, RobotStateStreamingClient

        username = os.environ.get("BOSDYN_CLIENT_USERNAME")
        password = os.environ.get("BOSDYN_CLIENT_PASSWORD")
        if not username or not password:
            raise ContractError("set the two BOSDYN_CLIENT credential environment variables")
        sdk = bosdyn.client.create_standard_sdk("spot-readiness-read")
        sdk.register_service_client(RobotStateStreamingClient)
        self.robot = sdk.create_robot(self.config.endpoint)
        self.connection_stage = "authentication"
        self.robot.authenticate(username, password, timeout=self.config.rpc_timeout_s)
        self.connection_stage = "time_sync"
        self._time_sync = self.robot.time_sync
        self._time_sync.wait_for_sync(timeout_sec=self.config.rpc_timeout_s)
        self.connection_stage = "state_service"
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        if streaming:
            self.connection_stage = "streaming_state_service"
            self.stream_client = self.robot.ensure_client(
                RobotStateStreamingClient.default_service_name
            )
        self.connection_stage = "connected"

    def read_state(self):
        message = self.state_client.get_robot_state(timeout=self.config.rpc_timeout_s)
        return decode_full_state(message)

    def read_body_height(self):
        message = self.state_client.get_robot_state(timeout=self.config.rpc_timeout_s)
        result = body_height_from_state(message)
        result["state_age_s"] = self.robot_now() - result["robot_time_s"]
        if abs(result["state_age_s"]) > .25:
            raise ContractError("initial body-height state older than 250 ms")
        return result

    def robot_now(self):
        if not self.robot.time_sync.endpoint.has_established_time_sync:
            raise ContractError("time synchronization lost")
        return seconds(self.robot.time_sync.endpoint.robot_timestamp_from_local_secs(time.time()))

    def snapshot(self):
        from google.protobuf.json_format import MessageToDict
        from bosdyn.api import robot_state_pb2
        from bosdyn.client.estop import EstopClient
        from bosdyn.client.license import LicenseClient
        from bosdyn.client.payload import PayloadClient
        from bosdyn.client.robot_id import RobotIdClient

        timeout = self.config.rpc_timeout_s
        robot_id = self.robot.ensure_client(RobotIdClient.default_service_name).get_id(
            timeout=timeout
        )
        version = robot_id.software_release.version
        firmware = f"{version.major_version}.{version.minor_version}.{version.patch_level}"
        hardware = self.state_client.get_robot_hardware_configuration(timeout=timeout)
        if not hardware.skeleton.urdf:
            raise ContractError("robot returned no morphology description")
        self.robot_urdf = hardware.skeleton.urdf
        payloads = self.robot.ensure_client(PayloadClient.default_service_name).list_payloads(
            timeout=timeout
        )
        features = self.robot.ensure_client(LicenseClient.default_service_name).get_feature_enabled(
            [self.config.joint_control_feature], timeout=timeout
        )
        state = self.state_client.get_robot_state(timeout=timeout)
        estop = self.robot.ensure_client(EstopClient.default_service_name).get_status(
            timeout=timeout
        )
        # Full-state names use dots (fl.hx, arm0.sh0); streamed arrays use SDK order.
        names = [j.name.replace(".", "_") for j in state.kinematic_state.joint_states]
        fault_count = (
            len(state.system_fault_state.faults)
            + len(state.behavior_fault_state.faults)
            + len(state.service_fault_state.faults)
        )
        result = {
            "serial": robot_id.serial_number,
            "firmware": firmware,
            "sdk_version": "5.1.1",
            "joint_names": names,
            "has_arm": self.robot.has_arm(timeout=timeout),
            "arm_stowed": state.manipulator_state.stow_state
            == robot_state_pb2.ManipulatorState.STOWSTATE_STOWED,
            "joint_control_licensed": bool(features.get(self.config.joint_control_feature, False)),
            "joint_control_feature_code": self.config.joint_control_feature,
            **estop_observation(self.config, state, estop),
            "active_fault_count": fault_count,
            "fault_details": fault_details(state),
            "battery_percent": min(
                (b.charge_percentage.value for b in state.battery_states), default=0
            ),
            "motors_off": state.power_state.motor_power_state
            == robot_state_pb2.PowerState.STATE_OFF,
            "robot_model_sha256": hashlib.sha256(hardware.skeleton.urdf.encode()).hexdigest(),
            "payloads": sorted(
                [
                    {
                        "name": p.name,
                        "guid": p.GUID,
                        "authorized": p.is_authorized,
                        "enabled": p.is_enabled,
                        "mass_volume_properties": MessageToDict(p.mass_volume_properties),
                        "body_tform_payload": MessageToDict(p.body_tform_payload),
                        "mount_tform_payload": MessageToDict(p.mount_tform_payload),
                        "mount_frame_name": p.mount_frame_name,
                    }
                    for p in payloads
                ],
                key=lambda p: p["guid"],
            ),
        }
        result["payload_config_sha256"] = canonical_hash({"payloads": result["payloads"]})
        result["identity_matches"] = (
            result["serial"] == self.config.expected_serial
            and firmware == self.config.expected_firmware
        )
        result["joint_layout_matches"] = len(names) == 19 and set(names) == set(JOINTS)
        return result

    def health(self):
        from bosdyn.api import estop_pb2
        from bosdyn.client.estop import EstopClient

        state = self.state_client.get_robot_state(timeout=self.config.rpc_timeout_s)
        estop = self.robot.ensure_client(EstopClient.default_service_name).get_status(
            timeout=self.config.rpc_timeout_s
        )
        return {
            "time": time.monotonic(),
            **estop_observation(self.config, state, estop),
            "fault_count": len(state.system_fault_state.faults)
            + len(state.behavior_fault_state.faults)
            + len(state.service_fault_state.faults),
            "fault_details": fault_details(state),
            "battery_percent": min(
                (b.charge_percentage.value for b in state.battery_states), default=0
            ),
            "estop_endpoints": [
                {
                    "id": e.endpoint.unique_id,
                    "role": e.endpoint.role,
                    "allowed": e.stop_level == estop_pb2.ESTOP_LEVEL_NONE,
                }
                for e in estop.endpoints
            ],
        }

    def start_stream(self, duration):
        from bosdyn.api.robot_state_pb2 import RobotStateStreamRequest

        self._stream = self.stream_client._stub.GetRobotStateStream(
            RobotStateStreamRequest(), timeout=duration + self.config.rpc_timeout_s
        )

        def receive():
            previous = None
            try:
                for message in self._stream:
                    if self._stop.is_set():
                        break
                    state = decode_state(message)
                    self.mailbox.publish(state)
                    if previous is not None:
                        self._acquisition_gaps.append(state.robot_time_s - previous.robot_time_s)
                        self._receive_gaps.append(
                            state.received_monotonic_s - previous.received_monotonic_s
                        )
                    previous = state
                    self._received_count += 1
                if not self._stop.is_set():
                    raise ContractError("state stream ended unexpectedly")
            except BaseException as exc:
                if not self._stop.is_set():
                    self.mailbox.fail(exc)

        self._thread = threading.Thread(target=receive, name="spot-state", daemon=True)
        self._thread.start()
        return self.mailbox.get(self.config.rpc_timeout_s)

    def stream_statistics(self):
        return {
            "received_count": self._received_count,
            "acquisition_gap_s": percentiles(self._acquisition_gaps.copy()),
            "receive_gap_s": percentiles(self._receive_gaps.copy()),
        }

    @property
    def received_count(self):
        return self._received_count

    def close(self):
        self._stop.set()
        self.mailbox.close()
        if self._stream:
            self._stream.cancel()
        error = None
        if self._thread:
            try:
                bounded_join(self._thread, self.config.rpc_timeout_s)
            except ContractError as exc:
                error = exc
        if self._time_sync is not None:
            # Accessing robot.time_sync starts it. Cleanup after failed authentication
            # must never create a new service connection or background thread.
            stopper = threading.Thread(target=self._time_sync.stop, daemon=True)
            stopper.start()
            bounded_join(stopper, self.config.rpc_timeout_s)
        if error:
            raise error


def validate_snapshot(snapshot, config, manifest=None, envelope=None, for_control=False):
    checks = {
        "identity_firmware": snapshot["identity_matches"],
        "joint_layout": snapshot["joint_layout_matches"],
        "physical_arm": snapshot["has_arm"],
        "joint_control_license": snapshot["joint_control_licensed"],
        "arm_stowed": snapshot["arm_stowed"],
        "no_active_faults": snapshot["active_fault_count"] == 0,
        "external_estop_ready": snapshot["estop_ready"],
    }
    if manifest:
        checks["robot_model"] = snapshot["robot_model_sha256"] == manifest.robot_model_sha256
        checks["payload_configuration"] = (
            snapshot["payload_config_sha256"] == manifest.payload_config_sha256
        )
    if envelope:
        checks["battery"] = snapshot["battery_percent"] >= envelope.min_battery_percent
        if envelope.arm_motion_guard == 'observe':
            # Keep the raw stow observation in snapshot; this is an explicit
            # alternative prerequisite, not a claim that a displaced arm is stowed.
            checks.pop('arm_stowed')
            checks['arm_harness_observation'] = (
                getattr(manifest, 'adapter', None) in ('relic84', 'relic-exploy'))
    if for_control:
        checks["motors_initially_off"] = snapshot["motors_off"]
    return checks

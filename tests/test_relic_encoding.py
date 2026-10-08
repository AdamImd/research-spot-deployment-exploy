import numpy as np
import pytest

from spot_deploy.contracts import ContractError
from spot_deploy.safety import Guard
from spot_deploy.sdk_control import command_proto


def test_per_command_feedforward_reaches_sdk_without_changing_gains(manifest, envelope, state):
    ff = np.linspace(-.5, .5, 19)
    command = Guard(manifest, envelope).command(
        np.zeros(19), state, state.received_monotonic_s, state.robot_time_s,
        state.received_monotonic_s, .001, feedforward=ff)
    message = command_proto(command, manifest, envelope).joint_command
    np.testing.assert_allclose(message.load, ff, atol=1e-7)
    np.testing.assert_allclose(message.gains.k_q_p, manifest.gains.kp)
    np.testing.assert_array_equal(message.velocity, np.zeros(19))
    assert message.extrapolation_duration.nanos == 0


@pytest.mark.parametrize("ff", [[float("nan")]*19, [0.]*18, [3.]*19])
def test_feedforward_is_validated_before_encoding(manifest, envelope, state, ff):
    with pytest.raises(ContractError, match="feedforward"):
        Guard(manifest, envelope).command(
            np.zeros(19), state, state.received_monotonic_s, state.robot_time_s,
            state.received_monotonic_s, .001, feedforward=ff)

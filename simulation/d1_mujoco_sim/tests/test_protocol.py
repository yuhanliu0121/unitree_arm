import json

import numpy as np
import pytest

from d1_mujoco_sim.protocol import (
    ProtocolError,
    full_joint_targets_deg,
    parse_command,
    qpos_to_sdk_angles,
    sdk_angles_to_qpos,
)


def test_full_joint_command() -> None:
    payload = json.dumps(
        {
            "seq": 42,
            "address": 1,
            "funcode": 2,
            "data": {
                "mode": 1,
                **{f"angle{i}": float(i * 10) for i in range(7)},
            },
        }
    )
    command = parse_command(payload)
    targets, mode = full_joint_targets_deg(command)
    assert command.seq == 42
    assert mode == 1
    np.testing.assert_allclose(targets, np.arange(7) * 10.0)


def test_invalid_joint_command_is_rejected() -> None:
    with pytest.raises(ProtocolError):
        parse_command('{"seq":1,"address":9,"funcode":2,"data":{}}')


def test_sdk_and_urdf_units_round_trip() -> None:
    angles = np.array([10, -20, 30, -40, 50, -60, 15], dtype=float)
    qpos = sdk_angles_to_qpos(angles, -30.0, 60.0, 0.03)
    recovered = qpos_to_sdk_angles(qpos, -30.0, 60.0, 0.03)
    np.testing.assert_allclose(recovered, angles, atol=1e-10)

from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any

import numpy as np


ARM_JOINT_COUNT = 6
TOTAL_JOINT_COUNT = 7


class ProtocolError(ValueError):
    """Raised when a D1 JSON command is malformed or unsupported."""


@dataclass(frozen=True)
class D1Command:
    seq: int
    funcode: int
    data: dict[str, Any]


def parse_command(payload: str) -> D1Command:
    try:
        message = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"invalid JSON: {exc.msg}") from exc

    if not isinstance(message, dict):
        raise ProtocolError("command must be a JSON object")
    if message.get("address") != 1:
        raise ProtocolError("command address must be 1")

    seq = message.get("seq")
    funcode = message.get("funcode")
    if not isinstance(seq, int):
        raise ProtocolError("seq must be an integer")
    if not isinstance(funcode, int) or funcode not in range(1, 8):
        raise ProtocolError("funcode must be an integer in [1, 7]")

    data = message.get("data", {})
    if not isinstance(data, dict):
        raise ProtocolError("data must be an object")
    return D1Command(seq=seq, funcode=funcode, data=data)


def full_joint_targets_deg(command: D1Command) -> tuple[np.ndarray, int]:
    if command.funcode != 2:
        raise ProtocolError("full_joint_targets_deg requires funcode=2")
    mode = command.data.get("mode")
    if mode not in (0, 1):
        raise ProtocolError("mode must be 0 or 1")
    try:
        targets = np.asarray(
            [command.data[f"angle{i}"] for i in range(TOTAL_JOINT_COUNT)],
            dtype=np.float64,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ProtocolError("angle0 through angle6 must be numeric") from exc
    if not np.all(np.isfinite(targets)):
        raise ProtocolError("joint targets must be finite")
    return targets, mode


def receive_ack(seq: int, accepted: bool) -> str:
    return json.dumps(
        {
            "seq": seq,
            "address": 3,
            "funcode": 1,
            "data": {"recv_status": int(accepted)},
        },
        separators=(",", ":"),
    )


def execution_ack(seq: int, succeeded: bool) -> str:
    return json.dumps(
        {
            "seq": seq,
            "address": 3,
            "funcode": 2,
            "data": {"exec_status": int(succeeded)},
        },
        separators=(",", ":"),
    )


def joint_feedback_json(angles_deg: np.ndarray) -> str:
    data = {f"angle{i}": float(angles_deg[i]) for i in range(TOTAL_JOINT_COUNT)}
    return json.dumps(
        {"seq": 10, "address": 2, "funcode": 1, "data": data},
        separators=(",", ":"),
    )


def status_feedback_json(enabled: bool, powered: bool) -> str:
    return json.dumps(
        {
            "seq": 10,
            "address": 2,
            "funcode": 3,
            "data": {
                "enable_status": int(enabled),
                "power_status": int(powered),
                # Real hardware was observed reporting 0 during normal use.
                "error_status": 0,
            },
        },
        separators=(",", ":"),
    )


def sdk_angles_to_qpos(
    angles_deg: np.ndarray,
    closed_angle_deg: float,
    open_angle_deg: float,
    gripper_travel_m: float,
) -> np.ndarray:
    angles = np.asarray(angles_deg, dtype=np.float64)
    if angles.shape != (TOTAL_JOINT_COUNT,):
        raise ValueError("expected seven SDK angles")
    qpos = np.empty(TOTAL_JOINT_COUNT, dtype=np.float64)
    qpos[:ARM_JOINT_COUNT] = np.deg2rad(angles[:ARM_JOINT_COUNT])
    ratio = (angles[6] - closed_angle_deg) / (
        open_angle_deg - closed_angle_deg
    )
    qpos[6] = gripper_travel_m * np.clip(ratio, 0.0, 1.0)
    return qpos


def qpos_to_sdk_angles(
    qpos: np.ndarray,
    closed_angle_deg: float,
    open_angle_deg: float,
    gripper_travel_m: float,
) -> np.ndarray:
    positions = np.asarray(qpos, dtype=np.float64)
    if positions.shape != (TOTAL_JOINT_COUNT,):
        raise ValueError("expected seven MuJoCo joint positions")
    angles = np.empty(TOTAL_JOINT_COUNT, dtype=np.float64)
    angles[:ARM_JOINT_COUNT] = np.rad2deg(positions[:ARM_JOINT_COUNT])
    ratio = np.clip(positions[6] / gripper_travel_m, 0.0, 1.0)
    angles[6] = closed_angle_deg + ratio * (
        open_angle_deg - closed_angle_deg
    )
    return angles

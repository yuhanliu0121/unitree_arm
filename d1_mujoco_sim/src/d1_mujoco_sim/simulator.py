from __future__ import annotations

from collections import deque
import logging
import time
from typing import Callable

import mujoco
import numpy as np

from .model import JOINT_NAMES
from .protocol import (
    D1Command,
    ProtocolError,
    execution_ack,
    full_joint_targets_deg,
    joint_feedback_json,
    parse_command,
    qpos_to_sdk_angles,
    receive_ack,
    sdk_angles_to_qpos,
    status_feedback_json,
)


LOGGER = logging.getLogger(__name__)


class D1Simulator:
    def __init__(
        self,
        model: mujoco.MjModel,
        config: dict,
        manual_control: bool = False,
    ) -> None:
        self.model = model
        self.data = mujoco.MjData(model)
        self.config = config
        self.manual_control = manual_control
        gripper = config["gripper"]
        self.closed_angle_deg = float(gripper["closed_angle_deg"])
        self.open_angle_deg = float(gripper["open_angle_deg"])
        self.gripper_travel_m = float(gripper["travel_m"])
        self.gripper_closing_preload_m = float(
            config["controller"].get("gripper_closing_preload_m", 0.0)
        )

        initial_angles = np.asarray(
            config["simulation"]["initial_sdk_angles_deg"],
            dtype=np.float64,
        )
        initial_qpos = sdk_angles_to_qpos(
            initial_angles,
            self.closed_angle_deg,
            self.open_angle_deg,
            self.gripper_travel_m,
        )
        self.qpos_ids = np.asarray(
            [
                model.jnt_qposadr[
                    mujoco.mj_name2id(
                        model,
                        mujoco.mjtObj.mjOBJ_JOINT,
                        name,
                    )
                ]
                for name in JOINT_NAMES
            ],
            dtype=np.int32,
        )
        self.data.qpos[self.qpos_ids] = initial_qpos
        self.commanded_qpos = initial_qpos.copy()
        self.filtered_qpos = initial_qpos.copy()
        self.data.ctrl[:] = initial_qpos
        mujoco.mj_forward(self.model, self.data)

        self.enabled = np.ones(7, dtype=bool)
        self.powered = True
        self.mode = 0
        self.last_command_time = -np.inf
        self.last_feedback_time = -np.inf
        self._base_gain = self.model.actuator_gainprm[:, 0].copy()
        self._base_bias = self.model.actuator_biasprm[:, :3].copy()
        mobile_base_name = str(
            config.get("mobile_base", {}).get("body_name", "go2_base")
        )
        mobile_base_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            mobile_base_name,
        )
        self.mobile_base_mocap_id = (
            int(model.body_mocapid[mobile_base_id])
            if mobile_base_id >= 0
            else -1
        )

    @property
    def joint_positions(self) -> np.ndarray:
        return self.data.qpos[self.qpos_ids].copy()

    @property
    def sdk_angles_deg(self) -> np.ndarray:
        return qpos_to_sdk_angles(
            self.joint_positions,
            self.closed_angle_deg,
            self.open_angle_deg,
            self.gripper_travel_m,
        )

    @property
    def mobile_base_xy(self) -> np.ndarray:
        if self.mobile_base_mocap_id < 0:
            raise RuntimeError("This model has no movable Go2 base")
        return self.data.mocap_pos[self.mobile_base_mocap_id, :2].copy()

    def set_mobile_base_xy(self, x: float, y: float) -> None:
        """Set the GT platform position while preserving z and orientation."""
        if self.mobile_base_mocap_id < 0:
            raise RuntimeError("This model has no movable Go2 base")
        xy = np.asarray([x, y], dtype=np.float64)
        if not np.isfinite(xy).all():
            raise ValueError("Mobile-base x and y must be finite")
        self.data.mocap_pos[self.mobile_base_mocap_id, :2] = xy
        mujoco.mj_forward(self.model, self.data)

    def _set_target_deg(self, targets_deg: np.ndarray, mode: int) -> None:
        target_qpos = sdk_angles_to_qpos(
            targets_deg,
            self.closed_angle_deg,
            self.open_angle_deg,
            self.gripper_travel_m,
        )
        if targets_deg[6] <= self.closed_angle_deg:
            target_qpos[6] = -self.gripper_closing_preload_m
        ranges = self.model.actuator_ctrlrange
        self.commanded_qpos = np.clip(
            target_qpos,
            ranges[:, 0],
            ranges[:, 1],
        )
        self.mode = mode

    def apply_command(self, command: D1Command) -> None:
        if self.manual_control:
            raise ProtocolError(
                "DDS motion commands are disabled in manual-control mode"
            )
        data = command.data
        if command.funcode == 1:
            joint_id = data.get("id")
            angle = data.get("angle")
            if not isinstance(joint_id, int) or joint_id not in range(7):
                raise ProtocolError("joint id must be in [0, 6]")
            if not isinstance(angle, (int, float)) or not np.isfinite(angle):
                raise ProtocolError("angle must be finite and numeric")
            targets = qpos_to_sdk_angles(
                self.commanded_qpos,
                self.closed_angle_deg,
                self.open_angle_deg,
                self.gripper_travel_m,
            )
            targets[joint_id] = float(angle)
            delay_ms = data.get("delay_ms", 0)
            if not isinstance(delay_ms, (int, float)) or delay_ms < 0:
                raise ProtocolError("delay_ms must be non-negative")
            self._set_target_deg(targets, self.mode)
            self.enabled[joint_id] = True
        elif command.funcode == 2:
            targets, mode = full_joint_targets_deg(command)
            self._set_target_deg(targets, mode)
            self.enabled[:] = True
        elif command.funcode == 4:
            joint_id = data.get("id")
            mode = data.get("mode")
            if not isinstance(joint_id, int) or joint_id not in range(7):
                raise ProtocolError("joint id must be in [0, 6]")
            if not isinstance(mode, (int, float)):
                raise ProtocolError("enable mode must be numeric")
            self.enabled[joint_id] = mode >= 1000
        elif command.funcode == 5:
            mode = data.get("mode")
            if not isinstance(mode, (int, float)):
                raise ProtocolError("enable mode must be numeric")
            self.enabled[:] = mode >= 1000
        elif command.funcode == 6:
            power = data.get("power")
            if power == 1:
                self.powered = True
            elif power == 0:
                # Match the observed firmware: ACK succeeds but power remains on.
                LOGGER.warning("power=0 acknowledged but intentionally has no effect")
            else:
                raise ProtocolError("power must be 0 or 1")
        elif command.funcode == 7:
            self._set_target_deg(np.zeros(7, dtype=np.float64), self.mode)
            self.enabled[:] = True
        else:
            raise ProtocolError(f"unsupported funcode {command.funcode}")

    def handle_payload(self, payload: str) -> tuple[D1Command | None, list[str]]:
        try:
            command = parse_command(payload)
            self.apply_command(command)
        except ProtocolError as exc:
            LOGGER.warning("Rejected D1 command: %s", exc)
            try:
                seq = int(__import__("json").loads(payload).get("seq", 0))
            except (ValueError, TypeError, AttributeError):
                seq = 0
            return None, [receive_ack(seq, False)]
        # Real execution ACK is early/unreliable. The simulator sends it early
        # so completion logic cannot accidentally rely on physical convergence.
        return command, [
            receive_ack(command.seq, True),
            execution_ack(command.seq, True),
        ]

    def step(self) -> None:
        if self.manual_control:
            # In passive-viewer mode MuJoCo writes slider edits directly into
            # data.ctrl. Preserve those values instead of replacing them with
            # the DDS target/filter on every physics step.
            np.clip(
                self.data.ctrl,
                self.model.actuator_ctrlrange[:, 0],
                self.model.actuator_ctrlrange[:, 1],
                out=self.data.ctrl,
            )
            mujoco.mj_step(self.model, self.data)
            return

        dt = self.model.opt.timestep
        controller = self.config["controller"]
        tau = float(controller[f"mode_{self.mode}_time_constant_s"])
        alpha = 1.0 - np.exp(-dt / max(tau, dt))
        self.filtered_qpos += alpha * (
            self.commanded_qpos - self.filtered_qpos
        )

        active = self.enabled & self.powered
        for index, is_active in enumerate(active):
            if is_active:
                self.model.actuator_gainprm[index, 0] = self._base_gain[index]
                self.model.actuator_biasprm[index, :3] = self._base_bias[index]
            else:
                self.model.actuator_gainprm[index, 0] = 0.0
                self.model.actuator_biasprm[index, :3] = 0.0
        self.data.ctrl[:] = self.filtered_qpos
        mujoco.mj_step(self.model, self.data)

    def feedback_due(self, now: float) -> bool:
        period = 1.0 / float(self.config["dds"]["feedback_rate_hz"])
        if now - self.last_feedback_time >= period:
            self.last_feedback_time = now
            return True
        return False

    def feedback_payloads(self) -> tuple[list[float], list[str]]:
        angles = self.sdk_angles_deg.tolist()
        feedback = [
            joint_feedback_json(np.asarray(angles)),
            status_feedback_json(bool(np.any(self.enabled)), self.powered),
        ]
        return angles, feedback

    def run(
        self,
        take_commands: Callable[[], list[str]],
        write_feedback: Callable[[str], None],
        write_joint_feedback: Callable[[list[float]], None],
        write_scene_state: Callable[[], None] | None = None,
        publish_camera: Callable[[], None] | None = None,
        camera_rate_hz: float = 10.0,
        publish_visualization: Callable[[], None] | None = None,
        visualization_rate_hz: float = 25.0,
        viewer=None,
        duration_s: float | None = None,
    ) -> None:
        started = time.monotonic()
        next_step = started
        realtime = bool(self.config["simulation"].get("realtime", True))
        command_period = 1.0 / float(
            self.config["controller"]["command_rate_limit_hz"]
        )
        pending_payloads: deque[str] = deque()
        last_scene_state_time = -np.inf
        scene_state_period = 1.0 / float(
            self.config.get("scene_state", {}).get("publish_rate_hz", 25.0)
        )
        last_camera_time = -np.inf
        camera_period = 1.0 / camera_rate_hz
        last_visualization_time = -np.inf
        visualization_period = 1.0 / visualization_rate_hz
        last_viewer_sync_time = -np.inf
        viewer_sync_period = 1.0 / float(
            self.config["simulation"].get("viewer_sync_rate_hz", 60.0)
        )

        while duration_s is None or time.monotonic() - started < duration_s:
            if viewer is not None and not viewer.is_running():
                break
            for payload in take_commands():
                pending_payloads.append(payload)

            now = time.monotonic()
            if (
                pending_payloads
                and now - self.last_command_time >= command_period
            ):
                _, responses = self.handle_payload(pending_payloads.popleft())
                for response in responses:
                    write_feedback(response)
                self.last_command_time = now

            self.step()
            if self.feedback_due(now):
                angles, payloads = self.feedback_payloads()
                write_joint_feedback(angles)
                for payload in payloads:
                    write_feedback(payload)

            if (
                write_scene_state is not None
                and now - last_scene_state_time >= scene_state_period
            ):
                write_scene_state()
                last_scene_state_time = now

            if (
                publish_camera is not None
                and now - last_camera_time >= camera_period
            ):
                publish_camera()
                last_camera_time = now

            if (
                publish_visualization is not None
                and now - last_visualization_time >= visualization_period
            ):
                publish_visualization()
                last_visualization_time = now

            if (
                viewer is not None
                and now - last_viewer_sync_time >= viewer_sync_period
            ):
                viewer.sync()
                last_viewer_sync_time = now
            if realtime:
                next_step += self.model.opt.timestep
                delay = next_step - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -0.25:
                    next_step = time.monotonic()

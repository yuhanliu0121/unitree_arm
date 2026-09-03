#!/usr/bin/env python3
"""Temporary loopback proxy that freezes the physical D1 Joint6 command.

This is a commissioning aid for an arm with an unavailable gripper actuator.
It must never be used to claim that a physical grasp or release succeeded.
"""

import argparse
import math
import selectors
import signal
import socket
import statistics
import struct
import sys
from collections import deque


PACKET_MAGIC = 0x44314350
PACKET_VERSION = 2
PACKET_COMMAND = 1
PACKET_FEEDBACK = 2
PACKET = struct.Struct("<IHHQIIII7d")


class Joint6BypassState:
    def __init__(
        self,
        sample_count=5,
        stability_deg=0.3,
        retention_margin_deg=4.0,
        arm_stationary_deg=2.5,
        open_angle_deg=60.0,
    ):
        self.samples = deque(maxlen=sample_count)
        self.stability_deg = stability_deg
        self.retention_margin_deg = retention_margin_deg
        self.arm_stationary_deg = arm_stationary_deg
        self.open_angle_deg = open_angle_deg
        self.frozen_deg = None
        self.virtual_deg = None
        self.latest_arm_deg = None

    def observe_feedback(self, angles_deg):
        values = tuple(float(value) for value in angles_deg)
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            raise ValueError("feedback does not contain seven finite joint angles")
        self.latest_arm_deg = values[:6]
        physical_joint6 = values[6]
        if self.frozen_deg is None:
            self.samples.append(physical_joint6)
            if len(self.samples) == self.samples.maxlen:
                spread = max(self.samples) - min(self.samples)
                if spread <= self.stability_deg:
                    self.frozen_deg = statistics.median(self.samples)
                    self.virtual_deg = self.frozen_deg
        output = list(values)
        if self.virtual_deg is not None:
            output[6] = self.virtual_deg
        return tuple(output)

    def rewrite_command(self, angles_deg):
        if self.frozen_deg is None or self.latest_arm_deg is None:
            raise RuntimeError("Joint6 stable-angle latch is not ready")
        values = tuple(float(value) for value in angles_deg)
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            raise ValueError("command does not contain seven finite joint angles")
        maximum_arm_delta = max(
            abs(values[index] - self.latest_arm_deg[index]) for index in range(6)
        )
        simulated_gripper = maximum_arm_delta <= self.arm_stationary_deg
        if simulated_gripper:
            requested = values[6]
            if requested >= self.open_angle_deg - 1.0:
                self.virtual_deg = min(requested, self.open_angle_deg)
            else:
                self.virtual_deg = min(
                    self.open_angle_deg, requested + self.retention_margin_deg
                )
        output = list(values)
        output[6] = self.frozen_deg
        return tuple(output), simulated_gripper


def unpack_packet(data):
    if len(data) != PACKET.size:
        raise ValueError(f"packet size {len(data)} does not equal {PACKET.size}")
    values = list(PACKET.unpack(data))
    if values[0] != PACKET_MAGIC or values[1] != PACKET_VERSION:
        raise ValueError("invalid packet header")
    return values


def replace_angles(values, angles_deg):
    values[8:15] = angles_deg
    return PACKET.pack(*values)


def make_socket(bind_port=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    if bind_port is not None:
        sock.bind(("127.0.0.1", bind_port))
    return sock


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--command-listen-port", type=int, required=True)
    parser.add_argument("--command-forward-port", type=int, required=True)
    parser.add_argument("--feedback-listen-port", type=int, required=True)
    parser.add_argument("--feedback-forward-port", type=int, required=True)
    parser.add_argument("--sample-count", type=int, default=5)
    parser.add_argument("--stability-deg", type=float, default=0.3)
    parser.add_argument("--retention-margin-deg", type=float, default=4.0)
    parser.add_argument("--arm-stationary-deg", type=float, default=2.5)
    parser.add_argument("--open-angle-deg", type=float, default=60.0)
    options = parser.parse_args()
    ports = (
        options.command_listen_port,
        options.command_forward_port,
        options.feedback_listen_port,
        options.feedback_forward_port,
    )
    if any(port <= 0 or port > 65535 for port in ports) or len(set(ports)) != 4:
        parser.error("all four ports must be distinct values in [1, 65535]")
    if options.sample_count < 3 or options.stability_deg <= 0.0:
        parser.error("sample-count must be at least 3 and stability-deg must be positive")
    if options.retention_margin_deg <= 0.0 or options.arm_stationary_deg <= 0.0:
        parser.error("retention and stationary thresholds must be positive")
    return options


def main():
    options = parse_args()
    state = Joint6BypassState(
        sample_count=options.sample_count,
        stability_deg=options.stability_deg,
        retention_margin_deg=options.retention_margin_deg,
        arm_stationary_deg=options.arm_stationary_deg,
        open_angle_deg=options.open_angle_deg,
    )
    command_in = make_socket(options.command_listen_port)
    feedback_in = make_socket(options.feedback_listen_port)
    forward = make_socket()
    selector = selectors.DefaultSelector()
    selector.register(command_in, selectors.EVENT_READ, "command")
    selector.register(feedback_in, selectors.EVENT_READ, "feedback")
    stopping = False
    pending_command = None

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(
        "*** JOINT6 BYPASS ACTIVE: physical Joint6 will be frozen; "
        "grasp/release results are simulated and are not physical validation ***",
        flush=True,
    )

    def forward_command(data):
        values = unpack_packet(data)
        if values[2] != PACKET_COMMAND:
            raise ValueError(f"unexpected command-side packet kind {values[2]}")
        rewritten, simulated_gripper = state.rewrite_command(values[8:15])
        requested_joint6 = values[14]
        forward.sendto(
            replace_angles(values, rewritten),
            ("127.0.0.1", options.command_forward_port),
        )
        print(
            "JOINT6_BYPASS command forwarded: "
            f"requested={requested_joint6:.2f}deg physical={state.frozen_deg:.2f}deg "
            f"virtual={state.virtual_deg:.2f}deg "
            f"kind={'virtual_gripper' if simulated_gripper else 'arm_motion'}",
            flush=True,
        )

    try:
        while not stopping:
            for key, _ in selector.select(timeout=0.2):
                data, _ = key.fileobj.recvfrom(PACKET.size + 1)
                try:
                    if key.data == "feedback":
                        values = unpack_packet(data)
                        if values[2] != PACKET_FEEDBACK:
                            raise ValueError(
                                f"unexpected feedback-side packet kind {values[2]}"
                            )
                        was_ready = state.frozen_deg is not None
                        rewritten = state.observe_feedback(values[8:15])
                        forward.sendto(
                            replace_angles(values, rewritten),
                            ("127.0.0.1", options.feedback_forward_port),
                        )
                        if not was_ready and state.frozen_deg is not None:
                            print(
                                "*** JOINT6 BYPASS READY: "
                                f"latched physical angle={state.frozen_deg:.2f}deg ***",
                                flush=True,
                            )
                            if pending_command is not None:
                                forward_command(pending_command)
                                pending_command = None
                    else:
                        if state.frozen_deg is None:
                            pending_command = data
                            print(
                                "JOINT6_BYPASS buffered command while collecting "
                                "stable physical feedback",
                                flush=True,
                            )
                        else:
                            forward_command(data)
                except (RuntimeError, ValueError) as error:
                    print(f"JOINT6_BYPASS rejected packet: {error}", file=sys.stderr, flush=True)
    finally:
        selector.close()
        command_in.close()
        feedback_in.close()
        forward.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

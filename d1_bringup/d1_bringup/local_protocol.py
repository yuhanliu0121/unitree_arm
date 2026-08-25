"""Decoder for the loopback JointPacket shared with d1-streaming-control."""

import struct
from typing import Tuple


PACKET_MAGIC = 0x44314350
PACKET_VERSION = 2
PACKET_FEEDBACK = 2
PACKET_STATUS = 3

# C++ JointPacket v2 layout: header/profile fields followed by seven angles.
PACKET = struct.Struct("<IHHQIIII7d")


def decode_joint_packet(data: bytes) -> Tuple[int, Tuple[float, ...]]:
    if len(data) != PACKET.size:
        raise ValueError(f"invalid JointPacket size {len(data)} (expected {PACKET.size})")
    unpacked = PACKET.unpack(data)
    if unpacked[0:2] != (PACKET_MAGIC, PACKET_VERSION):
        raise ValueError("invalid JointPacket header")
    return int(unpacked[2]), tuple(float(value) for value in unpacked[8:15])

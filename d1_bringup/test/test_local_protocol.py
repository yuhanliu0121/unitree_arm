import struct

import pytest

from d1_bringup.local_protocol import (
    PACKET,
    PACKET_FEEDBACK,
    PACKET_MAGIC,
    PACKET_VERSION,
    decode_joint_packet,
)


def test_joint_packet_v2_matches_cpp_layout_and_preserves_angles():
    angles = (-8.2, -88.2, 91.0, -1.5, -8.5, -0.6, 58.9)
    payload = PACKET.pack(
        PACKET_MAGIC, PACKET_VERSION, PACKET_FEEDBACK, 42,
        2, 10806, 800, 800, *angles,
    )

    assert PACKET.size == 88
    packet_kind, decoded = decode_joint_packet(payload)
    assert packet_kind == PACKET_FEEDBACK
    assert decoded == pytest.approx(angles)


def test_legacy_80_byte_packet_is_rejected():
    legacy = struct.pack(
        "<IHHQII7d", PACKET_MAGIC, 1, PACKET_FEEDBACK, 42, 0, 0,
        *([0.0] * 7),
    )

    with pytest.raises(ValueError, match="size"):
        decode_joint_packet(legacy)

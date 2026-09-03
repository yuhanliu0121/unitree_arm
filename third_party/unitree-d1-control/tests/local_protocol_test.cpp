#include "d1_control/local_protocol.hpp"
#include "d1_control/joint_interval.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>

int main()
{
  using d1_control::JointPacket;
  using d1_control::PacketKind;
  using d1_control::enforceMinimumJointInterval;
  using d1_control::kD1JointCount;
  using d1_control::packet_header_is_valid;
  using d1_control::packet_is_valid;

  JointPacket packet;
  packet.kind = PacketKind::command;
  packet.sequence = 42U;
  packet.duration_ms = 50U;
  packet.acceleration_ms = 10U;
  packet.deceleration_ms = 15U;
  for (std::size_t index = 0; index < kD1JointCount; ++index)
  {
    packet.angle_deg[index] = static_cast<double>(index) - 3.0;
  }

  if (sizeof(packet) != 88U || !packet_header_is_valid(packet) ||
    !packet_is_valid(packet, PacketKind::command) ||
    packet_is_valid(packet, PacketKind::feedback) ||
    packet.sequence != 42U || packet.duration_ms != 50U ||
    packet.acceleration_ms != 10U || packet.deceleration_ms != 15U ||
    std::abs(packet.angle_deg[6] - 3.0) > 1e-12)
  {
    std::cerr << "local protocol contract failed" << std::endl;
    return 1;
  }
  if (enforceMinimumJointInterval(1U, 10U) != 10U ||
    enforceMinimumJointInterval(9U, 10U) != 10U ||
    enforceMinimumJointInterval(10U, 10U) != 10U ||
    enforceMinimumJointInterval(33U, 10U) != 33U)
  {
    std::cerr << "minimum joint interval policy failed" << std::endl;
    return 1;
  }
  return 0;
}

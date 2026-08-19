#ifndef D1_ROS2_CONTROL__LOCAL_PROTOCOL_HPP_
#define D1_ROS2_CONTROL__LOCAL_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <type_traits>

namespace d1_ros2_control
{

constexpr std::uint32_t kLocalProtocolMagic = 0x44314350U;  // "D1CP"
constexpr std::uint16_t kLocalProtocolVersion = 1U;
constexpr std::size_t kD1JointCount = 7U;

enum class PacketKind : std::uint16_t
{
  command = 1U,
  feedback = 2U,
  status = 3U,
  power_command = 4U,
  enable_command = 5U,
};

struct JointPacket
{
  std::uint32_t magic{kLocalProtocolMagic};
  std::uint16_t version{kLocalProtocolVersion};
  PacketKind kind{PacketKind::command};
  std::uint64_t sequence{0U};
  std::uint32_t smoothing_mode{0U};
  std::uint32_t reserved{0U};
  std::array<double, kD1JointCount> angle_deg{};
};

static_assert(std::is_trivially_copyable<JointPacket>::value);
static_assert(sizeof(JointPacket) == 80U);

inline bool packet_is_valid(const JointPacket & packet, PacketKind expected_kind)
{
  return packet.magic == kLocalProtocolMagic &&
         packet.version == kLocalProtocolVersion &&
         packet.kind == expected_kind;
}

inline bool packet_header_is_valid(const JointPacket & packet)
{
  return packet.magic == kLocalProtocolMagic &&
         packet.version == kLocalProtocolVersion;
}

}  // namespace d1_ros2_control

#endif  // D1_ROS2_CONTROL__LOCAL_PROTOCOL_HPP_

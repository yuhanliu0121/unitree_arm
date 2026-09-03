#ifndef D1_CONTROL__LOCAL_PROTOCOL_HPP_
#define D1_CONTROL__LOCAL_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <type_traits>

namespace d1_control
{

constexpr std::uint32_t kLocalProtocolMagic = 0x44314350U;  // "D1CP"
constexpr std::uint16_t kLocalProtocolVersion = 2U;
constexpr std::size_t kD1JointCount = 7U;

enum class MotionProfile : std::uint32_t
{
  legacy_unsmoothed = 0U,
  legacy_vendor_smooth = 1U,
  common_arrival = 2U,
  uniform_joint_speed = 3U,
};

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
  // Requested actuator interpolation interval.  A value of zero preserves
  // the legacy funcode transport semantics used by the simulator.
  std::uint32_t duration_ms{0U};
  // Native long-segment profile. These fields are used when smoothing_mode
  // is common_arrival or uniform_joint_speed. For uniform_joint_speed,
  // duration_ms is the estimated duration of the joint with the largest
  // displacement; the onboard executor derives one nominal angular velocity
  // from it and converts every joint displacement to the delay accepted by the
  // vendor mode-1 setRawAngle() primitive.
  std::uint32_t acceleration_ms{0U};
  std::uint32_t deceleration_ms{0U};
  std::array<double, kD1JointCount> angle_deg{};
};

static_assert(std::is_trivially_copyable<JointPacket>::value);
static_assert(sizeof(JointPacket) == 88U);

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

}  // namespace d1_control

#endif  // D1_CONTROL__LOCAL_PROTOCOL_HPP_

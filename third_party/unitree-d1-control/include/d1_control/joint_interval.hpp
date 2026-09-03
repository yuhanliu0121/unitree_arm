#pragma once

#include <algorithm>
#include <cstdint>

namespace d1_control
{

constexpr std::uint16_t enforceMinimumJointInterval(
  const std::uint16_t requested_ms, const std::uint16_t minimum_ms)
{
  return std::max(requested_ms, minimum_ms);
}

}  // namespace d1_control

#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace d1_manipulation
{
enum class ArmTaskState : std::uint8_t
{
  INITIALIZING = 0,
  READY_STOWED = 1,
  READY_CARRY = 2,
  PICKING = 3,
  DROPPING = 4,
  RECOVERING_TO_STOWED = 5,
  RECOVERING_TO_CARRY = 6,
  FAULTED = 7,
};

enum class PayloadState : std::uint8_t
{
  EMPTY = 0,
  HELD = 1,
  UNKNOWN = 2,
};

enum class CanonicalPose : std::uint8_t
{
  STOWED = 0,
  CARRY = 1,
  OTHER = 2,
  UNKNOWN = 3,
};

enum class ArmTaskEvent : std::uint8_t
{
  INITIALIZATION_SUCCEEDED = 0,
  INITIALIZATION_FAILED = 1,
  START_PICK = 2,
  PICK_SUCCEEDED = 3,
  PICK_REPOSITION_REQUIRED = 4,
  START_DROP = 5,
  DROP_SUCCEEDED = 6,
  DROP_REPOSITION_REQUIRED = 7,
  RECOVERY_SUCCEEDED = 8,
  UPDATE_PHASE = 9,
  FAULT = 10,
};

struct ArmTaskSnapshot
{
  ArmTaskState state{ArmTaskState::INITIALIZING};
  PayloadState payload{PayloadState::UNKNOWN};
  CanonicalPose pose{CanonicalPose::UNKNOWN};
};

class ArmTaskStateMachine
{
public:
  ArmTaskStateMachine();
  ~ArmTaskStateMachine();
  ArmTaskStateMachine(ArmTaskStateMachine&&) noexcept;
  ArmTaskStateMachine& operator=(ArmTaskStateMachine&&) noexcept;
  ArmTaskStateMachine(const ArmTaskStateMachine&) = delete;
  ArmTaskStateMachine& operator=(const ArmTaskStateMachine&) = delete;

  bool process(ArmTaskEvent event);
  ArmTaskSnapshot snapshot() const;
  static const char* name(ArmTaskState state);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace d1_manipulation

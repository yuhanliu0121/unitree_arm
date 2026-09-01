#include <gtest/gtest.h>

#include "d1_manipulation/arm_task_state_machine.hpp"

namespace d1_manipulation
{
TEST(ArmTaskStateMachine, PickAndDropHappyPath)
{
  ArmTaskStateMachine machine;
  EXPECT_EQ(machine.snapshot().state, ArmTaskState::INITIALIZING);
  EXPECT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
  EXPECT_TRUE(machine.process(ArmTaskEvent::START_PICK));
  EXPECT_TRUE(machine.process(ArmTaskEvent::PICK_SUCCEEDED));
  EXPECT_EQ(machine.snapshot().state, ArmTaskState::READY_CARRY);
  EXPECT_EQ(machine.snapshot().payload, PayloadState::HELD);
  EXPECT_TRUE(machine.process(ArmTaskEvent::START_DROP));
  EXPECT_TRUE(machine.process(ArmTaskEvent::DROP_SUCCEEDED));
  EXPECT_EQ(machine.snapshot().state, ArmTaskState::READY_STOWED);
  EXPECT_EQ(machine.snapshot().payload, PayloadState::EMPTY);
}

TEST(ArmTaskStateMachine, RecoverableFailuresReachCanonicalReadyStates)
{
  ArmTaskStateMachine machine;
  ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
  ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
  ASSERT_TRUE(machine.process(ArmTaskEvent::PICK_REPOSITION_REQUIRED));
  EXPECT_EQ(machine.snapshot().state, ArmTaskState::RECOVERING_TO_STOWED);
  ASSERT_TRUE(machine.process(ArmTaskEvent::RECOVERY_SUCCEEDED));
  EXPECT_EQ(machine.snapshot().state, ArmTaskState::READY_STOWED);

  ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
  ASSERT_TRUE(machine.process(ArmTaskEvent::PICK_SUCCEEDED));
  ASSERT_TRUE(machine.process(ArmTaskEvent::START_DROP));
  ASSERT_TRUE(machine.process(ArmTaskEvent::DROP_REPOSITION_REQUIRED));
  EXPECT_EQ(machine.snapshot().state, ArmTaskState::RECOVERING_TO_CARRY);
  ASSERT_TRUE(machine.process(ArmTaskEvent::RECOVERY_SUCCEEDED));
  EXPECT_EQ(machine.snapshot().state, ArmTaskState::READY_CARRY);
}

TEST(ArmTaskStateMachine, RejectsInvalidAndConcurrentOperations)
{
  ArmTaskStateMachine machine;
  ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
  EXPECT_FALSE(machine.process(ArmTaskEvent::START_DROP));
  ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
  EXPECT_FALSE(machine.process(ArmTaskEvent::START_PICK));
  EXPECT_FALSE(machine.process(ArmTaskEvent::START_DROP));
  EXPECT_EQ(machine.snapshot().state, ArmTaskState::PICKING);
}

TEST(ArmTaskStateMachine, PhaseUpdatesAreAcceptedOnlyWhileWorkIsActive)
{
  ArmTaskStateMachine machine;
  EXPECT_FALSE(machine.process(ArmTaskEvent::UPDATE_PHASE));
  ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
  EXPECT_FALSE(machine.process(ArmTaskEvent::UPDATE_PHASE));
  ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
  EXPECT_TRUE(machine.process(ArmTaskEvent::UPDATE_PHASE));
  ASSERT_TRUE(machine.process(ArmTaskEvent::PICK_REPOSITION_REQUIRED));
  EXPECT_TRUE(machine.process(ArmTaskEvent::UPDATE_PHASE));
  ASSERT_TRUE(machine.process(ArmTaskEvent::FAULT));
  EXPECT_FALSE(machine.process(ArmTaskEvent::UPDATE_PHASE));
}

TEST(ArmTaskStateMachine, EveryOperationalStateCanFault)
{
  const auto expect_fault = [](auto prepare) {
      ArmTaskStateMachine machine;
      prepare(machine);
      EXPECT_TRUE(machine.process(ArmTaskEvent::FAULT));
      EXPECT_EQ(machine.snapshot().state, ArmTaskState::FAULTED);
      EXPECT_EQ(machine.snapshot().payload, PayloadState::UNKNOWN);
      EXPECT_EQ(machine.snapshot().pose, CanonicalPose::UNKNOWN);
      EXPECT_FALSE(machine.process(ArmTaskEvent::START_PICK));
      EXPECT_FALSE(machine.process(ArmTaskEvent::START_DROP));
    };
  expect_fault([](auto&) {});
  expect_fault([](auto& machine) {ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));});
  expect_fault([](auto& machine) {
      ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
      ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
    });
  expect_fault([](auto& machine) {
      ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
      ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
      ASSERT_TRUE(machine.process(ArmTaskEvent::PICK_REPOSITION_REQUIRED));
    });
  expect_fault([](auto& machine) {
      ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
      ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
      ASSERT_TRUE(machine.process(ArmTaskEvent::PICK_SUCCEEDED));
    });
  expect_fault([](auto& machine) {
      ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
      ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
      ASSERT_TRUE(machine.process(ArmTaskEvent::PICK_SUCCEEDED));
      ASSERT_TRUE(machine.process(ArmTaskEvent::START_DROP));
    });
  expect_fault([](auto& machine) {
      ASSERT_TRUE(machine.process(ArmTaskEvent::INITIALIZATION_SUCCEEDED));
      ASSERT_TRUE(machine.process(ArmTaskEvent::START_PICK));
      ASSERT_TRUE(machine.process(ArmTaskEvent::PICK_SUCCEEDED));
      ASSERT_TRUE(machine.process(ArmTaskEvent::START_DROP));
      ASSERT_TRUE(machine.process(ArmTaskEvent::DROP_REPOSITION_REQUIRED));
    });
}
}  // namespace d1_manipulation

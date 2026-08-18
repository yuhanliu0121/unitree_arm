#include <gtest/gtest.h>

#include <string>

#include <unistd.h>

#include "d1_manipulation/task_lock.hpp"

TEST(TaskLock, ExcludesAnotherServerAndReleasesCleanly)
{
  const std::string path = "/tmp/d1_task_lock_test_" + std::to_string(::getpid());
  d1_manipulation::TaskLock pick(path);
  d1_manipulation::TaskLock drop(path);

  ASSERT_TRUE(pick.tryAcquire());
  EXPECT_FALSE(drop.tryAcquire());
  pick.release();
  EXPECT_TRUE(drop.tryAcquire());
  drop.release();
  ::unlink(path.c_str());
}

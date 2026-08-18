#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <mutex>
#include <string>
#include <utility>

namespace d1_manipulation
{

// Cross-process, process-death-safe mutual exclusion for public arm tasks.
// Linux releases the advisory lock automatically if a server process exits.
class TaskLock
{
public:
  explicit TaskLock(std::string path) : path_(std::move(path)) {}

  ~TaskLock() { release(); }

  TaskLock(const TaskLock&) = delete;
  TaskLock& operator=(const TaskLock&) = delete;

  bool tryAcquire()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (fd_ >= 0) return false;
    const int candidate = ::open(path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0660);
    if (candidate < 0) return false;
    if (::flock(candidate, LOCK_EX | LOCK_NB) != 0) {
      ::close(candidate);
      return false;
    }
    fd_ = candidate;
    return true;
  }

  void release()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    if (fd_ < 0) return;
    ::flock(fd_, LOCK_UN);
    ::close(fd_);
    fd_ = -1;
  }

private:
  std::string path_;
  std::mutex mutex_;
  int fd_{-1};
};

}  // namespace d1_manipulation

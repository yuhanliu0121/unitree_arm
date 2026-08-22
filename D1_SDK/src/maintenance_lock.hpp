#ifndef D1_SDK__MAINTENANCE_LOCK_HPP_
#define D1_SDK__MAINTENANCE_LOCK_HPP_

#include <sys/file.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace d1_tools
{

class ExclusiveHardwareLease
{
public:
    explicit ExclusiveHardwareLease(const std::string& tool_name)
      : tool_name_(tool_name)
    {
        fd_ = ::open(kLockPath, O_RDWR | O_CREAT, 0666);
        if (fd_ < 0)
        {
            throw std::runtime_error(
                std::string("Cannot open D1 ownership lock: ") + std::strerror(errno));
        }
        if (!TryAcquire())
        {
            const auto owner = ReadOwner();
            const auto owner_pid = RealBringupPid(owner);
            if (owner_pid <= 1)
            {
                throw std::runtime_error(
                    "D1 command ownership is held by another maintenance command: " + owner);
            }
            std::cout << "Stopping active real_bringup PID " << owner_pid
                      << " before direct SDK maintenance...\n" << std::flush;
            if (::kill(owner_pid, SIGINT) != 0 && errno != ESRCH)
            {
                throw std::runtime_error(
                    std::string("Failed to stop real_bringup: ") + std::strerror(errno));
            }
            if (!WaitForLease(std::chrono::seconds(12)))
            {
                std::cerr << "real_bringup did not release ownership after SIGINT; "
                             "sending SIGTERM.\n";
                if (::kill(owner_pid, SIGTERM) != 0 && errno != ESRCH)
                {
                    throw std::runtime_error(
                        std::string("Failed to terminate real_bringup: ") +
                        std::strerror(errno));
                }
                if (!WaitForLease(std::chrono::seconds(8)))
                {
                    throw std::runtime_error(
                        "real_bringup did not release D1 ownership; direct command refused");
                }
            }
            std::cout << "Physical ROS control stack stopped; direct SDK ownership acquired.\n";
        }
        WriteOwner();
    }

    ~ExclusiveHardwareLease()
    {
        if (fd_ >= 0)
        {
            ::flock(fd_, LOCK_UN);
            ::close(fd_);
        }
    }

    ExclusiveHardwareLease(const ExclusiveHardwareLease&) = delete;
    ExclusiveHardwareLease& operator=(const ExclusiveHardwareLease&) = delete;

private:
    static constexpr const char* kLockPath = "/tmp/d1_real_bringup.lock";

    bool TryAcquire()
    {
        while (::flock(fd_, LOCK_EX | LOCK_NB) != 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                return false;
            }
            throw std::runtime_error(
                std::string("Cannot acquire D1 ownership lock: ") + std::strerror(errno));
        }
        return true;
    }

    bool WaitForLease(const std::chrono::steady_clock::duration timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline)
        {
            if (TryAcquire())
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return TryAcquire();
    }

    std::string ReadOwner() const
    {
        char buffer[256]{};
        const auto count = ::pread(fd_, buffer, sizeof(buffer) - 1, 0);
        if (count <= 0)
        {
            return "unknown owner";
        }
        std::string owner(buffer, static_cast<std::size_t>(count));
        const auto newline = owner.find_first_of("\r\n");
        if (newline != std::string::npos)
        {
            owner.resize(newline);
        }
        return owner;
    }

    static pid_t RealBringupPid(const std::string& owner)
    {
        std::string pid_text;
        constexpr const char prefix[] = "real_bringup ";
        if (owner.rfind(prefix, 0) == 0)
        {
            pid_text = owner.substr(sizeof(prefix) - 1);
        }
        else if (!owner.empty() && owner.find_first_not_of("0123456789") == std::string::npos)
        {
            // Backward compatibility with lock files written before ownership labels.
            pid_text = owner;
        }
        else
        {
            return -1;
        }
        char* end = nullptr;
        const long parsed = std::strtol(pid_text.c_str(), &end, 10);
        if (end == pid_text.c_str() || *end != '\0' || parsed <= 1)
        {
            return -1;
        }
        return static_cast<pid_t>(parsed);
    }

    void WriteOwner()
    {
        const std::string owner = "maintenance " + tool_name_ + " " +
            std::to_string(static_cast<long>(::getpid())) + "\n";
        if (::ftruncate(fd_, 0) != 0 || ::pwrite(fd_, owner.data(), owner.size(), 0) < 0)
        {
            throw std::runtime_error(
                std::string("Cannot record D1 ownership: ") + std::strerror(errno));
        }
    }

    std::string tool_name_;
    int fd_{-1};
};

}  // namespace d1_tools

#endif  // D1_SDK__MAINTENANCE_LOCK_HPP_

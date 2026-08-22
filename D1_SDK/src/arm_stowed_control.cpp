#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace
{

constexpr char kCommandTopic[] = "rt/arm_Command";
constexpr char kAngleTopic[] = "current_servo_angle";
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr double kOpenGripperDegrees = 60.0;
constexpr std::array<double, 6> kStowedRadians{0.0, -1.54, 1.55, 0.0, 0.0, 0.0};

struct AngleState
{
    std::mutex mutex;
    std::condition_variable changed;
    std::array<double, 7> angles{};
    std::uint64_t sequence = 0;
};

AngleState g_state;

void HandleAngles(const void* raw_message)
{
    const auto* message =
        static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw_message);
    const std::array<double, 7> angles{
        message->servo0_data_(),
        message->servo1_data_(),
        message->servo2_data_(),
        message->servo3_data_(),
        message->servo4_data_(),
        message->servo5_data_(),
        message->servo6_data_(),
    };
    if (!std::all_of(
            angles.begin(), angles.end(),
            [](const double value) { return std::isfinite(value); }))
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(g_state.mutex);
        g_state.angles = angles;
        ++g_state.sequence;
    }
    g_state.changed.notify_all();
}

class Arguments
{
public:
    Arguments(int argc, char** argv)
    {
        for (int index = 1; index < argc; ++index)
        {
            const std::string name = argv[index];
            if (name == "-h" || name == "--help")
            {
                help_ = true;
                continue;
            }
            if (name.rfind("--", 0) != 0 || index + 1 >= argc)
            {
                throw std::runtime_error("Invalid argument: " + name);
            }
            values_[name] = argv[++index];
        }
    }

    bool help() const { return help_; }

    std::string get(const std::string& name, const std::string& fallback = "") const
    {
        const auto found = values_.find(name);
        return found == values_.end() ? fallback : found->second;
    }

private:
    bool help_ = false;
    std::unordered_map<std::string, std::string> values_;
};

double ParseDouble(const std::string& text, const char* name)
{
    std::size_t parsed = 0;
    const double value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value))
    {
        throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
    }
    return value;
}

int ParseInt(const std::string& text, const char* name)
{
    std::size_t parsed = 0;
    const int value = std::stoi(text, &parsed);
    if (parsed != text.size())
    {
        throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
    }
    return value;
}

void PrintUsage(const char* program)
{
    std::cout
        << "Usage: " << program
        << " [--interface IFACE] [--timeout SEC] [--tolerance-deg DEG]"
        << " --confirm STOWED_MOVE\n"
        << "Moves D1 Joint0..5 to [0, -1.54, 1.55, 0, 0, 0] radians with\n"
        << "one complete funcode=2 command and fully opens Joint6 to 60 degrees.\n"
        << "Default interface: enp3s0. Stop every other D1 command publisher first.\n";
}

std::array<double, 7> WaitForAngles(const std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(g_state.mutex);
    if (!g_state.changed.wait_for(lock, timeout, [] { return g_state.sequence > 0; }))
    {
        throw std::runtime_error("No current_servo_angle feedback before timeout");
    }
    return g_state.angles;
}

std::array<double, 7> StowedTarget()
{
    std::array<double, 7> target{};
    for (std::size_t index = 0; index < kStowedRadians.size(); ++index)
    {
        target[index] = kStowedRadians[index] * kRadiansToDegrees;
    }
    target[6] = kOpenGripperDegrees;
    return target;
}

double MaximumJointError(
    const std::array<double, 7>& actual,
    const std::array<double, 7>& target)
{
    double maximum = 0.0;
    for (std::size_t index = 0; index < 7; ++index)
    {
        maximum = std::max(maximum, std::abs(actual[index] - target[index]));
    }
    return maximum;
}

std::array<double, 7> ReadAnglesFile(const std::string& path)
{
    std::ifstream input(path);
    std::array<double, 7> angles{};
    for (double& angle : angles)
    {
        if (!(input >> angle) || !std::isfinite(angle))
        {
            throw std::runtime_error("Invalid measured-angle file: " + path);
        }
    }
    return angles;
}

int MeasurePhase(const std::string& interface, const std::string& state_file)
{
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>
        subscriber(kAngleTopic);
    subscriber.InitChannel(HandleAngles);
    const auto current = WaitForAngles(std::chrono::seconds(5));
    std::ofstream output(state_file, std::ios::trunc);
    if (!output)
    {
        throw std::runtime_error("Cannot write measured-angle file: " + state_file);
    }
    output << std::setprecision(17);
    for (const double angle : current)
    {
        output << angle << '\n';
    }
    return output ? 0 : 1;
}

int CommandPhase(
    const std::string& interface,
    const std::string& state_file,
    const int sequence,
    const double tolerance_deg)
{
    const auto current = ReadAnglesFile(state_file);
    if (current[6] < -31.0 || current[6] > 63.5)
    {
        throw std::runtime_error("Measured Joint6 is outside the characterized range");
    }

    const auto target = StowedTarget();
    const double maximum_delta = MaximumJointError(current, target);
    if (maximum_delta <= tolerance_deg)
    {
        std::cout << "Arm and gripper are already within " << tolerance_deg
                  << " deg of STOWED; no command sent.\n";
        return 0;
    }

    // D1095 ignored a mode=1 correction whose largest error was 3.7 deg.
    // Use the characterized, faster small-motion mode only for final settling.
    const int mode = maximum_delta <= 5.0 ? 0 : 1;
    std::ostringstream json;
    json << std::fixed << std::setprecision(7)
         << "{\"seq\":" << sequence
         << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":" << mode;
    for (std::size_t index = 0; index < target.size(); ++index)
    {
        json << ",\"angle" << index << "\":" << target[index];
    }
    json << "}}";

    std::cout << std::fixed << std::setprecision(3)
              << "Current arm angles: [";
    for (std::size_t index = 0; index < 6; ++index)
    {
        std::cout << (index == 0 ? "" : ", ") << current[index];
    }
    std::cout << "] deg\nMaximum delta: " << maximum_delta
              << " deg; selected mode=" << mode
              << "; opening Joint6 to " << kOpenGripperDegrees << " deg\n";

    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>
        publisher(kCommandTopic);
    publisher.InitChannel();
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    unitree_arm::msg::dds_::ArmString_ command{};
    command.data_() = json.str();
    if (!publisher.Write(command))
    {
        throw std::runtime_error("DDS publisher.Write returned false");
    }
    std::cout << "Sent one complete funcode=2 STOWED target.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    return 0;
}

int VerifyPhase(
    const std::string& interface,
    const double timeout_s,
    const double tolerance_deg)
{
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>
        subscriber(kAngleTopic);
    subscriber.InitChannel(HandleAngles);

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_s);
    int stable_samples = 0;
    std::array<double, 7> latest{};
    while (std::chrono::steady_clock::now() < deadline)
    {
        std::unique_lock<std::mutex> lock(g_state.mutex);
        const auto observed = g_state.sequence;
        g_state.changed.wait_for(
            lock,
            std::chrono::milliseconds(300),
            [observed] { return g_state.sequence != observed; });
        if (g_state.sequence == 0)
        {
            continue;
        }
        latest = g_state.angles;
        lock.unlock();
        const auto target = StowedTarget();
        const double error = MaximumJointError(latest, target);
        stable_samples = error <= tolerance_deg ? stable_samples + 1 : 0;
        if (stable_samples >= 3)
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "STOWED reached: max Joint0..6 error=" << error
                      << " deg; Joint6=" << latest[6] << " deg.\n";
            return 0;
        }
    }
    const auto target = StowedTarget();
    std::cerr << std::fixed << std::setprecision(3)
              << "STOWED verification timed out: max Joint0..6 error="
              << MaximumJointError(latest, target) << " deg.\n";
    return 2;
}

int RunChild(const std::string& executable, const std::vector<std::string>& arguments)
{
    const pid_t child = ::fork();
    if (child < 0)
    {
        throw std::runtime_error("fork failed");
    }
    if (child == 0)
    {
        std::vector<char*> argv;
        argv.push_back(const_cast<char*>(executable.c_str()));
        for (const auto& argument : arguments)
        {
            argv.push_back(const_cast<char*>(argument.c_str()));
        }
        argv.push_back(nullptr);
        ::execv(executable.c_str(), argv.data());
        _exit(127);
    }
    int status = 0;
    if (::waitpid(child, &status, 0) < 0)
    {
        throw std::runtime_error("waitpid failed");
    }
    return WIFEXITED(status) ? WEXITSTATUS(status) : 128;
}

}  // namespace

int main(int argc, char** argv)
{
    try
    {
        const Arguments arguments(argc, argv);
        if (arguments.help())
        {
            PrintUsage(argv[0]);
            return 0;
        }
        const std::string interface = arguments.get("--interface", "enp3s0");
        const double timeout_s = ParseDouble(arguments.get("--timeout", "25"), "timeout");
        const double tolerance_deg = ParseDouble(
            arguments.get("--tolerance-deg", "1.0"), "tolerance-deg");
        const int sequence = ParseInt(
            arguments.get(
                "--seq",
                std::to_string(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count() %
                    1000000000)),
            "seq");
        if (timeout_s <= 0.0 || tolerance_deg <= 0.0 || tolerance_deg > 2.0)
        {
            throw std::runtime_error("timeout must be positive and tolerance 0..2 deg");
        }

        const std::string phase = arguments.get("--phase");
        const std::string state_file = arguments.get("--state-file");
        if (phase == "measure")
        {
            if (state_file.empty())
            {
                throw std::runtime_error("measure phase requires --state-file");
            }
            return MeasurePhase(interface, state_file);
        }
        if (phase == "command")
        {
            if (state_file.empty())
            {
                throw std::runtime_error("command phase requires --state-file");
            }
            return CommandPhase(interface, state_file, sequence, tolerance_deg);
        }
        if (phase == "verify")
        {
            return VerifyPhase(interface, timeout_s, tolerance_deg);
        }
        if (!phase.empty())
        {
            throw std::runtime_error("Unknown internal phase: " + phase);
        }
        if (arguments.get("--confirm") != "STOWED_MOVE")
        {
            throw std::runtime_error("Publishing requires --confirm STOWED_MOVE");
        }

        std::cout
            << "WARNING: this commands the physical D1. Stop real_bringup and every\n"
            << "other D1 command publisher, support the arm, and clear its workspace.\n"
            << std::flush;
        const std::string executable = std::filesystem::absolute(argv[0]).string();
        std::array<char, 64> state_template{};
        const std::string template_text = "/tmp/d1_stowed_angles.XXXXXX";
        std::copy(template_text.begin(), template_text.end(), state_template.begin());
        const int state_fd = ::mkstemp(state_template.data());
        if (state_fd < 0)
        {
            throw std::runtime_error("mkstemp failed");
        }
        ::close(state_fd);
        const std::string state_file_path = state_template.data();
        const std::vector<std::string> common{
            "--interface", interface,
            "--timeout", std::to_string(timeout_s),
            "--tolerance-deg", std::to_string(tolerance_deg),
            "--seq", std::to_string(sequence),
            "--state-file", state_file_path,
        };
        auto measure_arguments = common;
        measure_arguments.insert(measure_arguments.end(), {"--phase", "measure"});
        const int measure_result = RunChild(executable, measure_arguments);
        if (measure_result != 0)
        {
            ::unlink(state_file_path.c_str());
            return measure_result;
        }
        auto command_arguments = common;
        command_arguments.insert(command_arguments.end(), {"--phase", "command"});
        const int command_result = RunChild(executable, command_arguments);
        if (command_result != 0)
        {
            ::unlink(state_file_path.c_str());
            return command_result;
        }
        auto verify_arguments = common;
        verify_arguments.insert(verify_arguments.end(), {"--phase", "verify"});
        const int verify_result = RunChild(executable, verify_arguments);
        ::unlink(state_file_path.c_str());
        return verify_result;
    }
    catch (const std::exception& error)
    {
        std::cerr << "ERROR: " << error.what() << "\n\n";
        PrintUsage(argv[0]);
        return 1;
    }
}

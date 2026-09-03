#include "ArmString_.hpp"
#include "PubServoInfo_.hpp"
#include "SetServoDumping_.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

using namespace std::chrono_literals;
using ArmString = unitree_arm::msg::dds_::ArmString_;
using ServoFeedback = unitree_arm::msg::dds_::PubServoInfo_;
using DampingCommand = unitree_arm::msg::dds_::SetServoDumping_;
constexpr std::size_t kJointCount = 7U;
constexpr std::array<double, kJointCount> kLower{-159.6, -88.3, -92.8, -150.3, -92.3, -158.4, -30.0};
constexpr std::array<double, kJointCount> kUpper{161.0, 92.8, 88.6, 152.1, 105.9, 159.2, 60.0};

enum class Tool {single, multiple, delta, enable};

constexpr Tool compiled_tool()
{
#if defined(D1_TOOL_SINGLE)
  return Tool::single;
#elif defined(D1_TOOL_MULTIPLE)
  return Tool::multiple;
#elif defined(D1_TOOL_DELTA)
  return Tool::delta;
#elif defined(D1_TOOL_ENABLE)
  return Tool::enable;
#else
#error "A D1 joint tool compile definition is required"
#endif
}

class Arguments
{
public:
  Arguments(int argc, char ** argv)
  {
    for (int index = 1; index < argc; ++index)
    {
      const std::string key = argv[index];
      if (key == "-h" || key == "--help") {help_ = true; continue;}
      if (key.rfind("--", 0) == 0)
      {
        if (index + 1 >= argc) throw std::runtime_error("missing value for " + key);
        values_[key] = argv[++index];
      }
      else positional_.push_back(key);
    }
  }

  bool help() const {return help_;}
  bool has(const std::string & key) const {return values_.count(key) != 0U;}
  std::string get(const std::string & key, const std::string & fallback = "") const
  {
    const auto found = values_.find(key);
    return found == values_.end() ? fallback : found->second;
  }
  const std::vector<std::string> & positional() const {return positional_;}

private:
  std::map<std::string, std::string> values_;
  std::vector<std::string> positional_;
  bool help_{false};
};

int parse_int(const std::string & text, const char * name)
{
  std::size_t consumed = 0U;
  const int value = std::stoi(text, &consumed);
  if (consumed != text.size()) throw std::runtime_error(std::string("invalid ") + name);
  return value;
}

double parse_double(const std::string & text, const char * name)
{
  std::size_t consumed = 0U;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value))
    throw std::runtime_error(std::string("invalid ") + name);
  return value;
}

std::array<double, kJointCount> parse_angles(const std::string & text)
{
  std::array<double, kJointCount> result{};
  std::istringstream stream(text);
  std::string item;
  std::size_t count = 0U;
  while (std::getline(stream, item, ','))
  {
    if (count >= result.size()) throw std::runtime_error("--angles requires 7 values");
    result[count++] = parse_double(item, "angle");
  }
  if (count != result.size()) throw std::runtime_error("--angles requires 7 values");
  return result;
}

void validate_target(const std::array<double, kJointCount> & target)
{
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    if (target[joint] < kLower[joint] || target[joint] > kUpper[joint])
    {
      std::ostringstream error;
      error << "Joint" << joint << " target " << target[joint]
            << " exceeds [" << kLower[joint] << ", " << kUpper[joint] << "] deg";
      throw std::runtime_error(error.str());
    }
  }
}

void validate_joint_target(const std::size_t joint, const double target)
{
  if (target < kLower[joint] || target > kUpper[joint])
  {
    std::ostringstream error;
    error << "Joint" << joint << " target " << target
          << " exceeds [" << kLower[joint] << ", " << kUpper[joint] << "] deg";
    throw std::runtime_error(error.str());
  }
}

class FeedbackState
{
public:
  void receive(const void * raw)
  {
    const auto & message = *static_cast<const ServoFeedback *>(raw);
    std::lock_guard<std::mutex> lock(mutex_);
    angles_ = {message.servo0_data_(), message.servo1_data_(), message.servo2_data_(),
      message.servo3_data_(), message.servo4_data_(), message.servo5_data_(),
      message.servo6_data_()};
    valid_ = std::all_of(angles_.begin(), angles_.end(), [](double value)
      {return std::isfinite(value);});
    changed_.notify_all();
  }

  std::array<double, kJointCount> wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!changed_.wait_for(lock, 3s, [this]() {return valid_;}))
      throw std::runtime_error("no current_servo_angle feedback within 3 seconds");
    return angles_;
  }

private:
  std::mutex mutex_;
  std::condition_variable changed_;
  std::array<double, kJointCount> angles_{};
  bool valid_{false};
};

std::uint32_t duration_for(
  const std::array<double, kJointCount> & current,
  const std::array<double, kJointCount> & target,
  const int requested)
{
  if (requested != 0)
  {
    if (requested < 1500 || requested > 30000)
      throw std::runtime_error("duration/delay must be 0 or in [1500, 30000] ms");
    return static_cast<std::uint32_t>(requested);
  }
  double maximum_delta = 0.0;
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
    maximum_delta = std::max(maximum_delta, std::abs(target[joint] - current[joint]));
  return std::clamp(
    static_cast<std::uint32_t>(std::ceil(maximum_delta / 15.0 * 1000.0)),
    1500U, 30000U);
}

std::string make_segment(
  const std::array<double, kJointCount> & target, const std::uint32_t duration_ms)
{
  const std::uint32_t ramp = std::min(800U,
    static_cast<std::uint32_t>(std::lround(duration_ms * 0.15)));
  const auto sequence = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  std::ostringstream output;
  output << std::fixed << std::setprecision(7)
         << "{\"seq\":" << sequence << ",\"duration_ms\":" << duration_ms
         << ",\"acceleration_ms\":" << ramp << ",\"deceleration_ms\":" << ramp
         << ",\"angles_deg\":[";
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    if (joint != 0U) output << ',';
    output << target[joint];
  }
  output << "]}";
  return output.str();
}

void send_unload()
{
  unitree::robot::ChannelPublisher<DampingCommand> publisher("set_servo_dumping");
  publisher.InitChannel();
  std::this_thread::sleep_for(700ms);
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    DampingCommand command;
    command.seq_() = static_cast<std::int32_t>(joint + 1U);
    command.id_() = static_cast<std::uint8_t>(joint);
    command.power_() = 0U;
    if (!publisher.Write(command)) throw std::runtime_error("DDS unload publish failed");
  }
  std::this_thread::sleep_for(300ms);
  publisher.CloseChannel();
}

void print_usage(const char * program, const Tool tool)
{
  switch (tool)
  {
    case Tool::single:
      std::cout << "Usage: " << program << " --joint 0..6 --angle DEG "
        "[--delay-ms 0|1500..30000] --interface IFACE --confirm MOVE\n";
      break;
    case Tool::multiple:
      std::cout << "Usage: " << program << " --angles J0,...,J6 --mode 65535 "
        "[--duration-ms 0|1500..30000] --interface IFACE --confirm MOVE\n"
        "       mode 0 requires --confirm UNLOAD_ALL.\n";
      break;
    case Tool::delta:
      std::cout << "Usage: " << program
        << " <joint 0..6> <delta_deg within 30> <legacy_mode 0|1> <interface>\n";
      break;
    case Tool::enable:
      std::cout << "Usage: " << program << " --mode 0|65535 --interface IFACE "
        "--confirm UNLOAD_ALL|ENABLE_ALL\n";
      break;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const Tool tool = compiled_tool();
  try
  {
    const Arguments arguments(argc, argv);
    if (arguments.help()) {print_usage(argv[0], tool); return EXIT_SUCCESS;}

    std::string interface = arguments.get("--interface");
    int requested_duration = 0;
    int mode = 65535;
    int joint = -1;
    double value = 0.0;
    if (tool == Tool::delta)
    {
      if (arguments.positional().size() != 4U)
        throw std::runtime_error("joint_delta_control requires four positional arguments");
      joint = parse_int(arguments.positional()[0], "joint");
      value = parse_double(arguments.positional()[1], "delta");
      mode = parse_int(arguments.positional()[2], "legacy mode");
      interface = arguments.positional()[3];
      if (std::abs(value) > 30.0 || value == 0.0)
        throw std::runtime_error("delta must satisfy 0 < abs(delta) <= 30 degrees");
      if (mode != 0 && mode != 1) throw std::runtime_error("legacy mode must be 0 or 1");
    }
    else
    {
      if (interface.empty()) throw std::runtime_error("--interface is required");
      if (tool == Tool::single)
      {
        joint = parse_int(arguments.get("--joint"), "joint");
        value = parse_double(arguments.get("--angle"), "angle");
        requested_duration = parse_int(arguments.get("--delay-ms", "0"), "delay");
        if (arguments.get("--confirm") != "MOVE")
          throw std::runtime_error("--confirm MOVE is required");
      }
      else if (tool == Tool::multiple)
      {
        mode = parse_int(arguments.get("--mode"), "mode");
        requested_duration = parse_int(
          arguments.get("--duration-ms", "0"), "duration");
        const std::string confirmation = mode == 0 ? "UNLOAD_ALL" : "MOVE";
        if (arguments.get("--confirm") != confirmation)
          throw std::runtime_error("--confirm " + confirmation + " is required");
      }
      else
      {
        mode = parse_int(arguments.get("--mode"), "mode");
        const std::string confirmation = mode == 0 ? "UNLOAD_ALL" : "ENABLE_ALL";
        if (arguments.get("--confirm") != confirmation)
          throw std::runtime_error("--confirm " + confirmation + " is required");
      }
    }
    if (joint < -1 || joint >= static_cast<int>(kJointCount))
      throw std::runtime_error("joint must be in [0, 6]");
    if (mode != 0 && mode != 1 && mode != 65535)
      throw std::runtime_error("mode must be 0, 1 or 65535 as appropriate");

    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    if ((tool == Tool::multiple || tool == Tool::enable) && mode == 0)
    {
      std::cout << "WARNING: unloading all physical joints. Support the arm.\n";
      send_unload();
      unitree::robot::ChannelFactory::Instance()->Release();
      return EXIT_SUCCESS;
    }

    FeedbackState feedback;
    unitree::robot::ChannelSubscriber<ServoFeedback> subscriber("current_servo_angle");
    subscriber.InitChannel([&feedback](const void * raw) {feedback.receive(raw);}, 8);
    auto current = feedback.wait();
    auto target = current;
    if (tool == Tool::single) target[static_cast<std::size_t>(joint)] = value;
    else if (tool == Tool::delta) target[static_cast<std::size_t>(joint)] += value;
    else if (tool == Tool::multiple) target = parse_angles(arguments.get("--angles"));
    // Do not reject an unchanged joint merely because this physical unit is
    // a fraction of a degree outside the generic URDF limit. Validate only
    // the joint changed by a single/delta command; full-pose commands must
    // still satisfy every generic limit. ENABLE_ALL holds live feedback.
    if (tool == Tool::single || tool == Tool::delta)
      validate_joint_target(static_cast<std::size_t>(joint), target[static_cast<std::size_t>(joint)]);
    else if (tool == Tool::multiple)
      validate_target(target);
    const auto duration = tool == Tool::enable ? 1500U :
      duration_for(current, target, requested_duration);
    std::cout << "Native seven-joint target duration=" << duration << " ms target_deg=[";
    for (std::size_t index = 0; index < target.size(); ++index)
    {
      if (index != 0U) std::cout << ',';
      std::cout << std::fixed << std::setprecision(2) << target[index];
    }
    std::cout << "]\n";
    unitree::robot::ChannelPublisher<ArmString> publisher("d1_native_joint_segment");
    publisher.InitChannel();
    std::this_thread::sleep_for(300ms);
    ArmString command;
    command.data_() = make_segment(target, duration);
    if (!publisher.Write(command)) throw std::runtime_error("DDS segment publish failed");
    std::this_thread::sleep_for(std::chrono::milliseconds(duration + 300U));
    publisher.CloseChannel();
    subscriber.CloseChannel();
    unitree::robot::ChannelFactory::Instance()->Release();
    return EXIT_SUCCESS;
  }
  catch (const std::exception & error)
  {
    std::cerr << "ERROR: " << error.what() << "\n\n";
    print_usage(argv[0], tool);
    return EXIT_FAILURE;
  }
}

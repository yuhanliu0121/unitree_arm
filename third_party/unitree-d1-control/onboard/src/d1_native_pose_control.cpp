#include "CSerialPort/SerialPort.h"
#include "FashionStar/UServo/FashionStar_UartServo.h"
#include "FashionStar/UServo/FashionStar_UartServoProtocol.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using fsuservo::FSUS_Protocol;
using fsuservo::FSUS_Servo;

constexpr std::size_t kJointCount = 7U;
constexpr char kServoPort[] = "/dev/ttyS4";
constexpr char kConfirmation[] = "NATIVE_POSE_MOVE";
constexpr std::uint16_t kMaximumDurationMs = 30000U;
constexpr double kMaximumDeltaDeg = 120.0;

struct Config
{
  std::string target;
  std::string execution{"native"};
  std::uint16_t duration_ms{6000U};
  std::uint16_t acceleration_ms{800U};
  std::uint16_t deceleration_ms{800U};
  std::uint32_t period_us{1000U};
  std::uint16_t servo_duration_ms{5U};
  bool confirmed{false};
};

int parse_integer(const std::string & value, const std::string & option)
{
  std::size_t consumed = 0U;
  const int parsed = std::stoi(value, &consumed);
  if (consumed != value.size())
  {
    throw std::invalid_argument("invalid integer for " + option + ": " + value);
  }
  return parsed;
}

Config parse_arguments(int argc, char ** argv)
{
  Config config;
  for (int index = 1; index < argc; ++index)
  {
    const std::string option = argv[index];
    if (option == "--help")
    {
      std::cout
        << "Usage: d1_native_pose_control --target zero|stowed "
        << "[--execution native|streamed] "
        << "[--duration-ms N] [--acceleration-ms N] [--deceleration-ms N] "
        << "[--period-us N] [--servo-duration-ms N] "
        << "--confirm " << kConfirmation << '\n';
      std::exit(0);
    }
    if (index + 1 >= argc)
    {
      throw std::invalid_argument("missing value for " + option);
    }
    const std::string value = argv[++index];
    if (option == "--target")
    {
      config.target = value;
    }
    else if (option == "--execution")
    {
      config.execution = value;
    }
    else if (option == "--duration-ms")
    {
      const int parsed = parse_integer(value, option);
      if (parsed <= 0 || parsed > kMaximumDurationMs)
      {
        throw std::invalid_argument("duration must be in [1, 30000] ms");
      }
      config.duration_ms = static_cast<std::uint16_t>(parsed);
    }
    else if (option == "--acceleration-ms")
    {
      const int parsed = parse_integer(value, option);
      if (parsed < 0 || parsed > kMaximumDurationMs)
      {
        throw std::invalid_argument("acceleration must be in [0, 30000] ms");
      }
      config.acceleration_ms = static_cast<std::uint16_t>(parsed);
    }
    else if (option == "--deceleration-ms")
    {
      const int parsed = parse_integer(value, option);
      if (parsed < 0 || parsed > kMaximumDurationMs)
      {
        throw std::invalid_argument("deceleration must be in [0, 30000] ms");
      }
      config.deceleration_ms = static_cast<std::uint16_t>(parsed);
    }
    else if (option == "--period-us")
    {
      const int parsed = parse_integer(value, option);
      if (parsed < 500 || parsed > 100000)
      {
        throw std::invalid_argument("period must be in [500, 100000] us");
      }
      config.period_us = static_cast<std::uint32_t>(parsed);
    }
    else if (option == "--servo-duration-ms")
    {
      const int parsed = parse_integer(value, option);
      if (parsed < 1 || parsed > 1000)
      {
        throw std::invalid_argument("servo duration must be in [1, 1000] ms");
      }
      config.servo_duration_ms = static_cast<std::uint16_t>(parsed);
    }
    else if (option == "--confirm")
    {
      config.confirmed = value == kConfirmation;
    }
    else
    {
      throw std::invalid_argument("unknown option: " + option);
    }
  }

  if (config.target != "zero" && config.target != "stowed")
  {
    throw std::invalid_argument("--target must be zero or stowed");
  }
  if (config.execution != "native" && config.execution != "streamed")
  {
    throw std::invalid_argument("--execution must be native or streamed");
  }
  if (!config.confirmed)
  {
    throw std::invalid_argument(
            std::string("physical motion requires --confirm ") + kConfirmation);
  }
  if (static_cast<std::uint32_t>(config.acceleration_ms) + config.deceleration_ms >
    config.duration_ms)
  {
    throw std::invalid_argument("acceleration + deceleration must not exceed duration");
  }
  return config;
}

double quintic_position_scale(const double progress)
{
  const double s = std::clamp(progress, 0.0, 1.0);
  return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

void dispatch_native(
  std::array<FSUS_Servo, kJointCount> & servos,
  const std::array<double, kJointCount> & target,
  const Config & config)
{
  const auto dispatch_started = std::chrono::steady_clock::now();
  for (std::size_t joint = 0; joint < kJointCount; ++joint)
  {
    servos[joint].setRawAngle(
      static_cast<float>(target[joint]), config.duration_ms);
  }
  const double dispatch_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - dispatch_started).count();
  std::cout << "Seven native commands dispatched in " << std::fixed
            << std::setprecision(3) << dispatch_ms << " ms\n";
}

void dispatch_streamed(
  std::array<FSUS_Servo, kJointCount> & servos,
  const std::array<double, kJointCount> & current,
  const std::array<double, kJointCount> & target,
  const Config & config)
{
  using Clock = std::chrono::steady_clock;
  const auto period = std::chrono::microseconds(config.period_us);
  const std::uint64_t point_count =
    static_cast<std::uint64_t>(config.duration_ms) * 1000U / config.period_us + 1U;
  const auto started = Clock::now();
  double dispatch_sum_us = 0.0;
  double dispatch_max_us = 0.0;
  std::uint64_t late_points = 0U;
  double maximum_lateness_us = 0.0;

  for (std::uint64_t point = 0U; point < point_count; ++point)
  {
    const auto deadline = started + period * point;
    std::this_thread::sleep_until(deadline);
    const auto woke = Clock::now();
    const double lateness_us = std::chrono::duration<double, std::micro>(
      woke - deadline).count();
    if (lateness_us > static_cast<double>(config.period_us))
    {
      ++late_points;
      maximum_lateness_us = std::max(maximum_lateness_us, lateness_us);
    }

    const double progress = point_count == 1U ? 1.0 :
      static_cast<double>(point) / static_cast<double>(point_count - 1U);
    const double scale = quintic_position_scale(progress);
    const auto dispatch_started = Clock::now();
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
    {
      const double angle = current[joint] + (target[joint] - current[joint]) * scale;
      servos[joint].setRawAngle(
        static_cast<float>(angle), config.servo_duration_ms);
    }
    const double dispatch_us = std::chrono::duration<double, std::micro>(
      Clock::now() - dispatch_started).count();
    dispatch_sum_us += dispatch_us;
    dispatch_max_us = std::max(dispatch_max_us, dispatch_us);
  }

  std::cout << "Streamed " << point_count << " seven-joint points at requested period="
            << config.period_us << " us servo_duration=" << config.servo_duration_ms
            << " ms mean_serial_burst=" << std::fixed << std::setprecision(1)
            << dispatch_sum_us / static_cast<double>(point_count)
            << " us max_serial_burst=" << dispatch_max_us
            << " us late_points=" << late_points
            << " max_lateness=" << maximum_lateness_us << " us\n";
}

std::array<double, kJointCount> target_for(
  const std::string & name, const std::array<double, kJointCount> & current)
{
  auto target = current;
  if (name == "zero")
  {
    std::fill(target.begin(), target.begin() + 6, 0.0);
  }
  else
  {
    target[0] = 0.0;
    target[1] = -1.54 * 180.0 / M_PI;
    target[2] = 1.55 * 180.0 / M_PI;
    target[3] = 0.0;
    target[4] = 0.0;
    target[5] = 0.0;
  }
  // This experiment changes only the six arm joints. Preserve the measured
  // gripper angle so the comparison cannot accidentally release an object.
  target[6] = current[6];
  return target;
}

void print_angles(const char * label, const std::array<double, kJointCount> & angles)
{
  std::cout << label << " [";
  for (std::size_t joint = 0; joint < angles.size(); ++joint)
  {
    if (joint != 0U)
    {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(1) << angles[joint];
  }
  std::cout << "] deg\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try
  {
    const Config config = parse_arguments(argc, argv);
    // The vendor serial library has an unsafe process-exit destructor on the
    // D1 image. Keep the protocol alive until process termination and use
    // _Exit after flushing output, matching the proven maintenance tools.
    auto * protocol = new FSUS_Protocol(kServoPort, FSUS_DEFAULT_BAUDRATE);
    std::array<FSUS_Servo, kJointCount> servos{
      FSUS_Servo(0, protocol), FSUS_Servo(1, protocol),
      FSUS_Servo(2, protocol), FSUS_Servo(3, protocol),
      FSUS_Servo(4, protocol), FSUS_Servo(5, protocol),
      FSUS_Servo(6, protocol)};

    std::array<double, kJointCount> current{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
    {
      current[joint] = servos[joint].queryRawAngle();
      if (!std::isfinite(current[joint]))
      {
        throw std::runtime_error("non-finite joint feedback");
      }
    }
    const auto target = target_for(config.target, current);
    double maximum_delta = 0.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
    {
      maximum_delta = std::max(maximum_delta, std::abs(target[joint] - current[joint]));
    }
    if (maximum_delta > kMaximumDeltaDeg)
    {
      throw std::runtime_error("maximum joint delta exceeds 120 degrees");
    }

    print_angles("Current:", current);
    print_angles("Target: ", target);
    std::cout << "Execution=" << config.execution << " duration=" << config.duration_ms
              << " ms max_delta=" << std::fixed << std::setprecision(1)
              << maximum_delta << " deg\n";
    if (config.execution == "native")
    {
      std::cout << "Native acceleration=" << config.acceleration_ms
                << " ms deceleration=" << config.deceleration_ms << " ms\n";
      dispatch_native(servos, target, config);
    }
    else
    {
      dispatch_streamed(servos, current, target, config);
    }

    const int settling_ms = config.execution == "native" ?
      static_cast<int>(config.duration_ms) + 300 : 300;
    std::this_thread::sleep_for(std::chrono::milliseconds(settling_ms));
    std::array<double, kJointCount> final{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint)
    {
      final[joint] = servos[joint].queryRawAngle();
    }
    print_angles("Final:  ", final);
    std::cout.flush();
    std::cerr.flush();
    std::_Exit(EXIT_SUCCESS);
  }
  catch (const std::exception & error)
  {
    std::cerr << "d1_native_pose_control: " << error.what() << '\n';
    return 1;
  }
}

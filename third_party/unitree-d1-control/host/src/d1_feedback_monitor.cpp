#include "PubServoInfo_.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using namespace std::chrono_literals;
using ServoFeedback = unitree_arm::msg::dds_::PubServoInfo_;
std::atomic<bool> g_stop{false};

void stop_handler(int) {g_stop.store(true);}

int parse_int(const std::string & text, const char * name)
{
  std::size_t consumed = 0U;
  const int value = std::stoi(text, &consumed);
  if (consumed != text.size()) throw std::runtime_error(std::string("invalid ") + name);
  return value;
}

struct State
{
  std::mutex mutex;
  std::chrono::steady_clock::time_point first{};
  std::chrono::steady_clock::time_point last{};
  std::uint64_t samples{0U};
};

}  // namespace

int main(int argc, char ** argv)
{
  try
  {
    std::string interface;
    int seconds = 0;
    int first_option = 1;
#if defined(D1_INTERFACE_PROBE)
    if (argc > 1 && std::string(argv[1]) == "monitor") first_option = 2;
#endif
    for (int index = first_option; index < argc; ++index)
    {
      const std::string option = argv[index];
      if (option == "-h" || option == "--help")
      {
#if defined(D1_INTERFACE_PROBE)
        std::cout << "Usage: " << argv[0]
                  << " monitor --interface IFACE [--seconds N]\n";
#else
        std::cout << "Usage: " << argv[0]
                  << " --interface IFACE [--seconds N] [--feedback 0|1]\n";
#endif
        return EXIT_SUCCESS;
      }
      if (index + 1 >= argc) throw std::runtime_error("missing value for " + option);
      const std::string value = argv[++index];
      if (option == "--interface") interface = value;
      else if (option == "--seconds") seconds = parse_int(value, "seconds");
      else if (option == "--feedback")
      {
        // Retained for old CLI compatibility. The enhanced executor exposes
        // trustworthy joint feedback directly and no longer needs rt/arm_Feedback.
        const int enabled = parse_int(value, "feedback");
        if (enabled != 0 && enabled != 1)
          throw std::runtime_error("--feedback must be 0 or 1");
      }
      else throw std::runtime_error("unknown option: " + option);
    }
    if (interface.empty()) throw std::runtime_error("--interface is required");
    if (seconds < 0) throw std::runtime_error("--seconds must be non-negative");

    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
    State state;
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    unitree::robot::ChannelSubscriber<ServoFeedback> subscriber("current_servo_angle");
    subscriber.InitChannel([&state](const void * raw)
      {
        const auto & message = *static_cast<const ServoFeedback *>(raw);
        const auto now = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(state.mutex);
          if (state.samples == 0U) state.first = now;
          state.last = now;
          ++state.samples;
        }
        std::cout << std::fixed << std::setprecision(3) << "angles_deg=["
                  << message.servo0_data_() << ',' << message.servo1_data_() << ','
                  << message.servo2_data_() << ',' << message.servo3_data_() << ','
                  << message.servo4_data_() << ',' << message.servo5_data_() << ','
                  << message.servo6_data_() << "]\n";
      }, 8);

    const auto deadline = seconds == 0 ? std::chrono::steady_clock::time_point::max() :
      std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    while (!g_stop.load() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(100ms);
    subscriber.CloseChannel();
    unitree::robot::ChannelFactory::Instance()->Release();

    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.samples == 0U) throw std::runtime_error("no joint feedback received");
    const double elapsed = state.samples > 1U ?
      std::chrono::duration<double>(state.last - state.first).count() : 0.0;
    const double rate = elapsed > 0.0 ? static_cast<double>(state.samples - 1U) / elapsed : 0.0;
    std::cout << "samples=" << state.samples << " measured_rate_hz="
              << std::fixed << std::setprecision(2) << rate << '\n';
    return EXIT_SUCCESS;
  }
  catch (const std::exception & error)
  {
    std::cerr << "ERROR: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}

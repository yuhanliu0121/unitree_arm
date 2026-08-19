#include "d1_ros2_control/local_protocol.hpp"

#include "ArmString_.hpp"
#include "PubServoInfo_.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <arpa/inet.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

using d1_ros2_control::JointPacket;
using d1_ros2_control::PacketKind;
using d1_ros2_control::kD1JointCount;
using ArmString = unitree_arm::msg::dds_::ArmString_;
using ServoInfo = unitree_arm::msg::dds_::PubServoInfo_;
using Publisher = unitree::robot::ChannelPublisher<ArmString>;
using Subscriber = unitree::robot::ChannelSubscriber<ServoInfo>;
using StatusSubscriber = unitree::robot::ChannelSubscriber<ArmString>;

volatile sig_atomic_t g_stop_requested = 0;

void request_stop(int)
{
  g_stop_requested = 1;
}

struct Config
{
  enum class Direction
  {
    both,
    command,
    feedback,
    status,
  };

  int domain_id{42};
  std::string interface;
  std::string command_topic{"rt/arm_Command"};
  std::string feedback_topic{"current_servo_angle"};
  std::string status_topic{"rt/arm_Feedback"};
  int command_port{15000};
  int feedback_port{15001};
  Direction direction{Direction::both};
};

bool command_enabled(const Config & config)
{
  return config.direction == Config::Direction::both ||
         config.direction == Config::Direction::command;
}

bool feedback_enabled(const Config & config)
{
  return config.direction == Config::Direction::both ||
         config.direction == Config::Direction::feedback;
}

bool status_enabled(const Config & config)
{
  return config.direction == Config::Direction::status;
}

const char * direction_name(const Config::Direction direction)
{
  switch (direction)
  {
    case Config::Direction::both:
      return "both";
    case Config::Direction::command:
      return "command";
    case Config::Direction::feedback:
      return "feedback";
    case Config::Direction::status:
      return "status";
  }
  return "unknown";
}

int parse_int(const std::string & value, const std::string & option)
{
  std::size_t consumed = 0;
  const int parsed = std::stoi(value, &consumed);
  if (consumed != value.size())
  {
    throw std::invalid_argument("invalid value for " + option + ": " + value);
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
        << "Usage: d1_dds_gateway [--domain N] [--interface NAME] "
        << "[--direction both|command|feedback|status] "
        << "[--command-topic TOPIC] [--feedback-topic TOPIC] [--status-topic TOPIC] "
        << "[--command-port PORT] [--feedback-port PORT]\n";
      std::exit(0);
    }
    if (index + 1 >= argc)
    {
      throw std::invalid_argument("missing value for " + option);
    }
    const std::string value = argv[++index];
    if (option == "--domain")
    {
      config.domain_id = parse_int(value, option);
    }
    else if (option == "--interface")
    {
      config.interface = value;
    }
    else if (option == "--direction")
    {
      if (value == "both")
      {
        config.direction = Config::Direction::both;
      }
      else if (value == "command")
      {
        config.direction = Config::Direction::command;
      }
      else if (value == "feedback")
      {
        config.direction = Config::Direction::feedback;
      }
      else if (value == "status")
      {
        config.direction = Config::Direction::status;
      }
      else
      {
        throw std::invalid_argument("invalid direction: " + value);
      }
    }
    else if (option == "--command-topic")
    {
      config.command_topic = value;
    }
    else if (option == "--feedback-topic")
    {
      config.feedback_topic = value;
    }
    else if (option == "--status-topic")
    {
      config.status_topic = value;
    }
    else if (option == "--command-port")
    {
      config.command_port = parse_int(value, option);
    }
    else if (option == "--feedback-port")
    {
      config.feedback_port = parse_int(value, option);
    }
    else
    {
      throw std::invalid_argument("unknown option: " + option);
    }
  }

  if (config.domain_id < 0 || config.domain_id > 232 ||
      config.command_port <= 0 || config.command_port > 65535 ||
      config.feedback_port <= 0 || config.feedback_port > 65535 ||
      config.command_port == config.feedback_port)
  {
    throw std::invalid_argument("domain or local UDP port is out of range");
  }
  return config;
}

class Gateway
{
public:
  explicit Gateway(Config config)
  : config_(std::move(config))
  {
  }

  ~Gateway()
  {
    stop();
  }

  bool start()
  {
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
      std::cerr << "d1_dds_gateway: socket failed: " << std::strerror(errno) << '\n';
      return false;
    }

    if (command_enabled(config_))
    {
      sockaddr_in command_address{};
      command_address.sin_family = AF_INET;
      command_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      command_address.sin_port = htons(static_cast<std::uint16_t>(config_.command_port));
      if (::bind(
          socket_fd_, reinterpret_cast<const sockaddr *>(&command_address),
          sizeof(command_address)) < 0)
      {
        std::cerr << "d1_dds_gateway: bind 127.0.0.1:" << config_.command_port
                  << " failed: " << std::strerror(errno) << '\n';
        return false;
      }
    }

    if (feedback_enabled(config_) || status_enabled(config_))
    {
      feedback_address_.sin_family = AF_INET;
      feedback_address_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      feedback_address_.sin_port = htons(
        static_cast<std::uint16_t>(config_.feedback_port));
    }

    try
    {
      if (config_.interface.empty())
      {
        unitree::robot::ChannelFactory::Instance()->Init(config_.domain_id);
      }
      else
      {
        unitree::robot::ChannelFactory::Instance()->Init(
          config_.domain_id, config_.interface);
      }
      dds_initialized_ = true;
      if (feedback_enabled(config_))
      {
        subscriber_ = std::make_unique<Subscriber>(config_.feedback_topic);
        accepting_feedback_.store(true, std::memory_order_release);
        subscriber_->InitChannel(
          [this](const void * raw_message)
          {
            handle_feedback(raw_message);
          });
      }
      if (status_enabled(config_))
      {
        status_subscriber_ = std::make_unique<StatusSubscriber>(config_.status_topic);
        accepting_feedback_.store(true, std::memory_order_release);
        status_subscriber_->InitChannel(
          [this](const void * raw_message)
          {
            handle_status(raw_message);
          });
      }
      if (command_enabled(config_))
      {
        publisher_ = std::make_unique<Publisher>(config_.command_topic);
        publisher_->InitChannel();
      }
    }
    catch (const std::exception & error)
    {
      std::cerr << "d1_dds_gateway: DDS setup failed: " << error.what() << '\n';
      stop();
      return false;
    }

    started_ = true;
    std::cout << "D1 DDS gateway ready: direction="
              << direction_name(config_.direction)
              << " domain=" << config_.domain_id;
    if (command_enabled(config_))
    {
      std::cout << " command=" << config_.command_topic
                << " local_command=127.0.0.1:" << config_.command_port;
    }
    if (feedback_enabled(config_))
    {
      std::cout << " feedback=" << config_.feedback_topic
                << " local_feedback=127.0.0.1:" << config_.feedback_port;
    }
    if (status_enabled(config_))
    {
      std::cout << " status=" << config_.status_topic
                << " local_status=127.0.0.1:" << config_.feedback_port;
    }
    std::cout << std::endl;
    return true;
  }

  int run()
  {
    if (!command_enabled(config_))
    {
      while (!g_stop_requested)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      return 0;
    }

    pollfd descriptor{};
    descriptor.fd = socket_fd_;
    descriptor.events = POLLIN;
    while (!g_stop_requested)
    {
      const int result = ::poll(&descriptor, 1, 100);
      if (result < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        std::cerr << "d1_dds_gateway: poll failed: " << std::strerror(errno) << '\n';
        return 1;
      }
      if (result > 0 && (descriptor.revents & POLLIN) != 0)
      {
        receive_commands();
      }
    }
    return 0;
  }

private:
  void receive_commands()
  {
    while (true)
    {
      JointPacket packet;
      const auto received = ::recv(socket_fd_, &packet, sizeof(packet), MSG_DONTWAIT);
      if (received < 0)
      {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
          std::cerr << "d1_dds_gateway: receive failed: "
                    << std::strerror(errno) << '\n';
        }
        return;
      }
      if (received != static_cast<ssize_t>(sizeof(packet)) ||
          !d1_ros2_control::packet_header_is_valid(packet))
      {
        std::cerr << "d1_dds_gateway: ignored invalid local command packet\n";
        continue;
      }
      if (packet.kind == PacketKind::power_command)
      {
        publish_management_command(6, "\"power\":1");
        continue;
      }
      if (packet.kind == PacketKind::enable_command)
      {
        publish_management_command(5, "\"mode\":65535");
        continue;
      }
      if (packet.kind != PacketKind::command)
      {
        std::cerr << "d1_dds_gateway: ignored unsupported local command kind\n";
        continue;
      }
      bool finite = true;
      for (const double angle : packet.angle_deg)
      {
        finite = finite && std::isfinite(angle);
      }
      if (!finite || packet.smoothing_mode > 1U)
      {
        std::cerr << "d1_dds_gateway: ignored unsafe local command packet\n";
        continue;
      }
      publish_native_command(packet);
    }
  }

  void publish_native_command(const JointPacket & packet)
  {
    if (native_sequence_ == 10U)
    {
      ++native_sequence_;
    }
    std::ostringstream payload;
    payload.setf(std::ios::fixed);
    payload.precision(7);
    payload << "{\"seq\":" << native_sequence_++
            << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":"
            << packet.smoothing_mode;
    for (std::size_t index = 0; index < kD1JointCount; ++index)
    {
      payload << ",\"angle" << index << "\":" << packet.angle_deg[index];
    }
    payload << "}}";

    ++command_count_;
    if (command_count_ == 1U)
    {
      std::cout << "d1_dds_gateway: forwarding command #" << command_count_
                << " local_seq=" << packet.sequence
                << " payload=" << payload.str() << std::endl;
    }

    ArmString message;
    message.data_() = payload.str();
    if (!publisher_->Write(message))
    {
      std::cerr << "d1_dds_gateway: native DDS command publish failed\n";
    }
  }

  void publish_management_command(const int funcode, const char * data)
  {
    if (native_sequence_ == 10U)
    {
      ++native_sequence_;
    }
    std::ostringstream payload;
    payload << "{\"seq\":" << native_sequence_++
            << ",\"address\":1,\"funcode\":" << funcode
            << ",\"data\":{" << data << "}}";
    ArmString message;
    message.data_() = payload.str();
    if (!publisher_->Write(message))
    {
      std::cerr << "d1_dds_gateway: native DDS management publish failed\n";
      return;
    }
    std::cout << "d1_dds_gateway: published hardware preparation funcode="
              << funcode << std::endl;
  }

  void handle_feedback(const void * raw_message)
  {
    if (!accepting_feedback_.load(std::memory_order_acquire) || socket_fd_ < 0)
    {
      return;
    }
    const auto * feedback = static_cast<const ServoInfo *>(raw_message);
    JointPacket packet;
    packet.kind = PacketKind::feedback;
    packet.sequence = feedback_sequence_.fetch_add(1U, std::memory_order_relaxed);
    packet.angle_deg = {
      feedback->servo0_data_(), feedback->servo1_data_(),
      feedback->servo2_data_(), feedback->servo3_data_(),
      feedback->servo4_data_(), feedback->servo5_data_(),
      feedback->servo6_data_(),
    };
    ::sendto(
      socket_fd_, &packet, sizeof(packet), MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&feedback_address_),
      sizeof(feedback_address_));
  }

  static bool extract_status_value(
    const std::string & payload, const char * key, int & output)
  {
    const std::string token = std::string("\"") + key + "\"";
    const auto key_position = payload.find(token);
    if (key_position == std::string::npos)
    {
      return false;
    }
    const auto colon = payload.find(':', key_position + token.size());
    if (colon == std::string::npos)
    {
      return false;
    }
    char * end = nullptr;
    const long parsed = std::strtol(payload.c_str() + colon + 1, &end, 10);
    if (end == payload.c_str() + colon + 1)
    {
      return false;
    }
    output = static_cast<int>(parsed);
    return true;
  }

  void handle_status(const void * raw_message)
  {
    if (!accepting_feedback_.load(std::memory_order_acquire) || socket_fd_ < 0)
    {
      return;
    }
    const auto * message = static_cast<const ArmString *>(raw_message);
    int enable_status = 0;
    int power_status = 0;
    int error_status = 0;
    if (!extract_status_value(message->data_(), "enable_status", enable_status) ||
        !extract_status_value(message->data_(), "power_status", power_status) ||
        !extract_status_value(message->data_(), "error_status", error_status))
    {
      return;
    }
    JointPacket packet;
    packet.kind = PacketKind::status;
    packet.sequence = status_sequence_.fetch_add(1U, std::memory_order_relaxed);
    packet.angle_deg[0] = static_cast<double>(enable_status);
    packet.angle_deg[1] = static_cast<double>(power_status);
    packet.angle_deg[2] = static_cast<double>(error_status);
    ::sendto(
      socket_fd_, &packet, sizeof(packet), MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&feedback_address_),
      sizeof(feedback_address_));
  }

  void stop()
  {
    accepting_feedback_.store(false, std::memory_order_release);
    if (subscriber_)
    {
      subscriber_->CloseChannel();
      subscriber_.reset();
    }
    if (status_subscriber_)
    {
      status_subscriber_->CloseChannel();
      status_subscriber_.reset();
    }
    if (publisher_)
    {
      publisher_->CloseChannel();
      publisher_.reset();
    }
    if (dds_initialized_)
    {
      unitree::robot::ChannelFactory::Instance()->Release();
      dds_initialized_ = false;
    }
    if (socket_fd_ >= 0)
    {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
    started_ = false;
  }

  Config config_;
  int socket_fd_{-1};
  sockaddr_in feedback_address_{};
  std::atomic<bool> accepting_feedback_{false};
  std::atomic<std::uint64_t> feedback_sequence_{1U};
  std::atomic<std::uint64_t> status_sequence_{1U};
  std::uint32_t native_sequence_{1U};
  std::uint64_t command_count_{0U};
  std::unique_ptr<Publisher> publisher_;
  std::unique_ptr<Subscriber> subscriber_;
  std::unique_ptr<StatusSubscriber> status_subscriber_;
  bool started_{false};
  bool dds_initialized_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  try
  {
    const Config config = parse_arguments(argc, argv);
    ::signal(SIGINT, request_stop);
    ::signal(SIGTERM, request_stop);
    Gateway gateway(config);
    if (!gateway.start())
    {
      return 1;
    }
    return gateway.run();
  }
  catch (const std::exception & error)
  {
    std::cerr << "d1_dds_gateway: " << error.what() << '\n';
    return 2;
  }
}

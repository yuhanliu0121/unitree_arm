#include "d1_ros2_control/d1_system_hardware.hpp"

#include "d1_ros2_control/local_protocol.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>

namespace d1_ros2_control
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kJointCount = kD1JointCount;
const auto kLogger = rclcpp::get_logger("d1_ros2_control");

double parameter_as_double(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  double default_value)
{
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end()
           ? default_value
           : std::stod(iterator->second);
}

int parameter_as_int(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  int default_value)
{
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end()
           ? default_value
           : std::stoi(iterator->second);
}

std::string parameter_as_string(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  const std::string & default_value)
{
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end()
           ? default_value
           : iterator->second;
}

bool parameter_as_bool(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  bool default_value)
{
  const auto value = parameter_as_string(
    info, name, default_value ? "true" : "false");
  if (value == "true" || value == "1")
  {
    return true;
  }
  if (value == "false" || value == "0")
  {
    return false;
  }
  throw std::invalid_argument(name + " must be true or false");
}

}  // namespace

class D1SystemHardware::Impl
{
public:
  ~Impl()
  {
    if (socket_fd >= 0)
    {
      ::close(socket_fd);
    }
  }

  bool receive_latest(
    std::array<double, kJointCount> & output,
    std::chrono::steady_clock::time_point & timestamp,
    bool & updated)
  {
    updated = false;
    while (true)
    {
      JointPacket packet;
      const auto received = ::recv(
        socket_fd, &packet, sizeof(packet), MSG_DONTWAIT);
      if (received < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          break;
        }
        RCLCPP_ERROR(kLogger, "Local feedback receive failed: %s", std::strerror(errno));
        return false;
      }
      if (received != static_cast<ssize_t>(sizeof(packet)) ||
          !packet_header_is_valid(packet))
      {
        RCLCPP_WARN(kLogger, "Ignored an invalid local D1 feedback packet");
        continue;
      }
      if (packet.kind == PacketKind::status)
      {
        enable_status = static_cast<int>(packet.angle_deg[0]);
        power_status = static_cast<int>(packet.angle_deg[1]);
        error_status = static_cast<int>(packet.angle_deg[2]);
        latest_status_time = std::chrono::steady_clock::now();
        status_received = true;
        ++status_count;
        continue;
      }
      if (packet.kind != PacketKind::feedback)
      {
        RCLCPP_WARN(kLogger, "Ignored an unexpected local D1 packet kind");
        continue;
      }
      bool finite = true;
      for (const double angle : packet.angle_deg)
      {
        finite = finite && std::isfinite(angle);
      }
      if (!finite)
      {
        RCLCPP_WARN(kLogger, "Ignored non-finite D1 feedback");
        continue;
      }
      latest_sdk_angles_deg = packet.angle_deg;
      latest_feedback_time = std::chrono::steady_clock::now();
      feedback_received = true;
      updated = true;
    }

    if (!feedback_received)
    {
      return false;
    }
    output = latest_sdk_angles_deg;
    timestamp = latest_feedback_time;
    return true;
  }

  bool send(const JointPacket & packet)
  {
    const auto sent = ::sendto(
      socket_fd, &packet, sizeof(packet), 0,
      reinterpret_cast<const sockaddr *>(&gateway_address),
      sizeof(gateway_address));
    if (sent != static_cast<ssize_t>(sizeof(packet)))
    {
      RCLCPP_ERROR(kLogger, "Local command send failed: %s", std::strerror(errno));
      return false;
    }
    return true;
  }

  int socket_fd{-1};
  sockaddr_in gateway_address{};
  std::array<double, kJointCount> latest_sdk_angles_deg{};
  std::chrono::steady_clock::time_point latest_feedback_time{};
  bool feedback_received{false};
  std::chrono::steady_clock::time_point latest_status_time{};
  int enable_status{0};
  int power_status{0};
  int error_status{0};
  bool status_received{false};
  std::uint64_t status_count{0U};
};

D1SystemHardware::D1SystemHardware() = default;

D1SystemHardware::~D1SystemHardware()
{
  shutdown_transport();
}

hardware_interface::CallbackReturn D1SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    gateway_host_ = parameter_as_string(info, "gateway_host", "127.0.0.1");
    command_port_ = parameter_as_int(info, "command_port", 15000);
    feedback_port_ = parameter_as_int(info, "feedback_port", 15001);
    command_rate_hz_ = parameter_as_double(info, "command_rate_hz", 10.0);
    feedback_timeout_s_ = parameter_as_double(info, "feedback_timeout_s", 0.5);
    initial_feedback_timeout_s_ = parameter_as_double(
      info, "initial_feedback_timeout_s", 10.0);
    hardware_prepare_timeout_s_ = parameter_as_double(
      info, "hardware_prepare_timeout_s", 5.0);
    smoothing_mode_ = parameter_as_int(info, "smoothing_mode", 0);
    prepare_hardware_ = parameter_as_bool(info, "prepare_hardware", false);
    gripper_closed_angle_deg_ = parameter_as_double(
      info, "gripper_closed_angle_deg", -30.0);
    gripper_open_angle_deg_ = parameter_as_double(
      info, "gripper_open_angle_deg", 60.0);
    gripper_travel_m_ = parameter_as_double(info, "gripper_travel_m", 0.03);
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(kLogger, "Invalid hardware parameter: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.joints.size() != kJointCount || command_rate_hz_ <= 0.0 ||
      feedback_timeout_s_ <= 0.0 || initial_feedback_timeout_s_ <= 0.0 ||
      hardware_prepare_timeout_s_ <= 0.0 ||
      command_port_ <= 0 || command_port_ > 65535 ||
      feedback_port_ <= 0 || feedback_port_ > 65535 ||
      command_port_ == feedback_port_ || gripper_travel_m_ <= 0.0 ||
      gripper_open_angle_deg_ <= gripper_closed_angle_deg_ ||
      (smoothing_mode_ != 0 && smoothing_mode_ != 1))
  {
    RCLCPP_ERROR(kLogger, "Invalid D1 ros2_control configuration");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    const auto expected_name = "Joint" + std::to_string(index);
    const auto & joint = info.joints[index];
    if (joint.name != expected_name || joint.command_interfaces.size() != 1 ||
        joint.state_interfaces.size() != (index == 6 ? 2U : 1U) ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        (index == 6 &&
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(
        kLogger, "Joint contract mismatch at index %zu (expected %s)",
        index, expected_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    try
    {
      lower_limits_[index] = std::stod(joint.command_interfaces[0].min);
      upper_limits_[index] = std::stod(joint.command_interfaces[0].max);
    }
    catch (const std::exception &)
    {
      RCLCPP_ERROR(kLogger, "Joint %s requires numeric min/max", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    state_position_[index] = std::numeric_limits<double>::quiet_NaN();
    state_velocity_[index] = std::numeric_limits<double>::quiet_NaN();
    command_position_[index] = std::numeric_limits<double>::quiet_NaN();
  }

  RCLCPP_INFO(
    kLogger, "Initialized D1 hardware contract: gateway %s:%d, command %.2f Hz",
    gateway_host_.c_str(), command_port_, command_rate_hz_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
D1SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION,
      &state_position_[index]);
  }
  interfaces.emplace_back(
    info_.joints[6].name, hardware_interface::HW_IF_VELOCITY,
    &state_velocity_[6]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
D1SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION,
      &command_position_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn D1SystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (configured_)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  impl_ = std::make_unique<Impl>();
  impl_->socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (impl_->socket_fd < 0)
  {
    RCLCPP_ERROR(kLogger, "Failed to create local D1 socket: %s", std::strerror(errno));
    shutdown_transport();
    return hardware_interface::CallbackReturn::ERROR;
  }

  sockaddr_in local_address{};
  local_address.sin_family = AF_INET;
  local_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local_address.sin_port = htons(static_cast<std::uint16_t>(feedback_port_));
  if (::bind(
      impl_->socket_fd, reinterpret_cast<const sockaddr *>(&local_address),
      sizeof(local_address)) < 0)
  {
    RCLCPP_ERROR(
      kLogger, "Failed to bind local D1 feedback port %d: %s",
      feedback_port_, std::strerror(errno));
    shutdown_transport();
    return hardware_interface::CallbackReturn::ERROR;
  }

  impl_->gateway_address.sin_family = AF_INET;
  impl_->gateway_address.sin_port = htons(static_cast<std::uint16_t>(command_port_));
  if (::inet_pton(
      AF_INET, gateway_host_.c_str(), &impl_->gateway_address.sin_addr) != 1)
  {
    RCLCPP_ERROR(kLogger, "gateway_host must be an IPv4 address: %s", gateway_host_.c_str());
    shutdown_transport();
    return hardware_interface::CallbackReturn::ERROR;
  }

  configured_ = true;
  RCLCPP_INFO(
    kLogger, "Configured isolated D1 gateway transport: feedback 127.0.0.1:%d",
    feedback_port_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn D1SystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  shutdown_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn D1SystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!configured_ || !impl_)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(initial_feedback_timeout_s_);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (copy_feedback_to_state(false))
    {
      if (prepare_hardware_ && !prepare_physical_hardware())
      {
        return hardware_interface::CallbackReturn::ERROR;
      }
      command_position_ = state_position_;
      last_write_time_ = {};
      active_ = true;
      RCLCPP_INFO(kLogger, "Activated D1 hardware from live joint feedback");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_ERROR(
    kLogger, "No D1 gateway feedback received within %.2f seconds",
    initial_feedback_timeout_s_);
  return hardware_interface::CallbackReturn::ERROR;
}

bool D1SystemHardware::prepare_physical_hardware()
{
  if (!impl_)
  {
    return false;
  }

  const auto wait_for_status = [this](
    const auto & predicate, const char * phase,
    const std::uint64_t minimum_status_count)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(hardware_prepare_timeout_s_);
    while (std::chrono::steady_clock::now() < deadline)
    {
      copy_feedback_to_state(false);
      if (impl_->status_received)
      {
        if (impl_->error_status != 0)
        {
          RCLCPP_ERROR(
            kLogger, "D1 reports error_status=%d during %s",
            impl_->error_status, phase);
          return false;
        }
        if (impl_->status_count >= minimum_status_count && predicate())
        {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_ERROR(
      kLogger,
      "D1 hardware preparation timed out during %s "
      "(status_received=%s power=%d enable=%d error=%d)",
      phase, impl_->status_received ? "true" : "false",
      impl_->power_status, impl_->enable_status, impl_->error_status);
    return false;
  };

  if (!wait_for_status(
      [this]() {return impl_->status_received;}, "initial status", 1U))
  {
    return false;
  }

  if (impl_->power_status != 1)
  {
    JointPacket power;
    power.kind = PacketKind::power_command;
    power.sequence = sequence_++;
    const auto next_status = impl_->status_count + 1U;
    if (!impl_->send(power) ||
        !wait_for_status(
          [this]() {return impl_->power_status == 1;}, "power on", next_status))
    {
      return false;
    }
  }

  // D1 firmware can report enable_status=1 even after a weak/partial enable.
  // Always request the empirically validated full-enable value before motion.
  JointPacket enable;
  enable.kind = PacketKind::enable_command;
  enable.sequence = sequence_++;
  const auto next_status = impl_->status_count + 1U;
  if (!impl_->send(enable) ||
      !wait_for_status(
        [this]() {return impl_->power_status == 1 && impl_->enable_status == 1;},
        "full enable", next_status))
  {
    return false;
  }

  RCLCPP_INFO(
    kLogger, "D1 physical preparation passed: power=%d enable=%d error=%d",
    impl_->power_status, impl_->enable_status, impl_->error_status);
  return true;
}

hardware_interface::CallbackReturn D1SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (active_)
  {
    if (copy_feedback_to_state(false))
    {
      command_position_ = state_position_;
      publish_command(true);
    }
    active_ = false;
  }
  RCLCPP_INFO(kLogger, "Deactivated D1 hardware with a measured-position hold");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type D1SystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return copy_feedback_to_state(active_)
           ? hardware_interface::return_type::OK
           : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type D1SystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_)
  {
    return hardware_interface::return_type::OK;
  }
  return publish_command(false)
           ? hardware_interface::return_type::OK
           : hardware_interface::return_type::ERROR;
}

bool D1SystemHardware::copy_feedback_to_state(bool require_fresh)
{
  if (!impl_)
  {
    return false;
  }
  std::array<double, kJointCount> sdk_angles{};
  std::chrono::steady_clock::time_point feedback_time;
  bool updated = false;
  if (!impl_->receive_latest(sdk_angles, feedback_time, updated))
  {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - feedback_time).count();
  if (require_fresh && age > feedback_timeout_s_)
  {
    RCLCPP_ERROR(
      kLogger, "D1 feedback stale for %.3f s (limit %.3f s)",
      age, feedback_timeout_s_);
    return false;
  }
  if (require_fresh && prepare_hardware_)
  {
    if (!impl_->status_received)
    {
      RCLCPP_ERROR(kLogger, "D1 hardware status has not been received");
      return false;
    }
    const double status_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - impl_->latest_status_time).count();
    if (status_age > feedback_timeout_s_)
    {
      RCLCPP_ERROR(
        kLogger, "D1 hardware status stale for %.3f s (limit %.3f s)",
        status_age, feedback_timeout_s_);
      return false;
    }
    if (impl_->power_status != 1 || impl_->enable_status != 1 ||
        impl_->error_status != 0)
    {
      RCLCPP_ERROR(
        kLogger, "D1 hardware became NOT_READY: power=%d enable=%d error=%d",
        impl_->power_status, impl_->enable_status, impl_->error_status);
      return false;
    }
  }

  if (!updated)
  {
    return true;
  }

  std::array<double, kJointCount> new_position{};
  for (std::size_t index = 0; index < 6; ++index)
  {
    new_position[index] = sdk_angles[index] * kPi / 180.0;
  }
  const double ratio = std::clamp(
    (sdk_angles[6] - gripper_closed_angle_deg_) /
      (gripper_open_angle_deg_ - gripper_closed_angle_deg_),
    0.0, 1.0);
  new_position[6] = gripper_travel_m_ * ratio;

  const double sample_period = last_state_sample_time_.time_since_epoch().count() == 0
    ? 0.0
    : std::chrono::duration<double>(feedback_time - last_state_sample_time_).count();
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    state_velocity_[index] = sample_period > 1e-6 &&
      std::isfinite(state_position_[index])
      ? (new_position[index] - state_position_[index]) / sample_period
      : 0.0;
    state_position_[index] = new_position[index];
  }
  last_state_sample_time_ = feedback_time;
  return true;
}

bool D1SystemHardware::publish_command(bool ignore_rate_limit)
{
  if (!impl_)
  {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  const double minimum_period = 1.0 / command_rate_hz_;
  if (!ignore_rate_limit && last_write_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_write_time_).count() < minimum_period)
  {
    return true;
  }

  std::array<double, kJointCount> clamped{};
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    if (!std::isfinite(command_position_[index]))
    {
      RCLCPP_ERROR(kLogger, "Non-finite command for Joint%zu", index);
      return false;
    }
    clamped[index] = std::clamp(
      command_position_[index], lower_limits_[index], upper_limits_[index]);
  }

  JointPacket packet;
  packet.kind = PacketKind::command;
  packet.sequence = sequence_++;
  packet.smoothing_mode = static_cast<std::uint32_t>(smoothing_mode_);
  for (std::size_t index = 0; index < 6; ++index)
  {
    packet.angle_deg[index] = clamped[index] * 180.0 / kPi;
  }
  const double gripper_ratio = std::clamp(
    clamped[6] / gripper_travel_m_, 0.0, 1.0);
  packet.angle_deg[6] = gripper_closed_angle_deg_ + gripper_ratio *
    (gripper_open_angle_deg_ - gripper_closed_angle_deg_);

  if (!impl_->send(packet))
  {
    return false;
  }
  last_write_time_ = now;
  return true;
}

void D1SystemHardware::shutdown_transport()
{
  active_ = false;
  configured_ = false;
  last_state_sample_time_ = {};
  impl_.reset();
}

}  // namespace d1_ros2_control

PLUGINLIB_EXPORT_CLASS(
  d1_ros2_control::D1SystemHardware,
  hardware_interface::SystemInterface)

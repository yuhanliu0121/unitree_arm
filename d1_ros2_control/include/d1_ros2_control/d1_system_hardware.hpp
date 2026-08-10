#ifndef D1_ROS2_CONTROL__D1_SYSTEM_HARDWARE_HPP_
#define D1_ROS2_CONTROL__D1_SYSTEM_HARDWARE_HPP_

#include "d1_ros2_control/visibility_control.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace d1_ros2_control
{

class D1_ROS2_CONTROL_PUBLIC D1SystemHardware final
  : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(D1SystemHardware)

  D1SystemHardware();
  ~D1SystemHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  class Impl;

  bool copy_feedback_to_state(bool require_fresh);
  bool publish_command(bool ignore_rate_limit);
  void shutdown_transport();

  std::unique_ptr<Impl> impl_;
  std::array<double, 7> state_position_{};
  std::array<double, 7> state_velocity_{};
  std::array<double, 7> command_position_{};
  std::array<double, 7> lower_limits_{};
  std::array<double, 7> upper_limits_{};

  std::string gateway_host_{"127.0.0.1"};
  int command_port_{15000};
  int feedback_port_{15001};
  double command_rate_hz_{10.0};
  double feedback_timeout_s_{0.5};
  double initial_feedback_timeout_s_{10.0};
  int smoothing_mode_{0};
  double gripper_closed_angle_deg_{-30.0};
  double gripper_open_angle_deg_{60.0};
  double gripper_travel_m_{0.03};
  std::uint64_t sequence_{1};
  bool configured_{false};
  bool active_{false};
  std::chrono::steady_clock::time_point last_write_time_{};
  std::chrono::steady_clock::time_point last_state_sample_time_{};
};

}  // namespace d1_ros2_control

#endif  // D1_ROS2_CONTROL__D1_SYSTEM_HARDWARE_HPP_

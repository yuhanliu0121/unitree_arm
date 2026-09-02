#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "d1_manipulation/srv/apply_task_event.hpp"

namespace d1_manipulation
{
struct TaskStateEventResult
{
  bool accepted{false};
  std::uint8_t state{0};
  std::uint8_t payload_state{0};
  std::uint8_t canonical_pose{0};
  std::string detail;
};

class TaskStateClient
{
public:
  explicit TaskStateClient(const rclcpp::Node::SharedPtr& node);

  TaskStateEventResult apply(
    std::uint8_t event,
    const std::string& phase = {},
    const std::string& failure_code = {},
    const std::string& detail = {},
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<srv::ApplyTaskEvent>::SharedPtr client_;
};
}  // namespace d1_manipulation

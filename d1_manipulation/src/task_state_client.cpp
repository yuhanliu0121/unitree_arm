#include "d1_manipulation/task_state_client.hpp"

#include <future>

namespace d1_manipulation
{
TaskStateClient::TaskStateClient(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  // Goal callbacks synchronously reserve a task state before accepting work.
  // Keep the service response out of the node's default mutually-exclusive
  // callback group, otherwise the response cannot run until that same goal
  // callback stops waiting for it.
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  client_ = node_->create_client<srv::ApplyTaskEvent>(
    "/arm/tasks/apply_state_event", rmw_qos_profile_services_default, callback_group_);
}

TaskStateEventResult TaskStateClient::apply(
  std::uint8_t event,
  const std::string& phase,
  const std::string& failure_code,
  const std::string& detail,
  std::chrono::milliseconds timeout)
{
  TaskStateEventResult output;
  if (!client_->wait_for_service(timeout)) {
    output.detail = "arm task state manager service is unavailable";
    return output;
  }
  auto request = std::make_shared<srv::ApplyTaskEvent::Request>();
  request->event = event;
  request->active_phase = phase;
  request->failure_code = failure_code;
  request->detail = detail;
  auto future = client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    output.detail = "arm task state transition timed out";
    return output;
  }
  const auto response = future.get();
  output.accepted = response->accepted;
  output.state = response->state;
  output.payload_state = response->payload_state;
  output.canonical_pose = response->canonical_pose;
  output.detail = response->detail;
  return output;
}
}  // namespace d1_manipulation

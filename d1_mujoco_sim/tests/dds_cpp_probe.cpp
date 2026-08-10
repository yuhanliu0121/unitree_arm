#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "ArmString_.hpp"
#include "PubServoInfo_.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace
{

std::atomic<float> g_joint0_deg{0.0F};
std::atomic<int> g_feedback_count{0};

void HandleAngles(const void* raw_message)
{
    const auto* message = static_cast<
        const unitree_arm::msg::dds_::PubServoInfo_*>(raw_message);
    g_joint0_deg.store(message->servo0_data_());
    g_feedback_count.fetch_add(1);
}

}  // namespace

int main()
{
    // Simulation uses a non-hardware domain by default to avoid commanding a
    // physical arm that may be connected to the same machine.
    unitree::robot::ChannelFactory::Instance()->Init(42);
    unitree::robot::ChannelSubscriber<
        unitree_arm::msg::dds_::PubServoInfo_>
        subscriber("current_servo_angle");
    subscriber.InitChannel(HandleAngles);
    unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>
        publisher("rt/arm_Command");
    publisher.InitChannel();

    std::this_thread::sleep_for(std::chrono::milliseconds(750));
    unitree_arm::msg::dds_::ArmString_ command;
    command.data_() =
        R"({"seq":4242,"address":1,"funcode":2,"data":{"mode":0,"angle0":10,"angle1":-10,"angle2":15,"angle3":5,"angle4":-5,"angle5":8,"angle6":45}})";
    publisher.Write(command);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    const int feedback_count = g_feedback_count.load();
    const float joint0_deg = g_joint0_deg.load();
    std::cout << "feedback_count=" << feedback_count
              << " joint0_deg=" << joint0_deg << std::endl;
    if (feedback_count < 10 || std::abs(joint0_deg - 10.0F) > 0.5F)
    {
        return 1;
    }
    return 0;
}

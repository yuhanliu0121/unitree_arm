#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/PubServoInfo_.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

constexpr double kPi = 3.14159265358979323846;

class D1JointStateBridge;
D1JointStateBridge* g_bridge = nullptr;

class D1JointStateBridge : public rclcpp::Node
{
public:
    D1JointStateBridge()
        : Node("d1_joint_state_bridge")
    {
        const std::string interface =
            declare_parameter<std::string>("interface", "eno1");
        const int domain_id = declare_parameter<int>("dds_domain_id", 0);
        feedback_topic_ = declare_parameter<std::string>(
            "feedback_topic", "current_servo_angle");
        const std::string joint_states_topic =
            declare_parameter<std::string>(
                "joint_states_topic", "/joint_states");
        closed_angle_deg_ =
            declare_parameter<double>("closed_angle_deg", -30.0);
        open_angle_deg_ =
            declare_parameter<double>("open_angle_deg", 60.0);
        finger_travel_m_ =
            declare_parameter<double>("finger_travel_m", 0.03);

        if (open_angle_deg_ <= closed_angle_deg_)
        {
            throw std::invalid_argument(
                "open_angle_deg must be greater than closed_angle_deg");
        }
        if (finger_travel_m_ <= 0.0)
        {
            throw std::invalid_argument(
                "finger_travel_m must be positive");
        }

        publisher_ = create_publisher<sensor_msgs::msg::JointState>(
            joint_states_topic, 10);

        unitree::robot::ChannelFactory::Instance()->Init(
            domain_id, interface);
        subscriber_ = std::make_unique<
            unitree::robot::ChannelSubscriber<
                unitree_arm::msg::dds_::PubServoInfo_>>(
                    feedback_topic_);

        g_bridge = this;
        subscriber_->InitChannel(HandleAngles);
        RCLCPP_INFO(
            get_logger(),
            "Read-only D1 feedback: DDS '%s' on '%s' -> ROS '%s'",
            feedback_topic_.c_str(),
            interface.c_str(),
            joint_states_topic.c_str());
    }

    ~D1JointStateBridge() override
    {
        if (g_bridge == this)
        {
            g_bridge = nullptr;
        }
    }

private:
    static void HandleAngles(const void* raw_message)
    {
        if (g_bridge == nullptr)
        {
            return;
        }
        const auto* feedback =
            static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(
                raw_message);
        g_bridge->Publish(*feedback);
    }

    void Publish(const unitree_arm::msg::dds_::PubServoInfo_& feedback)
    {
        const std::array<double, 6> arm_degrees = {
            feedback.servo0_data_(),
            feedback.servo1_data_(),
            feedback.servo2_data_(),
            feedback.servo3_data_(),
            feedback.servo4_data_(),
            feedback.servo5_data_(),
        };

        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        message.name = {
            "Joint0",
            "Joint1",
            "Joint2",
            "Joint3",
            "Joint4",
            "Joint5",
            "Joint6",
        };
        message.position.resize(message.name.size());
        for (std::size_t index = 0; index < arm_degrees.size(); ++index)
        {
            message.position[index] =
                arm_degrees[index] * kPi / 180.0;
        }

        const double gripper_angle_deg = feedback.servo6_data_();
        const double open_ratio = std::clamp(
            (gripper_angle_deg - closed_angle_deg_) /
                (open_angle_deg_ - closed_angle_deg_),
            0.0,
            1.0);
        message.position[6] = finger_travel_m_ * open_ratio;
        publisher_->publish(message);

        if (!received_first_feedback_)
        {
            received_first_feedback_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Received first D1 feedback and published /joint_states");
        }
    }

    std::string feedback_topic_;
    double closed_angle_deg_{-30.0};
    double open_angle_deg_{60.0};
    double finger_travel_m_{0.03};
    bool received_first_feedback_{false};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::unique_ptr<
        unitree::robot::ChannelSubscriber<
            unitree_arm::msg::dds_::PubServoInfo_>> subscriber_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<D1JointStateBridge>());
    rclcpp::shutdown();
    return 0;
}

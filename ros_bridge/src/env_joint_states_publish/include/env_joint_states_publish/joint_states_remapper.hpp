#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace env_joint_states_publish
{

class JointStatesRemapper : public rclcpp::Node
{
public:
    explicit JointStatesRemapper(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void jointStatesCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

} // namespace env_joint_states_publish

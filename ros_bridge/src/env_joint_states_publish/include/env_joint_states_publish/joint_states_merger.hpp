#pragma once

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

namespace env_joint_states_publish
{

class JointStatesMerger : public rclcpp::Node
{
public:
    explicit JointStatesMerger(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~JointStatesMerger();

private:
    // Callback for synchronized joint states
    void jointStatesCallback(
        const sensor_msgs::msg::JointState::ConstSharedPtr &motomini_msg,
        const sensor_msgs::msg::JointState::ConstSharedPtr &gantry_msg);

    // Message filter subscribers
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>>
        motomini_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>>
        gantry_sub_;

    // Synchronizer with approximate time policy
    using MySyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::JointState,
        sensor_msgs::msg::JointState>;
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    // Publisher for merged joint states
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr merged_pub_;

    // Synchronization mutex
    std::mutex mutex_;
};

} // namespace env_joint_states_publish

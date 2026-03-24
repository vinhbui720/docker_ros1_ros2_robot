#include "env_joint_states_publish/joint_states_merger.hpp"

using namespace std::chrono_literals;

namespace env_joint_states_publish
{

JointStatesMerger::JointStatesMerger(const rclcpp::NodeOptions &options)
    : Node("joint_states_merger", options)
{
    RCLCPP_INFO(get_logger(), "Initializing Joint States Merger node");

    // Create message filter subscribers
    motomini_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::JointState>>(
        this, "/motomini_joint_states");
    gantry_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::JointState>>(
        this, "/gantry_joint_states");

    // Create synchronizer with approximate time policy (500ms slop)
    sync_ = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), *motomini_sub_, *gantry_sub_);
    sync_->registerCallback(std::bind(
        &JointStatesMerger::jointStatesCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Create publisher for merged joint states
    merged_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    RCLCPP_INFO(get_logger(),
                "Joint States Merger ready. Subscribing to /motomini_joint_states "
                "and /gantry_joint_states, publishing to /joint_states");
}

JointStatesMerger::~JointStatesMerger()
{
}

void JointStatesMerger::jointStatesCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr &motomini_msg,
    const sensor_msgs::msg::JointState::ConstSharedPtr &gantry_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Create merged message
    auto merged = std::make_shared<sensor_msgs::msg::JointState>();
    merged->header.stamp = this->get_clock()->now();

    // Combine joint names, positions, velocities, and efforts
    merged->name.insert(merged->name.end(), motomini_msg->name.begin(),
                        motomini_msg->name.end());
    merged->name.insert(merged->name.end(), gantry_msg->name.begin(),
                        gantry_msg->name.end());

    merged->position.insert(merged->position.end(), motomini_msg->position.begin(),
                            motomini_msg->position.end());
    merged->position.insert(merged->position.end(), gantry_msg->position.begin(),
                            gantry_msg->position.end());

    merged->velocity.insert(merged->velocity.end(), motomini_msg->velocity.begin(),
                            motomini_msg->velocity.end());
    merged->velocity.insert(merged->velocity.end(), gantry_msg->velocity.begin(),
                            gantry_msg->velocity.end());

    merged->effort.insert(merged->effort.end(), motomini_msg->effort.begin(),
                          motomini_msg->effort.end());
    merged->effort.insert(merged->effort.end(), gantry_msg->effort.begin(),
                          gantry_msg->effort.end());

    merged_pub_->publish(*merged);

    RCLCPP_DEBUG(get_logger(),
                 "Published merged joint states: %zu motomini joints + %zu gantry joints",
                 motomini_msg->name.size(), gantry_msg->name.size());
}

} // namespace env_joint_states_publish

// ═══════════════════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<env_joint_states_publish::JointStatesMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

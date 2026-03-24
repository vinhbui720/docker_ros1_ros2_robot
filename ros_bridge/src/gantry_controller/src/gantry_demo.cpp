#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class MockGantryPub : public rclcpp::Node
{
public:
    MockGantryPub() : Node("mock_gantry_pub"), time_counter_(0.0)
    {
        // Matches the topic name from your ModbusHwBridge code
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("gantry_joint_states", 10);
        
        // Publish at roughly 20Hz
        timer_ = this->create_wall_timer(
            50ms, std::bind(&MockGantryPub::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Mock Gantry Publisher Started.");
    }

private:
    void timerCallback()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();
        
        // Matches the joint names from your ModbusHwBridge code
        msg.name = {"joint_x", "joint_z"};
        
        // Generate a smooth sine wave for testing (simulating movement in meters)
        double pos_x = std::sin(time_counter_);
        double pos_z = std::cos(time_counter_) * 0.5; // Z moves a bit less
        
        msg.position = {pos_x, pos_z};
        // Adding dummy velocity just in case your merger expects it
        msg.velocity = {0.1, 0.05}; 

        publisher_->publish(msg);
        time_counter_ += 0.05;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    double time_counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockGantryPub>());
    rclcpp::shutdown();
    return 0;
}
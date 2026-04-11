#include <string>
#include <cstdint>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <motoman_msgs/WriteSingleIO.h>

class MotominiToolServiceNode
{
public:
  MotominiToolServiceNode()
  {
    ros::NodeHandle pnh("~");
    pnh.param("tool_address", tool_address_, 10010);
    pnh.param<std::string>("write_single_io_service", write_single_io_service_, "/write_single_io");
    pnh.param("wait_for_service_timeout", wait_timeout_sec_, 5.0);

    io_client_ = nh_.serviceClient<motoman_msgs::WriteSingleIO>(write_single_io_service_);

    enable_srv_ = nh_.advertiseService("/tool_enable", &MotominiToolServiceNode::handleEnable, this);
    disable_srv_ = nh_.advertiseService("/tool_disable", &MotominiToolServiceNode::handleDisable, this);

    ROS_INFO("motomini_tool_service ready: /tool_enable and /tool_disable available");
  }

private:
  bool sendToolCommand(int value, std::string& message)
  {
    if (!ros::service::waitForService(write_single_io_service_, ros::Duration(wait_timeout_sec_)))
    {
      message = "Timed out waiting for " + write_single_io_service_;
      ROS_ERROR_STREAM(message);
      return false;
    }

    motoman_msgs::WriteSingleIO srv;
    srv.request.address = static_cast<uint32_t>(tool_address_);
    srv.request.value = static_cast<int32_t>(value);

    if (!io_client_.call(srv))
    {
      message = "Service call failed: " + write_single_io_service_;
      ROS_ERROR_STREAM(message);
      return false;
    }

    message = srv.response.message;
    return srv.response.success;
  }

  bool handleEnable(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::string message;
    const bool success = sendToolCommand(1, message);
    res.success = success;
    res.message = message;
    return true;
  }

  bool handleDisable(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    std::string message;
    const bool success = sendToolCommand(0, message);
    res.success = success;
    res.message = message;
    return true;
  }

  ros::NodeHandle nh_;
  ros::ServiceClient io_client_;
  ros::ServiceServer enable_srv_;
  ros::ServiceServer disable_srv_;

  int tool_address_{10010};
  double wait_timeout_sec_{5.0};
  std::string write_single_io_service_{"/write_single_io"};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motomini_tool_service");
  MotominiToolServiceNode node;
  ros::spin();
  return 0;
}
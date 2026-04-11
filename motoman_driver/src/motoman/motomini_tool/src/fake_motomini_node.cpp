#include <algorithm>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/StopMotion.h>
#include <motoman_msgs/ReadGroupIO.h>
#include <motoman_msgs/ReadMRegister.h>
#include <motoman_msgs/ReadSingleIO.h>
#include <motoman_msgs/SelectTool.h>
#include <motoman_msgs/WriteGroupIO.h>
#include <motoman_msgs/WriteMRegister.h>
#include <motoman_msgs/WriteSingleIO.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

class FakeMotominiNode
{
public:
  FakeMotominiNode()
    : action_server_(nh_, "/joint_trajectory_action", false)
  {
    ros::NodeHandle pnh("~");
    pnh.param("publish_rate_hz", publish_rate_hz_, 50.0);

    if (!pnh.getParam("joint_names", joint_names_) || joint_names_.empty())
    {
      joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    }

    current_positions_.assign(joint_names_.size(), 0.0);
    current_velocities_.assign(joint_names_.size(), 0.0);

    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("/feedback_states", 10);
    status_pub_ = nh_.advertise<industrial_msgs::RobotStatus>("/robot_status", 10);

    joint_path_sub_ = nh_.subscribe("/joint_path_command", 1, &FakeMotominiNode::jointPathCb, this);

    robot_enable_srv_ = nh_.advertiseService("/robot_enable", &FakeMotominiNode::robotEnableCb, this);
    robot_disable_srv_ = nh_.advertiseService("/robot_disable", &FakeMotominiNode::robotDisableCb, this);
    stop_motion_srv_ = nh_.advertiseService("/stop_motion", &FakeMotominiNode::stopMotionCb, this);

    read_single_io_srv_ = nh_.advertiseService("/read_single_io", &FakeMotominiNode::readSingleIoCb, this);
    write_single_io_srv_ = nh_.advertiseService("/write_single_io", &FakeMotominiNode::writeSingleIoCb, this);
    read_group_io_srv_ = nh_.advertiseService("/read_group_io", &FakeMotominiNode::readGroupIoCb, this);
    write_group_io_srv_ = nh_.advertiseService("/write_group_io", &FakeMotominiNode::writeGroupIoCb, this);
    read_mregister_srv_ = nh_.advertiseService("/read_mregister", &FakeMotominiNode::readMRegisterCb, this);
    write_mregister_srv_ = nh_.advertiseService("/write_mregister", &FakeMotominiNode::writeMRegisterCb, this);
    select_tool_srv_ = nh_.advertiseService("/select_tool", &FakeMotominiNode::selectToolCb, this);

    action_server_.registerGoalCallback(boost::bind(&FakeMotominiNode::goalCb, this));
    action_server_.registerPreemptCallback(boost::bind(&FakeMotominiNode::preemptCb, this));
    action_server_.start();

    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_), &FakeMotominiNode::timerCb, this);

    ROS_INFO("Fake MotoMini node ready with simulated topics/actions/services");
  }

private:
  void timerCb(const ros::TimerEvent&)
  {
    publishJointAndFeedback();
    publishRobotStatus();
  }

  void publishJointAndFeedback()
  {
    sensor_msgs::JointState joint_msg;
    control_msgs::FollowJointTrajectoryFeedback fb_msg;

    std::lock_guard<std::mutex> lock(state_mutex_);
    const ros::Time now = ros::Time::now();

    joint_msg.header.stamp = now;
    joint_msg.name = joint_names_;
    joint_msg.position = current_positions_;
    joint_msg.velocity = current_velocities_;
    joint_state_pub_.publish(joint_msg);

    fb_msg.header.stamp = now;
    fb_msg.joint_names = joint_names_;
    fb_msg.actual.positions = current_positions_;
    fb_msg.actual.velocities = current_velocities_;
    fb_msg.desired.positions = current_positions_;
    fb_msg.error.positions.assign(joint_names_.size(), 0.0);
    feedback_pub_.publish(fb_msg);
  }

  void publishRobotStatus()
  {
    industrial_msgs::RobotStatus status;
    status.header.stamp = ros::Time::now();
    status.mode.val = 2;
    status.e_stopped.val = 0;
    status.drives_powered.val = drives_powered_ ? 1 : 0;
    status.motion_possible.val = motion_possible_ ? 1 : 0;
    status.in_motion.val = in_motion_ ? 1 : 0;
    status.in_error.val = in_error_ ? 1 : 0;
    status.error_code = 0;
    status_pub_.publish(status);
  }

  void goalCb()
  {
    auto goal = action_server_.acceptNewGoal();

    if (!drives_powered_ || !motion_possible_)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_.setAborted(result, "Robot is disabled");
      return;
    }

    if (goal->trajectory.points.empty())
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      action_server_.setSucceeded(result);
      return;
    }

    const trajectory_msgs::JointTrajectoryPoint& final_point = goal->trajectory.points.back();
    std::vector<int> mapping;
    mapping.reserve(goal->trajectory.joint_names.size());
    for (const std::string& name : goal->trajectory.joint_names)
    {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
      if (it == joint_names_.end())
      {
        mapping.push_back(-1);
      }
      else
      {
        mapping.push_back(static_cast<int>(std::distance(joint_names_.begin(), it)));
      }
    }

    in_motion_ = true;
    const ros::Time started = ros::Time::now();
    const double duration = final_point.time_from_start.toSec();
    ros::Rate wait_rate(100.0);

    while (ros::ok() && (ros::Time::now() - started).toSec() < duration)
    {
      if (action_server_.isPreemptRequested() || !in_motion_)
      {
        in_motion_ = false;
        action_server_.setPreempted();
        return;
      }
      wait_rate.sleep();
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const size_t count = std::min(final_point.positions.size(), mapping.size());
      for (size_t i = 0; i < count; ++i)
      {
        const int mapped_idx = mapping[i];
        if (mapped_idx >= 0)
        {
          current_positions_[mapped_idx] = final_point.positions[i];
          current_velocities_[mapped_idx] = 0.0;
        }
      }
    }

    in_motion_ = false;
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    action_server_.setSucceeded(result);
  }

  void preemptCb()
  {
    in_motion_ = false;
    action_server_.setPreempted();
  }

  void jointPathCb(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {
    if (msg->points.empty())
    {
      return;
    }

    const trajectory_msgs::JointTrajectoryPoint& pt = msg->points.front();

    std::vector<int> mapping;
    mapping.reserve(msg->joint_names.size());
    for (const std::string& name : msg->joint_names)
    {
      auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
      if (it == joint_names_.end())
      {
        mapping.push_back(-1);
      }
      else
      {
        mapping.push_back(static_cast<int>(std::distance(joint_names_.begin(), it)));
      }
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    const size_t pos_count = std::min(pt.positions.size(), mapping.size());
    for (size_t i = 0; i < pos_count; ++i)
    {
      const int mapped_idx = mapping[i];
      if (mapped_idx >= 0)
      {
        current_positions_[mapped_idx] = pt.positions[i];
      }
    }

    const size_t vel_count = std::min(pt.velocities.size(), mapping.size());
    for (size_t i = 0; i < vel_count; ++i)
    {
      const int mapped_idx = mapping[i];
      if (mapped_idx >= 0)
      {
        current_velocities_[mapped_idx] = pt.velocities[i];
      }
    }
  }

  bool robotEnableCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    drives_powered_ = true;
    motion_possible_ = true;
    res.success = true;
    res.message = "Robot Enabled";
    return true;
  }

  bool robotDisableCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
  {
    drives_powered_ = false;
    motion_possible_ = false;
    in_motion_ = false;
    res.success = true;
    res.message = "Robot Disabled";
    return true;
  }

  bool stopMotionCb(industrial_msgs::StopMotion::Request&, industrial_msgs::StopMotion::Response& res)
  {
    in_motion_ = false;
    if (action_server_.isActive())
    {
      action_server_.setPreempted();
    }
    res.code.val = 0;
    return true;
  }

  bool readSingleIoCb(motoman_msgs::ReadSingleIO::Request& req, motoman_msgs::ReadSingleIO::Response& res)
  {
    const auto it = single_io_.find(req.address);
    res.value = (it == single_io_.end()) ? 0 : it->second;
    res.success = true;
    res.message = "Success";
    return true;
  }

  bool writeSingleIoCb(motoman_msgs::WriteSingleIO::Request& req, motoman_msgs::WriteSingleIO::Response& res)
  {
    single_io_[req.address] = req.value;
    res.success = true;
    res.message = "Success";
    return true;
  }

  bool readGroupIoCb(motoman_msgs::ReadGroupIO::Request& req, motoman_msgs::ReadGroupIO::Response& res)
  {
    const auto it = group_io_.find(req.address);
    res.value = (it == group_io_.end()) ? 0 : it->second;
    res.success = true;
    res.message = "Success";
    return true;
  }

  bool writeGroupIoCb(motoman_msgs::WriteGroupIO::Request& req, motoman_msgs::WriteGroupIO::Response& res)
  {
    group_io_[req.address] = req.value;
    res.success = true;
    res.message = "Success";
    return true;
  }

  bool readMRegisterCb(motoman_msgs::ReadMRegister::Request& req, motoman_msgs::ReadMRegister::Response& res)
  {
    const auto it = mregisters_.find(req.address);
    res.value = (it == mregisters_.end()) ? 0 : it->second;
    res.success = true;
    res.message = "Success";
    return true;
  }

  bool writeMRegisterCb(motoman_msgs::WriteMRegister::Request& req, motoman_msgs::WriteMRegister::Response& res)
  {
    mregisters_[req.address] = req.value;
    res.success = true;
    res.message = "Success";
    return true;
  }

  bool selectToolCb(motoman_msgs::SelectTool::Request& req, motoman_msgs::SelectTool::Response& res)
  {
    active_tool_by_group_[req.group_number] = req.tool_number;
    res.success = true;
    res.message = "Success";
    return true;
  }

  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Publisher joint_state_pub_;
  ros::Publisher feedback_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber joint_path_sub_;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;

  ros::ServiceServer robot_enable_srv_;
  ros::ServiceServer robot_disable_srv_;
  ros::ServiceServer stop_motion_srv_;

  ros::ServiceServer read_single_io_srv_;
  ros::ServiceServer write_single_io_srv_;
  ros::ServiceServer read_group_io_srv_;
  ros::ServiceServer write_group_io_srv_;
  ros::ServiceServer read_mregister_srv_;
  ros::ServiceServer write_mregister_srv_;
  ros::ServiceServer select_tool_srv_;

  std::mutex state_mutex_;

  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::vector<double> current_velocities_;

  bool motion_possible_{true};
  bool in_motion_{false};
  bool in_error_{false};
  bool drives_powered_{true};

  double publish_rate_hz_{50.0};

  std::map<uint32_t, int32_t> single_io_;
  std::map<uint32_t, uint8_t> group_io_;
  std::map<uint32_t, uint16_t> mregisters_;
  std::map<uint32_t, uint32_t> active_tool_by_group_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_motomini_node");
  FakeMotominiNode node;
  ros::spin();
  return 0;
}
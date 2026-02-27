#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory

def callback(msg):
    rospy.loginfo("Received JointTrajectory")
    rospy.loginfo("Joint names: %s", msg.joint_names)
    rospy.loginfo("Number of points: %d", len(msg.points))

def main():
    rospy.init_node("joint_trajectory_receiver")
    rospy.Subscriber(
        "/joint_trajectory",
        JointTrajectory,
        callback,
        queue_size=10
    )
    rospy.loginfo("ROS1 JointTrajectory receiver started")
    rospy.spin()

if __name__ == "__main__":
    main()

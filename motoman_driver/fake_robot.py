#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult

class FakeRobot:
    def __init__(self):
        rospy.init_node('fake_robot_node')
        self.server = actionlib.SimpleActionServer(
            '/joint_trajectory_action', 
            FollowJointTrajectoryAction, 
            self.execute, 
            False
        )
        self.server.start()
        rospy.loginfo("ROS 1 Fake Robot is ready on /joint_trajectory_action")

    def execute(self, goal):
        rospy.loginfo("Received trajectory goal!")
        rospy.sleep(1.0) # Pretend to move
        res = FollowJointTrajectoryResult()
        res.error_code = 0 # SUCCESS
        self.server.set_succeeded(res)

if __name__ == '__main__':
    FakeRobot()
    rospy.spin()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__("joint_trajectory_publisher")
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory",
            10
        )
        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2"]

        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 1

        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info("Published JointTrajectory")

def main():
    rclpy.init()
    node = JointTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import threading
import time

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from industrial_msgs.msg import RobotStatus
from industrial_msgs.srv import StopMotion
from industrial_msgs.srv import StopMotionResponse
from motoman_msgs.srv import ReadGroupIO
from motoman_msgs.srv import ReadGroupIOResponse
from motoman_msgs.srv import ReadMRegister
from motoman_msgs.srv import ReadMRegisterResponse
from motoman_msgs.srv import ReadSingleIO
from motoman_msgs.srv import ReadSingleIOResponse
from motoman_msgs.srv import SelectTool
from motoman_msgs.srv import SelectToolResponse
from motoman_msgs.srv import WriteGroupIO
from motoman_msgs.srv import WriteGroupIOResponse
from motoman_msgs.srv import WriteMRegister
from motoman_msgs.srv import WriteMRegisterResponse
from motoman_msgs.srv import WriteSingleIO
from motoman_msgs.srv import WriteSingleIOResponse
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
from trajectory_msgs.msg import JointTrajectory


class FakeMotominiNode:
    def __init__(self):
        rospy.init_node('fake_motomini_node')

        self.joint_names = rospy.get_param('~joint_names', [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ])
        self.publish_rate = rospy.get_param('~publish_rate_hz', 50.0)

        self.num_joints = len(self.joint_names)
        self.current_positions = [0.0] * self.num_joints
        self.current_velocities = [0.0] * self.num_joints

        self.motion_possible = True
        self.in_motion = False
        self.in_error = False
        self.drives_powered = True

        self.single_io = {}
        self.group_io = {}
        self.mregisters = {}
        self.active_tool_by_group = {}

        self.state_lock = threading.Lock()

        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.feedback_pub = rospy.Publisher('/feedback_states', FollowJointTrajectoryFeedback, queue_size=10)
        self.status_pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)

        rospy.Subscriber('/joint_path_command', JointTrajectory, self.joint_path_cb, queue_size=1)

        self.action_server = actionlib.SimpleActionServer(
            '/joint_trajectory_action',
            FollowJointTrajectoryAction,
            execute_cb=self.execute_trajectory_cb,
            auto_start=False
        )
        self.action_server.start()

        rospy.Service('/robot_enable', Trigger, self.handle_enable)
        rospy.Service('/robot_disable', Trigger, self.handle_disable)
        rospy.Service('/stop_motion', StopMotion, self.handle_stop_motion)

        rospy.Service('/read_single_io', ReadSingleIO, self.read_single_io_cb)
        rospy.Service('/write_single_io', WriteSingleIO, self.write_single_io_cb)
        rospy.Service('/read_group_io', ReadGroupIO, self.read_group_io_cb)
        rospy.Service('/write_group_io', WriteGroupIO, self.write_group_io_cb)
        rospy.Service('/read_mregister', ReadMRegister, self.read_mregister_cb)
        rospy.Service('/write_mregister', WriteMRegister, self.write_mregister_cb)
        rospy.Service('/select_tool', SelectTool, self.select_tool_cb)

        rospy.loginfo('Fake MotoMini node ready with simulated topics/actions/services')
        self.publish_loop()

    def publish_loop(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_joint_and_feedback_states()
            self.publish_robot_status()
            rate.sleep()

    def publish_joint_and_feedback_states(self):
        now = rospy.Time.now()

        with self.state_lock:
            positions = list(self.current_positions)
            velocities = list(self.current_velocities)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = now
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        self.joint_state_pub.publish(joint_state_msg)

        feedback_msg = FollowJointTrajectoryFeedback()
        feedback_msg.header.stamp = now
        feedback_msg.joint_names = self.joint_names
        feedback_msg.actual.positions = positions
        feedback_msg.actual.velocities = velocities
        feedback_msg.desired.positions = positions
        feedback_msg.error.positions = [0.0] * self.num_joints
        self.feedback_pub.publish(feedback_msg)

    def publish_robot_status(self):
        msg = RobotStatus()
        msg.header.stamp = rospy.Time.now()
        msg.mode.val = 2
        msg.e_stopped.val = 0
        msg.drives_powered.val = 1 if self.drives_powered else 0
        msg.motion_possible.val = 1 if self.motion_possible else 0
        msg.in_motion.val = 1 if self.in_motion else 0
        msg.in_error.val = 1 if self.in_error else 0
        msg.error_code = 0
        self.status_pub.publish(msg)

    def execute_trajectory_cb(self, goal):
        if not self.drives_powered or not self.motion_possible:
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
            self.action_server.set_aborted(result, 'Robot is disabled')
            return

        points = goal.trajectory.points
        if not points:
            result = FollowJointTrajectoryResult()
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self.action_server.set_succeeded(result)
            return

        index_by_name = {name: idx for idx, name in enumerate(self.joint_names)}
        mapped_indices = [index_by_name.get(name, -1) for name in goal.trajectory.joint_names]

        self.in_motion = True
        start_time = rospy.Time.now()
        total_duration = points[-1].time_from_start.to_sec()

        while (rospy.Time.now() - start_time).to_sec() < total_duration:
            if self.action_server.is_preempt_requested() or not self.in_motion:
                self.in_motion = False
                self.action_server.set_preempted()
                return
            time.sleep(0.01)

        final_point = points[-1]
        with self.state_lock:
            for pos_idx, value in enumerate(final_point.positions):
                if pos_idx >= len(mapped_indices):
                    break
                joint_idx = mapped_indices[pos_idx]
                if joint_idx >= 0:
                    self.current_positions[joint_idx] = value
                    self.current_velocities[joint_idx] = 0.0

        self.in_motion = False
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.action_server.set_succeeded(result)

    def joint_path_cb(self, msg):
        if not msg.points:
            return

        first_point = msg.points[0]
        index_by_name = {name: idx for idx, name in enumerate(self.joint_names)}
        mapped_indices = [index_by_name.get(name, -1) for name in msg.joint_names]

        with self.state_lock:
            for pos_idx, value in enumerate(first_point.positions):
                if pos_idx >= len(mapped_indices):
                    break
                joint_idx = mapped_indices[pos_idx]
                if joint_idx >= 0:
                    self.current_positions[joint_idx] = value

            if first_point.velocities:
                for vel_idx, value in enumerate(first_point.velocities):
                    if vel_idx >= len(mapped_indices):
                        break
                    joint_idx = mapped_indices[vel_idx]
                    if joint_idx >= 0:
                        self.current_velocities[joint_idx] = value

    def handle_enable(self, _req):
        self.drives_powered = True
        self.motion_possible = True
        return TriggerResponse(success=True, message='Robot Enabled')

    def handle_disable(self, _req):
        self.drives_powered = False
        self.motion_possible = False
        self.in_motion = False
        return TriggerResponse(success=True, message='Robot Disabled')

    def handle_stop_motion(self, _req):
        self.in_motion = False
        if self.action_server.is_active():
            self.action_server.set_preempted()
        return StopMotionResponse(code=0)

    def read_single_io_cb(self, req):
        value = self.single_io.get(req.address, 0)
        return ReadSingleIOResponse(message='Success', success=True, value=value)

    def write_single_io_cb(self, req):
        self.single_io[req.address] = req.value
        return WriteSingleIOResponse(message='Success', success=True)

    def read_group_io_cb(self, req):
        value = self.group_io.get(req.address, 0)
        return ReadGroupIOResponse(message='Success', success=True, value=value)

    def write_group_io_cb(self, req):
        self.group_io[req.address] = req.value
        return WriteGroupIOResponse(message='Success', success=True)

    def read_mregister_cb(self, req):
        value = self.mregisters.get(req.address, 0)
        return ReadMRegisterResponse(message='Success', success=True, value=value)

    def write_mregister_cb(self, req):
        self.mregisters[req.address] = req.value
        return WriteMRegisterResponse(message='Success', success=True)

    def select_tool_cb(self, req):
        self.active_tool_by_group[req.group_number] = req.tool_number
        return SelectToolResponse(message='Success', success=True)


if __name__ == '__main__':
    try:
        FakeMotominiNode()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
import actionlib
import threading
import time

# ROS Messages
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
from trajectory_msgs.msg import JointTrajectory
from industrial_msgs.msg import RobotStatus, TriState
from std_srvs.srv import Trigger, TriggerResponse
from industrial_msgs.srv import StopMotion, StopMotionResponse

# Motoman specific messages
# We wrap these in try-except in case you run this on a machine without motoman_msgs
try:
    from motoman_msgs.srv import (
        ReadSingleIO, ReadSingleIOResponse,
        WriteSingleIO, WriteSingleIOResponse,
        ReadGroupIO, ReadGroupIOResponse,
        WriteGroupIO, WriteGroupIOResponse,
        ReadMRegister, ReadMRegisterResponse,
        WriteMRegister, WriteMRegisterResponse,
        SelectTool, SelectToolResponse
    )
    MOTOMAN_MSGS_AVAILABLE = True
except ImportError:
    rospy.logwarn("motoman_msgs not found. IO services will not be advertised.")
    MOTOMAN_MSGS_AVAILABLE = False

class FakeMotomanRobot:
    def __init__(self):
        rospy.init_node('fake_motoman_robot')

        # --- CONFIGURATION ---
        # UPDATE THESE TO MATCH YOUR REAL ROBOT'S URDF JOINT NAMES
        # Common defaults: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        # Or: ['group_1/joint_1', 'group_1/joint_2', ...]
        self.joint_names = rospy.get_param('~joint_names', [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ])
        
        self.num_joints = len(self.joint_names)
        self.current_positions = [0.0] * self.num_joints
        self.lock = threading.Lock()
        
        # Robot Status State
        self.motion_possible = True
        self.in_motion = False
        self.in_error = False
        self.drives_powered = True

        # --- PUBLISHERS ---
        # 1. Joint States (standard ROS)
        self.joint_state_pub = rospy.Publisher('/motomini_joint_states', JointState, queue_size=10)
        
        # 2. Robot Status (Industrial Robot standard)
        self.status_pub = rospy.Publisher('/robot_status', RobotStatus, queue_size=10)
        
        # 3. Feedback (Motoman specific feedback loop)
        self.feedback_pub = rospy.Publisher('/feedback_states', FollowJointTrajectoryFeedback, queue_size=10)

        # --- SUBSCRIBERS ---
        # Listen for raw streaming commands
        rospy.Subscriber('/joint_path_command', JointTrajectory, self.joint_path_cb)

        # --- ACTION SERVERS ---
        # The main interface for MoveIt!
        self.as_name = '/joint_trajectory_action'
        self.action_server = actionlib.SimpleActionServer(
            self.as_name, 
            FollowJointTrajectoryAction, 
            execute_cb=self.execute_trajectory_cb, 
            auto_start=False
        )
        self.action_server.start()

        # --- SERVICES ---
        # Standard Industrial Services
        rospy.Service('/robot_enable', Trigger, self.handle_enable)
        rospy.Service('/robot_disable', Trigger, self.handle_disable)
        rospy.Service('/stop_motion', StopMotion, self.handle_stop)
        
        # Motoman Specific IO Services
        if MOTOMAN_MSGS_AVAILABLE:
            rospy.Service('/read_single_io', ReadSingleIO, self.mock_read_io)
            rospy.Service('/write_single_io', WriteSingleIO, self.mock_write_io)
            rospy.Service('/read_group_io', ReadGroupIO, self.mock_read_group)
            rospy.Service('/write_group_io', WriteGroupIO, self.mock_write_group)
            rospy.Service('/read_mregister', ReadMRegister, self.mock_read_mreg)
            rospy.Service('/write_mregister', WriteMRegister, self.mock_write_mreg)
            rospy.Service('/select_tool', SelectTool, self.mock_select_tool)

        rospy.loginfo("Fake Motoman Robot Ready!")
        rospy.loginfo(f"Simulating joints: {self.joint_names}")

        # Start the state publishing loop
        self.publish_loop()

    def publish_loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            self.publish_joint_states()
            self.publish_robot_status()
            rate.sleep()

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        with self.lock:
            msg.position = list(self.current_positions)
        msg.velocity = [0.0] * self.num_joints
        msg.effort = [0.0] * self.num_joints
        self.joint_state_pub.publish(msg)

    def publish_robot_status(self):
        msg = RobotStatus()
        msg.header.stamp = rospy.Time.now()
        # Direct integer assignment to avoid AttributeError if constants are missing
        # 1 = Manual, 2 = Auto
        msg.mode.val = 2 
        msg.e_stopped.val = 0
        msg.drives_powered.val = 1 if self.drives_powered else 0
        msg.motion_possible.val = 1 if self.motion_possible else 0
        msg.in_motion.val = 1 if self.in_motion else 0
        msg.in_error.val = 0
        msg.error_code = 0
        self.status_pub.publish(msg)

    # --- ACTION CALLBACK (For MoveIt) ---
    def execute_trajectory_cb(self, goal):
        rospy.loginfo("Received Trajectory Goal")
        
        trajectory = goal.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points

        if not points:
            self.action_server.set_succeeded()
            return

        # Simple simulation: Move to the final point over a short duration
        # (In a real sim you'd interpolate, here we just jump or slide strictly)
        
        self.in_motion = True
        
        # Simulate movement duration (speed up 2x for testing)
        # We take the time_from_start of the last point
        duration = points[-1].time_from_start.to_sec()
        
        # Mapping goal joint names to our internal indices
        indices = []
        for name in joint_names:
            if name in self.joint_names:
                indices.append(self.joint_names.index(name))
            else:
                indices.append(-1)

        # Go through points to simulate checking constraints/time
        # For this fake, we just sleep for the duration and update position
        start_time = rospy.Time.now()
        
        # Update Loop
        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Trajectory Preempted")
                self.action_server.set_preempted()
                self.in_motion = False
                return
            
            # Here we could interpolate. For simplicity, we just keep the previous pos
            # until the end, or perform a simple linear slide if we wanted complex code.
            # To be safe for testing: We will update to the FINAL position at the END.
            time.sleep(0.1)

        # Reached end of trajectory time: Snap to final positions
        final_point = points[-1]
        with self.lock:
            for i, val in enumerate(final_point.positions):
                idx = indices[i]
                if idx != -1:
                    self.current_positions[idx] = val
        
        self.in_motion = False
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.action_server.set_succeeded(result)
        rospy.loginfo("Trajectory Execution Complete")

    # --- TOPIC CALLBACK (For Streaming) ---
    def joint_path_cb(self, msg):
        # This topic is usually used for real-time streaming (servo)
        # We just snap to the last point immediately
        if not msg.points:
            return
            
        rospy.logdebug("Received streaming path command")
        target = msg.points[0].positions # Assuming single point streaming usually
        
        # Map names
        indices = []
        for name in msg.joint_names:
            if name in self.joint_names:
                indices.append(self.joint_names.index(name))
            else:
                indices.append(-1)
                
        with self.lock:
            for i, val in enumerate(target):
                idx = indices[i]
                if idx != -1:
                    self.current_positions[idx] = val

    # --- SERVICE HANDLERS ---
    def handle_enable(self, req):
        rospy.loginfo("Service: Robot Enabled")
        self.drives_powered = True
        self.motion_possible = True
        return TriggerResponse(success=True, message="Robot Enabled")

    def handle_disable(self, req):
        rospy.loginfo("Service: Robot Disabled")
        self.drives_powered = False
        return TriggerResponse(success=True, message="Robot Disabled")

    def handle_stop(self, req):
        rospy.loginfo("Service: Stop Motion")
        self.in_motion = False
        # In a real driver, this would cancel the active goal
        # 0 = STOP_COMMANDED
        return StopMotionResponse(code=0)

    # --- MOCK IO SERVICES ---
    def mock_read_io(self, req):
        # Always return 0
        return ReadSingleIOResponse(value=0)

    def mock_write_io(self, req):
        rospy.loginfo(f"Mock IO Write: Address {req.address}, Value {req.value}")
        return WriteSingleIOResponse(message="Success", success=True)

    def mock_read_group(self, req):
        return ReadGroupIOResponse(value=0)

    def mock_write_group(self, req):
        rospy.loginfo(f"Mock Group Write: Address {req.address}, Value {req.value}")
        return WriteGroupIOResponse(message="Success", success=True)

    def mock_read_mreg(self, req):
        return ReadMRegisterResponse(value=0)

    def mock_write_mreg(self, req):
        rospy.loginfo(f"Mock MRegister Write: Index {req.register_number}, Value {req.value}")
        return WriteMRegisterResponse(message="Success", success=True)
        
    def mock_select_tool(self, req):
        rospy.loginfo(f"Mock Tool Select: Group {req.group_number}, Tool {req.tool_number}")
        return SelectToolResponse(message="Success", success=True)

if __name__ == '__main__':
    try:
        node = FakeMotomanRobot()
    except rospy.ROSInterruptException:
        pass
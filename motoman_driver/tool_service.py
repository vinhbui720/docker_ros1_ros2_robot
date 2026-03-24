#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from motoman_msgs.srv import WriteSingleIO, WriteSingleIORequest

class YaskawaToolWrapper:
    def __init__(self):
        rospy.init_node('yaskawa_tool_wrapper_node')
        
        # The specific I/O address for the tool
        self.tool_address = 10010
        
        # 1. Setup the client to talk to the real Yaskawa controller
        self.yaskawa_service_name = '/write_single_io'
        rospy.loginfo("Waiting for Yaskawa service: %s", self.yaskawa_service_name)
        rospy.wait_for_service(self.yaskawa_service_name)
        self.io_client = rospy.ServiceProxy(self.yaskawa_service_name, WriteSingleIO)
        rospy.loginfo("Connected to Yaskawa I/O service!")

        # 2. Setup standard Trigger services that ROS 2 can easily call
        self.enable_srv = rospy.Service('/tool_enable', Trigger, self.handle_enable)
        self.disable_srv = rospy.Service('/tool_disable', Trigger, self.handle_disable)
        
        rospy.loginfo("Tool wrapper ready. You can now call /tool_enable or /tool_disable from ROS 2.")

    def send_tool_command(self, value):
        """Helper function to build and send the custom Yaskawa message."""
        req = WriteSingleIORequest()
        req.address = self.tool_address
        req.value = value
        
        try:
            response = self.io_client(req)
            return response.success, response.message
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False, str(e)

    def handle_enable(self, req):
        """Callback for the standard ROS 2 /tool_enable trigger."""
        rospy.loginfo("Received TOOL ENABLE request from ROS 2...")
        success, message = self.send_tool_command(1) # Send 1 to turn tool ON
        return TriggerResponse(success=success, message=message)

    def handle_disable(self, req):
        """Callback for the standard ROS 2 /tool_disable trigger."""
        rospy.loginfo("Received TOOL DISABLE request from ROS 2...")
        success, message = self.send_tool_command(0) # Send 0 to turn tool OFF
        return TriggerResponse(success=success, message=message)

if __name__ == '__main__':
    try:
        wrapper = YaskawaToolWrapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
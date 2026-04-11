#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from motoman_msgs.srv import WriteSingleIO, WriteSingleIORequest


class MotominiToolServiceNode:
    def __init__(self):
        rospy.init_node('motomini_tool_service')

        self.tool_address = rospy.get_param('~tool_address', 10010)
        self.write_single_io_service = rospy.get_param('~write_single_io_service', '/write_single_io')
        self.wait_timeout = rospy.get_param('~wait_for_service_timeout', 5.0)

        self.io_client = rospy.ServiceProxy(self.write_single_io_service, WriteSingleIO)

        rospy.Service('/tool_enable', Trigger, self.handle_enable)
        rospy.Service('/tool_disable', Trigger, self.handle_disable)

        rospy.loginfo('motomini_tool_service ready: /tool_enable and /tool_disable available')

    def send_tool_command(self, value):
        req = WriteSingleIORequest()
        req.address = self.tool_address
        req.value = value

        try:
            rospy.wait_for_service(self.write_single_io_service, timeout=self.wait_timeout)
            res = self.io_client(req)
            return res.success, res.message
        except rospy.ROSException as err:
            rospy.logerr('Timed out waiting for %s: %s', self.write_single_io_service, err)
            return False, str(err)
        except rospy.ServiceException as err:
            rospy.logerr('Service call to %s failed: %s', self.write_single_io_service, err)
            return False, str(err)

    def handle_enable(self, _req):
        success, message = self.send_tool_command(1)
        return TriggerResponse(success=success, message=message)

    def handle_disable(self, _req):
        success, message = self.send_tool_command(0)
        return TriggerResponse(success=success, message=message)


if __name__ == '__main__':
    try:
        MotominiToolServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
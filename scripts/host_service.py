#!/usr/bin/env python
import rospy
import time
from mambo_fly.srv import *
from std_msgs.msg import String
from mavros_msgs.msg import AttitudeTarget
from pyparrot.Minidrone import Mambo

class MamboHost:

    def __init__(self):
        self.host_name = rospy.get_param('~name')
        self.host_mac = rospy.get_param('~mac')
        self.service_connect = rospy.Service('/mambo_srv/' + self.host_name + '/connect', Connect, self.handle_connect)
        self.service_take_off = rospy.Service('/mambo_srv/' + self.host_name + '/takeoff', Connect, self.handle_takeoff)
        self.service_land = rospy.Service('/mambo_srv/' + self.host_name + '/land', Connect, self.handle_land)
        self.sub_disconnect = rospy.Subscriber('/mambo_srv/' + self.host_name + '/disconnect', String, self.callback_disconnect)
        self.sub_fly_command = rospy.Subscriber('/mambo_srv/' + self.host_name + '/fly_command', AttitudeTarget, self.callback_fly_command)

        self.mambo = Mambo(self.host_mac, use_wifi=False)

    def handle_connect(self, req):
        rospy.loginfo(rospy.get_caller_id() + ', I heard %s', req.connect)   
        if (req.connect):
            success = self.mambo.connect(num_retries=3)
            rospy.loginfo(rospy.get_caller_id() + ' connected: %s', success)
            return success
        else:
            rospy.loginfo(rospy.get_caller_id() + ' connection not requested.')
            return False

    def handle_takeoff(self, req):
        rospy.loginfo(rospy.get_caller_id() + ', I heard %s', req.connect)   
        if (req.connect):
            success_msg = self.mambo.takeoff()
            rospy.loginfo(rospy.get_caller_id() + " take off requested: %s", success_msg)    
            time.sleep(0.5)
            success = self.mambo.is_takeoff()
            rospy.loginfo(rospy.get_caller_id() + " has taken off: %s", success)    
            return success
        else:
            rospy.loginfo(rospy.get_caller_id() + ' take off not requested.')
            return False

    def handle_land(self, req):
        rospy.loginfo(rospy.get_caller_id() + ', I heard %s', req.connect)   
        if (req.connect):
            success = self.mambo.safe_land(5)
            rospy.loginfo(rospy.get_caller_id() + " landing: %s", success)    
            return True
        else:
            rospy.loginfo(rospy.get_caller_id() + ' landing not requested.')
            return False

    def callback_disconnect(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)    
        success = self.mambo.disconnect()
        # rospy.loginfo(rospy.get_caller_id() + ' disconnected: %s', success)

    def callback_fly_command(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' flight command: %f' % data.thrust + 'successful.')
        success = self.mambo.fly_direct(roll=data.orientation.x, pitch=data.orientation.y, yaw=0, vertical_movement=data.thrust, duration=0.2)


if __name__ == '__main__':
    rospy.init_node('mambo_host', anonymous=True)

    mambo = MamboHost()
    rospy.spin()
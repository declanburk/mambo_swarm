#!/usr/bin/env python3
import rospy
from mambo_fly.srv import *
from std_msgs.msg import String
from mavros_msgs.msg import AttitudeTarget
## Import Mambo fly class to handle conneciton and initialise with config.
from pyparrot.Minidrone import Mambo


def handle_connect(req):
    rospy.loginfo(rospy.get_caller_id() + ', I heard %s', req.connect)   
    if (req.connect):
        success = mambo.connect(num_retries=3)
        rospy.loginfo(rospy.get_caller_id() + ' connected: %s', success)
        return success
    else:
        rospy.loginfo(rospy.get_caller_id() + ' connection not requested.')
        return False

def disconnectcb(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)    
    success = mambo.disconnect()
    # rospy.loginfo(rospy.get_caller_id() + ' disconnected: %s', success)

def handle_takeoff(req):
    rospy.loginfo(rospy.get_caller_id() + ', I heard %s', req.connect)   
    if (req.connect):
        success_msg = mambo.takeoff()
        rospy.loginfo(rospy.get_caller_id() + " take off requested: %s", success_msg)    
        success = mambo.is_takeoff()
        rospy.loginfo(rospy.get_caller_id() + " has taken off: %s", success)    
        return success
    else:
        rospy.loginfo(rospy.get_caller_id() + ' take off not requested.')
        return False

def handle_land(req):
    rospy.loginfo(rospy.get_caller_id() + ', I heard %s', req.connect)   
    if (req.connect):
        success = mambo.safe_land(5)
        rospy.loginfo(rospy.get_caller_id() + " landing: %s", success)    
        return True
    else:
        rospy.loginfo(rospy.get_caller_id() + ' landing not requested.')
        return False

def flycb(data):
    rospy.loginfo(rospy.get_caller_id() + ' flight command: %f' % data.thrust + 'successful.')
    success = mambo.fly_direct(roll=data.orientation.x, pitch=data.orientation.y, yaw=0, vertical_movement=data.thrust, duration=0.2)

def mambo_host():
    # Subscribe to the mamboHost<MAC> message.
    name = rospy.get_param('~name')
    s1 = rospy.Service('/mambo_srv/' + name + '/connect', Connect, handle_connect)
    s2 = rospy.Subscriber('/mambo_srv/' + name + '/disconnect', String, disconnectcb)
    s3 = rospy.Service('/mambo_srv/' + name + '/takeoff', Connect, handle_takeoff)
    s4 = rospy.Service('/mambo_srv/' + name + '/land', Connect, handle_land)
    s5 = rospy.Subscriber('/mambo_srv/' + name + '/fly_command', AttitudeTarget, flycb)
    rospy.spin()

if __name__ == '__main__':
    # Initialise the node.
    rospy.init_node('mambo_host', anonymous=True)

    mamboAddr = rospy.get_param('~mac')
    mambo = Mambo(mamboAddr, use_wifi=False)
    mambo_host()
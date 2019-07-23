#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pyparrot.Minidrone import Mambo
import config

def connectcb(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)    
    if (data.data == 'Connected'):
        success = config.mambo.connect(num_retries=3)
        rospy.loginfo(rospy.get_caller_id() + 'connected: %s', success)
    else:
        rospy.loginfo(rospy.get_caller_id() + 'connection not requested.')        

def mamboHost():
    # Initialise the node.
    rospy.init_node('mamboHost', anonymous=True)
    # Subscribe to the mamboHost<MAC> message.
    rospy.Subscriber("mamboHostMAC", String, connectcb)
    rospy.spin()

if __name__ == '__main__':
    mamboHost()
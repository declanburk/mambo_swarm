#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def control():
    pub = rospy.Publisher("mamboHostMAC", String, queue_size=10)
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    for x in range(1):
        msg_str = 'Connected'
        rospy.loginfo(msg_str + ' at %s' % rospy.get_time())
        pub.publish(msg_str)
        # test
        rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
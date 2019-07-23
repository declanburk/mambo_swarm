#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

def test_publisher():
	pub1 = rospy.Publisher('/vrpn_client_node/Penne/twist', TwistStamped)
	pub2 = rospy.Publisher('/vrpn_client_node/Penne/pose', PoseStamped)
	rospy.init_node('test_pub', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		msg_str1 = TwistStamped()
		msg_str1.twist.linear.z = 3
		msg_str2 = PoseStamped()
		msg_str2.pose.position.x = 2
		rospy.loginfo(msg_str1.twist.linear)
		rospy.loginfo(msg_str2.pose.position)
		pub1.publish(msg_str1)
		pub2.publish(msg_str2)
		# test
		rate.sleep()

if __name__ == '__main__':
    try:
        test_publisher()
    except rospy.ROSInterruptException:
        pass
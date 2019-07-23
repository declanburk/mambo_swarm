#!/usr/bin/env python
## Modules
import rospy
import message_filters
from math import cos
from tf.transformations import euler_from_quaternion
## Messages
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import AccelStamped
from mavros_msgs.msg import AttitudeTarget

u = AttitudeTarget()

def mocapcb(pose, twist, accel):
	# ## Get orientation from mocap data and convert to Euler angles
	orientation_q = pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	now = rospy.get_rostime()
	f.write('%f ' % pose.pose.position.x + '%f ' % pose.pose.position.y + '%f ' % pose.pose.position.z + \
		'%f ' % twist.twist.linear.x + '%f ' % twist.twist.linear.y + '%f ' % twist.twist.linear.z + \
		'%f ' % accel.twist.linear.x + '%f ' % accel.twist.linear.y + '%f ' % accel.twist.linear.z + \
		'%f ' % u.body_rate.x + '%f ' % u.body_rate.y + \
		'%s' %  now.secs + '.%s' %  now.nsecs + '\n')

def call_back_fly_command(data):
	global u
	u = data 

if __name__ == '__main__':
	rospy.init_node('data_logger', anonymous=True)	
	name = rospy.get_param('~name')

	f = open('/home/declan/Documents/flight_data/' + name + '.txt','w')

	sub_pose = message_filters.Subscriber('/vrpn_client_node/' + name + '/pose', PoseStamped)
	sub_twist = message_filters.Subscriber('/vrpn_client_node/' + name + '/twist', TwistStamped)
	sub_accel = message_filters.Subscriber('/vrpn_client_node/' + name + '/accel', TwistStamped)

	sub_fly_commands = rospy.Subscriber('/mambo_srv/' + name + '/fly_command', AttitudeTarget, call_back_fly_command)

	ats = message_filters.ApproximateTimeSynchronizer([sub_pose, sub_twist, sub_accel], queue_size=10, slop=0.1)
	ats.registerCallback(mocapcb)

	while not rospy.is_shutdown():
		rospy.spin()

	f.close()
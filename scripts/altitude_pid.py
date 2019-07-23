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

class alt_gains():
	m = 0.063
	c12 = 1
	c13 = 1
	c14 = 1
	c15 = 1
	lam6 = 0
	lam7 = 0

chi6 = 0
chi7 = 0

xref = 0
yref = 0
zref = 1

def mocapcb(pose, twist):
	## Initialise Attitude Target message, the control input to be sent to host_service.py
	u = AttitudeTarget()
	# ## Get orientation from mocap data and convert to Euler angles
	# orientation_q = pose.pose.orientation
	# orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	# (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	# rospy.loginfo('%f' % roll + ' and %f' % pitch)

	## Calculate the velocity command from integral backstepping: https://doi.org/10.3929/ethz-a-010039365
	chi6 = 0	## IMPLEMENT INTEGRATION
	chi7 = 0	## IMPLEMENT INTEGRATION
	e10 = xref - twist.twist.linear.x
	e11 = yref - twist.twist.linear.y
	u.thrust = zref
	u.orientation.x = (alt_gains.m / u.thrust) * (((alt_gains.c12 + alt_gains.c13) * e10) + (alt_gains.lam6 * chi6))
	u.orientation.y = (alt_gains.m / u.thrust) * (((alt_gains.c14 + alt_gains.c15) * e11) + (alt_gains.lam7 * chi7))
	rospy.loginfo('Calculated z %f, phi %f and theta %f', u.thrust, u.orientation.x, u.orientation.y)    

if __name__ == '__main__':
	rospy.init_node('altitude_pid', anonymous=True)
	rate = rospy.Rate(20)
	
	ref = rospy.get_param('~alt_ref')
	name = rospy.get_param('~name')
	
	pub = rospy.Publisher('/mambo_con/' + name + '/zd_input', AttitudeTarget, queue_size=10)
	posesub = message_filters.Subscriber('/vrpn_client_node/' + name + '/pose', PoseStamped)
	twistsub = message_filters.Subscriber('/vrpn_client_node/' + name + '/twist', TwistStamped)
	# dsub = message_filters.Subscriber('/vrpn_client_node/' + name + '/accel', TwistStamped)

	ats = message_filters.ApproximateTimeSynchronizer([posesub, twistsub], queue_size=10, slop=0.1)
	ats.registerCallback(mocapcb)
	rospy.spin()

	rospy.loginfo('Tracking altitude: %f' % ref)
	# while not rospy.is_shutdown():
	# 	## Add the computation of u.
	# 	rospy.loginfo('Computation at %s' % rospy.get_time())
	# 	rospy.spin()
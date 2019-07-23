#!/usr/bin/env python
import sys
import rospy
import time
from multiprocessing import Pool
from mambo_fly.srv import *
from std_msgs.msg import String

import message_filters
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import AttitudeTarget

# Fix this import.
class alt_gains():
    m = 70.0
    # m = 0.063
    c12 = 1.0
    c13 = 1.0
    c14 = 1.0
    c15 = 1.0
    lam6 = 0.0
    lam7 = 0.0
    kp = 100.0

global chi6
global chi7
chi6 = 0.0
chi7 = 0.0

xdref = 0.0
ydref = 0.0
zdref = 0.0

ffactor = 0.9

def connect_client(name, req=True):
    # name = rospy.get_param('~name')
    rospy.wait_for_service('/mambo_srv/' + name + '/connect')
    try: 
        connect = rospy.ServiceProxy('/mambo_srv/' + name + '/connect', Connect)
        resp = connect(req)
        rospy.loginfo('Service call for connecting ' + name + ': %s' % resp.connected)
        return resp.connected
    except rospy.ServiceException:
        print('Service call for connection failed: ' + name)

def takeoff_client(name, req=True):
    # name = rospy.get_param('~name')
    rospy.wait_for_service('/mambo_srv/' + name + '/takeoff')
    try: 
        takeoff = rospy.ServiceProxy('/mambo_srv/' + name + '/takeoff', Connect)
        resp = takeoff(req)
        rospy.loginfo('Service call for take off for ' + name + ': %s' % resp.connected)
        return resp.connected
    except rospy.ServiceException:
        print('Service call for take off failed: ' + name)

def land_client(name, req=True):
    # name = rospy.get_param('~name')
    rospy.wait_for_service('/mambo_srv/' + name + '/land')
    try: 
        land = rospy.ServiceProxy('/mambo_srv/' + name + '/land', Connect)
        resp = land(req)
        rospy.loginfo('Service call for landing ' + name + ': %s' % resp.connected)
        return resp.connected
    except rospy.ServiceException:
        print('Service call for land failed: ' + name)

def collect_result(result):
    global results
    results.append(result)

def usage():
    return '%s' % sys.argv[0]

## Initialise Attitude Target message, the control input to be sent to host_service.py
u = AttitudeTarget()
init_control = 0

def mocapcb(pose, twist):
    # ## Get orientation from mocap data and convert to Euler angles
    # orientation_q = pose.pose.orientation
    # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # rospy.loginfo('%f' % roll + ' and %f' % pitch)
    global chi6
    global chi7
    ## Calculate the velocity command from integral backstepping: https://doi.org/10.3929/ethz-a-010039365
    if (init_control):
        e10 = xdref - twist.twist.linear.x
        e11 = ydref - twist.twist.linear.y
        chi6 = (ffactor * chi6) + e10  ## IMPLEMENT INTEGRATION
        chi7 = (ffactor * chi7) + e10    ## IMPLEMENT INTEGRATION
        u.thrust = alt_gains.kp * (zdref - twist.twist.linear.z)
        U1 = 1.0
        u.orientation.x = float((alt_gains.m / U1) * (((alt_gains.c12 + alt_gains.c13) * e10) + (alt_gains.lam6 * chi6)))
        u.orientation.y = float((alt_gains.m / U1) * (((alt_gains.c14 + alt_gains.c15) * e11) + (alt_gains.lam7 * chi7)))
        # rospy.loginfo('Calculated z %f, phi %f and theta %f', u.thrust, u.orientation.x, u.orientation.y)
        # rospy.loginfo('Velocity x error: %f' % chi6 + ' y error: %f.' % chi7)



results = [[False]]

if __name__ == '__main__':
    rospy.init_node('mambo_request', anonymous=True)
    rate = rospy.Rate(4)

    req = rospy.get_param('~request')
    names = [rospy.get_param(s) for s in rospy.get_param_names() if "mambo_request/name" in s]
    rospy.loginfo(names)

    for s in names:
        pub = rospy.Publisher('/mambo_srv/' + s + '/disconnect', String, queue_size=10)

        posesub = message_filters.Subscriber('/vrpn_client_node/' + s + '/pose', PoseStamped)
        twistsub = message_filters.Subscriber('/vrpn_client_node/' + s + '/twist', TwistStamped)
        # dsub = message_filters.Subscriber('/vrpn_client_node/' + name + '/accel', TwistStamped)
        ats = message_filters.ApproximateTimeSynchronizer([posesub, twistsub], queue_size=10, slop=0.1)
        ats.registerCallback(mocapcb)
        pub_fly_command = rospy.Publisher('/mambo_srv/' + s + '/fly_command', AttitudeTarget, queue_size=10)

    
    ## BLE Connection
    rospy.loginfo('Swarm connection request.')
    # result = False 
    while (not all(results[0])):
        time.sleep(4)
        for s in names:
            msg_str = 'Disconnect'
            pub.publish(msg_str)
            rospy.loginfo('Disconnection ' + s)

        results = []
        pool=Pool()
        r = pool.map_async(connect_client, names, callback=collect_result)
        pool.close()
        pool.join()
        # for s in names:
        #     rospy.loginfo(s)
        #     result = connect_client(req, s)
        #     rospy.loginfo('Connection ' + s + ': %s' % result)

    ## Take off
    results = [[False]]
    while (not all(results[0])):
        results = []
        pool=Pool()
        r = pool.map_async(takeoff_client, names, callback=collect_result)
        pool.close()
        pool.join()

    init_control = 1
    time.sleep(1)

    ## Control loop
    ## Fly left
    rospy.loginfo('Flying left.')
    for x in range(6):
        ydref = -1.0
        xdref = 0
        pub_fly_command.publish(u)
        rospy.loginfo('Would command thrust: %f' % u.thrust + 'and roll: %f' % u.orientation.x + 'and pitch: %f' % u.orientation.y)
        rospy.loginfo('Velocity x error: %f' % chi6 + ' y error: %f.' % chi7)
        rate.sleep()
    for x in range(6):
        ydref = 0.0
        xdref = 1.0
        pub_fly_command.publish(u)
        rospy.loginfo('Would command thrust: %f' % u.thrust + 'and roll: %f' % u.orientation.x + 'and pitch: %f' % u.orientation.y)
        rospy.loginfo('Velocity x error: %f' % chi6 + ' y error: %f.' % chi7)
        rate.sleep()
    for x in range(6):
        ydref = 1.0
        xdref = 0.0
        pub_fly_command.publish(u)
        rospy.loginfo('Would command thrust: %f' % u.thrust + 'and roll: %f' % u.orientation.x + 'and pitch: %f' % u.orientation.y)
        rospy.loginfo('Velocity x error: %f' % chi6 + ' y error: %f.' % chi7)
        rate.sleep()
    for x in range(6):
        ydref = 0.0
        xdref = -1.0
        pub_fly_command.publish(u)
        rospy.loginfo('Would command thrust: %f' % u.thrust + 'and roll: %f' % u.orientation.x + 'and pitch: %f' % u.orientation.y)
        rospy.loginfo('Velocity x error: %f' % chi6 + ' y error: %f.' % chi7)
        rate.sleep()
    ## Fly left
    rospy.loginfo('Stabilising.')
    # for x in range(3):
    #     ydref = 0.0
    #     pub_fly_command.publish(u)
    #     rospy.loginfo('Would command thrust: %f' % u.thrust + 'and roll: %f' % u.orientation.x + 'and pitch: %f' % u.orientation.y)
    #     rospy.loginfo('Velocity x error: %f' % chi6 + ' y error: %f.' % chi7)
    #     rate.sleep()
    # rospy.loginfo('Flying right.')
    ## Fly right
    # for x in range(15):
    #     ydref = 1.0
    #     pub_fly_command.publish(u)
    #     rospy.loginfo('Would command thrust: %f' % u.thrust + 'and roll: %f' % u.orientation.x + 'and pitch: %f' % u.orientation.y)
    #     rospy.loginfo('Velocity x error: %f' % chi6 + ' y error: %f.' % chi7)
    #     rate.sleep()


    ## Land
    results = [[False]]
    while (not all(results[0])):
        time.sleep(2)

        results = []
        pool=Pool()
        r = pool.map_async(land_client, names, callback=collect_result)
        pool.close()
        pool.join()

    ## Disconnect
    for s in names:
        msg_str = 'Disconnect'
        pub.publish(msg_str)
        rospy.loginfo('Disconnection ' + s)
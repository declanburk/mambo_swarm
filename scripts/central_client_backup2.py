#!/usr/bin/env python2
import sys
import rospy
import time
from pathos.pools import ProcessPool
import message_filters

from mambo_fly.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import AttitudeTarget

class GroundStation:

    def __init__(self):
        self.rate = rospy.Rate(4)
        self.connected = False
        self.took_off = False
        self.init_control = False

        self.m = 30
        self.c12 = 1
        self.c13 = 1
        self.c14 = 1
        self.c15 = 1
        self.c16 = 0.015
        self.c17 = 0.015
        self.lam6 = 0.015
        self.lam7 = 0.015
        self.chi6 = 0
        self.chi7 = 0
        self.ffactor = 1
        self.kp = 50

        self.pose = PoseStamped()
        self.twist = TwistStamped()
        self.accel = TwistStamped()
        self.u = AttitudeTarget()

        self.xdref = 0
        self.ydref = 0
        self.zdref = 0

        self.agent_names = [rospy.get_param(s) for s in rospy.get_param_names() if "mambo_request/name" in s]
        rospy.loginfo(self.agent_names)
        for s in self.agent_names:
            ## Publishers and subscribers for each agent.
            self.pub_disconnect = rospy.Publisher('/mambo_srv/' + s + '/disconnect', String, queue_size=10)
            self.pub_fly_command = rospy.Publisher('/mambo_srv/' + s + '/fly_command', AttitudeTarget, queue_size=10)
            self.sub_pose = rospy.Subscriber('/vrpn_client_node/' + s + '/pose', PoseStamped, self.callback_pose)
            self.sub_twist = rospy.Subscriber('/vrpn_client_node/' + s + '/twist', TwistStamped, self.callback_twist)
            self.sub_accel = rospy.Subscriber('/vrpn_client_node/' + s + '/accel', TwistStamped, self.callback_accel)

    ## A connection routine where the groundstation disconnects before attempting to connect a set number of times. 
        ## The disconnecting makes sure no hanging connections cause problems in the case of one quad in the swarm
        ## experiencing connection issues. Also, I have edited "btle.py" in the bluepy module to use much faster 
        ## timeouts. This approach of quickly retrying again helps to prevent timeouts accross the whole swarm in
        ## the case of one quad experiencing problems.
    def ble_routine(self, retries=5):
        count = 0
        while not self.connected:
            time.sleep(4)
            for s in self.agent_names:
                msg_str = 'Disconnect'
                self.pub_disconnect.publish(msg_str)
                rospy.loginfo('Disconnection ' + s)
            self.ble_connect()
            count = count + 1
            if count > retries:
                return False
        return True

    def ble_connect(self):    
        pool = ProcessPool()
        r = pool.map(self.client_connect, self.agent_names)
        self.connected = all(r)

    ## A take off routine that repeatedly sends the take off message and checks if successful or not. Again, this has
        ## proven more effective than waiting for a safe take off, to prevent swarm timeouts in the case one quad
        ## takes very long.
    def take_off_routine(self, retries=5):
        count = 0
        while not self.took_off:
            self.take_off()
            count = count + 1
            if count > retries:
                return False
        return True

    def take_off(self):
        pool = ProcessPool()
        r = pool.map(self.client_takeoff, self.agent_names)
        self.took_off = all(r)

    def land_routine(self):
        while self.took_off:
            pool = ProcessPool()
            r = pool.map(self.client_land, self.agent_names)
            rospy.loginfo('Landing responses:')
            rospy.loginfo(r)
            self.took_off = not all(r)
        return True

    def ble_disconnect(self):
        for s in self.agent_names:
            msg_str = 'Disconnect'
            self.pub_disconnect.publish(msg_str)
            rospy.loginfo('Disconnection ' + s)

    def off_board(self, num):
        self.init_control = True

        rospy.loginfo('Hover')
        self.xdref = 0.0
        self.ydref = 0.0
        self.u.body_rate.x = self.xdref
        self.u.body_rate.y = self.ydref
        for x in range(num):
            self.pub_fly_command.publish(self.u)
            self.rate.sleep()


        rospy.loginfo('Forward')
        self.xdref = 0.0
        self.ydref = 0.5
        self.u.body_rate.x = self.xdref
        self.u.body_rate.y = self.ydref
        for x in range(num):
            self.pub_fly_command.publish(self.u)
            self.rate.sleep()

        rospy.loginfo('Right')
        self.xdref = 0.5
        self.ydref = 0.0
        self.u.body_rate.x = self.xdref
        self.u.body_rate.y = self.ydref
        for x in range(num):
            self.pub_fly_command.publish(self.u)
            self.rate.sleep()
        
        rospy.loginfo('Back')
        self.xdref = 0.0
        self.ydref = -0.5
        self.u.body_rate.x = self.xdref
        self.u.body_rate.y = self.ydref
        for x in range(num):
            self.pub_fly_command.publish(self.u) 
            self.rate.sleep()

        rospy.loginfo('Left')
        self.xdref = -0.5
        self.ydref = 0.0
        self.u.body_rate.x = self.xdref
        self.u.body_rate.y = self.ydref
        for x in range(num):
            self.pub_fly_command.publish(self.u)
            self.rate.sleep()

        self.xdref = 0.0
        self.ydref = 0.0
        self.u.body_rate.x = self.xdref
        self.u.body_rate.y = self.ydref
        self.pub_fly_command.publish(self.u)
        self.rate.sleep()

        self.init_control = False
      

    ## The control input is calculated as part of the Vicon position callback.
    def callback_pose(self, data):
        self.pose = data
        ## Get orientation from mocap data and convert to Euler angles
        # orientation_q = pose.pose.orientation
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # rospy.loginfo('%f' % roll + ' and %f' % pitch)

        ## Calculate the velocity command from integral backstepping: https://doi.org/10.3929/ethz-a-010039365
        if (self.init_control):
            e10 = self.xdref - self.twist.twist.linear.x
            ed10 = -self.accel.twist.linear.x
            e11 = self.ydref - self.twist.twist.linear.y
            ed11 = -self.accel.twist.linear.y
            self.chi6 = (self.ffactor * self.chi6) + e10  ## IMPLEMENT INTEGRATION
            self.chi7 = (self.ffactor * self.chi7) + e11    ## IMPLEMENT INTEGRATION
            self.u.thrust = self.kp * (self.zdref - self.twist.twist.linear.z)
            U1 = 1.0
            self.u.orientation.x = float((self.m / U1) * (((self.c12 + self.c13) * e10) + (self.lam6 * self.chi6) + (self.c16 * ed10)))
            self.u.orientation.y = float((self.m / U1) * (((self.c14 + self.c15) * e11) + (self.lam7 * self.chi7) + (self.c17 * ed11)))
            # rospy.loginfo('Calculated z %f, phi %f and theta %f', u.thrust, u.orientation.x, u.orientation.y)
            # rospy.loginfo('Velocity x error: %f' % chi6 + ' y error: %f.' % chi7)

    ## Clients and callbacks required by ROS.
    def client_connect(self, name, req=True):
        rospy.wait_for_service('/mambo_srv/' + name + '/connect')
        try: 
            connect = rospy.ServiceProxy('/mambo_srv/' + name + '/connect', Connect)
            resp = connect(req)
            rospy.loginfo('Service call for connecting ' + name + ': %s' % resp.connected)
            return resp.connected
        except rospy.ServiceException:
            print('Service call for connection failed: ' + name)
            return False

    def client_takeoff(self, name, req=True):
        rospy.wait_for_service('/mambo_srv/' + name + '/takeoff')
        try: 
            takeoff = rospy.ServiceProxy('/mambo_srv/' + name + '/takeoff', Connect)
            resp = takeoff(req)
            rospy.loginfo('Service call for take off for ' + name + ': %s' % resp.connected)
            return resp.connected
        except rospy.ServiceException:
            print('Service call for take off failed: ' + name)
            return False

    def client_land(self, name, req=True):
        rospy.wait_for_service('/mambo_srv/' + name + '/land')
        try: 
            land = rospy.ServiceProxy('/mambo_srv/' + name + '/land', Connect)
            resp = land(req)
            rospy.loginfo('Service call for landing ' + name + ': %s' % resp.connected)
            return True
        except rospy.ServiceException:
            print('Service call for land failed: ' + name)
            return False

    def callback_twist(self, data):
        self.twist = data

    def callback_accel(self, data):
        self.accel = data

if __name__ == '__main__':
    rospy.init_node('mambo_groundstation', anonymous=True)
    print('Creating object')
    mambo = GroundStation()

    success = mambo.ble_routine()
    rospy.loginfo('Connection: %s.' % success)

    success = mambo.take_off_routine()
    rospy.loginfo('Take off: %s.' % success)

    time.sleep(2)

    mambo.off_board(12)

    time.sleep(1)

    success = mambo.land_routine()
    rospy.loginfo('Landing: %s.' % success)

    mambo.ble_disconnect
    rospy.loginfo('Disconnected.')
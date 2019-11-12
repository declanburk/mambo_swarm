#!/usr/bin/env python2
import sys
import rospy
import numpy as np
import time
from pathos.pools import ProcessPool
import message_filters

from structs import two_agents

from mambo_fly.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import AttitudeTarget

class GroundStation:

    def __init__(self):
        self.rate = rospy.Rate(4)
        self.connected = False
        self.took_off = False
        self.init_control = False

        self.m = 40
        self.c12 = 1
        self.c13 = 1
        self.c14 = 1
        self.c15 = 1
        self.c16 = 0.0
        self.c17 = 0.0
        self.lam6 = 0.015
        self.lam7 = 0.015
        self.ffactor = 1
        self.kp = 50

        self.agents = {}
        self.network = two_agents
        self.agent_names = [rospy.get_param(s) for s in rospy.get_param_names() if "mambo_request/name" in s]
        rospy.loginfo(self.agent_names)

        self.pub_disconnect = {}
        self.pub_fly_command = {}
        self.pub_init = {}

        for s in self.agent_names:
            neighbours = ()
            bearing = []
            for ix, x in enumerate(two_agents['edges']):
                if (s == x[0]):
                    y = np.divide(np.array(two_agents['bearing'][ix]), np.linalg.norm(np.array(two_agents['bearing'][ix])))
                    neighbours += (x[1],)
                    bearing.append((y[0], y[1], y[2]))
                elif (s == x[1]):
                    y = np.multiply(np.divide(np.array(two_agents['bearing'][ix]), np.linalg.norm(np.array(two_agents['bearing'][ix]))), -1)
                    neighbours += (x[0],)
                    bearing.append((y[0], y[1], y[2]))

            ## Dictionary of each agent's state information. The two zeros are the integration of velocity errors.
            self.agents[s] = {'pose': (0.0, 0.0, 0.0), 'twist': (0.0, 0.0, 0.0), 'accel': (0.0, 0.0, 0.0), 'u': AttitudeTarget(), 'ref': (0.0, 0.0, 0.0), 'chi6': 0.0, 'chi7': 0.0, 'neighbours': neighbours, 'bearing': bearing}
            # self.agents[s] = [PoseStamped(), TwistStamped(), TwistStamped(), AttitudeTarget(), (0.0, 0.0, 0.0), 0.0, neighbours]
            ## Publishers and subscribers for each agent.
            self.pub_disconnect[s] = rospy.Publisher('/mambo_srv/' + s + '/disconnect', String, queue_size=10)
            self.pub_fly_command[s] = rospy.Publisher('/mambo_srv/' + s + '/fly_command', AttitudeTarget, queue_size=10)
            self.pub_init[s] = rospy.Publisher('/mambo_srv/' + s + '/init_control', Bool, queue_size=10)
            self.sub_pose = rospy.Subscriber('/vrpn_client_node/' + s + '/pose', PoseStamped, self.callback_pose, s)
            self.sub_twist = rospy.Subscriber('/vrpn_client_node/' + s + '/twist', TwistStamped, self.callback_twist, s)
            self.sub_accel = rospy.Subscriber('/vrpn_client_node/' + s + '/accel', TwistStamped, self.callback_accel, s)

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
                self.pub_disconnect[s].publish(msg_str)
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
            self.pub_disconnect[s].publish(msg_str)
            rospy.loginfo('Disconnection ' + s)

    def off_board(self, num):
        self.init_control = True
        for s in self.agent_names:
            self.pub_init[s].publish(self.init_control)

        rospy.loginfo('Bearing controller')
        for x in range(num):
            for s in self.agent_names:
                self.pub_fly_command[s].publish(self.agents[s]['u'])    
            self.rate.sleep()

        self.init_control = False
        for s in self.agent_names:
            self.pub_init[s].publish(self.init_control)

    ## The control input is calculated as part of the Vicon position callback.
    def callback_pose(self, data, name):
        self.agents[name]['pose'] = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
        ## Calculate the reference velocity from bearing-only control: https://arxiv.org/abs/1408.6552
        v = np.array([0.0, 0.0, 0.0])
        for ix, x in enumerate(self.agents[name]['neighbours']):
            edge = np.array(np.subtract(self.agents[x]['pose'], self.agents[name]['pose']))
            bearing = np.divide(edge, np.linalg.norm(edge))
            bearing_d = np.array(self.agents[name]['bearing'][ix])
            ortho_op = np.subtract(np.identity(3), np.outer(bearing, bearing))
            v += np.multiply(np.matmul(ortho_op, bearing_d), -1) 
            ## Calculate the reference velocity from complementary bearing-only control: Declan Burke, 2019.
            bearing_cross = np.cross(bearing_d, bearing)
            scaled_bc = np.multiply(np.divide(bearing_cross, np.linalg.norm(bearing_cross)), np.sign(np.dot(bearing, bearing_d)))
            skew_sym = np.array([[0,np.multiply(bearing_d[2],-1), bearing_d[1]], \
                                [bearing_d[2],0,np.multiply(bearing_d[0],-1)], \
                                [np.multiply(bearing_d[1],-1), bearing_d[0], 0]])
            tanh_rate = 5
            sigma = np.square(np.tanh(np.multiply(np.linalg.norm(bearing_cross), tanh_rate)))
            v += np.multiply(np.multiply(np.matmul(np.matmul(ortho_op,skew_sym), scaled_bc), -1), sigma)
            v = np.multiply(v, 0.25*(1/np.sqrt(2)))
            # v = np.multiply(v, 0.25)
            # print(v)
        self.agents[name]['ref'] = (v[0], v[1], v[2])
        self.agents[name]['u'].body_rate.x = self.agents[name]['ref'][0]
        self.agents[name]['u'].body_rate.y = self.agents[name]['ref'][1]

        # print('Agent %s has reference %s' % (name, self.agents[name]['ref']))        
        ## Calculate the velocity command from integral backstepping: https://doi.org/10.3929/ethz-a-010039365
        agent = self.agents[name]
        if (self.init_control):
            e10 = agent['ref'][0] - agent['twist'][0]
            e11 = agent['ref'][1] - agent['twist'][1]
            ed10 = -agent['accel'][0]
            ed11 = -agent['accel'][1]
            agent['chi6'] = (self.ffactor * agent['chi6']) + e10    ## IMPLEMENT INTEGRATION
            agent['chi7'] = (self.ffactor * agent['chi7']) + e11    ## IMPLEMENT INTEGRATION
            self.agents[name]['u'].thrust = self.kp * (agent['ref'][2] - agent['twist'][2])

            U1 = 1.0
            self.agents[name]['u'].orientation.x = float((self.m / U1) * (((self.c12 + self.c13) * e10) + (self.lam6 * agent['chi6']) + (self.c16 * ed10)))
            self.agents[name]['u'].orientation.y = float((self.m / U1) * (((self.c14 + self.c15) * e11) + (self.lam7 * agent['chi7']) + (self.c17 * ed11)))
            # time_now = rospy.Time()
            # self.agents[name]['u'].header.stamp.secs = time_now.secs
            # self.agents[name]['u'].header.stamp.nsecs = time_now.nsecs
            # print(self.agents[name]['u'])

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

    def callback_twist(self, data, name):
        self.agents[name]['twist'] = (data.twist.linear.x, data.twist.linear.y, data.twist.linear.z)

    def callback_accel(self, data, name):
        self.agents[name]['accel'] = (data.twist.linear.x, data.twist.linear.y, data.twist.linear.z)

if __name__ == '__main__':
    rospy.init_node('mambo_groundstation', anonymous=True)
    print('Creating object')
    mambo = GroundStation()

    success = mambo.ble_routine()
    rospy.loginfo('Connection: %s.' % success)

    success = mambo.take_off_routine()
    rospy.loginfo('Take off: %s.' % success)

    time.sleep(2)

    mambo.off_board(120)

    time.sleep(1)

    success = mambo.land_routine()
    rospy.loginfo('Landing: %s.' % success)

    mambo.ble_disconnect
    rospy.loginfo('Disconnected.')
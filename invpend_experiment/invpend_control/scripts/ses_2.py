#!/usr/bin/env python

from __future__ import print_function

import math
import time
import numpy as np
import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
from controller import controller

class bcolors:
    """ For the purpose of print in terminal with colors """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class Cartpole(object):

    def __init__(self):
        rospy.init_node('Cartpole')
        self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.jstates_callback)
        self._pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=50)
        self._pub_pole_position_cmd = rospy.Publisher('/invpend/joint2_position_controller/command', Float64, queue_size=50)
        self._pub_set_pole = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=50)
        self._pub_set_cart = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=50)
        # init observation parameters
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0
        self.reward = 0
        self.out_range = False
        self.reset_stamp = time.time()
        self.time_elapse = 0.
        # init reset_env parameters
        self.reset_dur = .2  # reset duration, sec
        self.freq = 1000  # topics pub and sub frequency, Hz
        ## pole
        self.PoleState = LinkState()
        self.PoleState.link_name = 'pole'
        self.PoleState.pose.position = Point(0.0, -0.275, 0.0)  # pole's position w.r.t. world
        self.PoleState.reference_frame = 'cart'
        self.pole_length = 0.5
        ## cart
        self.CartState = LinkState()
        self.CartState.link_name = 'cart'
        self.CartState.pose.position = Point(0.0, 0.0, 0.0)  # pole's position w.r.t. world
        self.CartState.reference_frame = 'slidebar'
        # velocity control command
        self.cmd = 0
        self.positionpid = controller(10**-3, 5, -5, 1.0, 0.0, 0.1, 15)
        self.velocitypid = controller(10**-3, 5, -5, 100.0, 0.0, 5.0, 15)


    def jstates_callback(self, data):
        """ Callback function for subscribing /invpend/joint_states topic """
        self.pos_cart = data.position[1]
        self.vel_cart = data.velocity[1]
        self.pos_pole = data.position[0]
        self.vel_pole = data.velocity[0]

    def movePole(self, goal):
        self._pub_pole_position_cmd.publish(goal)

    def moveCart(self, goal):
        position, angle = goal
        rate = rospy.Rate(50)

        #self.movePole(angle)


        while not rospy.is_shutdown():
            #print("Current pose", self.pos_cart)
            #print("Current Velocity", self.vel_cart)
            error = position - self.pos_cart
            #print("error", error)
            if abs(error) < 0.1:
                break

            velocity = self.positionpid.step(error)
            vel_error = velocity - self.vel_cart
            self.cmd = self.velocitypid.step(vel_error)
            #print("vel", self.cmd)
            self._pub_vel_cmd.publish(self.cmd)
            self.movePole(angle)
            rate.sleep()

    def mission_planner(self, waypoints):
        rate = rospy.Rate(50)
        for location in waypoints:
            x, y = location

            theta = math.asin(abs(y) / self.pole_length)

            if y < 0:
                angle = math.pi / 2 + theta
            else:
                angle = math.pi /2 - theta

            if theta == 0:
                pos_cmd = x - self.pole_length
                angle = math.pi/2
            else:
                pos_cmd = x - (y/math.tan(theta))

            self.moveCart([pos_cmd, angle])
            # print("Cart Position : ",pos_cmd)
            #print("Angle : ",angle)
            # print("Actual Angle",self.pos_pole)
            print(bcolors.OKGREEN, "Reached Coordiantes : ", bcolors.ENDC,x,y)

            rate.sleep()

    def goToStart(self):
        self.moveCart([0.0, 0.0])
        rospy.spin()

def input_coordinates():
    coordinates = []
    n = int(input("Enter the number of waypoints :"))
    for i in range(0,n):
        print("Enter the waypoint",i+1)
        x = float(input("Enter the X-Coordinate :"))
        y = float(input("Enter the Y-Coordinate :"))
        if(abs(x)>2.5):
            x = float(np.clip(x,-2.5,2.5))
            print(bcolors.WARNING,"Entered X-Coordinate is too large limiting to :",x,bcolors.ENDC)
        if(abs(y)>0.5):
            y = float(np.clip(y,-0.5,0.5))
            print(bcolors.WARNING,"Entered Y-Coordinate is too large limiting to :", y,bcolors.ENDC)
        coordinates.append((x,y))
    coordinates.append((0.0,0.5))
    return coordinates

if __name__ == '__main__':
    cart = Cartpole()
     #cart.default_pos()
    # while not rospy.is_shutdown():
    #     cart._pub_pole_position_cmd.publish(0.0)
    #mission = [(0.0,0.0),(1.0,0),(1.5,0.25),(-1,0.5),(-2,0.25)]
    #mission = [(0.0,0.5)]
    print(bcolors.HEADER, "CART-POLE WAYPOINT SIMULATOR", bcolors.ENDC)
    mission = input_coordinates()
    cart.mission_planner(mission)
    print(bcolors.OKGREEN, "Returned to Original Position", bcolors.ENDC)


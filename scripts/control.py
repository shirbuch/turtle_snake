#!/usr/bin/env python

from __future__ import print_function

import sys
from time import sleep
import rospy
import random
from turtle_snake.srv import *
from std_srvs.srv import Empty

DEBUG = True

DISTANCES = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.10]
DISTANCES = [0.5 for i in range(10)]
DEGREES = [90 for i in range(10)]
DEGREES = [20, 50, 70, 90, 110, 130, 150, 180, 200, 250]

def reset_turtle():
    print("Resetting turtle location")
    clear_bg = rospy.ServiceProxy('reset', Empty)
    clear_bg()
    move_client(0)
    turn_client(0)
    sleep(1)

def move_client(distance):
    try:
        move = rospy.ServiceProxy('move', Move)
        resp = move(distance)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def turn_client(degrees):
    try:
        turn = rospy.ServiceProxy('turn', Turn)
        resp = turn(degrees)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.wait_for_service('move')
    rospy.wait_for_service('turn')

    reset_turtle()
    for i in range(10):        
        distance = DISTANCES[i]
        move_client(distance)
        sleep(2)

        degrees = DEGREES[i]
        turn_client(degrees)
        sleep(2)

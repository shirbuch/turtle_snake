#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from turtle_snake.srv import *
import random

MAP_SIDE_LEN = 10 # todo

DISTANCES = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
DEGREES = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

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

    for i in range(10):        
        distance = DISTANCES[i]
        move_client(distance)
        
        degrees = DEGREES[i]
        turn_client(degrees)

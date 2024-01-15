#!/usr/bin/env python

from __future__ import print_function

from time import sleep
import rospy
from turtle_snake.srv import *
from std_srvs.srv import Empty

DISTANCES = [10, 6, 9, 8.8, 8.7, 3, 8, 2, 2, 4]
DEGREES = [90, 10, 180, 270, 10, 93, 90, 90, 90, 90]

def reset_turtle():
    print("Resetting turtle location")
    clear_bg = rospy.ServiceProxy('reset', Empty)
    clear_bg()
    move_turtle(0)
    turn_turtle(0)
    sleep(1)

def move_turtle(distance):
    try:
        move = rospy.ServiceProxy('move', Move)
        resp = move(distance)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def turn_turtle(degrees):
    try:
        turn = rospy.ServiceProxy('turn', Turn)
        resp = turn(degrees)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Service logic
if __name__ == "__main__":
    rospy.wait_for_service('move')
    rospy.wait_for_service('turn')

    print("=====================================")
    print("=============  BREAK  ===============")
    print("=====================================")

    reset_turtle()
    for i in range(10):        
        distance = DISTANCES[i]
        move_turtle(distance)
        sleep(1)

        degrees = DEGREES[i]
        turn_turtle(degrees)
        sleep(1)
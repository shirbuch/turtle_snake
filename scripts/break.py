#!/usr/bin/env python

from __future__ import print_function

from time import sleep
import rospy
from turtle_snake.srv import *
from std_srvs.srv import Empty

DISTANCES = [3, 2, 2, 8.8, 8.7, 3, 4]
DEGREES = [90, 90, 90, 200, 270, 10, 10]

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

def send_angle(degrees):
    try:
        observer = rospy.ServiceProxy('observer', Observer)
        resp = observer(degrees)
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

    send_angle(0) # Signal that the turtle is reseted
    reset_turtle()
    for i in range(len(DISTANCES)):        
        distance = DISTANCES[i]
        degrees = DEGREES[i]

        move_turtle(distance)
        sleep(1)

        send_angle(degrees)

        turn_turtle(degrees)
        sleep(1)

    send_angle(-1) # Signal end

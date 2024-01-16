#!/usr/bin/env python

from __future__ import print_function

from time import sleep
import rospy
from turtle_snake.srv import *
from std_srvs.srv import Empty

DISTANCES = [4.5, 4.6, 9, 8.8, 8.7, 3, 8, 2.3, 6.5, 1.5]
DEGREES = [90, 91, 89, 92, 88, 93, 87, 94, 86, 95]

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
    print("============  CONTROL  ===============")
    print("=====================================")

    reset_turtle()
    send_angle(0) # Signal that the rturtle is reseted
    for i in range(10):        
        distance = DISTANCES[i]
        move_turtle(distance)
        sleep(1)

        degrees = DEGREES[i]
        turn_turtle(degrees)
        sleep(1)

        send_angle(degrees)

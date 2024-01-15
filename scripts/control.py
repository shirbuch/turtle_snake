#!/usr/bin/env python

from __future__ import print_function

from time import sleep
import rospy
from turtle_snake.srv import *
from std_srvs.srv import Empty
from turtlesim.msg import Pose

DISTANCES = [4, 3.9, 7.5, 7.2, 3, 5, 2, 2.5, 1, 1.2]
DEGREES = [90, 91, 92, 93, 94, 96, 97, 98, 99, 100]

current_pose = Pose()

def print_turtle_pose(Pose, message=None):
    print("----------------")
    if message:
        print(message)
    print("x:", Pose.x)
    print("y:", Pose.y)
    print("theta:", Pose.theta)
    print("----------------")

def pose_callback(msg):
    global current_pose
    current_pose = msg

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
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback) # subscribe to turtle position

    print("=====================================")
    print("============  CONTROL  ===============")
    print("=====================================")
    # todo deleteme and all current_pose related if needed
    # print_turtle_pose(current_pose, "Initial Turtle Pose")
    # print_turtle_pose(current_pose, "Final Turtle Pose")

    reset_turtle()
    for i in range(10):        
        distance = DISTANCES[i]
        move_client(distance)
        sleep(1)

        degrees = DEGREES[i]
        turn_client(degrees)
        sleep(1)

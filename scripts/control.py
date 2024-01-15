#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from turtle_snake.srv import *

def move_client(distance):
    rospy.wait_for_service('move')
    try:
        move = rospy.ServiceProxy('move', Move)
        resp = move(distance)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def turn_client(degrees):
    rospy.wait_for_service('turn')
    try:
        turn = rospy.ServiceProxy('turn', Turn)
        resp = turn(degrees)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "Usage: [move|turn] [distance||degrees]"

if __name__ == "__main__":
    while True:
        user_input = input("Enter your command: ")
        try:
            [command, amount] = user_input.split()
            amount = int(amount)
        except ValueError:
            print(usage())
            continue
        if command not in ["move", "turn"]:
            print(usage())
            continue

        print("Requesting %s of %s"%(command, amount))
        if command == "move":
            move_client(amount)
        elif command == "turn":
            turn_client(amount)

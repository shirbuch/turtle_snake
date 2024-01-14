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

def usage():
    return "%s [distance]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        distance = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s"%(distance))
    move_client(distance)
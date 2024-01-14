#!/usr/bin/env python

from __future__ import print_function

from turtle_snake.srv import Move,MoveResponse
import rospy

def handle_move(req):
    print("Moving length %s"%(req.distance))
    return MoveResponse()

def move_server():
    rospy.init_node('move_server')
    s = rospy.Service('move', Move, handle_move)
    print("Ready to move.")
    rospy.spin()

if __name__ == "__main__":
    move_server()
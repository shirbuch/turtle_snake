#!/usr/bin/env python

from __future__ import print_function

from turtle_snake.srv import Turn,TurnResponse
import rospy

def handle_turn(req):
    print("Turning %s degrees"%(req.degrees))
    return TurnResponse()

def turn_server():
    rospy.init_node('turn_server')
    s = rospy.Service('turn', Turn, handle_turn)
    print("Ready to turn.")
    rospy.spin()

if __name__ == "__main__":
    turn_server()
#!/usr/bin/env python

from __future__ import print_function

import rospy
from turtle_snake.srv import Observer,ObserverResponse

def handle_observer(req):
    try:
        print("Observer handling")
        return ObserverResponse()
    except rospy.ROSInterruptException as e:
        print(f"Error: {e}")

def observer_server():
    rospy.init_node('observer_server')
    s = rospy.Service('observer', Observer, handle_observer)
    print("Ready to observe.")
    rospy.spin()

if __name__ == "__main__":
    observer_server()
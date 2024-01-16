#!/usr/bin/env python

from __future__ import print_function
from math import sqrt

import rospy
from turtle_snake.srv import Observer,ObserverResponse
from turtlesim.msg import Pose

distances = []
angles = []

prev_pose = Pose()
current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def calculate_distance(first_pose, second_pose):
    return sqrt((second_pose.x - first_pose.x)**2 + (second_pose.y - first_pose.y)**2) # calculating distance traveled

def handle_observer(req):
    global angles
    global prev_pose

    try:
        if(req.angle == -1):
            if len(angles) != 10:
                print("Not enough segments!")
            return ObserverResponse()
        elif(req.angle != 0):
            # todo add wall check

            distance = round(calculate_distance(prev_pose, current_pose), 1)
            if distance in distances:
                print(f"Distance {distance} already used!")
            distances.append(distance)

            if req.angle in angles:
                print(f"Angle {req.angle} already used!")
            angles.append(req.angle)      
            
        prev_pose = current_pose
        return ObserverResponse()
    except rospy.ROSInterruptException as e:
        print(f"Error: {e}")

def observer_server():
    rospy.init_node('observer_server')
    s = rospy.Service('observer', Observer, handle_observer)
    print("Ready to observe.")
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback) # subscribe to turtle position
    rospy.spin()

if __name__ == "__main__":
    observer_server()
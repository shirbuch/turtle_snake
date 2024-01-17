#!/usr/bin/env python

from __future__ import print_function
from math import sqrt

import rospy
from turtle_snake.srv import Observer,ObserverResponse
from turtlesim.msg import Pose

PI = 3.1415926535897

MAX_X = 11.07
MIN_X = 0
MAX_Y = 11
MIN_Y = 0

distances = []
angles = []

prev_pose = Pose()
current_pose = Pose()

init = False

def pose_callback(msg):
    global prev_pose, current_pose, init
    current_pose = msg
    if not init:
        prev_pose = current_pose
        init = True

def calculate_distance(first_pose, second_pose):
    distance = sqrt((second_pose.x - first_pose.x)**2 + (second_pose.y - first_pose.y)**2) # calculating distance traveled
    return distance

# Wall facing checks
def facing_down(pose_degrees):
    return pose_degrees > 0 and pose_degrees < 180

def facing_up(pose_degrees):
    return pose_degrees > 180 and pose_degrees < 360

def facing_right(pose_degrees):
    return pose_degrees > 90 and pose_degrees < 270

def facing_left(pose_degrees):
    return (pose_degrees > 270 or pose_degrees < 90)

# Bumping wall checks
def bumping_lower_wall(pose_degrees, pose_y):
    return facing_down(pose_degrees) and pose_y <= MIN_Y

def bumping_upper_wall(pose_degrees, pose_y):
    return facing_up(pose_degrees) and pose_y > MAX_Y

def bumping_right_wall(pose_degrees, pose_x):
    return facing_right(pose_degrees) and pose_x >= MAX_X

def bumping_left_wall(pose_degrees, pose_x):
    return facing_left(pose_degrees) and pose_x <= MIN_X

def bumping_wall(pose):
    pose_degrees = 180 + pose.theta * 180/PI
    return bumping_right_wall(pose_degrees, pose.x) or bumping_left_wall(pose_degrees, pose.x) or bumping_upper_wall(pose_degrees, pose.y) or bumping_lower_wall(pose_degrees, pose.y)

# Service logic
def handle_observer(req):
    global angles
    global prev_pose

    try:
        if(req.angle == -1):
            if len(angles) != 10:
                print("Not enough segments!")
            return ObserverResponse()
        elif(req.angle != 0):
            if bumping_wall(current_pose):
                print("Bumped in wall!")

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
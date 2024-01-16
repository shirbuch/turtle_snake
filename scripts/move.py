#!/usr/bin/env python

from __future__ import print_function
from math import sqrt

import rospy
from turtle_snake.srv import Move,MoveResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

MAX_X = 11.07
MIN_X = 0
MAX_Y = 11
MIN_Y = 0

PI = 3.1415926535897

current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

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

def calculate_distance(first_pose, second_pose):
    return sqrt((second_pose.x - first_pose.x)**2 + (second_pose.y - first_pose.y)**2) # calculating distance traveled

# Service logic
def move_turtle(distance, speed=3):
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback) # subscribe to turtle position
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # publishing to the turtle velocity
    
    vel_msg = Twist()
    vel_msg.linear.x = speed # linear.x moves the turtle

    init_pose = current_pose # used for calculating distance traveled
    distance_traveled = 0
    
    while(distance_traveled < distance): # while loop, for our moving
        if bumping_wall(current_pose):
            break
        velocity_publisher.publish(vel_msg) # important, this is responsible for the moving itself
        distance_traveled = calculate_distance(init_pose, current_pose) # calculating distance traveled
    
    # stop the turtle
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    velocity_publisher.publish(vel_msg) # publishing to the turtle velocity node to stop

    return

def handle_move(req):
    try:
        distance = req.distance
        if distance < 0 or distance >= MAX_X - MIN_X:
            print("Unvalid distance %s"%(distance))
            return MoveResponse()
        elif distance != 0:
            print("Moving distance %s"%(distance))
        
        move_turtle(distance)
        return MoveResponse()
    except rospy.ROSInterruptException as e:
        print(f"Error: {e}")

def move_server():
    rospy.init_node('move_server')
    s = rospy.Service('move', Move, handle_move)
    print("Ready to move.")
    rospy.spin()

if __name__ == "__main__":
    move_server()
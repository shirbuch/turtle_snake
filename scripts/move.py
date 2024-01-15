#!/usr/bin/env python

from __future__ import print_function
from math import sqrt

import rospy
from turtle_snake.srv import Move,MoveResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

current_pose = Pose()
MAX_X = 11.096445
MIN_X = -0.023555
MAX_Y = 11.096445
MIN_Y = -0.023555

# todo: deleteme: distance <= 0 or (current_pose.x + distance >= MAX_X): # a check that we are good...
# todo: stop if reached wall

def pose_callback(msg):
    global current_pose
    current_pose = msg

def move_turtle(distance, speed=3):
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # publishing to the turtle velocity
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback) # subscribe to turtle position
    
    vel_msg = Twist()
    vel_msg.linear.x = speed # linear.x moves the turtle

    init_pos = current_pose # used for calculating distance traveled
    distance_traveled = 0
    
    while(distance_traveled < distance): # while loop, for our moving
        velocity_publisher.publish(vel_msg) # important, this is responsible for the moving itself
        distance_traveled = sqrt((current_pose.x - init_pos.x)**2 + (current_pose.y - init_pos.y)**2) # calculating distance traveled
    
    vel_msg.linear.x = 0 # stops the turtle
    vel_msg.linear.y = 0 # stops the turtle
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
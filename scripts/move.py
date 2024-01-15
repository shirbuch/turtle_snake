#!/usr/bin/env python

from __future__ import print_function

import rospy
from turtle_snake.srv import Move,MoveResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

current_pose = Pose()
MAX_X = 11.096445
MIN_X = -0.023555
MAX_Y = 11.096445
MIN_Y = -0.023555

# todo: deleteme: distance <= 0 or (current_pose.x + distance >= MAX_X): # a check that we are good...

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

def move_turtle(distance):
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # publishing to the turtle velocity
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback) # subscribe to turtle position
    
    vel_msg = Twist()
    speed = 1
    vel_msg.linear.x = abs(speed) # linear.x moves the turtle in the x axis (to the right)

    init_pos = current_pose.x # used for calculating distance traveled
    distance_traveled = 0
    print_turtle_pose(current_pose, "Initial Turtle Pose")
    
    while(distance_traveled < distance): # while loop, for our moving
        velocity_publisher.publish(vel_msg) # important, this is responsible for the moving itself
        distance_traveled = abs(current_pose.x - init_pos) # calculating distance traveled
    
    vel_msg.linear.x = 0 # stops the turtle
    velocity_publisher.publish(vel_msg) # publishing to the turtle velocity node to stop.

    print_turtle_pose(current_pose, "Final Turtle Pose")
    return

def handle_move(req):
    try:
        distance = req.distance
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
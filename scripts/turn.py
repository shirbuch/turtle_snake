#!/usr/bin/env python

from __future__ import print_function
from math import fabs

import rospy
from turtle_snake.srv import Turn,TurnResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

PI = 3.1415926535897
MIN_THETA = -PI
MAX_THETA = PI/6
angular_speed = 30*2*PI/360

current_pose = Pose()

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

def turn_turtle(degrees):
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    vel_msg = Twist()
    relative_angle = degrees*2*PI/360
    vel_msg.angular.z = fabs(angular_speed)
    
    init_angle = current_pose.theta
    angle_traveled = 0
    print_turtle_pose(current_pose, "Initial Turtle Pose")
    
    while(angle_traveled < init_angle + relative_angle):
        intermediate_init_angle = current_pose.theta
        
        velocity_publisher.publish(vel_msg)
        
        # todo: check if other side needed
        if current_pose.theta < 0 and intermediate_init_angle > 0:
            angle_traveled += current_pose.theta - MIN_THETA + MAX_THETA - intermediate_init_angle
        else:
            angle_traveled += fabs(current_pose.theta - intermediate_init_angle)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    print_turtle_pose(current_pose, "Final Turtle Pose")
    return

def handle_turn(req):
    try:
        degrees = req.degrees
        print("Turning %s degrees"%(degrees))
        turn_turtle(degrees)
        return TurnResponse()
    except rospy.ROSInterruptException as e:
        print(f"Error: {e}")

def turn_server():
    rospy.init_node('turn_server')
    s = rospy.Service('turn', Turn, handle_turn)
    print("Ready to turn.")
    rospy.spin()

if __name__ == "__main__":
    turn_server()
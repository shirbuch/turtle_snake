#!/usr/bin/env python

from __future__ import print_function
from math import fabs

import rospy
from turtle_snake.srv import Turn,TurnResponse
from geometry_msgs.msg import Twist

PI = 3.1415926535897

def turn_turtle(degrees, speed=45, clockwise=False): # speed is (degrees/sec)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Converting from degrees to radians
    angular_speed = speed*2*PI/360
    wanted_angle_change = degrees*2*PI/360

    # We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -fabs(angular_speed)
    else:
        vel_msg.angular.z = fabs(angular_speed)

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle_change = 0

    # Rotate the robot
    while(current_angle_change < wanted_angle_change):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle_change = angular_speed*(t1-t0)

    # Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    
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
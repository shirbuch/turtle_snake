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
segments = []

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

# Segment coliision checks, by geeksforgeeks
# A Python3 program to find if 2 given line segments intersect or not 
  
class Point: 
    def __init__(self, x, y): 
        self.x = x 
        self.y = y 
  
# Given three collinear points p, q, r, the function checks if  
# point q lies on line segment 'pr'  
def onSegment(p, q, r): 
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and 
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))): 
        return True
    return False
  
def orientation(p, q, r): 
    # to find the orientation of an ordered triplet (p,q,r) 
    # function returns the following values: 
    # 0 : Collinear points 
    # 1 : Clockwise points 
    # 2 : Counterclockwise 
      
    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/  
    # for details of below formula.  
      
    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)) 
    if (val > 0): 
          
        # Clockwise orientation 
        return 1
    elif (val < 0): 
          
        # Counterclockwise orientation 
        return 2
    else: 
          
        # Collinear orientation 
        return 0
  
# The main function that returns true if  
# the line segment 'p1q1' and 'p2q2' intersect. 
def doIntersect(p1,q1,p2,q2): 
      
    # Find the 4 orientations required for  
    # the general and special cases 
    o1 = orientation(p1, q1, p2) 
    o2 = orientation(p1, q1, q2) 
    o3 = orientation(p2, q2, p1) 
    o4 = orientation(p2, q2, q1) 
  
    # General case 
    if ((o1 != o2) and (o3 != o4)): 
        return True
  
    # Special Cases 
  
    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1 
    if ((o1 == 0) and onSegment(p1, p2, q1)): 
        return True
  
    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1 
    if ((o2 == 0) and onSegment(p1, q2, q1)): 
        return True
  
    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2 
    if ((o3 == 0) and onSegment(p2, p1, q2)): 
        return True
  
    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2 
    if ((o4 == 0) and onSegment(p2, q1, q2)): 
        return True
  
    # If none of the cases 
    return False
  
# Driver program to test above functions: 
p1 = Point(1, 1) 
q1 = Point(10, 1) 
p2 = Point(1, 2) 
q2 = Point(10, 2) 
  
if doIntersect(p1, q1, p2, q2): 
    print("Yes") 
else: 
    print("No") 
     
# This code is contributed by Ansh Riyal

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
    global segments

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

            current_segment = (Point(prev_pose.x, prev_pose.y), Point(current_pose.x, current_pose.y))
            for s in segments[:-1]:
                if doIntersect(s[0], s[1], current_segment[0], current_segment[1]):
                    print("Crossed line!")
            segments.append(current_segment)
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
Usage:
Whatever is with # may not be needed.

Build:
# cd ~/catkin_ws/
catkin_make
# source devel/setup.bash 

Run:
roscore
rosrun turtlesim turtlesim_node
roslaunch src/turtle_snake/launch/control.launch

#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 2
xterm  -e "roslaunch turtlebot3_gazebo turtlebot3_world.launch" &
sleep 5
xterm  -e "roslaunch service_bot amcl.launch" &
sleep 5
xterm  -e "roslaunch turtlebot3_gazebo view_navigation.launch" 

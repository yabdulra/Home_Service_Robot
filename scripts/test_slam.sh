#!/bin/sh
xterm  -e  " source /opt/ros/melodic/setup.bash; roscore" & 
sleep 2
xterm  -e "roslaunch turtlebot3_gazebo turtlebot3_world.launch" &
sleep 5
xterm  -e "roslaunch gmapping slam_gmapping.launch" &
sleep 5
xterm  -e "roslaunch turtlebot3_gazebo view_navigation.launch" &
sleep 5
xterm  -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py" 

#!/bin/sh
xterm  -e  " source /opt/ros/melodic/setup.bash; roscore" & 
sleep 2
xterm  -e "roslaunch turtlebot3_gazebo turtlebot3_world.launch" &
sleep 5
xterm  -e "roslaunch service_bot amcl.launch" &
sleep 5
xterm  -e "roslaunch turtlebot3_gazebo view_navigation.launch" &
sleep 5
xterm -e "roslaunch pick_objects pick_objects.launch" &
xterm -e "roslaunch add_markers add_markers.launch"

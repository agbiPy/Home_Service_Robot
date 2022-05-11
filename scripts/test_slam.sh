#!/bin/sh

# Define workspace variable
path_catkin_ws="/home/workspace/catkin_ws"

#Deploy turtlebot world enviroment
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/home_robot.world " &
sleep 5

#Perform slam_gmapping
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash;

roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 10

# Perform rviz
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

# control keyboard manually
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash;
roslaunch turtlebot_teleop keyboard_teleop.launch" 

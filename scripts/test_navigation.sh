#!/bin/sh

# Define workspace variable
path_catkin_ws="/home/workspace/catkin_ws"
#Deploy turtlebot enviroment
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash; 
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/home_service.world " &
sleep 5

#Perform localization
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch  map_file:=$(pwd)/../../src/map/home_robot_map.yaml" &
sleep 5


xterm -e "cd ${path_catkin_ws} && source devel/setup.bash;  roslaunch turtlebot_rviz_launchers view_navigation.launch"
 

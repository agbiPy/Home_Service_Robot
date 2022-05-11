#!/bin/sh

# Define workspace variable
path_catkin_ws="/home/workspace/catkin_ws"
#Deploy turtlebot environment
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/home_robot.world " & 
sleep 5

# Perform Localization
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../../src/map/home_robot_map.yaml " &
sleep 5


xterm -e "cd ${path_catkin_ws} && source devel/setup.bash; roslaunch add_markers home_robot_rviz.launch rviz_config_file:=$(pwd)/../../src/rvizConfig/home_robot.rviz" &
sleep 5

# Perform rviz for visualization
xterm -e "cd ${path_catkin_ws} && source devel/setup.bash; rosparam load $(pwd)/../Rob_rout/Pick_objects.yaml;
rosrun add_markers add_markers " &

xterm -e "cd ${path_catkin_ws} && source devel/setup.bash; rosparam load $(pwd)/../Rob_rout/Pick_objects.yaml;
rosrun pick_objects pick_objects_goals " &

#!/bin/sh


# launch my personal gazebo world with turtlebot  
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/my_robot/worlds/my_world.world " &
sleep 6 

# launch rviz with pre-defined setup
#xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
xterm -e " roslaunch my_robot view_navigation.launch " &
sleep 6

# launch amcl package with turtlebot configuration
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/map/gmapper-map.yaml initial_pose_x:=-0.8 initial_pose_y:=-1.2 initial_pose_a:=-0.0 " &


#!/bin/sh

# launch gazebo world
#xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
xterm -e " roslaunch my_robot world.launch spawn_my_robot:=true " &
sleep 5

# launch gmapping slam package
#xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
#xterm -e " rosrun gmapping slam_gmapping scan:=scan _base_frame:=robot_footprint " &
xterm -e " roslaunch gmapping slam_gmapping_my_robot.launch " &
sleep 5

# launch rviz with pre-defined setup
#xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
xterm -e " roslaunch my_robot view_navigation.launch " &
sleep 5

# launch teleop twist packate
#xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch " &
xterm -e " roslaunch my_robot telelop.launch " &



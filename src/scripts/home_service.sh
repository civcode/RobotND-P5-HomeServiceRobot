#!/bin/sh


# launch my personal gazebo world with turtlebot  
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/my_robot/worlds/my_world.world " &
sleep 6 

# launch rviz with pre-defined setup
xterm -e " roslaunch my_robot view_navigation.launch " &
sleep 6

# launch amcl package with turtlebot configuration
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/map/gmapper-map.yaml initial_pose_x:=-0.8 initial_pose_y:=-1.2 initial_pose_a:=-0.0 " &
sleep 6

# lauch add markers node
#xterm -e " rosrun add_markers add_markers_node --use_time_delay=false " &
xterm -e " roslaunch add_markers add_markers.launch " &
sleep 5

# launch robot position publisher
#xterm -e " rosrun robot_pose_publisher robot_pose_publisher " &
#sleep 5

# launch pick object node
xterm -e " rosrun pick_objects pick_objects_node " &




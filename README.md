# Robotic Software Engineer ND 
## Project 5: Home Service Robot

[image1]: ./img/RobotND-Project5-Gazebo-World.png  " "
[image2]: ./img/RobotND-Project5-RVIZ.gif " "

Simple ROS/Gazebo project with mapping and navigation to make a home service robot move objects.  

Gazebo world with mobile robot:
![][image1]

Rviz in an object pick up and drop off scenario:
![][image2]


## How To Use

### Clone repo as catkin_ws, download dependencies, initialize workspace and build
This project depends on the following ros packages which can be downloaded automatically with a shell script.

- slam_gmapping (https://github.com/ros-perception/slam_gmapping.git)

- turtlebot_teleop (https://github.com/turtlebot/turtlebot)

- turtelbot_rviz_launchers (https://github.com/turtlebot/turtlebot_interactions)

- turtlebot_gazebo (https://github.com/turtlebot/turtlebot_simulator)

- robot_pose_publisher (https://github.com/GT-RAIL/robot_pose_publisher.git)

- telelop_twist_keyboard (https://github.com/ros-teleop/teleop_twist_keyboard)

```
$ git clone https://github.com/civcode/RobotND-P5-HomeServiceRobot.git catkin_ws
$ cd catkin_ws/src 
$ sh download_dependencies.sh
$ catkin_init_workspace
$ cd .. && catkin_make
```

### Source the ROS Environment

```
$ source devel/setup.bash
```

## Launch Different Configurations

### SLAM with Gmapping 
Launches gazebo, robot model, rviz, gmapper and teleoperation package. Use the keyboard to navigate the robot and create a map.

```
$ ./src/scripts/test_slam.sh 
```
Save the map.
```
$ rosrun map_server map_saver -f <map-name>
```

### Localization and Navigation with AMCL (Adaptive Monte Carlo Localization)
Launches gazebo, robot model, map, rviz and amcl package. Use the "2D Nav Goal" button in the RViz toolbar to send a goal pose and start localizlation and navigation. 

```
$ ./src/scripts/test_navigation.sh
```

### Pick Objects 
Launches gazebo, robot model, map, rviz, amcl package and a pick objects node which sends different goal poses to the navigation package.

```
$ ./src/scripts/pick_objects.sh    
```

### Add Markers
Launches gazebo, robot model, map, rviz, amcl package and a add markers node which sends different marker messages to RViz. The blue cube represents an object, the green cylinder a target position.

```
$ ./src/scripts/add_markers.sh         
```

### Home Service Robot
Launches gazebo, robot model, map, rviz, amcl, robot pose publisher, pick objects node and add markers node. The idea is to simulate a scenario where the robot picks up an object, moves it and drops it off.   

The pick objects node sends the pick up position of the object to the navigation stack. The add markers node gets the object position from the parameter server and sends an object marker message to RViz. The add markers node uses the robot pose message to calculate the distance between the current position and the object. When the robot arrives at the object, the object marker is removed. Then, the pick objects node send the drop off position for the object, and the add markers node sends the target position marker message for the drop off position.  

```
$ ./src/scripts/home_service.sh            
```


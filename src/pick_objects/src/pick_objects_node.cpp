#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/program_options.hpp>
#include <math.h>

namespace po = boost::program_options;

using namespace std;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//const vector<double> goal_1 = {-3.0, 2.0, -M_PI/2};
//const vector<double> goal_2 = {2.0, -3.0, M_PI/2};

const vector<double> goal_1 = {-3.0, -3.3, 0.0};
const vector<double> goal_2 = {-2.5, 2.2, -M_PI/2};
const vector<double> goal_3 = {2.0, -3.0, M_PI/2};

const double delay_seconds = 3.0;

vector<double> yawAngle2Quaternion(double yaw) {
  vector<double> quaternion = {0.0, 0.0, 0.0, 0.0};
  
  double w = cos(yaw/2.0);
  double z = sin(yaw/2.0);
  
  quaternion[0] = w;
  quaternion[3] = z;

  return quaternion;
}
  
bool moveAndWait(MoveBaseClient* ac, string frame_id, vector<double> pose) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  //goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  //goal.target_pose.pose.position.x = vm["pose_x"].as<double>();
  //goal.target_pose.pose.position.y = vm["pose_y"].as<double>();
  //vector<double> quaternion = yawAngle2Quaternion(vm["yaw"].as<double>());
  //goal.target_pose.pose.orientation.w = quaternion[0];
  //goal.target_pose.pose.orientation.z = quaternion[3];
  
  goal.target_pose.pose.position.x = pose[0];
  goal.target_pose.pose.position.y = pose[1];
  vector<double> quaternion = yawAngle2Quaternion(pose[2]);
  goal.target_pose.pose.orientation.w = quaternion[0];
  goal.target_pose.pose.orientation.z = quaternion[3];
  
  //ROS_INFO("quaternion: [%.2f, %.2f, %.2f, %0.2f]", quaternion[0], quaternion[1], quaternion[2], quaternion[3]);


   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending position [%.2f, %.2f, %.2f]", pose[0], pose[1], pose[2]);
  ac->sendGoal(goal);

  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached its goal
  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Base moved as requested");
    return true;
  } else {
    ROS_INFO("Base failed to move for some reason");
    return false;
  }
  
  
}

int main(int argc, char** argv) {
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // parse command line options
  po::options_description desc("command options");
  po::variables_map vm;
  try {
    //po::options_description desc("command options");
    desc.add_options()
      ("help", "produce help message")
      ("pos_x", po::value<double>()->default_value(2.0), "x coordinate on map")
      ("pos_y", po::value<double>()->default_value(-3.0), "y coordinate on map")
      ("yaw", po::value<double>()->default_value(1.0), "yaw angle of the robot")
    ;

    //po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      cout << desc << "\n";
      return 0;
    }

    if (vm.count("pos_x")) {
      cout << "pos_x: " << vm["pos_x"].as<double>() << endl;
    } else {
      cout << "pos_x was not set" << endl;
    }

    if (vm.count("pos_y")) {
      cout << "pos_y: " << vm["pos_y"].as<double>() << endl;
    } else {
      cout << "pos_y was not set" << endl;
    }

    if (vm.count("yaw")) {
      cout << "yaw: " << vm["yaw"].as<double>() << endl;
    } else {
      cout << "yaw was not set" << endl;
    }
  } catch (exception& e) {
    cerr << "error: " << e.what() << endl;
    return 1;
  } catch (...) {
    cerr << "Exception of unknown Type.\n";
    return 1;
  }

  /*
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  //goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  //goal.target_pose.pose.position.x = vm["pose_x"].as<double>();
  //goal.target_pose.pose.position.y = vm["pose_y"].as<double>();
  //vector<double> quaternion = yawAngle2Quaternion(vm["yaw"].as<double>());
  //goal.target_pose.pose.orientation.w = quaternion[0];
  //goal.target_pose.pose.orientation.z = quaternion[3];
  
  goal.target_pose.pose.position.x = goal_1[0];
  goal.target_pose.pose.position.y = goal_1[1];
  vector<double> quaternion = yawAngle2Quaternion(goal_1[2]);
  goal.target_pose.pose.orientation.w =quaternion[0];
  goal.target_pose.pose.orientation.z = quaternion[3];
  

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  */
  while (ros::ok()) {
	  bool is_success;
	  is_success = moveAndWait(&ac, "map", goal_1);
	  if (is_success) {
	  	ROS_INFO("Reached pick up location successfully"); 
	  } else {
	    ROS_INFO("Could not reach pick up location"); 
	  }
	  ros::Duration(delay_seconds).sleep();
	  
	  is_success = moveAndWait(&ac, "map", goal_2);
	  if (is_success) {
	    ROS_INFO("Reached droop off location successfully"); 
	  } else {
	    ROS_INFO("Could not reach drop off location"); 
	  }
	  ros::Duration(delay_seconds).sleep();

	  is_success = moveAndWait(&ac, "map", goal_3);
	  if (is_success) {
	    ROS_INFO("Reached standby location successfully"); 
	  } else {
	    ROS_INFO("Could not reach drop off location"); 
	  }
	  ros::Duration(delay_seconds).sleep();

  }
  
  return 0;
}

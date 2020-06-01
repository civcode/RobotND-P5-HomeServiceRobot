#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/program_options.hpp>

using namespace std;

namespace po = boost::program_options;

//const vector<double> goal_1 = {-3.0, 2.0, -M_PI/2};
//const vector<double> goal_2 = {2.0, -3.0, M_PI/2};

//const vector<double> goal_1 = {-3.0, -3.2, 0.0};
//const vector<double> goal_2 = {-2.5, 2.2, -M_PI/2};
//const vector<double> goal_3 = {2.0, -3.0, M_PI/2};

const double distance_threshold = 0.2;

const double delay_seconds = 3.0;

vector<double> current_position = {1000.0, 1000.0};

/*
vector<double> yawAngle2Quaternion(double yaw) {
  vector<double> quaternion = {0.0, 0.0, 0.0, 0.0};
  
  double w = cos(yaw/2.0);
  double z = sin(yaw/2.0);
  
  quaternion[0] = w;
  quaternion[3] = z;

  return quaternion;
}
*/

double getDistance(vector<double> goal, vector<double> current_position) {
  double dx = goal[0] - current_position[0];
  double dy = goal[1] - current_position[1];
  //ROS_INFO("goal: [%0.2f, %0.2f]", goal[0], goal[1]);
  //ROS_INFO("current: [%0.2f, %0.2f]", current_position[0], current_position[1]);
  return sqrt(pow(dx,2)+pow(dy,2));
}

void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  //ROS_INFO("Received: [%s]")
  //ROS_INFO("Received: [%f]", msg->position.x);
  current_position[0] = msg->position.x;
  current_position[1] = msg->position.y;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(0.1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscribe to robot pose message
  ros::Subscriber sub = n.subscribe("/robot_pose", 10, robotPoseCallback);

  
  // Parse command line options
  po::options_description desc("command options");
  po::variables_map vm;
  try {
    desc.add_options()
      ("use_time_delay", po::value<bool>()->default_value(false), "uses constant time delay for markers")
    ;

    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("use_time_delay")) {
      cout << "use_time_delay: " << vm["use_time_delay"].as<bool>() << endl;
    } else {
      cout << "use_time_delay was not set" << endl;
    }
  } catch (exception& e) {
    cerr << "error: " << e.what() << endl;
    return 1;
  } catch (...) {
    cerr << "Exception of unknown Type.\n";
    return 1;
  }


  // get marker positions 
  vector<double> goal_1;
  vector<double> goal_2;
  vector<double> goal_3;

  if (vm["use_time_delay"].as<bool>()) {
    // use dummy positions
    double pos_1[] = {-3.0, -3.3, 0.0};
    double pos_2[] = {-2.5, 2.2, 0.0};
    double pos_3[] = {2.0, -3.0, 0.0};

    goal_1.assign(pos_1, pos_1+3);
    goal_2.assign(pos_2, pos_2+3);
    goal_3.assign(pos_3, pos_3+3);

  } else {
    // get marker positions from parameter server
    while(!n.getParam("/pick_objects/goal_1", goal_1) && ros::ok()) {ros::Duration(1.0).sleep();};
    while(!n.getParam("/pick_objects/goal_2", goal_2) && ros::ok()) {ros::Duration(1.0).sleep();};
    while(!n.getParam("/pick_objects/goal_3", goal_3) && ros::ok()) {ros::Duration(1.0).sleep();};
  }

  // Set our initial shape type to be a cube
  //uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set up marker
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map"; //"/my_frame";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
   // marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = goal_1[0];
    marker.pose.position.y = goal_1[1];
    marker.pose.position.z = 0;
    //vector<double> quaternion = yawAngle2Quaternion(goal_1[2]);
    //quaternion[0];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0/255.0f;
    marker.color.g = 153.0/255.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;

    ROS_INFO("Placing marker at pick up location");
    marker_pub.publish(marker);

    if (vm["use_time_delay"].as<bool>()) {
      ros::Duration(5.0).sleep();
    } else {
      while ((getDistance(goal_1, current_position) > distance_threshold) && ros::ok()) {
        ros::spinOnce();
        ROS_INFO("Robot position: [%0.2f, %0.2f]", current_position[0], current_position[1]);
        ROS_INFO("Distance to pickup: %0.2f", getDistance(goal_1, current_position));
        ros::Duration(1.0).sleep();
      }
      ros::Duration(delay_seconds).sleep();
    }
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    //ros::spinOnce();

    //ros::Duration(5.0).sleep();

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = goal_2[0];
    marker.pose.position.y = goal_2[1];
    marker.pose.position.z = 0;

    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 51.0/255.0f;
    marker.color.g = 204.0/255.0f;
    marker.color.b = 51.0/255.0f;
    marker.color.a = 1.0;

    ROS_INFO("Placing target marker at drop off location");
    marker_pub.publish(marker);

    if (vm["use_time_delay"].as<bool>()) {
      ros::Duration(5.0).sleep();
    } else {
      while ((getDistance(goal_2, current_position)) > distance_threshold && ros::ok()) {
        ros::spinOnce();
        ROS_INFO("Robot position: [%0.2f, %0.2f]", current_position[0], current_position[1]);
        ROS_INFO("Distance to dropdown: %0.2f", getDistance(goal_2, current_position));
        ros::Duration(1.0).sleep();
      }
      ros::Duration(delay_seconds).sleep();
    }
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    ROS_INFO("Placing marker at drop off location");
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.color.r = 0.0/255.0f;
    marker.color.g = 153.0/255.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker_pub.publish(marker);

    if (vm["use_time_delay"].as<bool>()) {
      ros::Duration(5.0).sleep();
    } else {
      while ((getDistance(goal_3, current_position)) > distance_threshold && ros::ok()) {
        ros::spinOnce();
        ROS_INFO("Robot position: [%0.2f, %0.2f]", current_position[0], current_position[1]);
        ROS_INFO("Distance to standby: %0.2f", getDistance(goal_3, current_position));
        ros::Duration(1.0).sleep();
      }
      ros::Duration(delay_seconds).sleep();
    }
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);


    // Cycle between different shapes
    /*
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
    */

    r.sleep();
  }
}

/*
 * Project: PID Tracking Controller for Turtlebot3
 * Author: Zheng Chen
 * Email: chenarlen2018@gmail.com
 * Date: 02.23.2020
 */

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include <boost/format.hpp>

#include "pid.h"
#include "pid_param.h"

using namespace std;

// cmd publisher
// ros::Publisher cmd_vel_pub;

// odometry data subscriber
ros::Subscriber odom_sub;
// cmd subscriber
ros::Subscriber cmd_sub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle n("~");

  // Create a Parameter object to read parameters from launch file
  Parameter param;

  // Read parameters with ros::NodeHandle
  param.config_from_ros_handle(n);

  // Create a PID object
  PID pid(param);

  // Define a cmd publisher
  pid.ctrl_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Config the pid parameters
  pid.config();

  // Receive the odometry data 
  odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 
                                              1000,
                                              boost::bind(&Odometry_Data::feed, &pid.odom_data, _1),
                                              ros::VoidConstPtr(),
                                              ros::TransportHints().tcpNoDelay());
  // Receive the cmd data from planner
  cmd_sub = n.subscribe<turtlebot3_msgs::Command>("/tb3_cmd", 
                                              1000,
                                              boost::bind(&Desired_State::feed, &pid.desired_data, _1),
                                              ros::VoidConstPtr(),
                                              ros::TransportHints().tcpNoDelay());
  ros::Rate r(2000.0); // ROS spins 2000 frames per second
  pid.last_ctrl_time = ros::Time::now();

  while (ros::ok())
  {
    // Process the control signal
    pid.process();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}



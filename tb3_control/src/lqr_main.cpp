/*
 * Project: LQR controller for Turtlebot3
 * Author: Zheng Chen
 * Email: chenarlen2018@gmail.com
 * Date: 03.13.2020
 * 
 */

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include <boost/format.hpp>
#include "lqr_control.h"
#include "lqr_param.h"

using namespace std;

// odometry data subscriber
ros::Subscriber odom_sub;
// cmd subscriber
ros::Subscriber cmd_sub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lqr_control_node");
    ros::NodeHandle nh("~");
    
    Parameter param;

    param.config_from_ros_handle(nh);

    LQR lqr(param);
    lqr.config();

    lqr.ctrl_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    lqr.goal_pub = nh.advertise<visualization_msgs::Marker>("/tb3_goal", 1);

    // Receive the odometry data 
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 
                                                1000,
                                                boost::bind(&Odometry_Data::feed, &lqr.odom_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    // Receive the cmd data from planner
    cmd_sub = nh.subscribe<turtlebot3_msgs::Command>("/tb3_cmd", 
                                                1000,
                                                boost::bind(&Desired_State::feed, &lqr.desired_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
    
    ros::Rate r(1/lqr.param.lqr_time_step); // ROS spins 2000 frames per second
    lqr.last_ctrl_time = ros::Time::now();
    int cmd_id = 0;

    while(ros::ok())
    {
        lqr.process(cmd_id);
        cmd_id++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
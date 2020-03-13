#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include "turtlebot3_msgs/Command.h"
#include "visualization_msgs/Marker.h"

using namespace std;

ros::Publisher cmd_pub;
ros::Publisher traj_pub;
ros::Time time_traj_start;

turtlebot3_msgs::Command cmd;
vector<Eigen::Vector3d> traj_cmd;

void cmdCallback(const ros::TimerEvent& e)
{
    static int count = 0;
    ros::Time time_now = ros::Time::now();
    if(count == 0)
    {
        time_traj_start = time_now;
    }
    double t_cur = (time_now - time_traj_start).toSec();

    cmd.header.stamp = time_now;
    cmd.header.frame_id = "odom";

    double n = 6;
    double radius = 3;
  
#if 0
    // Circle traj
    cmd.position.x = radius * cos(M_PI * t_cur / n + 3 * M_PI / 2);
    cmd.position.y = radius * sin(M_PI * t_cur / n + 3 * M_PI / 2) + radius;
    cmd.position.z = 0;
    double x_dot = -radius * M_PI * sin(M_PI * t_cur / n + 3 * M_PI / 2) / n;
    double y_dot =  radius * M_PI * cos(M_PI * t_cur / n + 3 * M_PI / 2) / n;
    cmd.velocity.x = x_dot;
    cmd.velocity.y = y_dot;
    cmd.velocity.z = 0;
#else
    #if 0
    // Eight shape traj
    cmd.position.x = radius * sin(M_PI * t_cur / n);
    cmd.position.y = radius * sin(M_PI * t_cur / n) * cos(M_PI * t_cur / n);
    cmd.position.z = 0;
    double x_dot = radius * M_PI * cos(M_PI * t_cur / n) / n;
    double y_dot = radius * M_PI * (pow(cos(M_PI * t_cur / n), 2) - pow(sin(M_PI * t_cur / n), 2)) / n;
    cmd.velocity.x = x_dot;
    cmd.velocity.y = y_dot;
    cmd.velocity.z = 0;
    #else
    // Heart shape traj

    // Flat-bottom heart
    cmd.position.x = 2 * radius * (1 - cos(M_PI * t_cur / n)) * cos(M_PI * t_cur / n);
    cmd.position.y = 2 * radius * (1 - cos(M_PI * t_cur / n)) * sin(M_PI * t_cur / n);
    cmd.position.z = 0;
    double x_dot = 2 * radius * sin(M_PI * t_cur / n) * cos(M_PI * t_cur / n) - 2 * radius * sin(M_PI * t_cur / n) * (1 - cos(M_PI * t_cur / n));
    double y_dot = 2 * radius * (cos(M_PI * t_cur / n) * (1 - cos(M_PI * t_cur / n)) + pow(sin(M_PI * t_cur / n), 2));

    cmd.velocity.x = x_dot;
    cmd.velocity.y = y_dot;
    cmd.velocity.z = 0;

    // True heart
    // cmd.position.y = 16 * pow(sin(M_PI * t_cur / n), 3);
    // cmd.position.x = 13 * cos(M_PI * t_cur / n) - 5 * cos(2 * M_PI * t_cur / n) - 2 * cos(3 * M_PI * t_cur / n) - cos(4 * M_PI * t_cur / n);
    // cmd.position.z = 0;
    // double y_dot = 48 * pow(sin(M_PI * t_cur / n), 2) * cos(M_PI * t_cur / n);
    // double x_dot = -13 * sin(M_PI * t_cur / n) + 10 * sin(2 * M_PI * t_cur / n) + 6 * sin(3 * M_PI * t_cur / n) + 4 * sin(4 * M_PI * t_cur / n);
    #endif
#endif

    if(x_dot != 0 )
    {
        cmd.yaw = atan2(y_dot, x_dot);
    }
    else
    {
        cmd.yaw = (y_dot > 0) ? M_PI / 2 : -M_PI / 2;
    }

    cmd_pub.publish(cmd);

    traj_cmd.push_back(Eigen::Vector3d(cmd.position.x,
                                       cmd.position.y,
                                       0));
    if(traj_cmd.size()>10000) traj_cmd.erase(traj_cmd.begin(), traj_cmd.begin()+1000);
    count++;
}

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
    visualization_msgs::Marker mk;
    mk.header.frame_id = "odom";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.id = id;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(path.size()); i++) {
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        mk.points.push_back(pt);
    }
    traj_pub.publish(mk);
}

void visCallback(const ros::TimerEvent& e)
{
    displayTrajWithColor(traj_cmd, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tb3_planner_node");
    ros::NodeHandle nh;

    // cmd hz: 100
    cmd_pub = nh.advertise<turtlebot3_msgs::Command>("tb3_cmd", 1);
    traj_pub = nh.advertise<visualization_msgs::Marker>("tb3_traj", 1);

    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
    ros::Timer vis_timer = nh.createTimer(ros::Duration(0.25), visCallback);
    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}
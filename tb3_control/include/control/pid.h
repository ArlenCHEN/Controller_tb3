#ifndef PID_H_
#define PID_H_

#include <tf/transform_listener.h>
#include <turtlebot3_msgs/Command.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>

#include "pid_param.h"
class Desired_State
{
    public:
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        double yaw;
        Eigen::Quaterniond q;
        Eigen::Vector3d a;

        turtlebot3_msgs::Command msg;

        Desired_State(){}
        void feed(turtlebot3_msgs::CommandConstPtr pMsg);
};

class Odometry_Data
{
    public:
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Quaterniond q;

        double yaw;
        // Angular velocity along up z axis
        double w;

        nav_msgs::Odometry msg;

        Odometry_Data(){}
        void feed(nav_msgs::OdometryConstPtr pMsg);
};

struct Controller_Output
{
    // Along body heading direction
    double linear_vel;
    // Along body up z axis
    double angular_vel;
};

class PID
{
public:
    Parameter& param;
    Desired_State desired_data;
    Odometry_Data odom_data;
    
    ros::Publisher ctrl_pub;
    ros::Time last_ctrl_time;

    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kv;
    double Kyaw;

    PID(Parameter& param_):param(param_)
    {
        is_configured = false;
    }
    void config_gain(const Parameter::Gain& gain);
    void config();
    void update(const Desired_State& des, const Odometry_Data& odom,
    Controller_Output& u);
    void publish_ctrl(const Controller_Output& u);
    void process();
    void process_control(const ros::Time& now_time);
    void process_cmd_control(Controller_Output& u);

private:
    bool is_configured;
};

#endif
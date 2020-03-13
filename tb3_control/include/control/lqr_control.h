#ifndef LQR_CONTROL_H_
#define LQR_CONTROL_H_

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <tf/transform_listener.h>
#include <turtlebot3_msgs/Command.h>
#include <nav_msgs/Odometry.h>
#include <uav_utils/geometry_utils.h>
#include <visualization_msgs/Marker.h>
#include "lqr_param.h"

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

class LQR
{
    public:
        Parameter& param;
        Desired_State desired_data;
        Odometry_Data odom_data;

        ros::Publisher ctrl_pub;
        ros::Publisher goal_pub;
        ros::Time last_ctrl_time;

        LQR(Parameter& param_):param(param_)
        {
            is_configured = false;

            lqr_Q = Eigen::MatrixXd::Identity(3, 3);
            lqr_R = Eigen::MatrixXd::Identity(2, 2);
            lqr_start = Eigen::MatrixXd::Zero(4, 1);
            lqr_end = Eigen::MatrixXd::Zero(4, 1);
        }

        void config();
        void config_params(const Parameter& params);
        void process(int cmd_id);
        void process_control(int cmd_id);
        void process_cmd_control(Controller_Output& u, int cmd_id);
        void update(int cmd_id, 
                    const Eigen::MatrixXd& lqr_end, 
                    const Odometry_Data& odom, 
                    Controller_Output& u);
        void update(int cmd_id,
                  const Desired_State& des,
                  const Odometry_Data& odom,
                  Controller_Output& u);
        void publish_ctrl(const Controller_Output& u);
        void publish_goal();
        
    private:
        bool is_configured;
        Eigen::MatrixXd lqr_Q;
        Eigen::MatrixXd lqr_R;
        Eigen::MatrixXd lqr_start;
        Eigen::MatrixXd lqr_end;
        Eigen::MatrixXd lqr_A;
        Eigen::MatrixXd lqr_B;
        std::vector<Eigen::MatrixXd> lqr_P;

        double lqr_time_step;
        double lqr_roh;
        double linear_speed_thresh;
        double angular_speed_thresh;
        int lqr_time_horizon;
        bool is_limit_speed;
};

#endif
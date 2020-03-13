#include "lqr_control.h"

using namespace Eigen;
using namespace std;

void Desired_State::feed(turtlebot3_msgs::CommandConstPtr pMsg)
{
    msg = *pMsg;
    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    yaw = msg.yaw;
}

void Odometry_Data::feed(nav_msgs::OdometryConstPtr pMsg)
{
    msg = *pMsg;
    tf::Point odom_pos;
    double odom_yaw;

    tf::pointMsgToTF(msg.pose.pose.position, odom_pos);
    odom_yaw = tf::getYaw(msg.pose.pose.orientation);

    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;
    q.w() = msg.pose.pose.orientation.w;

    // Updata odometry position
    p(0) = odom_pos.x();
    p(1) = odom_pos.y();
    p(2) = odom_pos.z();

    //update observed linear and angular speeds (real speeds published from simulation)
    v(0) = msg.twist.twist.linear.x;
    v(1) = msg.twist.twist.linear.y;
    v(2) = msg.twist.twist.linear.z;
    
    w = msg.twist.twist.angular.z;
    yaw = odom_yaw;
}

void LQR::config()
{
    config_params(param);
    is_configured = true;
}

void LQR::config_params(const Parameter& params)
{
    lqr_Q(0, 0) = params.lqr_matrix_params.q_0;
    lqr_Q(1, 1) = params.lqr_matrix_params.q_1;
    lqr_Q(2, 2) = params.lqr_matrix_params.q_2;

    lqr_R = params.lqr_roh * lqr_R;

    lqr_start(0, 0) = params.lqr_matrix_params.s_0;
    lqr_start(1, 0) = params.lqr_matrix_params.s_1;
    lqr_start(2, 0) = params.lqr_matrix_params.s_2;
    lqr_start(3, 0) = params.lqr_matrix_params.s_3;

    lqr_end(0, 0) = params.lqr_matrix_params.e_0;
    lqr_end(1, 0) = params.lqr_matrix_params.e_1;
    lqr_end(2, 0) = params.lqr_matrix_params.e_2;
    lqr_end(3, 0) = params.lqr_matrix_params.e_3;

    lqr_roh = params.lqr_roh;
    lqr_time_step = params.lqr_time_step;
    lqr_time_horizon = params.lqr_time_horizon;
    linear_speed_thresh = params.linear_speed_thresh;
    angular_speed_thresh = params.angular_speed_thresh;
    is_limit_speed = params.is_limit_speed;

    lqr_A = MatrixXd(3, 3);
    lqr_A << 1, 0, -lqr_end(3, 0) * sin(lqr_end(2, 0)) * lqr_time_step,
             0, 1,  lqr_end(3, 0) * cos(lqr_end(2, 0)) * lqr_time_step,
             0, 0,  1;
    
    lqr_B = MatrixXd(3, 2);
    lqr_B << cos(lqr_end(2, 0)) * lqr_time_step, 0,
            sin(lqr_end(2, 0)) * lqr_time_step, 0,
            0                                 , lqr_time_step;

    for(int i = 0; i <= lqr_time_horizon; i++)
    {
        lqr_P.push_back(MatrixXd::Zero(3, 3));
    }

    lqr_P[lqr_time_horizon] = lqr_Q;

    for(int i = lqr_time_horizon; i > 0; i--)
    {
        lqr_P[i-1] = lqr_Q + lqr_A.transpose() * lqr_P[i] * lqr_A - 
                     lqr_A.transpose() * lqr_P[i] * lqr_B * ( lqr_R + lqr_B.transpose() * lqr_P[i] * lqr_B ).inverse() * lqr_B.transpose() * lqr_P[i] * lqr_A;
    }
}

void  LQR::update(int cmd_id,
                  const Eigen::MatrixXd& lqr_end,
                  const Odometry_Data& odom,
                  Controller_Output& u)
{
    ROS_ASSERT_MSG(is_configured, "Gains for controller might not be initialized.");
    double yaw_curr = uav_utils::get_yaw_from_quaternion(odom.q);

    cout<<"cmd_id: "<<cmd_id<<endl;
    MatrixXd K_t = MatrixXd::Zero(2, 3);
    K_t = -(lqr_R + lqr_B.transpose() * lqr_P[cmd_id+1] * lqr_B).inverse() * lqr_B.transpose() * lqr_P[cmd_id + 1] * lqr_A;

    MatrixXd X_t = MatrixXd::Zero(3, 1);
    X_t << odom.p(0) - lqr_end(0, 0),
           odom.p(1) - lqr_end(1, 0),
           yaw_curr  - lqr_end(2, 0);
    
    ROS_INFO("X Error: %f; Y Error: %f; Yaw Error: %f", X_t(0, 0), X_t(1, 0), X_t(2, 0));
    MatrixXd U_t;
    U_t = K_t * X_t;

    ROS_INFO("Linear vel: %f; Angular vel: %f.", U_t(0, 0), U_t(1, 0));
    
    double limited_linear_speed;
    double limited_angular_speed;
    if(is_limit_speed)
    {
        if(U_t(0, 0) > linear_speed_thresh)
        {
            limited_linear_speed = linear_speed_thresh;
        }
        else if(U_t(0, 0) < -linear_speed_thresh)
        {
            limited_linear_speed = -linear_speed_thresh;
        }

        if(U_t(1, 0) > angular_speed_thresh)
        {
            limited_angular_speed = angular_speed_thresh;
        }
        else if(U_t(1, 0) < -angular_speed_thresh)
        {
            limited_angular_speed = -angular_speed_thresh;
        }
    }
    else
    {
        limited_linear_speed = U_t(0, 0);
        limited_angular_speed = U_t(1, 0);
    }
    
    u.linear_vel = limited_linear_speed;
    u.angular_vel = limited_angular_speed;
}

void  LQR::update(int cmd_id,
                  const Desired_State& des,
                  const Odometry_Data& odom,
                  Controller_Output& u)
{
    ROS_ASSERT_MSG(is_configured, "Gains for controller might not be initialized.");
    double yaw_curr = uav_utils::get_yaw_from_quaternion(odom.q);
    double yaw_des = des.yaw;

    lqr_A = MatrixXd(3, 3);
    lqr_A << 1, 0, -des.v(0) * sin(yaw_des) * lqr_time_step,
             0, 1,  des.v(0) * cos(yaw_des) * lqr_time_step,
             0, 0,  1;
     
    lqr_B = MatrixXd(3, 2);
    lqr_B << cos(yaw_des) * lqr_time_step, 0,
             sin(yaw_des) * lqr_time_step, 0,
             0                , lqr_time_step;

    for(int i = 0; i <= lqr_time_horizon; i++)
    {
        lqr_P.push_back(MatrixXd::Zero(3, 3));
    }

    lqr_P[lqr_time_horizon] = lqr_Q;

    for(int i = lqr_time_horizon; i > 0; i--)
    {
        lqr_P[i-1] = lqr_Q + lqr_A.transpose() * lqr_P[i] * lqr_A - 
                     lqr_A.transpose() * lqr_P[i] * lqr_B * ( lqr_R + lqr_B.transpose() * lqr_P[i] * lqr_B ).inverse() * lqr_B.transpose() * lqr_P[i] * lqr_A;
    }

    cout<<"cmd_id: "<<cmd_id<<endl;
    MatrixXd K_t = MatrixXd::Zero(2, 3);
    K_t = -(lqr_R + lqr_B.transpose() * lqr_P[cmd_id+1] * lqr_B).inverse() * lqr_B.transpose() * lqr_P[cmd_id + 1] * lqr_A;

    MatrixXd X_t = MatrixXd::Zero(3, 1);
    X_t << odom.p(0) - des.p(0),
           odom.p(1) - des.p(1),
           yaw_curr  - yaw_des;
    
    ROS_INFO("X Error: %f; Y Error: %f; Yaw Error: %f", X_t(0, 0), X_t(1, 0), X_t(2, 0));
    MatrixXd U_t;
    U_t = K_t * X_t;

    ROS_INFO("Linear vel: %f; Angular vel: %f.", U_t(0, 0), U_t(1, 0));
    
    double limited_linear_speed;
    double limited_angular_speed;
    if(is_limit_speed)
    {
        if(U_t(0, 0) > linear_speed_thresh)
        {
            limited_linear_speed = linear_speed_thresh;
        }
        else if(U_t(0, 0) < -linear_speed_thresh)
        {
            limited_linear_speed = -linear_speed_thresh;
        }

        if(U_t(1, 0) > angular_speed_thresh)
        {
            limited_angular_speed = angular_speed_thresh;
        }
        else if(U_t(1, 0) < -angular_speed_thresh)
        {
            limited_angular_speed = -angular_speed_thresh;
        }
    }
    else
    {
        limited_linear_speed = U_t(0, 0);
        limited_angular_speed = U_t(1, 0);
    }
    
    u.linear_vel = limited_linear_speed;
    u.angular_vel = limited_angular_speed;
}

void LQR::process_cmd_control(Controller_Output& u, int cmd_id)
{
    update(cmd_id, lqr_end, odom_data, u);
    // update(cmd_id, desired_data, odom_data, u);
}

void LQR::publish_ctrl(const Controller_Output& u)
{
    geometry_msgs::Twist tw_msg;
    tw_msg.linear.x = u.linear_vel;
    tw_msg.angular.z = u.angular_vel;
    ctrl_pub.publish(tw_msg);
}

void LQR::process_control(int cmd_id)
{
    Controller_Output u;
    process_cmd_control(u, cmd_id);
    publish_ctrl(u);
}

void LQR::publish_goal()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time();
    marker.ns = "tb3_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    // marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = lqr_end(0, 0);
    marker.pose.position.y = lqr_end(1, 0);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    goal_pub.publish(marker);
}

void LQR::process(int cmd_id)
{
    process_control(cmd_id);
    publish_goal();
}
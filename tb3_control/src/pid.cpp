#include "pid.h"
#include <uav_utils/geometry_utils.h>

using namespace std;

void Desired_State::feed(turtlebot3_msgs::CommandConstPtr pMsg)
{
    msg = *pMsg;
    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

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

void PID::config()
{
    config_gain(param.pid_gain);
    is_configured = true;
}

void PID::config_gain(const Parameter::Gain& gain)
{
    Kp.setZero();
    Kv.setZero();

    Kp(0, 0) = gain.Kp0;
    Kp(1, 1) = gain.Kp1;
    Kp(2, 2) = gain.Kp2;
    Kv(0, 0) = gain.Kv0;
    Kv(1, 1) = gain.Kv1;
    Kv(2, 2) = gain.Kv2;
    Kyaw = gain.Kyaw;
}

void PID::update(
    const Desired_State& des,
    const Odometry_Data& odom,
    Controller_Output& u
)
{
    ROS_ASSERT_MSG(is_configured, "Gains for controller might not be initialized.");
    Eigen::Vector3d e_p, e_v, u_e;
    double e_yaw = 0.0;
    
    double yaw_curr = uav_utils::get_yaw_from_quaternion(odom.q);
    double yaw_des = des.yaw;
    Eigen::Matrix3d wRc = uav_utils::rotz(yaw_curr);
    Eigen::Matrix3d cRw = wRc.transpose();

    e_p = des.p - odom.p;
    Eigen::Vector3d u_p = Kp * e_p;

    e_v = des.v - odom.v;
    Eigen::Vector3d u_v = Kv * e_v;

    e_yaw = yaw_des - yaw_curr;

    // Eliminate the discontinuity of yaw value in rviz
    while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
	while(e_yaw < -M_PI) e_yaw += (2 * M_PI);

    double u_yaw = Kyaw * e_yaw;

    u_e = Kp * e_p + Kv * e_v;

    u.linear_vel = sqrt(pow(u_e(0), 2) + pow(u_e(1), 2));
    u.angular_vel = Kyaw * e_yaw;
}

void PID::publish_ctrl(const Controller_Output& u)
{
    geometry_msgs::Twist tw_msg;
    tw_msg.linear.x = u.linear_vel;
    tw_msg.angular.z = u.angular_vel;
    ctrl_pub.publish(tw_msg);
}

void PID::process_cmd_control(Controller_Output& u)
{
    update(desired_data, odom_data, u);
}

void PID::process_control(const ros::Time& now_time)
{
    Controller_Output u;
    process_cmd_control(u);
    publish_ctrl(u);
}

void PID::process()
{
    ros::Time now_time = ros::Time::now();
    if ((now_time - last_ctrl_time).toSec() < (1.0 / param.ctrl_rate))
	{
		return;
	}
    last_ctrl_time = now_time;

    process_control(now_time);
}

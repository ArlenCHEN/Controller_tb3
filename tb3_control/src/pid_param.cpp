#include "pid_param.h"

Parameter::Parameter()
{
}

void Parameter::config_from_ros_handle(const ros::NodeHandle& nh)
{
    read_params(nh, "gain/Kp0", pid_gain.Kp0);
    read_params(nh, "gain/Kp1", pid_gain.Kp1);
    read_params(nh, "gain/Kp2", pid_gain.Kp2);

    read_params(nh, "gain/Kv0", pid_gain.Kv0);
    read_params(nh, "gain/Kv1", pid_gain.Kv1);
    read_params(nh, "gain/Kv2", pid_gain.Kv2);

    read_params(nh, "gain/Kyaw", pid_gain.Kyaw);

    // read_params(nh, "control_mode", control_mode);
    read_params(nh, "mass", mass);
    read_params(nh, "gravity", gravity);
    read_params(nh, "ctrl_rate", ctrl_rate);
};
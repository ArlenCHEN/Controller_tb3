#ifndef PID_PARAM_H_
#define PID_PARAM_H_

#include <iostream>

#include <ros/ros.h>

class Parameter
{
public:
    struct Gain
    {
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
        double Kyaw;

    };

    Gain pid_gain;
    double mass;
    double gravity;
    double ctrl_rate;
    
    Parameter();
    void config_from_ros_handle(const ros::NodeHandle& nh);

private:
    template<typename TName, typename TVal>
    void read_params(const ros::NodeHandle& nh, const TName& name, TVal& val)
    {
        if(nh.getParam(name, val))
        {
            // pass
        }
        else
        {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
};

#endif

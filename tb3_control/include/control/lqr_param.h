#ifndef LQR_PARAM_H_
#define LQR_PARAM_H_

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>

class Parameter
{
    public:
        struct Matrix_params
        {
            double q_0, q_1, q_2;
            double s_0, s_1, s_2, s_3;
            double e_0, e_1, e_2, e_3;
        };

        Matrix_params lqr_matrix_params;
        double lqr_time_step;
        double lqr_roh;
        double linear_speed_thresh;
        double angular_speed_thresh;
        int lqr_time_horizon;
        bool is_limit_speed;

        Parameter(){}
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
                ROS_ERROR_STREAM("Read param: "<< name << " failed.");
                ROS_BREAK();
            }
        };
};


#endif
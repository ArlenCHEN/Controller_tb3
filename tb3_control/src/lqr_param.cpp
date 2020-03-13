#include "lqr_param.h"

void Parameter::config_from_ros_handle(const ros::NodeHandle& nh)
{
    read_params(nh, "time_step", lqr_time_step);
    read_params(nh, "roh", lqr_roh);
    read_params(nh, "time_horizon", lqr_time_horizon);
    read_params(nh, "linear_speed_thresh", linear_speed_thresh);
    read_params(nh, "angular_speed_thresh", angular_speed_thresh);
    read_params(nh, "is_limit_speed", is_limit_speed);

    read_params(nh, "Q/q_0", lqr_matrix_params.q_0);
    read_params(nh, "Q/q_1", lqr_matrix_params.q_1);
    read_params(nh, "Q/q_2", lqr_matrix_params.q_2);
    
    read_params(nh, "start_point/s_0", lqr_matrix_params.s_0);
    read_params(nh, "start_point/s_1", lqr_matrix_params.s_1);
    read_params(nh, "start_point/s_2", lqr_matrix_params.s_2);
    read_params(nh, "start_point/s_3", lqr_matrix_params.s_3);

    read_params(nh, "end_point/e_0", lqr_matrix_params.e_0);
    read_params(nh, "end_point/e_1", lqr_matrix_params.e_1);
    read_params(nh, "end_point/e_2", lqr_matrix_params.e_2);
    read_params(nh, "end_point/e_3", lqr_matrix_params.e_3);
}
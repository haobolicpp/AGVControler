/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-29 16:06:48
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-11 16:54:46
 * @FilePath: /agv_controller/include/agv_net_ctrl/agv_ros_svr.hpp
 * @Description: 
 */
#ifndef AGV_ROS_SVR_H
#define AGV_ROS_SVR_H

#include "agv_tcp_ctrl.h"

#define ROS_SVR_BUFF_SIZE 32*1024*1024

st_tcp_connect_config * ros_svr_config_create();
int ros_svr_config_delete(st_tcp_connect_config *pt_config);

#endif // AGV_ROS_SVR_H
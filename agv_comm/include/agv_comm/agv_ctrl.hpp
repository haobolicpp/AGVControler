#ifndef AGV_CTRL_H
#define AGV_CTRL_H

#include "agv_comm/agv_type.h"
#include "agv_comm/agv_udp_ctrl.hpp"
#include "agv_comm/agv_tcp_ctrl.hpp"
#include "agv_comm/agv_ros_ctrl.hpp"
#include "agv_comm/agv_config.h"


#define AGV_CTRL_M1_VERSION 1
#define AGV_CTRL_S1_VERSION 1
#define AGV_CTRL_S2_VERSION 2


#define AGV_UDP_PORT      20001
#define AGV_TCP_PORT_CTRL 20002
#define AGV_TCP_PORT_DATA 20003
#define AGV_CLIENT_MAX 6

struct agv_ctrl
{
    

    st_udp_svr_ctrl t_udp_ctrl;
    st_tcp_connect_ctrl t_connect_ctrl;
    st_agv_base_config t_agv_base_config;

    CAgvRosCtrl *po_ros_ctrl;
};


int agv_ctrl_init(st_agv_ctrl *pt_agv_ctrl, ros::NodeHandle *nh);
int agv_ctrl_run(st_agv_ctrl *pt_agv_ctrl);
int agv_ctrl_routine(st_agv_ctrl *pt_agv_ctrl);

int agv_ctrl_process(st_agv_event *pt_event, 
               st_tcp_connect_info *pt_connect_info, 
               CAgvRosCtrl *po_ros_ctrl);

#endif // AGV_CTRL_H
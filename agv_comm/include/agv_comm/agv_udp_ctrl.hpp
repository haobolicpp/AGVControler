
#ifndef AGV_UDP_CTRL_H
#define AGV_UDP_CTRL_H

#include <pthread.h>
#include <agv_comm/sys_queue.h>
#include <agv_comm/agv_type.h>


#define AGV_UDP_BUFF_SIZE 1024



struct udp_svr_ctrl
{
    pthread_t thread_handler_udp_svr;
    LIST_HEAD(, agv_cmd_handler) lh_cmd_handler;
};

int udp_svr_ctrl_init(st_agv_ctrl *pt_agv_ctrl);

int udp_svr_regist_handler(st_udp_svr_ctrl *pt_udp_svr_ctrl, st_agv_cmd_handler *pt_handler);

int agv_udp_svr_run(st_agv_ctrl *pt_agv_ctrl);



#endif // AGV_UDP_CTRL_H









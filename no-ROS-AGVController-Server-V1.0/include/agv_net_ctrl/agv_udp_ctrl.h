/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-08 17:04:19
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 10:59:42
 * @FilePath: /agv_controller/include/agv_net_ctrl/agv_udp_ctrl.h
 * @Description: UDP服务
 */

#ifndef AGV_UDP_CTRL_H
#define AGV_UDP_CTRL_H

#include <pthread.h>

#include "sys_queue.h"
#include "agv_type.h"


#define AGV_UDP_BUFF_SIZE 1024

typedef struct udp_svr_ctrl st_udp_svr_ctrl;

struct udp_svr_ctrl
{
    pthread_t thread_handler_udp_svr;
    LIST_HEAD(, tcp_cmd_handler) lh_cmd_handler;
};

int udp_svr_ctrl_init(AgvCtrl *pt_agv_ctrl);

int udp_svr_regist_handler(st_udp_svr_ctrl *pt_udp_svr_ctrl, st_udp_cmd_handler *pt_handler);

int agv_udp_svr_run(AgvCtrl *pt_agv_ctrl);



#endif // AGV_UDP_CTRL_H









/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-08 17:04:19
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 15:06:05
 * @FilePath: /agv_controller/include/agv_net_ctrl/agv_tcp_svr.h
 * @Description: TCP Server头文件
 */

#ifndef AGV_TCP_SVR_H
#define AGV_TCP_SVR_H

#include <pthread.h>

#include "sys_queue.h"

#include "agv_cmd.h"
#include "agv_tcp_ctrl.h"
#include "agv_tcp_svr.h"



typedef struct robot_tcp_svr
{
    pthread_t th_tcp_svr;

    int connect_max_;
    int connect_now_;

    pthread_mutex_t mutex_connect;
    LIST_HEAD(, tcp_connect_info) lh_connect_pool;
    LIST_HEAD(, tcp_connect_info) lh_connect_work;

    // pthread_mutex_t mutex_work;


    void *pRoot;

}st_robot_tcp_svr;


int robot_tcp_svr_init(st_robot_tcp_svr *pt_tcp_svr, int connect_max, void *pRoot);
int robot_tcp_svr_run(st_robot_tcp_svr *pt_tcp_svr);
int robot_tcp_svr_deinit(st_robot_tcp_svr *pt_tcp_svr);


#endif // AGV_TCP_SVR_H






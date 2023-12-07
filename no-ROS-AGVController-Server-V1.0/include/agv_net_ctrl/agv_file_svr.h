/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-08 17:04:19
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 11:13:27
 * @FilePath: /agv_controller/include/agv_net_ctrl/agv_file_svr.h
 * @Description: 大文件传输服务
 */
#ifndef AGV_FILE_SVR_H
#define AGV_FILE_SVR_H

#include "agv_tcp_ctrl.h"

#define AGV_FILE_MAX_SIZE 32*1024*1024

st_tcp_connect_config * file_svr_config_create();
int file_svr_config_delete(st_tcp_connect_config *pt_config);



#endif // AGV_FILE_SVR_H
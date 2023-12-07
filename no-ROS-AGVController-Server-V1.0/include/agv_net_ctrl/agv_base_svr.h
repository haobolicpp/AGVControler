/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-08 17:04:19
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-11 13:44:11
 * @FilePath: /agv_controller/include/agv_net_ctrl/agv_base_svr.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef AGV_BASE_SVR_H
#define AGV_BASE_SVR_H

#include "agv_tcp_ctrl.h"

#define BASE_SVR_BUFF_SIZE 1024*1024


st_tcp_connect_config * base_svr_config_create();
int base_svr_config_delete(st_tcp_connect_config *pt_config);


#endif // AGV_BASE_SVR_H





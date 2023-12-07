/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-11 13:44:37
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-11 17:46:24
 * @FilePath: /agv_controller/include/agv_net_ctrl/agv_tcp_inst.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef AGV_TCP_CLIENT_H
#define AGV_TCP_CLIENT_H

#include <stdint.h>

//通信线程前端发送和接受缓冲区大小
#define AGV_TCP_BUFF_SIZE 8192

int tcp_connect_instance_create(st_tcp_connect_info *pt_info);

#endif // AGV_TCP_CLIENT_H
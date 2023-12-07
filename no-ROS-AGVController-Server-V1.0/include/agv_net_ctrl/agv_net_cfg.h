/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-11 16:53:46
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 13:44:08
 * @FilePath: /agv_controller/include/agv_net_ctrl/agv_net_cfg.h
 * @Description: 用户通信规格定义
 */
#ifndef AGV_NET_CFG_H
#define AGV_NET_CFG_H

#define AGV_UDP_PORT      20001
#define AGV_TCP_PORT_BASE 20002
#define AGV_TCP_PORT_FILE 20003
#define AGV_TCP_PORT_ROS  20004

//用户服务类型定义
typedef enum
{
    E_AGV_SVR_BASE = 1,
    E_AGV_SVR_FILE = 2,
    E_AGV_SVR_ROS  = 3,
    E_AGV_SVR_MAX
}E_TCP_CONNECT_TYPE;

//用户周期传输事件类型定义
typedef enum
{
    TCP_EVENT_HEART_BEAT = 0,
    AGV_EVENT_SUB_BASE_INFO = 1,
    AGV_EVENT_SUB_MAP_INFO = 2,
    AGV_EVENT_SUB_PATH_INFO = 3,
    AGV_EVENT_PUB_VEL_CMD = 4,
    AGV_EVENT_CALL = 5,
    AGV_EVENT_MAX
}E_TCP_EVENT_TYPE;


#endif // AGV_NET_CFG_H
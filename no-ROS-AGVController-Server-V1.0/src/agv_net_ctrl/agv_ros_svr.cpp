/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-29 16:07:51
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 10:44:24
 * @FilePath: /agv_controller/src/agv_net_ctrl/agv_ros_svr.cpp
 * @Description: 
 */
#include <stdio.h>
#include <stdlib.h>

#include "agv_type.h"
#include "agv_cmd.h"

#include "agv_ros_svr.h"
#include "agv_tcp_ctrl.h"

/**
 * @name: ros_svr_config_create
 * @des:  向ROS客户端提供数据的服务
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
st_tcp_connect_config * ros_svr_config_create()
{
    st_tcp_connect_config *pt_config = 
           (st_tcp_connect_config *) malloc(sizeof(st_tcp_connect_config));
    
    if (pt_config == NULL)
    {
        return NULL;
    }

    pt_config->connect_type = E_AGV_SVR_ROS;
    pt_config->stream_buffer_size = ROS_SVR_BUFF_SIZE;

    LIST_INIT(&pt_config->lh_cmd_handler);
    st_tcp_cmd_handler *pt_handler;

    //暂时不添加命令解析
    return pt_config;
}

/**
 * @name: ros_svr_config_create
 * @des: 
 * @param {st_tcp_connect_config} *pt_config
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int ros_svr_config_delete(st_tcp_connect_config *pt_config)
{
    if (pt_config != NULL)
    {
        free(pt_config);
        pt_config = NULL;
    }
    return 0;
}


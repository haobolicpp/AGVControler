/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-08 17:04:19
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 16:19:35
 * @FilePath: /agv_controller/src/agv_common/agv_cmd.cpp
 * @Description: 消息队列通信 与 消息处理方法
 */

#include "agv_cmd.h"

/**
 * @name: robot_msg_init
 * @des:  初始化用于线程间通信的消息
 * @param {st_robot_msg} *pt_msg
 * @param {st_robot_msg_header} t_msg_header
 * @param {uint8_t} *pbody
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int robot_msg_init(st_robot_msg *pt_msg, st_robot_msg_header header, uint8_t *pbody)
{
    if (pt_msg == NULL || header.s32_len < 0)
    {
        return -1;
    }

    //组建消息头部
    memcpy(&pt_msg->t_msg_header, &header, sizeof(st_robot_msg_header));
    pt_msg->res_flag = 0;
    //简单消息(仅有头部信息描述)
    if (header.s32_len == 0)
    {
        //nothing todo
    }
    //短消息
    else if (header.s32_len > 0 && header.s32_len <= AGV_CMD_S_BODY_LEN && pbody != NULL)
    {
        memcpy(pt_msg->t_msg_body.sbody, pbody, header.s32_len);
    }
    //长消息
    else if (header.s32_len > AGV_CMD_S_BODY_LEN && pbody != NULL)
    {
        pt_msg->t_msg_body.lbody = NULL;
        pt_msg->t_msg_body.lbody = (uint8_t *)malloc(header.s32_len);
        if (pt_msg->t_msg_body.lbody == NULL)
        {
            return -1;
        }
        memcpy(pt_msg->t_msg_body.lbody, pbody, header.s32_len);
        pt_msg->res_flag = 1; //标记长消息
    }
    else
    {
        //非法构造
        return -1;
    }

    return 0;
}


/**
 * @name: robot_msg_release
 * @des:  释放消息资源
 * @param {st_robot_msg} *pt_msg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int robot_msg_release(st_robot_msg *pt_msg)
{
    if (pt_msg == NULL)
    {
        return -1;
    }

    memset(&pt_msg->t_msg_header, 0, sizeof(st_robot_msg_header));

    //带有资源的消息判定
    if (pt_msg->t_msg_header.s32_len > AGV_CMD_S_BODY_LEN 
        &&pt_msg->res_flag == 1 
        && pt_msg->t_msg_body.lbody != NULL)
    {
        pt_msg->res_flag = 0;
        free(pt_msg->t_msg_body.lbody);
        pt_msg->t_msg_body.lbody = NULL;
    }

    return 0;
}


/**
 * @name: robot_msg_get_body
 * @des:  获取消息体指针
 * @param {st_robot_msg} *pt_msg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
uint8_t* robot_msg_get_body(st_robot_msg *pt_msg)
{
    if (pt_msg == NULL)
    {
        return NULL;
    }
    
    //简单消息
    if (pt_msg->t_msg_header.s32_len <= 0)
    {
        return NULL;
    }
    
    //短消息
    else if (pt_msg->t_msg_header.s32_len > 0 
        && pt_msg->res_flag == 0
        && pt_msg->t_msg_header.s32_len <= AGV_CMD_S_BODY_LEN)
    {
        return pt_msg->t_msg_body.sbody;
    }

    //长消息
    else if (pt_msg->t_msg_header.s32_len > AGV_CMD_S_BODY_LEN 
        && pt_msg->res_flag != 0)
    {
        return pt_msg->t_msg_body.lbody;
    }

    //错误
    else
    {
        return NULL;
    }
}
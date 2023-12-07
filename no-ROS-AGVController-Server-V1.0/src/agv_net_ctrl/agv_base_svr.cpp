#include <stdio.h>
#include <stdlib.h>

#include "agv_type.h"
#include "agv_cmd.h"
#include "agv_base_svr.h"
#include "agv_tcp_ctrl.h"
#include "agv_map_ctrl.h"
#include "AgvCtrl.h"


int base_svr_set_target_station(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);
int base_svr_cancel_target_move(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);

int base_svr_mani_word(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);
int base_svr_mani_request(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);

/**
 * @name: base_svr_config_create
 * @des:  控制命令服务配置
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
st_tcp_connect_config * base_svr_config_create()
{
    st_tcp_connect_config *pt_config = 
           (st_tcp_connect_config *) malloc(sizeof(st_tcp_connect_config));
    
    if (pt_config == NULL)
    {
        return NULL;
    }

    pt_config->connect_type = E_AGV_SVR_BASE;
    pt_config->stream_buffer_size = BASE_SVR_BUFF_SIZE;

    LIST_INIT(&pt_config->lh_cmd_handler);
    st_tcp_cmd_handler *pt_handler;

//todo ADD the cmd process element
//-------------------------register map download req--------------------
    pt_handler = agv_cmd_handler_create("set_target_station", 
        AGV_CMD_C_CTRL, 
        AGV_CMD_T_CTRL_SET_STAION,
        base_svr_set_target_station);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    pt_handler = agv_cmd_handler_create("cancel_target_move", 
        AGV_CMD_C_CTRL, 
        AGV_CMD_T_CTRL_CANCEL_MOVE,
        base_svr_cancel_target_move);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    pt_handler = agv_cmd_handler_create("mani_request", 
        AGV_CMD_C_CTRL, 
        AGV_CMD_T_CTRL_MANI_REQ,
        base_svr_mani_request);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    pt_handler = agv_cmd_handler_create("mani_word", 
        AGV_CMD_C_CTRL, 
        AGV_CMD_T_CTRL_MANI_WORD,
        base_svr_mani_word);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    return pt_config;
}


/**
 * @name: base_svr_config_delete
 * @des:  控制命令服务清除
 * @param {st_tcp_connect_config} *pt_config
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int base_svr_config_delete(st_tcp_connect_config *pt_config)
{
    if (pt_config != NULL)
    {
        free(pt_config);
        pt_config = NULL;
    }
    return 0;
}



/**
 * @name: base_svr_set_target_station
 * @des:  设置导航目标命令
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int base_svr_set_target_station(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;

    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *> (pt_connect_info->pRoot);

    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    uint8_t *pt_msg_body = cmd_in + AGV_COMMON_HEADER_LEN;
    
    int target_station = *(int *)(pt_msg_body + 0);
    float target_x = *(float *)(pt_msg_body + 4);
    float target_y = *(float *)(pt_msg_body + 8);
    float target_w = *(float *)(pt_msg_body + 12);

    printf("INFO: TCP Base Server Get HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);
    printf("INFO: TCP Base Server Set Target Station id[%d]-x[%f]-y[%f]-w[%f]\n",
        target_station, target_x, target_y, target_w);


    //V1.02 使用路径规划器和PID实现AGV的轨迹跟踪控制
    TAgvPose2D tTargetPose;
    tTargetPose.x = target_x;
    tTargetPose.y = target_y;
    tTargetPose.phi = target_w;
    ret = poAgvCtrl->CallMoveToTarget(pt_connect_info, tTargetPose);
    if (ret == 0)
    {
        *olen = 0; //等待底盘规划模块发送路径消息给CLient
        return 0;
    }

    //路径跟踪任务请求失败回复
    cmd_offset = 0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_1;
    cmd_out[cmd_offset++] = AGV_FRAME_H_2;
    cmd_out[cmd_offset++] = AGV_FRAME_H_3;

    pt_msg_header->u16_type = pt_msg_header->u16_type | 0x8000;
    pt_msg_header->u16_src = 0x10;
    pt_msg_header->u16_dest = 0x1;
    pt_msg_header->s32_len = 1;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_robot_msg_header));
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);

    //Return Msg Body 1 is Call Failed
    uint8_t res = 1;
    memcpy((cmd_out + cmd_offset), &res, 1);
    cmd_offset = cmd_offset + 1;

     *olen = cmd_offset;

     return 0;

}

/**
 * @name: base_svr_cancel_target_move
 * @des:  导航目标撤销命令
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int base_svr_cancel_target_move(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;

    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *> (pt_connect_info->pRoot);

    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    uint8_t *pt_msg_body = cmd_in + AGV_COMMON_HEADER_LEN;
    

    printf("INFO: TCP Base Server Get HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);

    poAgvCtrl->CallCancalTarget(); 


    //Failed
    //ADD HEADER
    cmd_offset = 0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_1;
    cmd_out[cmd_offset++] = AGV_FRAME_H_2;
    cmd_out[cmd_offset++] = AGV_FRAME_H_3;

    pt_msg_header->u16_type = pt_msg_header->u16_type | 0x8000;
    pt_msg_header->u16_src = 0x10;
    pt_msg_header->u16_dest = 0x1;
    pt_msg_header->s32_len = 0;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_robot_msg_header));
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);

    *olen = cmd_offset;

    return 0;
}


/**
 * @name: base_svr_mani_word
 * @des:  遥控操作命令
 * @param {void} *data
 * @param {uint8_t} *cmd_in
 * @param {int} ilen
 * @param {uint8_t} *cmd_out
 * @param {int} *olen
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int base_svr_mani_word(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;

    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;

    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *> (pt_connect_info->pRoot);

    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    uint8_t *pt_msg_body = cmd_in + AGV_COMMON_HEADER_LEN;

    int mani_word = *(int *)pt_msg_body;
    mani_word = mani_word & 0x0000000f;
    float mlv = poAgvCtrl->mani_linear_vel_def;
    float mav = poAgvCtrl->mani_angular_vel_def;
    //每次收到上位机遥控命令字就复位看门狗，标明当前控制有效
    poAgvCtrl->ManiWatchDogReset();

    switch (mani_word)
    {
        case 0b0000: //没有按键按下
            poAgvCtrl->SetChassisVel(0, 0);
        break;

        case 0b0001: //↑按键按下 前进
            poAgvCtrl->SetChassisVel(mlv, 0);
        break;

        case 0b0010: //↓按键按下 后退
            poAgvCtrl->SetChassisVel(-mlv, 0);
        break;

        case 0b0100: //← 逆时针原地旋转
            poAgvCtrl->SetChassisVel(0, mav); //PI/4
        break;

        case 0b1000: //→ 顺时针原地旋转
            poAgvCtrl->SetChassisVel(0, -mav); //PI/4
        break;

        case 0b0101: //↖ 
            poAgvCtrl->SetChassisVel(mlv, mav); //PI/4
        break;

        case 0b1001: //↗ 
            poAgvCtrl->SetChassisVel(mlv, -mav); //PI/4
        break;

        case 0b0110: //↙ 
            poAgvCtrl->SetChassisVel(-mlv, mav); //PI/4
        break;

        case 0b1010: //↘
            poAgvCtrl->SetChassisVel(-mlv, -mav); //PI/4
        break;

        default:
            poAgvCtrl->SetChassisVel(0, 0);
        break;
    }



    return 0;
}

/**
 * @name: base_svr_mani_request
 * @des:  遥控操作切换
 * @param {void} *data
 * @param {uint8_t} *cmd_in
 * @param {int} ilen
 * @param {uint8_t} *cmd_out
 * @param {int} *olen
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int base_svr_mani_request(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;

    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *> (pt_connect_info->pRoot);

    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    uint8_t *pt_msg_body = cmd_in + AGV_COMMON_HEADER_LEN;

    int mode_req_result;
    int mode = *(int *)pt_msg_body;

    //准备应答数据
    cmd_offset = 0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_1;
    cmd_out[cmd_offset++] = AGV_FRAME_H_2;
    cmd_out[cmd_offset++] = AGV_FRAME_H_3;

    pt_msg_header->u16_type = pt_msg_header->u16_type | 0x8000;
    pt_msg_header->u16_src = 0x10;
    pt_msg_header->u16_dest = 0x1;
    pt_msg_header->s32_len = 4;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_robot_msg_header));
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);

    //申请将其切换至手动遥控状态
    if (mode == 1)
    {
        if (false == poAgvCtrl->RequestChassisMode(1))
        {
            mode_req_result = -1;
            memcpy((cmd_out + cmd_offset), &mode_req_result, 4);
            cmd_offset = cmd_offset + 4;
            *olen = cmd_offset;
            return -1;
        }
        //注册周期循环事件 发布遥控速度指令
        ret = tcp_connect_regist_event(pt_connect_info, AGV_EVENT_PUB_VEL_CMD, 2);
        if (ret != 0)
        {
            mode_req_result = -1;
            memcpy((cmd_out + cmd_offset), &mode_req_result, 4);
            cmd_offset = cmd_offset + 4;
            *olen = cmd_offset;
            return -1;
        }
        //所有准备就绪 切换至遥控状态成功
        mode_req_result = 1;
        memcpy((cmd_out + cmd_offset), &mode_req_result, 4);
        cmd_offset = cmd_offset + 4;
        *olen = cmd_offset;
        return 0;
    }

    //申请将其切换至自主导航状态
    else if (mode == 0)
    {
        if (false == poAgvCtrl->RequestChassisMode(0))
        {
            mode_req_result = -1;
            memcpy((cmd_out + cmd_offset), &mode_req_result, 4);
            cmd_offset = cmd_offset + 4;
            *olen = cmd_offset;
            return -1;
        }
        //注销 发布遥控速度指令的 周期循环事件
        ret = tcp_connect_remove_event(pt_connect_info, AGV_EVENT_PUB_VEL_CMD);
        if (ret != 0)
        {
            mode_req_result = -1;
            memcpy((cmd_out + cmd_offset), &mode_req_result, 4);
            cmd_offset = cmd_offset + 4;
            *olen = cmd_offset;
            return -1;
        }
        //所有准备就绪 切换至遥控状态成功
        mode_req_result = 0;
        memcpy((cmd_out + cmd_offset), &mode_req_result, 4);
        cmd_offset = cmd_offset + 4;
        *olen = cmd_offset;
        return 0;
    }

    //无效的切换
    else
    {
        mode_req_result = -1;
        memcpy((cmd_out + cmd_offset), &mode_req_result, 4);
        cmd_offset = cmd_offset + 4;
        *olen = cmd_offset;
        return 0;
    }

}
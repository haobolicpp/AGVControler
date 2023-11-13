#include <stdio.h>
#include <stdlib.h>

#include <agv_comm/agv_type.h>
#include <agv_comm/agv_cmd.hpp>
#include <agv_comm/agv_base_svr.hpp>
#include <agv_comm/agv_tcp_ctrl.hpp>
#include <agv_comm/agv_map_ctrl.h>


int base_svr_set_target_station(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);
int base_svr_cancel_target_move(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);

int base_svr_mani_word(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);
int base_svr_mani_request(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);

/******************************************************************************
* 函数名称: base_svr_config_create()
* 作 用 域: 
* 功能描述: 创建文件服务配置结构实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
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
    st_agv_cmd_handler *pt_handler;

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


/******************************************************************************
* 函数名称: tcp_connect_config_create()
* 作 用 域: 
* 功能描述: 创建对应实例的服务配置结构实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int base_svr_config_delete(st_tcp_connect_config *pt_config)
{
    if (pt_config != NULL)
    {
        free(pt_config);
        pt_config = NULL;
    }
    return 0;
}



/******************************************************************************
* 函数名称: base_svr_set_target_station()
* 作 用 域: 
* 功能描述: 创建对应实例的服务配置结构实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int base_svr_set_target_station(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    st_agv_msg_header *pt_msg_header = (st_agv_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    uint8_t *pt_msg_body = cmd_in + AGV_COMMON_HEADER_LEN;
    
    int target_station = *(int *)(pt_msg_body + 0);
    float target_x = *(float *)(pt_msg_body + 4);
    float target_y = *(float *)(pt_msg_body + 8);
    float target_w = *(float *)(pt_msg_body + 12);

    printf("HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);
    printf("Get Target Station id[%d]-x[%f]-y[%f]-w[%f]\n",target_station, target_x, target_y, target_w);

    /*TODO 
    //1.路径规划
    //2.传递move_base
    //3.*/
    double dRotateToAngle = 0; //TODO
    std::vector<TPointd> global_path = pt_connect_info->po_ros_ctrl->m_pAGVMapCtrl->get_path(pt_connect_info->po_ros_ctrl->tBaseInfo.tPt, target_station, dRotateToAngle);

    if (!global_path.empty())
    {
        //发送move_base
        ret = pt_connect_info->po_ros_ctrl->CallMoveToTarget(pt_connect_info, dRotateToAngle, target_w, global_path);

        //Success
        if (ret == 0)
        {
            //发送AGVClient路径
            //ADD HEADER
            cmd_offset = 0;
            cmd_out[cmd_offset++] = AGV_FRAME_H_0;
            cmd_out[cmd_offset++] = AGV_FRAME_H_1;
            cmd_out[cmd_offset++] = AGV_FRAME_H_2;
            cmd_out[cmd_offset++] = AGV_FRAME_H_3;

            pt_msg_header->u16_type = AGV_CMD_T_CTRL_GLOBAL_PATH;
            pt_msg_header->u16_src = 0x10;
            pt_msg_header->u16_dest = 0x1;
            pt_msg_header->s32_len = 4 + 8 * global_path.size();

            memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_agv_msg_header));
            cmd_offset = cmd_offset + sizeof(st_agv_msg_header);

            uint32_t pathNum = global_path.size();
            memcpy((cmd_out + cmd_offset), &pathNum, 4);
            cmd_offset += 4;

            for (int i = 0; i < global_path.size(); i++)
            {
                float fx = (float)(global_path[i].dX);
                float fy = (float)(global_path[i].dY);
                printf("fx:%.6f, fy:%.6f\n", fx, fy);
                memcpy((cmd_out + cmd_offset), &fx, 4);
                cmd_offset += 4;
                memcpy((cmd_out + cmd_offset), &fy, 4);
                cmd_offset += 4;
            }

            *olen = cmd_offset;

            printf("send global path sucess!");
            return 0;
        }
    }

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
    pt_msg_header->s32_len = 1;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_agv_msg_header));
    cmd_offset = cmd_offset + sizeof(st_agv_msg_header);

    //Return Msg Body 1 is Call Failed
    uint8_t res = 1;
    memcpy((cmd_out + cmd_offset), &res, 1);
    cmd_offset = cmd_offset + 1;

     *olen = cmd_offset;

     return 0;

}

/******************************************************************************
* 函数名称: base_svr_cancel_target_move()
* 作 用 域: 
* 功能描述: 创建对应实例的服务配置结构实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int base_svr_cancel_target_move(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    st_agv_msg_header *pt_msg_header = (st_agv_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    uint8_t *pt_msg_body = cmd_in + AGV_COMMON_HEADER_LEN;
    

    printf("HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);

    pt_connect_info->po_ros_ctrl->CallCancalTarget(); 

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

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_agv_msg_header));
    cmd_offset = cmd_offset + sizeof(st_agv_msg_header);

    *olen = cmd_offset;

    return 0;
}


/******************************************************************************
* 函数名称: base_svr_mani_word()
* 作 用 域: 
* 功能描述: 远程遥控指令解析与执行↑ ↓ ← → ↖ ↗
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年12月5日
******************************************************************************/
int base_svr_mani_word(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;

    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    st_agv_msg_header *pt_msg_header = (st_agv_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    uint8_t *pt_msg_body = cmd_in + AGV_COMMON_HEADER_LEN;

    int mani_word = *(int *)pt_msg_body;
    mani_word = mani_word & 0x0000000f;
    float mlv = pt_connect_info->po_ros_ctrl->mani_linear_vel_def;
    float mav = pt_connect_info->po_ros_ctrl->mani_angular_vel_def;
    //每次收到上位机遥控命令字就复位看门狗，标明当前控制有效
    pt_connect_info->po_ros_ctrl->ManiWatchDogReset();

    switch (mani_word)
    {
        case 0b0000: //没有按键按下
            pt_connect_info->po_ros_ctrl->SetChassisVel(0, 0);
        break;

        case 0b0001: //↑按键按下 前进
            pt_connect_info->po_ros_ctrl->SetChassisVel(mlv, 0);
        break;

        case 0b0010: //↓按键按下 后退
            pt_connect_info->po_ros_ctrl->SetChassisVel(-mlv, 0);
        break;

        case 0b0100: //← 逆时针原地旋转
            pt_connect_info->po_ros_ctrl->SetChassisVel(0, mav); //PI/4
        break;

        case 0b1000: //→ 顺时针原地旋转
            pt_connect_info->po_ros_ctrl->SetChassisVel(0, -mav); //PI/4
        break;

        case 0b0101: //↖ 
            pt_connect_info->po_ros_ctrl->SetChassisVel(mlv, mav); //PI/4
        break;

        case 0b1001: //↗ 
            pt_connect_info->po_ros_ctrl->SetChassisVel(mlv, -mav); //PI/4
        break;

        case 0b0110: //↙ 
            pt_connect_info->po_ros_ctrl->SetChassisVel(-mlv, mav); //PI/4
        break;

        case 0b1010: //↘
            pt_connect_info->po_ros_ctrl->SetChassisVel(-mlv, -mav); //PI/4
        break;

        default:
            pt_connect_info->po_ros_ctrl->SetChassisVel(0, 0);
        break;
    }



    return 0;
}

/******************************************************************************
* 函数名称: base_svr_mani_request()
* 作 用 域: 
* 功能描述: 远程遥控/自动导航 切换
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年12月5日
******************************************************************************/
int base_svr_mani_request(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    st_agv_msg_header *pt_msg_header = (st_agv_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
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

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_agv_msg_header));
    cmd_offset = cmd_offset + sizeof(st_agv_msg_header);

    //申请将其切换至手动遥控状态
    if (mode == 1)
    {
        if (false == pt_connect_info->po_ros_ctrl->RequestChassisMode(1))
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
        if (false == pt_connect_info->po_ros_ctrl->RequestChassisMode(0))
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
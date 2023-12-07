/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-17 11:02:40
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-13 11:31:46
 * @FilePath: /agv_controller/src/agv_ros_ctrl/AgvRosSvr.cpp
 * @Description: ROS数据监控服务
 */

#include "AgvRosSvr.h"
#include "AgvCtrl.h"
#include "agv_cmd.h"

/**
 * @name: 
 * @des: 
 * @param {* args} *
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
AgvRosSvr::AgvRosSvr(AgvCtrl *poAgvCtrl)
{
    poAgvCtrl_ = poAgvCtrl;
}

/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
AgvRosSvr::~AgvRosSvr()
{
}


/**
 * @name: AsyncMsgPost
 * @des:  异步消息发送至ROS调试端
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRosSvr::AsyncMsgPost(TRobotMsg *ptAgvMsg)
{
    //ROS调试端未连接
    if (!bRosConnected || ptRosConnectInfo_ ==NULL)
    {
        RobotMsgRelease(ptAgvMsg);
        return -1;
    }

    int s32Ret;
    s32Ret = mq_send(ptRosConnectInfo_->q_svr_handler, (char *)ptAgvMsg, sizeof(st_robot_msg), 0);
    if (s32Ret < 0)
    {
        printf("ERROR: Agv Ros Svr Msg Post: %d\n",errno);
        RobotMsgRelease(ptAgvMsg);
        return -1;
    }


    return 0;
}

/**
 * @name: 
 * @des: 
 * @param {st_tcp_connect_info} *tRosConnectInfo
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRosSvr::Init(st_tcp_connect_info *ptRosConnectInfo)
{
    ptRosConnectInfo_ = ptRosConnectInfo;
    bRosConnected = true;

    //发送当前全局地图给ROS端
    int s32Ret;
    int s32Len;
    TRobotMsg tAgvMsg;
    TRobotMsgHeader tAgvMsgHeader;

    memset(pSerializeBuff, 0, SERIALIZE_LENGTH_MAX);
    s32Ret = SerializeOccupancyGrid(poAgvCtrl_->tMapGrid_, pSerializeBuff, s32Len);
    if (s32Ret < 0)
    {
        printf("ERROR: Agv Ros Svr Serialize MapGrid Failed!\n");
    }

    //构造消息发送至ROS连接服务线程
    tAgvMsgHeader.u16_class = AGV_ROS_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_ROS_T_GLOBAL_MAP;
    tAgvMsgHeader.s32_len = s32Len;

    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, pSerializeBuff);
    s32Ret = AsyncMsgPost(&tAgvMsg);
    if (s32Ret < 0)
    {
        printf("ERROR: Agv Ros Svr Post MapGrid Failed!\n");
    }
    

    return 0;
}
/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRosSvr::DeInit()
{
    bRosConnected = false;
    ptRosConnectInfo_ = NULL;
    return 0;
}


/**
 * @name: SendLocalMapGrid
 * @des:  发送局部地图
 * @param {TOccupancyGrid} &tMapGrid_
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvRosSvr::SendLocalMapGrid(TOccupancyGrid &tMapGrid_)
{
    int s32Ret;
    int s32Len;
    TRobotMsg tAgvMsg = {0};
    TRobotMsgHeader tAgvMsgHeader = {0};

    if(!bRosConnected)
    {
        return -1;
    }

    memset(pSerializeBuff, 0, SERIALIZE_LENGTH_MAX);
    s32Ret = SerializeOccupancyGrid(tMapGrid_, pSerializeBuff, s32Len);
    if (s32Ret < 0)
    {
        printf("ERROR: Agv Ros Svr Serialize MapGrid Failed!\n");
    }

    //构造消息发送至ROS连接服务线程
    tAgvMsgHeader.u16_class = AGV_ROS_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_ROS_T_LOCAL_MAP;
    tAgvMsgHeader.s32_len = s32Len;
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, pSerializeBuff);

    s32Ret = AsyncMsgPost(&tAgvMsg);
    if (s32Ret < 0)
    {
        return -1;
    }

    return 0;
}
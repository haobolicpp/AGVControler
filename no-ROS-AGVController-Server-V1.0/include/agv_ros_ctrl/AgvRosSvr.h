/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-17 11:02:39
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-13 11:26:54
 * @FilePath: /agv_controller/include/agv_ros_ctrl/AgvRosSvr.h
 * @Description: 
 */

#ifndef AGVROSSVR_H
#define AGVROSSVR_H

#include "agv_type.h"
#include "agv_cmd.h"
#include "agv_tcp_ctrl.h"


class AgvRosSvr
{
private:
    /* data */
public:
    AgvRosSvr(AgvCtrl *poAgvCtrl);
    ~AgvRosSvr();

public:

    AgvCtrl *poAgvCtrl_;

    bool bRosConnected = false;
    st_tcp_connect_info *ptRosConnectInfo_;

    int Init(st_tcp_connect_info *ptRosConnectInfo);
    int DeInit();
    int AsyncMsgPost(TRobotMsg *ptAgvMsg);

    int SendLocalMapGrid(TOccupancyGrid &tMapGrid_);
    int SendGlobalMapGrid(TOccupancyGrid &tMapGrid_);
    uint8_t pSerializeBuff[SERIALIZE_LENGTH_MAX];

};




#endif // AGVROSSVR_H
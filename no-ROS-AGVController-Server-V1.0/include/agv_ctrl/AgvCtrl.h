/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-08 17:04:19
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-13 11:18:58
 * @FilePath: /agv_controller/include/agv_ctrl/AgvCtrl.h
 * @Description: Agv控制类定义
 */

#ifndef AGVCTRL_H
#define AGVCTRL_H

#include <pthread.h>
#include <mqueue.h>
#include <boost/thread/recursive_mutex.hpp>

#include "agv_type.h"
#include "agv_cmd.h"
#include "agv_config.h"
#include "agv_map_ctrl.h"
#include "agv_tcp_svr.h"
#include "agv_udp_ctrl.h"

#include "AgvSlammer.h"
#include "AgvTf.h"
#include "agv_chassis_ctrl.h"
#include "AgvSickScanDriver.h"
#include "AgvRvdCtrl.h"
#include "AgvRosSvr.h"


enum class E_AGV_CTRL_STATE
{
    INIT = 0,
    NAVI,
    NAVI2SLAM,
    SLAM,
    SLAM2NAVI,
    ERROR,
    MAX
};

class AgvSickScanDriver;

class AgvCtrl
{

public:
    int s32AgvCtrlFreq;
    bool bHasInited;
    bool bHasStarted;
    mqd_t qAgvCtrlSvr;
    pthread_t thAgvCtrlSvr;

public:
    //AGV Paramter Config
    st_agv_base_config t_agv_base_config;
    //UDP Server Management
    st_udp_svr_ctrl t_udp_ctrl;
    //TCP Connect Management
    st_robot_tcp_svr t_agv_tcp_svr;
    //全局地图管理与全局路径规划
    CAGVMapCtrl *poAgvMapCtrl;//地图对象

    //agv slammer模块
    AgvSlammer *poAgvSlammer;
    //agv tf
    AgvTf   *poAgvTf;
    //agv chassis
    AgvChassisCtrl *poAgvChassisCtrl;
    //Laser Scan
    AgvSickScanDriver *poAgvSickScanDriver;
    // Rvd 视觉定位与对接
    AgvRvdCtrl *poAgvRvdCtrl;

public:
    AgvCtrl(/* args */);
    ~AgvCtrl();

public:
    E_AGV_CTRL_STATE ctrlState;
    int Init();
    int Start();
    int AsyncMsgPost(const TRobotMsg *ptAgvMsg);
    int AsyncMsgHandler(const TRobotMsg *ptAgvMsg);
    int LoopAgvCtrlSvr();
    static void *LoopAgvCtrlSvrStatic(void *arg);


public:
    boost::recursive_mutex *lockBaseInfo_;
    TBaseInfo tBaseInfo_;
    int SetBaseInfo(TBaseInfo &tBaseInfo);
    int SetBaseInfoAgvPos(double X, double Y, double A);
    int SetBaseInfoAgvVel(double Lv, double Av);
    int GetBaseInfoAgvVel(double &Lv, double &Av);
    int GetBaseInfo(TBaseInfo &tBaseInfo);
    int UpdateBaseInfo();
    boost::recursive_mutex *lockMapInfo_;
    TMapInfo tMapInfo_;
    bool MapValid_ = false;
    int SetMapInfo(const TOccupancyGrid &tOccupancyGrid);
    int SetMapInfo(const TMapInfo &tMapInfo);
    int GetMapInfo(TOccupancyGrid &tOccupancyGrid);
    int GetMapInfo(TMapInfo &tMapInfo);

//Rotine事件处理
public:
    int AgvCtrlRoutine();
    int AgvCtrlProcess(tcp_period_event *pt_event, st_tcp_connect_info *pt_connect_info);
    int EventSubBaseInfoProcess(tcp_period_event *pt_event, st_tcp_connect_info *pt_connect_info);
    int EventSubMapInfoProcess(tcp_period_event *pt_event, st_tcp_connect_info *pt_connect_info);
    int EventPubVelCmdProcess(tcp_period_event *pt_event, st_tcp_connect_info *pt_connect_info);

//运动任务请求指令的处理
public:
    int CallChangeModeSvr(bool arg);
    int CallMoveToTarget(st_tcp_connect_info *ptConnectInfo, TAgvPose2D tTargetPose);
    int CallCancalTarget();

//底盘控制相关
public:
    int mani_watch_dog;
    float mani_linear_vel;
    float mani_angular_vel;
    float mani_linear_vel_def;
    float mani_angular_vel_def;
    bool RequestChassisMode(uint8_t mode_req);
    int PubManiCmdVel(float linear_vel, float angular_vel);
    void ManiWatchDogReset();
    void SetChassisVel(float linear_vel, float angular_vel);

//二次定位与对接相关
public:
    int RvdImageRequest(st_tcp_connect_info *ptConnectInfo);


//检查位置跳变与调试信息记录
    TAgvPose2D tCheckPose2DPre_;
    Rigid3d    r3M2OPre_;
    Rigid3d    r3O2BPre_;
    int CheckPose2DViolentChange();

    //当前栅格地图
    TOccupancyGrid tMapGrid_;
    TOccupancyGrid tMapLocal_;

    //ROS调试接口
    AgvRosSvr *poAgvRosSvr_;


};




#endif // AGVCTRL_H
/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 08:32:38
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 16:58:26
 * @FilePath: /agv_controller/include/agv_chassis_ctrl/agv_chassis_ctrl.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CHASSIS_MESSAGE_H
#define CHASSIS_MESSAGE_H

#include <map>
#include <functional>
#include <string>
#include <stdint.h>
#include <pthread.h>
#include <mqueue.h>
#include <sys/time.h>

#include "agv_type.h"
#include "AgvCtrl.h"
#include "AgvTf.h"
#include "TrackingPlanner.h"
#include "TrackingController.h"
#include "RemoteController.h"
//MotorController类声明 在AgvChassisCtrl中可见
class MotorController;


enum class ChassisCtrlState
{
    INIT,
    READY,
    STOP,
    RUN,
    SUSPEND,
    ERROR,
    RESET
};

//AgvChassisCtrl类定义
class AgvChassisCtrl
{
public:
    AgvCtrl *pt_agv_ctrl_;
    bool bHasInited;
    bool bHasStarted;

    AgvTf *poAgvTf;

    MotorController *po_motor_ctrl_;
    int s32ChassisCtrlFreq;
    int s32MotorStateCtrlFreq;

    TrackingPlanner *poTrackingPlanner;       //规划器
    TrackingController *poTrackingController; //控制器

    RemoteController *poRemoteController;     //远程遥控平滑控制器
    double remoteV;
    double remoteW;

public:
    AgvChassisCtrl(AgvCtrl *pt_agv_ctrl);
    ~AgvChassisCtrl();
    int Init();
    int Start();

    //线程与消息队列服务
    mqd_t qChassisSvr;
    pthread_t thChassisSvr;
    pthread_t thMotorStateSvr;
    int AsyncMsgPost(TRobotMsg *ptAgvMsg);
    int AsyncMsgHandler(TRobotMsg *ptAgvMsg);
    int LoopChassisSvr();
    int LoopMotorStateSvr();
    static void *LoopChassisSvrStatic(void *arg);
    static void *LoopMotorStateSvrStatic(void *arg);

private:
    std::chrono::system_clock::time_point current_time_;
    std::chrono::system_clock::time_point last_time_;

public:
    //Tracking Control State Machine
    bool bRemoteCtrl; //遥控操作的允许标志
    int  s32SafeLevel_;
    using ChassisCtrlStateFunc = std::function<int (TRobotMsg *ptAgvMsg)>;
    ChassisCtrlStateFunc fChassisCtrlStateFunc;
    ChassisCtrlState eChassisCtrlState;
    int ChassisCtrlStateInit(TRobotMsg *ptAgvMsg);
    int ChassisCtrlStateReady(TRobotMsg *ptAgvMsg);
    int ChassisCtrlStateStop(TRobotMsg *ptAgvMsg); 
    int ChassisCtrlStateRun(TRobotMsg *ptAgvMsg);
    int ChassisCtrlStateSuspend(TRobotMsg *ptAgvMsg);
    int ChassisCtrlStateError(TRobotMsg *ptAgvMsg);
    int ChassisCtrlStateReset(TRobotMsg *ptAgvMsg);

    int s32StateDelay_ = 0;
    int s32StateCount_ = 0;

    bool IsIdle();
    int ChassisCtrlPeriod();
    int ChassisOdomPeriod();
    int ChassisReadyReport();
    int ChassisErrorReport();

//跟踪任务现场数据 TODO后续组织任务 重写该部分 需要相关的数据结构支持
public:
    int s32MissionFrom_;
    TAgvPose2D tMissionOrgin_;
    TAgvPose2D tMissionTarget_;
//信息显示与调试

    //Debug
    FILE *pLogOdom;
    bool bLogOdom;
    int StartLogOdom();
    int StopLogOdom();

};


#endif 

/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 10:38:05
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 17:49:14
 * @FilePath: /AGVController-Server-V1.0/include/agv_rvd_ctrl/agv_rvd_ctrl.h
 * @Description: 
 */
#ifndef AGV_RVD_CTRL_H
#define AGV_RVD_CTRL_H

#include <stdint.h>
#include <pthread.h>
#include <mqueue.h>

#include "AgvTf.h"
#include "agv_type.h"
#include "agv_cmd.h"

#include "MvCameraControl.h"

#include "apriltag.h"
#include "apriltag_pose.h" //pose estimation lib
#include "tag36h11.h"

#include "hitbot_interface.h"
#include "ControlBeanEx.h"

enum class AgvRvdState
{
    INIT,
    READY,
    RUN,
    RESET,
    ERROR
};

class AgvRvdCtrl
{
private:
    mqd_t qRvdCtrl;
    pthread_t thRvdCtrl;

    /* data */
public:
    AgvCtrl *pt_agv_ctrl;
    AgvTf *poAgvTf_;
    bool bHasInited;
    bool bHasStarted;
    int s32CtrlFreq;

public:
    AgvRvdCtrl(AgvCtrl *pt_agv_ctrl, AgvTf *poAgvTf);
    ~AgvRvdCtrl();

    //主流程相关方法
    AgvRvdState State;
    static void *LoopRvdCtrlStatic(void *arg);
    int LoopRvdCtrl();
    int Init();
    int Start();
    int Stop();

    //消息投递与处理相关方法
    int AsyncMsgPost(TRobotMsg *ptAgvMsg);
    int AsyncMsgHanler(TRobotMsg *ptAgvMsg);
    int AsyncMsgHandlerInReady(TRobotMsg *ptAgvMsg);

    //相机取图相关
    MV_CC_DEVICE_INFO_LIST stDeviceList; //设备列表
    void * pCameraHandle; //设备句柄
    MVCC_INTVALUE stImageParam;
    char pImageName_[256];
    char pImageDir_[256];
    int CameraInit();
    int CameraDeInit();
    int CameraInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
    int CameraGetImage();

    //视觉二次定位
    apriltag_family_t *ptTagFamily;
    apriltag_detector_t *ptTagDetector;
    apriltag_detection_info_t tTagInfo;
    int TargetDetectInit();
    int TargetDetectDeInit();
    int TargetFrameCalc();

    //执行机构控制相关
    ControlBeanEx *ptHitbot;
    int ActuatorInit();
    int ActuatorDeInit();
    int ActuatorExecute();
    int ActuatorHome();

    //静态变换 坐标树待完善------------------------------------
    //机器人到相机的变换 9点标定
    double R33RobotToCamera[9];
    double T31RobotToCamera[3];
    //机器人到工具坐标系的变换 机械尺寸给定
    double R33RobotToTool[9];
    double T31RobotToTool[3];
    //靶标到目标的变换 机械给定
    double R33TagToGoalA[9];
    double T31TagToGoalA[3];
    double R33TagToGoalB[9];
    double T31TagToGoalB[3];
    //待计算的 目标在机器人坐标系下的描述
    //double R33RobotToGoalA[9];
    bool   validGoalA;
    double T31RobotToGoalA[3];
    double thetaRobotGoalA;
    //double R33RobotToGoalB[9];
    bool   validGoalB;
    double T31RobotToGoalB[3];
    double thetaRobotGoalB;

    float f32TransCameraToTag_[20];

    //计算目标在机器人坐标系下的位姿
    int StaticTransInit();
    int CalcTransActuatorToGoal(const double *R33CameraToTag, const double *T31CameraToTag);
    //--------------------------------------------------------

};


#endif // AGV_RVD_CTRL_H
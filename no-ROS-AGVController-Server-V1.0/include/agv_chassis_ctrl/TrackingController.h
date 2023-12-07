/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-05-04 15:12:43
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-26 14:21:11
 * @FilePath: /scout_ws/src/traking_ctrl/include/traking_ctrl/TrackingController.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef TRACKINGCONTROLLER_H
#define TRACKINGCONTROLLER_H

#include <map>
#include <string>
#include <functional>

#include "global.h"
#include "ApproxDiffTrans.h"

#define AGV_TRAC_NODE_MAX 65536
#define AGV_TRAC_ERRX_MAX 0.5
#define AGV_TRAC_ERRY_MAX 0.5
#define AGV_TRAC_ERRPHI_MAX 3.14/4

#define TRACK_ERR_PARA_INVALID 1
#define TRACK_ERR_TRACK_LOST 2


enum class TrackingState
{
    CTRL_IDLE = 0,
    ORG_ROTATE,
    TRACKING,
    END_ROTATE,
    CTRL_ERROR
};


class TrackingController
{
private:
    /* data */

public:
    double deltaT;                  //采样周期

    double L, Tau;                  //控制器参数
    double Kp_s, Ki_s, Kd_s;        //s方向PID参数
    double Kp_n, Ki_n, Kd_n;        //n方向PID参数
    double Kp_phi, Ki_phi, Kd_phi;  //phi方向PID参数

    double Err_x, Err_y;
    double Err_s, Err_n, Err_phi;   //误差项

    double IntErr_s;           //s方向误差积分项
    double IntErr_n;           //n方向误差积分项
    double IntErr_phi;         //phi误差积分项

    double PreErr_s, DifErr_s;    //s方向误差微分项
    double PreErr_n, DifErr_n;    
    double PreErr_phi, DifErr_phi;
    ApproxDiffTrans nDiffTrans;
    ApproxDiffTrans sDiffTrans;
    ApproxDiffTrans phiDiffTrans;

    double Vmax, Wmax, Vmin, Wmin; //速度限制
    double AccMax, WccMax, Vcur_, Wcur_;
    double Wrot;  //原地旋转期望角速度

    int WrotDir;  //原地旋转方向
    int AllowBackMotion;

public:
    TrackingController(/* args */);
    ~TrackingController();

//运行时暂存数据
public:
    TAgvMotionNode ptRefMotionNode[AGV_TRAC_NODE_MAX];
    int s32RefTotal;
    int s32CurT;

public:
    int Init(double f64CtrlPeriod);
    int Reset(double f64CtrlPeriod);
    int Start(const TAgvMotionNode *ptPlanNode, 
        const int s32PlanNodeTotal, const TAgvMotionNode *ptCurNode); //轨迹跟踪
    int Stop();
    int Error(int s32Err);

    //Tracking Control State Machine
    using TrackingStateFunc = std::function<int (const TAgvMotionNode *ptCurNode, 
        TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode)>;
    TrackingStateFunc fTrakingStateFunc;
    TrackingState eTrackingState;
    int TrackingStateCtrlIdle(const TAgvMotionNode *ptCurNode, TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode);
    int TrackingStateOrgRotate(const TAgvMotionNode *ptCurNode, TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode);
    int TrackingStateTracking(const TAgvMotionNode *ptCurNode, TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode);
    int TrackingStateEndRotate(const TAgvMotionNode *ptCurNode, TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode);
    int TrackingStateCtrlError(const TAgvMotionNode *ptCurNode, TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode);
    //Util Fuction
    double VelLimitedProcess(double Vin, double Vcur);
    double WrotLimitedProcess(double Win, double Wcur);
    int ResetSource();

//显示和调试
public:
    int s32Err_;
    std::map<TrackingState, std::string> mapTrackingState;
};




#endif // TRACKINGCONTROLLER_H
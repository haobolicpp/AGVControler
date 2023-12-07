/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-05-04 15:12:17
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-28 15:37:57
 * @FilePath: /scout_ws/src/traking_ctrl/src/TrackingController.cpp
 * @Description: 双轮差速移动机器人模型的轨迹跟踪控制方法
 */
#include <string>
#include <map>
#include <functional>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <math.h>

#include <iostream>

#include "agv_type.h"
#include "TrackingController.h"
#include "ApproxDiffTrans.h"

/**
 * @name: 轨迹跟踪控制器构造
 * @des: 
 * @param {* args} *
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
TrackingController::TrackingController(/* args */)
{
    deltaT = 0.02; //默认0.05
    Vmax = 1.2;
    Wmax = 3.14159 / 4;
    Vmin = 0.01;
    Wmin = 0.02;
    AccMax = 8;
    WccMax = 3.14159 * 2;

    Wrot = 3.14159 / 4;
    AllowBackMotion = 1;

    L = 1;
    Tau = 1;
    Kp_s = 1/Tau;
    Kp_n = 2/(L*L);
    Kp_phi = 1/L;

    Ki_s = 0.05;
    Ki_n = 0.1;
    Ki_phi = 0;

    Kd_s = 0.5;
    Kd_n = 1;
    Kd_phi = 0.1;

    Err_s = 0;
    Err_n = 0;
    Err_phi = 0;
    Err_x = 0;
    Err_y = 0;

    WrotDir = 0;
    eTrackingState = TrackingState::CTRL_IDLE;
    fTrakingStateFunc = std::bind(&TrackingController::TrackingStateCtrlIdle, 
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
}

/**
 * @name: 轨迹跟踪控制器析构
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
TrackingController::~TrackingController()
{
}


/**
 * @name: Init
 * @des:  初始化控制器资源
 * @param {double} f64CtrlPeriod
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::Init(double f64CtrlPeriod)
{
    //控制周期设定不低于1HZ
    if (f64CtrlPeriod < 0 || f64CtrlPeriod > 1)
    {
        return -1;
    }
    deltaT = f64CtrlPeriod;

    memset(ptRefMotionNode, 0, sizeof(TAgvMotionNode) * AGV_TRAC_NODE_MAX);
    s32RefTotal = 0;
    s32CurT = 0;

    // mapTrackingState.insert(std::pair<TrackingState,std::string>
    //     (TrackingState::CTRL_IDLE, "CTRL_IDLE"));
    // mapTrackingState.insert(std::pair<TrackingState,std::string>
    //     (TrackingState::ORG_ROTATE, "ORG_ROTATE"));
    // mapTrackingState.insert(std::pair<TrackingState,std::string>
    //     (TrackingState::TRACKING, "TRACKING"));
    // mapTrackingState.insert(std::pair<TrackingState,std::string>
    //     (TrackingState::END_ROTATE, "END_ROTATE"));
    // mapTrackingState.insert(std::pair<TrackingState,std::string>
    //     (TrackingState::CTRL_ERROR, "CTRL_ERROR"));

    return 0;
}

/**
 * @name: Reset
 * @des:  重置控制器资源
 * @param {double} f64CtrlPeriod
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::Reset(double f64CtrlPeriod)
{
    //控制周期设定不低于1HZ
    if (f64CtrlPeriod < 0 || f64CtrlPeriod > 1)
    {
        return -1;
    }
    deltaT = f64CtrlPeriod;
    memset(ptRefMotionNode, 0, sizeof(TAgvMotionNode) * AGV_TRAC_NODE_MAX);
    s32RefTotal = 0;
    s32CurT = 0;

    //重置控制器状态为IDLE
    s32Err_ = 0;
    eTrackingState = TrackingState::CTRL_IDLE;
    fTrakingStateFunc = std::bind(&TrackingController::TrackingStateCtrlIdle, 
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    return 0;
}

/**
 * @name: Error
 * @des:  控制器设置为Error状态
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::Error(int s32Err)
{
    s32Err_ = s32Err;

    // std::string strStatePre = mapTrackingState[eTrackingState];
    eTrackingState = TrackingState::CTRL_ERROR;
    // std::string strStateCur = mapTrackingState[eTrackingState];
    fTrakingStateFunc = std::bind(&TrackingController::TrackingStateCtrlError, 
        this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    
    // std::cout << "Error: Tracking Control State" << strStatePre <<"->" << strStateCur;
    // printf("Error: Tracking Control State [%s]->[%s] With Err[%d]\n", 
    //     strStatePre.c_str(), strStateCur.c_str(), s32Err_);
    printf("Error: Tracking Control State ErrX[%f]-ErrY[%f]-ErrPhi[%f]\n", 
    Err_x, Err_y, Err_phi);
    
    return 0;
}

/**
 * @name: 
 * @des: 
 * @param {TAgvMotionNode} *ptPlanNode
 * @param {int} s32PlanNodeTotal
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::Start(const TAgvMotionNode *ptPlanNode, 
    const int s32PlanNodeTotal, const TAgvMotionNode *ptCurNode)
{
    //参数合法性判断
    if (s32PlanNodeTotal > AGV_TRAC_NODE_MAX 
        || eTrackingState != TrackingState::CTRL_IDLE
        || ptPlanNode == NULL || ptCurNode == NULL)
    {
        Error(TRACK_ERR_PARA_INVALID);
        return -1;
    }

    memcpy(ptRefMotionNode, ptPlanNode, sizeof(TAgvMotionNode) * s32PlanNodeTotal);
    s32RefTotal = s32PlanNodeTotal;
    s32CurT = 0;

    ResetSource();

    //初始位置合法性判断
    Err_x =  -ptRefMotionNode[0].tPose2D.x + ptCurNode->tPose2D.x;
    Err_y =  -ptRefMotionNode[0].tPose2D.y + ptCurNode->tPose2D.y;
    if (fabs(Err_x) > AGV_TRAC_ERRX_MAX || fabs(Err_y) > AGV_TRAC_ERRY_MAX)
    {
        Error(TRACK_ERR_TRACK_LOST);
        return -1;
    }

    Err_phi = -ptRefMotionNode[0].tPose2D.phi + ptCurNode->tPose2D.phi;
    if (Err_phi > 3.14159)
    {
        Err_phi = Err_phi - 2 * 3.14159;
    }
    else if (Err_phi <= (- 3.14159))
    {
        Err_phi = Err_phi + 2 * 3.14159;
    }
    WrotDir = Err_phi > 0.0 ? -1 : 1;

    eTrackingState = TrackingState::ORG_ROTATE;
    fTrakingStateFunc = std::bind(&TrackingController::TrackingStateOrgRotate, 
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    return 0;
}

/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::Stop()
{
    return 0;
}


/**
 * @name: TrackingStateCtrlIdle
 * @des: 
 * @param {TAgvMotionNode} *ptCurNode
 * @param {TAgvTwist2D} *ptTwist2D
 * @param {TAgvMotionNode} *ptRefNode
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::TrackingStateCtrlIdle(const TAgvMotionNode *ptCurNode, 
        TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode)
{
    return 0;
}

/**
 * @name: TrackingStateOrgRotate
 * @des: 
 * @param {TAgvMotionNode} *ptCurNode
 * @param {TAgvTwist2D} *ptTwist2D
 * @param {TAgvMotionNode} *ptRefNode
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::TrackingStateOrgRotate(const TAgvMotionNode *ptCurNode, 
        TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode)
{
    int s32Ret;
    double Vout = 0;
    double Wout = 0;
    
//计算Frenet坐标系需求误差---------------------------------------
    Err_x =  -ptRefMotionNode[s32CurT].tPose2D.x + ptCurNode->tPose2D.x;
    Err_y =  -ptRefMotionNode[s32CurT].tPose2D.y + ptCurNode->tPose2D.y;
    if (fabs(Err_x) > AGV_TRAC_ERRX_MAX || fabs(Err_y) > AGV_TRAC_ERRY_MAX)
    {
        Error(TRACK_ERR_TRACK_LOST);
        ptTwist2D->v = 0.0;
        ptTwist2D->w = 0.0;
        return -1;
    }

    //记录参考点
    memcpy(ptRefNode, &ptRefMotionNode[s32CurT], sizeof(TAgvMotionNode));

//TODO 角度误差计算要考虑临界！！！！ 角度限制在-pi-pi
    Err_phi = -ptRefMotionNode[s32CurT].tPose2D.phi + ptCurNode->tPose2D.phi;
    if (Err_phi > 3.14159)
    {
        Err_phi = Err_phi - 2 * 3.14159;
    }
    else if (Err_phi <= (- 3.14159))
    {
        Err_phi = Err_phi + 2 * 3.14159;
    }

//退出条件----------------------------------------------------------
    if (fabs(Err_phi) < 0.03 && fabs(ptCurNode->tTwist2D.w) < 0.02)
    {
        ResetSource();
        eTrackingState = TrackingState::TRACKING;
        fTrakingStateFunc = std::bind(&TrackingController::TrackingStateTracking, 
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        ptTwist2D->v = 0.0;
        ptTwist2D->w = 0.0;
        return 0;
    }
//-----------------------------------------------------------------
    // Wout = std::min(fabs(Err_phi), Wrot);
    // Wout = Wout * WrotDir;
    // Vout = 0;
    // ptTwist2D->v = Vout;
    // ptTwist2D->w = Wout;

    double Wctrl = 0;
    double RcKp = 1;
    double RcKi = 0.5;
    if (fabs(Err_phi) < 0.2)
    {
        IntErr_phi = IntErr_phi + Err_phi * deltaT;
    }
    //PI
    Wctrl = -(RcKp * Err_phi + RcKi * IntErr_phi);
    Wout = WrotLimitedProcess(Wctrl, ptCurNode->tTwist2D.w);
    Vout = 0;
    ptTwist2D->v = Vout;
    ptTwist2D->w = Wout;
    // printf("ErrorPhi:%f RefPhi:%f CurPhi:%f  CtrlW:%f  IntPhi:%f\n", 
    //     Err_phi,  ptRefMotionNode[s32CurT].tPose2D.phi, ptCurNode->tPose2D.phi, Wout, IntErr_phi);

    return 0;
}
/**
 * @name: TrackingStateTracking
 * @des:  PID控制跟踪阶段
 * @param {TAgvMotionNode} *ptCurNode
 * @param {TAgvTwist2D} *ptTwist2D
 * @param {TAgvMotionNode} *ptRefNode
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::TrackingStateTracking(const TAgvMotionNode *ptCurNode, 
        TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode)
{
    int s32Ret;
    double Vout = 0;
    double Wout = 0;
    double Wk = 0; //控制率计算的曲率给定量

//A已执行到到规划的终点 终点保存有目标角度信息 不再作用于跟踪参考 
    if (s32CurT >= (s32RefTotal - 1)) 
    {
        s32CurT = s32RefTotal - 1;
        memcpy(ptRefNode, &ptRefMotionNode[s32CurT], sizeof(TAgvMotionNode)); //输出当前参考点
        ptTwist2D->v = 0.0;
        ptTwist2D->w = 0.0;
        //等待机器人停止 惯性停止
        if (fabs(ptCurNode->tTwist2D.v) < 1e-3 && fabs(ptCurNode->tTwist2D.w) < 1e-3)
        {
            Err_phi = -ptRefMotionNode[s32CurT].tPose2D.phi + ptCurNode->tPose2D.phi;
            if (Err_phi > 3.14159)
            {
                Err_phi = Err_phi - 2 * 3.14159;
            }
            else if (Err_phi <= (- 3.14159))
            {
                Err_phi = Err_phi + 2 * 3.14159;
            }
            ResetSource();
            WrotDir = Err_phi > 0.0 ? -1 : 1; //确定旋转方向
            eTrackingState = TrackingState::END_ROTATE;
            fTrakingStateFunc = std::bind(&TrackingController::TrackingStateEndRotate, 
                this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
            return 2;
        }
        return 0;
    }
//B未执行到到规划的终点 继续执行PID轨迹跟踪控制
    memcpy(ptRefNode, &ptRefMotionNode[s32CurT], sizeof(TAgvMotionNode));
//B1计算Frenet坐标系需求误差---------------------------------------
    Err_x =  -ptRefMotionNode[s32CurT].tPose2D.x + ptCurNode->tPose2D.x;
    Err_y =  -ptRefMotionNode[s32CurT].tPose2D.y + ptCurNode->tPose2D.y;
//B2 TODO 角度误差计算要考虑临界！！！！ 角度限制在-pi-pi
    Err_phi = -ptRefMotionNode[s32CurT].tPose2D.phi + ptCurNode->tPose2D.phi;
    if (Err_phi > 3.14159)
    {
        Err_phi = Err_phi - 2 * 3.14159;
    }
    else if (Err_phi <= (- 3.14159))
    {
        Err_phi = Err_phi + 2 * 3.14159;
    }

//B3 跟踪失败 误差超出阈值-------------------------------------------
    if (fabs(Err_x) > AGV_TRAC_ERRX_MAX || 
        fabs(Err_y) > AGV_TRAC_ERRY_MAX ||
        fabs(Err_phi) > AGV_TRAC_ERRPHI_MAX )
    {
        Error(TRACK_ERR_TRACK_LOST);
        ptTwist2D->v = 0.0;
        ptTwist2D->w = 0.0;
        return -1;
    }

//B4 换算至Frenet--------------------------------------------------
    Err_s = cos(ptRefMotionNode[s32CurT].tPose2D.phi) * Err_x + 
        sin(ptRefMotionNode[s32CurT].tPose2D.phi) * Err_y;
    Err_n = cos(ptRefMotionNode[s32CurT].tPose2D.phi) * Err_y -
        sin(ptRefMotionNode[s32CurT].tPose2D.phi) * Err_x;
    
//B5 误差死区控制--------------------------------------------------
    if(fabs(Err_s) < 0.01)
    {
        Err_s = 0;
    }
    if(fabs(Err_n) < 0.01)
    {
        Err_n = 0;
    }
    if(fabs(Err_phi) < 0.03) //1
    {
        Err_phi = 0;
    }

//B6 PID控制实现--------------------------------------------------
    //1-积分项更新
    IntErr_s = IntErr_s + Err_s * deltaT;
    IntErr_n = IntErr_n + Err_n * deltaT;
    IntErr_phi = IntErr_phi + Err_n * deltaT;
    
    //2-微分项更新
    DifErr_s = sDiffTrans.Calc(Err_s);
    DifErr_n = nDiffTrans.Calc(Err_n);

    //3-计算控线速度控制量 前馈反馈控制
    Vout = ptRefMotionNode[s32CurT].tTwist2D.v 
        - (Kp_s * Err_s + Ki_s * IntErr_s + Kd_s * DifErr_s);
    //4-线速度控制量限幅与死区判定
    Vout = VelLimitedProcess(Vout, ptCurNode->tTwist2D.v);
    // printf("P[%d]-Vout [%f]\n", s32CurT, Vout);

    //5-计算角速度控制量 前馈反馈控制
    Wk = Kp_n * Err_n + Ki_n * IntErr_n + Kd_n * DifErr_n
        + Kp_phi * Err_phi; //Todo Phi PID
    Wout = ptRefMotionNode[s32CurT].tTwist2D.w - Vout * Wk;
    //6-角速度限幅与死区处理
    if (fabs(Wout) > Wmax)
    {
        Wout = Wout > 0 ? Wmax : -Wmax;
    }
    else if(fabs(Wout) < Wmin)
    {
        Wout = 0.0;
    }
    ptTwist2D->v = Vout;
    ptTwist2D->w = Wout;
    s32CurT = s32CurT + 1; //更新控制参考点索引

    return 0;
}
/**
 * @name: TrackingStateEndRotate
 * @des:  PID控制终点旋转阶段
 * @param {TAgvMotionNode} *ptCurNode
 * @param {TAgvTwist2D} *ptTwist2D
 * @param {TAgvMotionNode} *ptRefNode
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::TrackingStateEndRotate(const TAgvMotionNode *ptCurNode, 
        TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode)
{
    int s32Ret;
    double Vout = 0;
    double Wout = 0;
    
//计算Frenet坐标系需求误差---------------------------------------
    Err_x =  -ptRefMotionNode[s32CurT].tPose2D.x + ptCurNode->tPose2D.x;
    Err_y =  -ptRefMotionNode[s32CurT].tPose2D.y + ptCurNode->tPose2D.y;
    if (fabs(Err_x) > AGV_TRAC_ERRX_MAX || fabs(Err_y) > AGV_TRAC_ERRY_MAX)
    {
        Error(TRACK_ERR_TRACK_LOST);
        ptTwist2D->v = 0.0;
        ptTwist2D->w = 0.0;
        return -1;
    }

    //记录参考点
    memcpy(ptRefNode, &ptRefMotionNode[s32CurT], sizeof(TAgvMotionNode));

//TODO 角度误差计算要考虑临界！！！！ 角度限制在-pi-pi
    Err_phi = -ptRefMotionNode[s32CurT].tPose2D.phi + ptCurNode->tPose2D.phi;
    if (Err_phi > 3.14159)
    {
        Err_phi = Err_phi - 2 * 3.14159;
    }
    else if (Err_phi <= (- 3.14159))
    {
        Err_phi = Err_phi + 2 * 3.14159;
    }

//退出条件----------------------------------------------------------
    if (fabs(Err_phi) < 0.03 && fabs(ptCurNode->tTwist2D.w) < 0.02)
    {
        ResetSource();
        eTrackingState = TrackingState::CTRL_IDLE;
        fTrakingStateFunc = std::bind(&TrackingController::TrackingStateCtrlIdle, 
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        ptTwist2D->v = 0.0;
        ptTwist2D->w = 0.0;
        return 1;
    }
//-----------------------------------------------------------------
    // Wout = std::min(fabs(Err_phi), Wrot);
    // Wout = Wout * WrotDir;
    // Vout = 0.0;
    // ptTwist2D->v = Vout;
    // ptTwist2D->w = Wout;

    double Wctrl = 0;
    double RcKp = 1;
    double RcKi = 0.5;
    if (fabs(Err_phi) < 0.2)
    {
        IntErr_phi = IntErr_phi + Err_phi * deltaT;
    }
    //PI
    Wctrl = -(RcKp * Err_phi + RcKi * IntErr_phi);
    Wout = WrotLimitedProcess(Wctrl, ptCurNode->tTwist2D.w);
    Vout = 0;
    ptTwist2D->v = Vout;
    ptTwist2D->w = Wout;

    // printf("ErrorPhi:%f RefPhi:%f CurPhi:%f  CtrlW:%f  IntPhi:%f\n", 
    //     Err_phi,  ptRefMotionNode[s32CurT].tPose2D.phi, ptCurNode->tPose2D.phi, Wout, IntErr_phi);

    return 0;
}

/**
 * @name: TrackingStateCtrlError
 * @des:  PID控制异常状态
 * @param {TAgvMotionNode} *ptCurNode
 * @param {TAgvTwist2D} *ptTwist2D
 * @param {TAgvMotionNode} *ptRefNode
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingController::TrackingStateCtrlError(const TAgvMotionNode *ptCurNode, 
        TAgvTwist2D *ptTwist2D, TAgvMotionNode *ptRefNode)
{
    ptTwist2D->v = 0.0;
    ptTwist2D->w = 0.0;
    return 0;
}



/**
 * @name: 
 * @des: 
 * @param {double} Vin
 * @param {double} Vcur
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
double TrackingController::VelLimitedProcess(double Vin, double Vcur)
{
    double Vout = Vin;

    //最大加速度限制
    double VlimL = Vcur - AccMax * deltaT;
    double VlimH = Vcur + AccMax * deltaT;

    //最大速度限制
    VlimH = VlimH > Vmax ? Vmax : VlimH;

    if (AllowBackMotion == 0) //不允许倒车运动
    {
        //最小速度限制
        VlimL = VlimL < Vmin ? Vmin : VlimL;
    }
    else  //允许倒车运动
    {
        VlimL = VlimL < -Vmax ? -Vmax : VlimL;
    }

    //速度限幅
    if (fabs(Vout) < Vmin)
    {
        Vout = 0.0;
    }
    else if (Vout > VlimH)
    {
        Vout = VlimH;
    }
    else if (Vout < VlimL)
    {
        Vout = VlimL;
    }
    else
    {
        Vout = Vin;
    }

    return Vout;
}


/**
 * @name: 
 * @des: 
 * @param {double} Win
 * @param {double} Wcur
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
double TrackingController::WrotLimitedProcess(double Win, double Wcur)
{   
    double Wout = 0;
    if (Win > Wmax)
    {
        Wout = Wmax;
    }

    else if (Win < -Wmax)
    {
        Wout = -Wmax;
    }

    else
    {
        Wout = Win;
    }

    return Wout;
}


int TrackingController::ResetSource()
{
    IntErr_s = 0;
    IntErr_n = 0;
    IntErr_phi = 0;
    PreErr_s = 0;
    DifErr_s = 0;
    PreErr_n = 0;
    DifErr_n = 0;
    PreErr_phi = 0;
    DifErr_phi = 0;
    WrotDir = 0;
    //近似微分环节初始化
    nDiffTrans.Init(0.9, 1, deltaT);
    sDiffTrans.Init(0.95, 1, deltaT);
    phiDiffTrans.Init(0.9, 1, deltaT);

    return 0;
}
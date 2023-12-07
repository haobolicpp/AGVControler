/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-07-28 16:10:39
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-31 08:49:28
 * @FilePath: /agv_controller/src/agv_chassis_ctrl/RemoteController.cpp
 * @Description: 远程遥控平滑运动控制器
 */
#include <math.h>
#include "RemoteController.h"
/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
RemoteController::RemoteController(/* args */)
{
    vOut_ = 0.0;
    wOut_ = 0.0;

    Tv_ = 0.0;
    Tw_ = 0.0;

    vMax_ = 1.0;   //最大线速度输出允许
    vMin_ = 0.01;  //速度死区限制
    wMax_ = PI/2;   //最大角速度输出允许
    wMin_ = 0.02;  //角速度死区限制

    CtrlFreq_ = 50; //默认采样频率50Hz
}
/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
RemoteController::~RemoteController()
{
}

/**
 * @name: Init
 * @des:  初始化远程遥控速度给定平滑控制器
 * @param {double} f64CtrlFreq 控制频率 平滑时间常数和控制频率相关 
 * TODO 连续到离散的变化
 * @param {double} Tv 线速度平滑时间常数
 * @param {double} Tw 角速度平滑时间常数
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int RemoteController::Init(double f64CtrlFreq, double Tv, double Tw)
{
    CtrlFreq_ = f64CtrlFreq;
    Tv_ = Tv;
    Tw_ = Tw;

    vOut_ = 0.0;
    wOut_ = 0.0;

    return 0;
}
/**
 * @name: Reset
 * @des:  控制器重置
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int RemoteController::Reset()
{
    vOut_ = 0.0;
    wOut_ = 0.0;

    return 0;
}

/**
 * @name: RemoteCtrlPeriod
 * @des:  远程遥控速度给定平滑控制器 周期输出
 * @param {TAgvTwist2D} *tTwistIn
 * @param {TAgvTwist2D} *tTwistOut
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int RemoteController::RemoteCtrlPeriod(TAgvTwist2D *tTwistIn, TAgvTwist2D *tTwistOut)
{
    //线速度平滑
    vOut_ = Tv_ * vOut_ + (1 - Tv_) * tTwistIn->v;
    if (fabs(vOut_) < vMin_)
    {
        tTwistOut->v = 0.0;
    }
    else if (vOut_ > vMax_)
    {
        vOut_ = vMax_;
    }
    else if (vOut_ < -vMax_)
    {
        vOut_ = -vMax_;
    }
    else
    {
        tTwistOut->v = vOut_;
    }
    //角速度平滑
    wOut_ = Tw_ * wOut_ + (1 - Tw_) * tTwistIn->w;
    if (fabs(wOut_) < wMin_)
    {
        tTwistOut->w = 0.0;
    }
    else if (wOut_ > wMax_)
    {
        wOut_ = wMax_;
    }
    else if (wOut_ < -wMax_)
    {
        wOut_ = -wMax_;
    }
    else
    {
        tTwistOut->w = wOut_;
    }

    // printf("smooth out [%f]-[%f]\n", vOut_, wOut_);

    return 0;
}
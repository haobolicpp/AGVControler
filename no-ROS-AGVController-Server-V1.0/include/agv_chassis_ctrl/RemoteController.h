/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-07-28 16:07:56
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-31 08:20:05
 * @FilePath: /agv_controller/include/agv_chassis_ctrl/RemoteController.h
 * @Description: 远程遥控的平滑运动控制
 */
#ifndef REMOTECONTROLLER_H
#define REMOTECONTROLLER_H


#include "agv_type.h"

//在远程给定的突变线速度和角速度情况下 做一阶平滑滤波处理后输出
//G(S) = 1/(TS + 1)


class RemoteController
{
private:
    /* data */
    double vOut_;
    double wOut_;

    double Tv_;
    double Tw_;

    double CtrlFreq_;
    double vMax_;
    double vMin_;
    double wMax_;
    double wMin_;

public:
    RemoteController(/* args */);
    ~RemoteController();

public:
    int Init(double f64CtrlFreq, double Tv, double Tw);
    int Reset();
    int RemoteCtrlPeriod(TAgvTwist2D *tTwistIn, TAgvTwist2D *tTwistOut);
};




#endif // REMOTECONTROLLER_H
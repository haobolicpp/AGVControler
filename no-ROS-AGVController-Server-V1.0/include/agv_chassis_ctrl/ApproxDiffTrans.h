/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-05-10 13:37:48
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-05-10 13:49:50
 * @FilePath: /scout_ws/src/traking_ctrl/include/traking_ctrl/ApproxDiffTrans.h
 * @Description: 近似微分环节 抑制噪声
 */
#ifndef APPROXDIFFTRANS_H
#define APPROXDIFFTRANS_H

class ApproxDiffTrans
{
private:
    /* data */

public:
    double T_ = 1;
    double K_ = 1;
    double delta_t_ = 0.1;

    double Yk_1 = 0;
    double Xk_1 = 0;

public:
    ApproxDiffTrans(/* args */);
    ~ApproxDiffTrans();

    int Init(double T, double K, double delta_t); //定采样周期实现
    double Calc(double Xk);

};




#endif // APPROXDIFFTRANS_H
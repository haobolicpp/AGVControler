/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-05-10 13:38:24
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-05-10 14:33:16
 * @FilePath: /scout_ws/src/traking_ctrl/src/ApproxDiffTrans.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "ApproxDiffTrans.h"

ApproxDiffTrans::ApproxDiffTrans(/* args */)
{
}

ApproxDiffTrans::~ApproxDiffTrans()
{
}

/**
 * @name: 
 * @des: 
 * @param {double} T
 * @param {double} K
 * @param {double} delta_t
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int ApproxDiffTrans::Init(double T, double K, double delta_t)
{
    T_ = T;
    K_ = K;
    delta_t_ = delta_t;

    Yk_1 = 0.0;
    Xk_1 = 0.0;

    return 0;
}

/**
 * @name: 
 * @des: 
 * @param {double} Xk 近似微分环节输入
 * @return {*} Yk 近似微分环节的输出
 * @author: yang.cheng
 * @ver: 1.01
 */
double ApproxDiffTrans::Calc(double Xk)
{
    double Yk;

    double T_div_Dt = T_ / delta_t_;
    double K_div_Dt = K_ / delta_t_;

    Yk = (T_div_Dt / (T_div_Dt + 1)) * Yk_1
        + (K_div_Dt / (T_div_Dt + 1)) * (Xk - Xk_1);
    
    //History Update
    Xk_1 = Xk;
    Yk_1 = Yk;

    return Yk;
}

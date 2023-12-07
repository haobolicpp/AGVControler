/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-11 16:09:34
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-11 17:20:36
 * @FilePath: /agv_controller/src/agv_slammer/amcl/AmclGuassian.cpp
 * @Description: Amcl 高斯采样C++
 */


#include <random>
#include "amcl/AmclGuassian.h"

static std::default_random_engine AmclRandGen;
/**
 * @name: AmclGuassianInit
 * @des:  Amcl高斯采样初始化
 * @param {int} seed
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
void AmclGuassianInit(unsigned int seed)
{
    AmclRandGen = std::default_random_engine(seed);
}

/**
 * @name: AmclGuassianSample
 * @des:  Amcl高斯采样
 * @param {double} sigma
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
double AmclGuassianSample(double sigma)
{
    std::normal_distribution<double> randNorm(0, sigma);
    return randNorm(AmclRandGen);
}
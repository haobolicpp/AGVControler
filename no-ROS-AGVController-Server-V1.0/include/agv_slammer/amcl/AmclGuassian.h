/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-11 16:10:46
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-11 17:07:17
 * @FilePath: /agv_controller/include/agv_slammer/amcl/AmclGuassian.h
 * @Description: 高斯分布C++调用
 */
#ifndef AMCLGUASSIAN_H
#define AMCLGUASSIAN_H

#ifdef __cplusplus
extern "C"{
#endif

void AmclGuassianInit(unsigned int seed);
double AmclGuassianSample(double sigma);

#ifdef __cplusplus
}
#endif


#endif // AMCLGUASSIAN_H
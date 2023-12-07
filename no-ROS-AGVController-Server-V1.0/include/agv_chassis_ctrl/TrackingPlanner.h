/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-05-04 15:13:02
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-26 10:14:10
 * @FilePath: /scout_ws/src/traking_ctrl/include/traking_ctrl/TrackingPlanner.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef TRACKINGPLANNER_H
#define TRACKINGPLANNER_H

#include "agv_type.h"

#define AGV_PLAN_NODE_MAX 65536

class TrackingPlanner
{
public:
    
    double Vmax, Vmin;
    double Wmax, Wmin;
    double AccMax, AccMin;
    double Lmax, Lmin;

public:
    TrackingPlanner(/* args */);
    ~TrackingPlanner();

public:
    TAgvMotionNode ptRefPlanNode[AGV_PLAN_NODE_MAX];
    int s32PlanTotal;

public:
    int Init();
    int Reset();
    
    int BestOrgRef;     //初始最优角度参考点索引
    int LinearPlan(const TAgvPose2D *ptPoseOrg, const TAgvPose2D *ptPoseEnd, 
                   double period, double vExpect, double accExpect,
                   int *planNodeTotal);
    
    //半圆弧规划
    int HalfCyclePlan(const TAgvPose2D *ptPoseOrg, const TAgvPose2D *ptPoseEnd,
                    double period, double vExpect, double accExpect,
                    int *planNodeTotal);

    //仿真校准参考的规划
    int SimLinearPlan(const TAgvPose2D *ptPoseOrg, const TAgvPose2D *ptPoseEnd, 
                double period, double vExpect, double accExpect,
                int *planNodeTotal);

    //圆弧规划
    int CyclePlanTest(const TAgvPose2D *ptPoseOrg, const TAgvPose2D *ptPoseEnd,
                double period, double vExpect, double accExpect,
                int *planNodeTotal);
    
};


#endif // TRACKINGPLANNER_H
/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-05-04 15:11:57
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-28 15:27:11
 * @FilePath: /scout_ws/src/traking_ctrl/src/TrackingPlanner.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#include "global.h"
#include "agv_type.h"
#include "TrackingPlanner.h"
/**
 * @name: 
 * @des: 
 * @param {* args} *
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
TrackingPlanner::TrackingPlanner(/* args */)
{
    Vmin = 0.1;
    Vmax = 4;
    AccMax = 10;
    AccMin = 0.1;
    Wmin = 3.14159 / 36;
    Wmax = 2 * 3.14159;
    Lmax = 50;
    Lmin = 0.05;
}
/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
TrackingPlanner::~TrackingPlanner()
{
}

/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingPlanner::Init()
{
    //初始化规划资源
    memset(ptRefPlanNode, 0, sizeof(TAgvMotionNode) * AGV_PLAN_NODE_MAX);
    s32PlanTotal = 0;
    return 0;
}
/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingPlanner::Reset()
{   
    //重置规划资源
    memset(ptRefPlanNode, 0, sizeof(TAgvMotionNode) * AGV_PLAN_NODE_MAX);
    s32PlanTotal = 0;
    return 0;
}
/**
 * @name: 
 * @des: 
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingPlanner::LinearPlan(const TAgvPose2D *ptPoseOrg, const TAgvPose2D *ptPoseEnd, 
                double period, double vExpect, double accExpect,
                int *planNodeTotal)
{
    int s32Ret;
    s32PlanTotal = 0;
//------------------------参数限制-----------------
//todo 距离过短限制
    if (period > 1.0 || period < 0.0)
    {
        return -1;
    }
    if (vExpect > Vmax || vExpect < Vmin)   //线速度限制
    {
        return -1;
    }
    if (accExpect > AccMax || accExpect < AccMin)
    {
        return -1;
    }

    double delta_t = period; //采样周期 
    double pi = 3.14159;
//---------------------初始与末端位姿赋值------------------
    double poseOrgX, poseOrgY, poseOrgPhi; //初始位姿赋值
    double poseEndX, poseEndY, poseEndPhi;//末端位姿赋值

    poseOrgX = ptPoseOrg->x;
    poseOrgY = ptPoseOrg->y;
    poseOrgPhi = ptPoseOrg->phi;

    poseEndX = ptPoseEnd->x;
    poseEndY = ptPoseEnd->y;
    poseEndPhi = ptPoseEnd->phi;

//------------------轨迹规划初始化规划-----------------------------------
    double dist, dist_xd, dist_yd, dist_xp, dist_yp, dist_itr;
    double v_triangle, v_itr, t_itr;
    double t_a, l_a, t_v, l_v, t_d; //临时变量
    double traj_theta;
    int itr, is_finish;

    //----计算轨迹总长与轨迹法向量
    dist_xd = poseEndX - poseOrgX;
    dist_yd = poseEndY - poseOrgY;
    dist = sqrt((dist_xd * dist_xd) + (dist_yd * dist_yd)); 
    dist_xp = dist_xd / dist;
    dist_yp = dist_yd / dist;
    traj_theta = atan2(dist_yd, dist_xd);
    //------轨迹长度非法
    if (dist > Lmax || dist < Lmin)
    {
        return -1;
    }

    //----初始化输出量
    memset(ptRefPlanNode, 0, sizeof(TAgvMotionNode) * AGV_PLAN_NODE_MAX);
    ptRefPlanNode[0].tPose2D.x = poseOrgX;
    ptRefPlanNode[0].tPose2D.y = poseOrgY;
    ptRefPlanNode[0].tPose2D.phi = traj_theta;
    ptRefPlanNode[0].tTwist2D.v = accExpect * delta_t;
    ptRefPlanNode[0].tTwist2D.w = 0; //直线运动的规划

    //----初始化迭代量
    v_itr = 0; 
    t_itr = 0;
    dist_itr = 0;
    is_finish = 0;
    //----判断是否轨迹过短导致的三角形加减速
    v_triangle = sqrt(accExpect * dist);
//------------------三角形加减速规划------------------------
    if (v_triangle < vExpect) 
    {
        t_a = v_triangle/accExpect; //加速阶段时间终点
        for (itr = 1; itr < AGV_PLAN_NODE_MAX; itr++) //从第一个点开始
        {
            t_itr = t_itr + delta_t; //更新时间
            if (t_itr <= t_a) //加速阶段
            {
                dist_itr = 0.5 * accExpect * t_itr * t_itr; //二分之一at方
                v_itr = accExpect * t_itr;
            }
            else if (t_itr > t_a && t_itr <= 2*t_a)//减速阶段
            {
                dist_itr = (0.5 * accExpect * t_a * t_a) 
                    + (v_triangle * (t_itr - t_a))
                    - (0.5 * accExpect * (t_itr - t_a) * (t_itr - t_a));
                v_itr = v_triangle - accExpect * (t_itr - t_a);
            }
            else //剩余时间段
            {
                dist_itr = dist;
                v_itr = 0;
                is_finish = 1;
            }

            //增量计算投影至XY方向
            ptRefPlanNode[itr].tPose2D.x = poseOrgX + dist_xp * dist_itr;
            ptRefPlanNode[itr].tPose2D.y = poseOrgY + dist_yp * dist_itr;
            ptRefPlanNode[itr].tPose2D.phi = traj_theta;
            ptRefPlanNode[itr].tTwist2D.v = v_itr;
            ptRefPlanNode[itr].tTwist2D.w = 0; //直线运动的规划
            if (is_finish == 1)
            {
                //s32PlanTotal = itr;
                break;
            }
        }//end_for
    }//end if //三角形情况
//---------------------梯形加减速情形------------------------
    else 
    {
        t_a = vExpect / accExpect;
        l_a = 0.5 * accExpect * t_a * t_a;
        t_v = (dist - vExpect * vExpect / accExpect) / vExpect;
        l_v = vExpect * t_v;
        t_d = vExpect / accExpect;
        for (itr = 1; itr < AGV_PLAN_NODE_MAX; itr++) //从第一个点开始
        {
            t_itr = t_itr + delta_t; //更新时间
            if (t_itr <= t_a) //加速阶段
            {
                dist_itr = 0.5 * accExpect * t_itr * t_itr; //二分之一at方
                v_itr = accExpect * t_itr;
            }
            else if (t_itr > t_a && t_itr <= (t_a + t_v) ) //匀速阶段
            {
                dist_itr = l_a + (t_itr - t_a) * vExpect;
                v_itr = vExpect;
            }
            else if (t_itr > (t_a + t_v) && t_itr <= (t_a + t_v + t_d)) //减速阶段
            {
                dist_itr = (l_a + l_v + (t_itr - t_a - t_v) * vExpect) -
                    (0.5 * accExpect * (t_itr - t_a - t_v) * (t_itr - t_a - t_v));
                v_itr = vExpect - accExpect * (t_itr - t_a - t_v);
            }
            else //剩余时间阶段
            {
                dist_itr = dist;
                v_itr = 0;
                is_finish = 1;
            }

            //增量计算投影至XY方向
            ptRefPlanNode[itr].tPose2D.x = poseOrgX + dist_xp * dist_itr;
            ptRefPlanNode[itr].tPose2D.y = poseOrgY + dist_yp * dist_itr;
            ptRefPlanNode[itr].tPose2D.phi = traj_theta;
            ptRefPlanNode[itr].tTwist2D.v = v_itr;
            ptRefPlanNode[itr].tTwist2D.w = 0; //直线运动的规划

            if (is_finish == 1)
            {
                //s32PlanTotal = itr;
                break;
            }
        }//end_for
    }//end else //梯形情况

    //规划未完成(最大点数超出预期)
    if (is_finish != 1)
    {
        *planNodeTotal = 0;
        return -1;
    }

    //TODO 为使到达目标的位置更加精确，可以多补充一些目标位姿点作为停止缓存参考
    //在允许倒车的情况下 通过PID控制 精确定位至目标点
    //补齐目标位姿
    itr = itr + 1;

    for (int i = 0; i < 50; i++)
    {
        ptRefPlanNode[itr].tPose2D.x = ptPoseEnd->x;
        ptRefPlanNode[itr].tPose2D.y = ptPoseEnd->y;
        ptRefPlanNode[itr].tPose2D.phi = traj_theta;
        ptRefPlanNode[itr].tTwist2D.v = 0;
        ptRefPlanNode[itr].tTwist2D.w = 0; 
        itr = itr + 1; //总数包含索引0
    }

    ptRefPlanNode[itr].tPose2D.x = ptPoseEnd->x;
    ptRefPlanNode[itr].tPose2D.y = ptPoseEnd->y;
    ptRefPlanNode[itr].tPose2D.phi = ptPoseEnd->phi;
    ptRefPlanNode[itr].tTwist2D.v = 0;
    ptRefPlanNode[itr].tTwist2D.w = 0; //直线运动的规划
    //规划完成
    itr = itr + 1; //总数包含索引0

    *planNodeTotal = itr;

    return 0;
}


/**
 * @name: HalfCyclePlan
 * @des:  半圆弧规划
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingPlanner::HalfCyclePlan(const TAgvPose2D *ptPoseOrg, const TAgvPose2D *ptPoseEnd,
                double period, double vExpect, double accExpect,
                int *planNodeTotal)
{
    int s32Ret;
    int s32PlanTotal = 0;
    double delta_t = period; //采样周期 
    double pi = 3.14159;

    if (period > 1.0 || period < 0.0)
    {
        return -1;
    }
    if (vExpect > Vmax || vExpect < Vmin)   //线速度限制
    {
        return -1;
    }
    if (accExpect > AccMax || accExpect < AccMin)
    {
        return -1;
    }

    double R = 0.5* sqrt((ptPoseOrg->x - ptPoseEnd->x) * (ptPoseOrg->x - ptPoseEnd->x)
                        + (ptPoseOrg->y - ptPoseEnd->y) * (ptPoseOrg->y - ptPoseEnd->y));
    double L = 2 * pi * R; //周长
    double Cx = 0.5 * (ptPoseEnd->x - ptPoseOrg->x); //圆心X
    double Cy = 0.5 * (ptPoseEnd->y - ptPoseOrg->y); //圆心Y
    double f64Div = L / vExpect; //份数

    //---初始期望位姿赋值---------------------------
    double PhiOrg;
    PhiOrg = atan2((ptPoseEnd->y - ptPoseOrg->y), (ptPoseEnd->x - ptPoseOrg->x)) - (0.5 * pi);
    memset(ptRefPlanNode, 0, sizeof(TAgvMotionNode) * AGV_PLAN_NODE_MAX);
    ptRefPlanNode[0].tPose2D.x = ptPoseOrg->x;
    ptRefPlanNode[0].tPose2D.y = ptPoseOrg->y;
    ptRefPlanNode[0].tPose2D.phi = PhiOrg;
    ptRefPlanNode[0].tTwist2D.v = vExpect;
    ptRefPlanNode[0].tTwist2D.w = vExpect / R; //r = v/w

    //----开始规划---------------------------------
    int DivTotal = floor(f64Div / delta_t);
    double phi_itr = PhiOrg - (0.5 * pi);
    double phi_plan;
    double w_itr = vExpect / R;
    int itr;

    for (itr = 1; itr < DivTotal; itr++)
    {
        phi_itr = phi_itr + w_itr * delta_t;
        phi_plan = phi_itr + (0.5 * pi);
        if (phi_plan >= pi)
        {
            phi_plan = phi_plan - 2 * pi;
        }
        else if (phi_plan < -pi)
        {
            phi_plan = phi_plan + 2 * pi;
        }

        ptRefPlanNode[itr].tPose2D.x = Cx + R * cos(phi_itr);
        ptRefPlanNode[itr].tPose2D.y = Cy + R * sin(phi_itr);
        ptRefPlanNode[itr].tPose2D.phi = phi_plan;
        ptRefPlanNode[itr].tTwist2D.v = vExpect;
        ptRefPlanNode[itr].tTwist2D.w = vExpect / R; //r = v/w
    }

    //终点位姿
    ptRefPlanNode[itr].tPose2D.x =  ptPoseEnd->x;
    ptRefPlanNode[itr].tPose2D.x =  ptPoseEnd->y;
    ptRefPlanNode[itr].tPose2D.phi = phi_plan;
    ptRefPlanNode[itr].tTwist2D.v = 0;
    ptRefPlanNode[itr].tTwist2D.w = 0; //r = v/w

    s32PlanTotal = itr;
    *planNodeTotal = itr;

    return 0;
}



int TrackingPlanner::SimLinearPlan(const TAgvPose2D *ptPoseOrg, const TAgvPose2D *ptPoseEnd, 
                double period, double vExpect, double accExpect,
                int *planNodeTotal)
{
    int s32Ret;
    s32PlanTotal = 0;
//------------------------参数限制-----------------
    if (period > 1.0 || period < 0.0)
    {
        return -1;
    }
    if (vExpect > Vmax || vExpect < Vmin)   //线速度限制
    {
        return -1;
    }
    if (accExpect > AccMax || accExpect < AccMin)
    {
        return -1;
    }

    double delta_t = period; //采样周期 
    double pi = 3.14159;
//---------------------初始与末端位姿赋值------------------
    double poseOrgX, poseOrgY, poseOrgPhi; //初始位姿赋值
    double poseEndX, poseEndY, poseEndPhi;//末端位姿赋值

    poseOrgX = ptPoseOrg->x;
    poseOrgY = ptPoseOrg->y;
    poseOrgPhi = ptPoseOrg->phi;

    poseEndX = ptPoseEnd->x;
    poseEndY = ptPoseEnd->y;
    poseOrgPhi = ptPoseEnd->phi;

//------------------轨迹规划初始化规划-----------------------------------
    double dist, dist_xd, dist_yd, dist_xp, dist_yp, dist_itr;
    double v_triangle, v_itr, t_itr;
    double t_a, l_a, t_v, l_v, t_d; //临时变量
    double traj_theta;
    int itr, is_finish;

    //----计算轨迹总长与轨迹法向量
    dist_xd = poseEndX - poseOrgX;
    dist_yd = poseEndY - poseOrgY;
    dist = sqrt((dist_xd * dist_xd) + (dist_yd * dist_yd)); 
    dist_xp = dist_xd / dist;
    dist_yp = dist_yd / dist;
    traj_theta = atan2(dist_yd, dist_xd);
    //------轨迹长度非法
    if (dist > Lmax || dist < Lmin)
    {
        return -1;
    }

    //----初始化输出量
    memset(ptRefPlanNode, 0, sizeof(TAgvMotionNode) * AGV_PLAN_NODE_MAX);
    ptRefPlanNode[0].tPose2D.x = poseOrgX;
    ptRefPlanNode[0].tPose2D.y = poseOrgY;
    ptRefPlanNode[0].tPose2D.phi = traj_theta;
    ptRefPlanNode[0].tTwist2D.v = accExpect * delta_t;
    ptRefPlanNode[0].tTwist2D.w = 0; //直线运动的规划

    //-------仿真校准变换
    double plan_o_x;
    double plan_o_y;
    double plan_o_phi;
    double sim_offset_x = 0.1;
    double sim_offset_y = 0.2;
    double sim_offset_phi = 2*pi / 3;
    //----初始化迭代量
    v_itr = 0; 
    t_itr = 0;
    dist_itr = 0;
    is_finish = 0;
    //----判断是否轨迹过短导致的三角形加减速
    v_triangle = sqrt(accExpect * dist);
//------------------三角形加减速规划------------------------
    if (v_triangle < vExpect) 
    {
        t_a = v_triangle/accExpect; //加速阶段时间终点
        for (itr = 1; itr < AGV_PLAN_NODE_MAX; itr++) //从第一个点开始
        {
            t_itr = t_itr + delta_t; //更新时间
            if (t_itr <= t_a) //加速阶段
            {
                dist_itr = 0.5 * accExpect * t_itr * t_itr; //二分之一at方
                v_itr = accExpect * t_itr;
            }
            else if (t_itr > t_a && t_itr <= 2*t_a)//减速阶段
            {
                dist_itr = (0.5 * accExpect * t_itr * t_itr) 
                    + (v_triangle * (t_itr - t_a))
                    - (0.5 * accExpect * (t_itr - t_a) * (t_itr - t_a));
            }
            else //剩余时间段
            {
                dist_itr = dist;
                v_itr = 0;
                is_finish = 1;
            }

            //增量计算投影至XY方向
            ptRefPlanNode[itr].tPose2D.x = poseOrgX + dist_xp * dist_itr;
            ptRefPlanNode[itr].tPose2D.y = poseOrgY + dist_yp * dist_itr;
            ptRefPlanNode[itr].tPose2D.phi = traj_theta;
            ptRefPlanNode[itr].tTwist2D.v = v_itr;
            ptRefPlanNode[itr].tTwist2D.w = 0; //直线运动的规划
            if (is_finish == 1)
            {
                s32PlanTotal = itr;
                break;
            }
        }//end_for
    }//end if //三角形情况
//---------------------梯形加减速情形------------------------
    else 
    {
        t_a = vExpect / accExpect;
        l_a = 0.5 * accExpect * t_a * t_a;
        t_v = (dist - vExpect * vExpect / accExpect) / vExpect;
        l_v = vExpect * t_v;
        t_d = vExpect / accExpect;
        for (itr = 1; itr < AGV_PLAN_NODE_MAX; itr++) //从第一个点开始
        {
            t_itr = t_itr + delta_t; //更新时间
            if (t_itr <= t_a) //加速阶段
            {
                dist_itr = 0.5 * accExpect * t_itr * t_itr; //二分之一at方
                v_itr = accExpect * t_itr;
            }
            else if (t_itr > t_a && t_itr <= (t_a + t_v) ) //匀速阶段
            {
                dist_itr = l_a + (t_itr - t_a) * vExpect;
                v_itr = vExpect;
            }
            else if (t_itr > (t_a + t_v) && t_itr <= (t_a + t_v + t_d)) //减速阶段
            {
                dist_itr = (l_a + l_v + (t_itr - t_a - t_v) * vExpect) -
                    (0.5 * accExpect * (t_itr - t_a - t_v) * (t_itr - t_a - t_v));
                v_itr = vExpect - accExpect * (t_itr - t_a - t_v);
            }
            else //剩余时间阶段
            {
                dist_itr = dist;
                v_itr = 0;
                is_finish = 1;
            }

            plan_o_x = poseOrgX + dist_xp * dist_itr;
            plan_o_y = poseOrgY + dist_yp * dist_itr;
            plan_o_phi = traj_theta;

            if (itr == floor(0.5*(dist/vExpect)/delta_t))
            {
                sim_offset_phi = pi/36 + 0.03;
                sim_offset_x = 1;
                sim_offset_y = -0.5;
            }

            if (itr > floor(0.5*(dist/vExpect)/delta_t)) //全局校准
            {
                
                ptRefPlanNode[itr].tPose2D.x = 
                    cos(sim_offset_phi) * plan_o_x - sin(sim_offset_phi) * plan_o_y + sim_offset_x;
                ptRefPlanNode[itr].tPose2D.y = 
                    sin(sim_offset_phi) * plan_o_x + cos(sim_offset_phi) * plan_o_y + sim_offset_y;
                ptRefPlanNode[itr].tPose2D.phi = plan_o_phi + sim_offset_phi;
                ptRefPlanNode[itr].tTwist2D.v = v_itr;
                ptRefPlanNode[itr].tTwist2D.w = 0; //直线运动的规划

            }
            else
            {
                ptRefPlanNode[itr].tPose2D.x = plan_o_x;
                ptRefPlanNode[itr].tPose2D.y = plan_o_y;
                ptRefPlanNode[itr].tPose2D.phi = plan_o_phi;
                ptRefPlanNode[itr].tTwist2D.v = v_itr;
                ptRefPlanNode[itr].tTwist2D.w = 0; //直线运动的规划
            }
            //增量计算投影至XY方向


            if (is_finish == 1)
            {
                s32PlanTotal = itr;
                break;
            }
        }//end_for
    }//end else //梯形情况

    //规划未完成
    if (is_finish != 1)
    {
        *planNodeTotal = 0;
        return -1;
    }

    //规划完成
    *planNodeTotal = s32PlanTotal;
    return 0;    
}



/**
 * @name: CyclePlanTest
 * @des:  圆弧规划
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int TrackingPlanner::CyclePlanTest(const TAgvPose2D *ptPoseOrg, const TAgvPose2D *ptPoseEnd,
                double period, double vExpect, double accExpect,
                int *planNodeTotal)
{
    int s32Ret;
    int s32PlanTotal = 0;
    double delta_t = period; //采样周期 
    double pi = 3.14159;

    if (period > 1.0 || period < 0.0)
    {
        return -1;
    }
    if (vExpect > Vmax || vExpect < Vmin)   //线速度限制
    {
        return -1;
    }
    if (accExpect > AccMax || accExpect < AccMin)
    {
        return -1;
    }
    double d = 1;
    double M = sqrt((ptPoseOrg->x - ptPoseEnd->x) * (ptPoseOrg->x - ptPoseEnd->x)
                        + (ptPoseOrg->y - ptPoseEnd->y) * (ptPoseOrg->y - ptPoseEnd->y));
    double R = 0.5 * d + 0.125 * M * M / d;
    double theta = 2 * asin(0.5 * M / R);
    double L = theta * R; //弧长
    double Cx = 0.5 * (ptPoseEnd->x + ptPoseOrg->x) - (ptPoseEnd->y - ptPoseOrg->y) * (R - d) / M; //圆心X
    double Cy = 0.5 * (ptPoseEnd->y + ptPoseOrg->y) + (ptPoseEnd->x - ptPoseOrg->x) * (R - d) / M; //圆心Y

    double f64Div = L / vExpect; //份数

    //---初始期望位姿赋值---------------------------
    double PhiOrg;
    PhiOrg = atan2((ptPoseOrg->x - Cx) / R, (-ptPoseOrg->y + Cy) / R);
    memset(ptRefPlanNode, 0, sizeof(TAgvMotionNode) * AGV_PLAN_NODE_MAX);
    ptRefPlanNode[0].tPose2D.x = ptPoseOrg->x;
    ptRefPlanNode[0].tPose2D.y = ptPoseOrg->y;
    ptRefPlanNode[0].tPose2D.phi = PhiOrg;
    ptRefPlanNode[0].tTwist2D.v = vExpect;
    ptRefPlanNode[0].tTwist2D.w = vExpect / R; //r = v/w

    //----开始规划---------------------------------
    int DivTotal = floor(f64Div / delta_t);
    double phi_itr = 0.0;
    double phi_plan;
    double tmp_x;
    double tmp_y;
    double w_itr = vExpect / R;
    int itr;

    for (itr = 1; itr < DivTotal; itr++)
    {
        phi_itr = phi_itr + w_itr * delta_t;
        tmp_x = (cos(phi_itr) * (ptPoseOrg->x - Cx) - sin(phi_itr) * (ptPoseOrg->y - Cy)) + Cx;
        tmp_y = (sin(phi_itr) * (ptPoseOrg->x - Cx) + cos(phi_itr) * (ptPoseOrg->y - Cy)) + Cy; 
        ptRefPlanNode[itr].tPose2D.x = tmp_x;
        ptRefPlanNode[itr].tPose2D.y = tmp_y;
        phi_plan = atan2((tmp_x - Cx) / R, -(tmp_y - Cy) / R);
        if (phi_plan >= pi)
        {
            phi_plan = phi_plan - 2 * pi;
        }
        else if (phi_plan < -pi)
        {
            phi_plan = phi_plan + 2 * pi;
        }
        ptRefPlanNode[itr].tPose2D.phi = phi_plan;
        ptRefPlanNode[itr].tTwist2D.v = vExpect;
        ptRefPlanNode[itr].tTwist2D.w = vExpect / R; //r = v/w
        
    }

    //终点位姿
    ptRefPlanNode[itr].tPose2D.x =  ptPoseEnd->x;
    ptRefPlanNode[itr].tPose2D.x =  ptPoseEnd->y;
    ptRefPlanNode[itr].tPose2D.phi = phi_plan;
    ptRefPlanNode[itr].tTwist2D.v = 0;
    ptRefPlanNode[itr].tTwist2D.w = 0; //r = v/w

    s32PlanTotal = itr;
    *planNodeTotal = itr;

    return 0;
}
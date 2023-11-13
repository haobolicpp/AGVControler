
#include <cmath>
#include <angles/angles.h>
#include "move_base/start_rotate_control.h"

namespace move_base {

#ifndef PI
#define PI                 3.14159265358979f
#endif

#define START_YAW_TOLERANCE 0.04 //起始角度认为到了的误差范围 5°
#define EPSILON_VEL 0.01 //判断速度是否为0的误差 1cm/s
#define IS_VEL_ZERO(v) (v>-EPSILON_VEL && v<EPSILON_VEL) 

CStartRotateControl::CStartRotateControl()
{
    m_eControlState = EStartRotateControlState::State_Init;
    m_bAngleDiffOK = false;
}

CStartRotateControl::~CStartRotateControl()
{

}

void CStartRotateControl::InitDesAngle(double dDesAngle)
{
    m_dDesAngle = dDesAngle;
    m_eControlState = EStartRotateControlState::State_Init;
    m_bAngleDiffOK = false;
}

bool CStartRotateControl::IsNeedControl(TBGI_Pose tCurrentPose)
{
    if (m_eControlState == EStartRotateControlState::State_Init)//判断是否需要进行旋转
    {
        double ang_diff = angles::shortest_angular_distance(tCurrentPose.dth, m_dDesAngle);
        if (fabs(ang_diff) <= START_YAW_TOLERANCE)
        {
            m_eControlState = EStartRotateControlState::State_Control_Finished;
            return false;
        }
        else
        {
            ROS_INFO("robot angle:%f, des angle:%f need rotate at start!", tCurrentPose.dth, m_dDesAngle);
            m_eControlState = EStartRotateControlState::State_Controlling;
        }
    }
    else if (m_eControlState == EStartRotateControlState::State_Controlling)
    {
        return true;
    }
    else if (m_eControlState == EStartRotateControlState::State_Control_Finished)
    {
        return false;
    }

    return true;
}

double CStartRotateControl::GetRotateVel(TBGI_Pose tCurrentPose, TBGI_Vel tCurrentVel, double dthMax, double dthAcc)
{
    dthAcc;
    if (!m_bAngleDiffOK)
    {
        double ang_diff = angles::shortest_angular_distance(tCurrentPose.dth, m_dDesAngle);
        if (fabs(ang_diff) <= START_YAW_TOLERANCE)
        {
            //返回角速度0
            m_bAngleDiffOK = true;
            return 0.0;
        }
        else
        {
            //计算发送的角速度
            double vel_send = std::min(dthMax, fabs(ang_diff)); //不能超过最大角速度

            if (ang_diff < 0)
            {
                vel_send = -vel_send;
            }
            
            
            ROS_INFO("robot angle:%f, des angle:%f, vel_send:%f", 180 * tCurrentPose.dth / PI, 180 * m_dDesAngle / PI , 180 * vel_send / PI );

            return vel_send;
        }
    }
    

    //角度满足，判断是否彻底停下来了
    if (m_bAngleDiffOK)
    {
        ROS_INFO("m_bAngleDiffOK, xv:%f, yv:%f, thv:%f", tCurrentVel.dx_v, tCurrentVel.dy_v, tCurrentVel.dth_v);
        if (IS_VEL_ZERO(tCurrentVel.dx_v) && IS_VEL_ZERO(tCurrentVel.dy_v) && IS_VEL_ZERO(tCurrentVel.dth_v))
        {
            ROS_INFO("State_Control_Finished");
            m_eControlState = EStartRotateControlState::State_Control_Finished;
            m_bAngleDiffOK = false;
        }
    }
    return 0.0;
}

double CStartRotateControl::TwoPointAngle(double x0, double y0, double x1, double y1)
{
    double th = 0;
    double del_x, del_y = 0;

    del_x = x1 - x0;
    del_y = y1 - y0;
    th = acos(fabs(del_x) / sqrt(del_x*del_x + del_y*del_y));
    if (del_x>0 && del_y<0)
    {
        th = -th; 
    }
    else if (del_x<0 && del_y>0)
    {
        th = PI - th;
    }
    else if (del_x<0 && del_y<0)
    {
        th = -PI + th;
    }
    else{}

    return th;
}

}

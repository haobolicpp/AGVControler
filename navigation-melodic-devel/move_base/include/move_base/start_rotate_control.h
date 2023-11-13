/**
 * @file start_rotate_control.h
 * @author your name (you@domain.com)
 * @brief 起始时选装控制
 * @version 0.1
 * @date 2021-03-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef START_ROTATE_CONTROL_H_
#define START_ROTATE_CONTROL_H_

#include <string>

#include <ros/ros.h>

namespace move_base {

    //位姿信息描述
    typedef struct TBGI_Pose{
        double dx;
        double dy;
        double dth; //角度范围-PI到PI
    }TBGI_Pose;

    //速度信息描述
    typedef struct TBGI_Vel{
        double dx_v;
        double dy_v;
        double dth_v; 
    }TBGI_Vel;

    //开始旋转控制状态枚举
    enum class EStartRotateControlState
    {
        State_Init,             //未开始控制
        State_Controlling,      //控制中
        State_Control_Finished  //控制完成
    };

    class CStartRotateControl
    {
        public:
        CStartRotateControl();
        ~CStartRotateControl();

        /**
         * @brief 初始化目标角度信息
         * 
         * @param dDesAngle 目标角度（-PI ~ PI）
         */
        void InitDesAngle(double dDesAngle);


        /**
         * @brief 是否需要进行初始旋转控制
         * 
         * @param tCurrentPose 当前机器人位姿
         * @return true 
         * @return false 
         */
        bool IsNeedControl(TBGI_Pose tCurrentPose);

        /**
         * @brief 进行旋转控制
         * 
         * @param tCurrentPose 机器人当前位姿
         * @param tCurrentVel 机器人当前速度
         * @param dthMax 机器人角速度最大值
         * @param dthAcc 机器人角速度加速度
         * @return double 返回要发送的角速度
         */
        double GetRotateVel(TBGI_Pose tCurrentPose, TBGI_Vel tCurrentVel, double dthMax, double dthAcc);

        private:
        /**
         * @brief 计算两个点的夹角，返回的范围是-PI~PI
         * 
         * @param x0 起始
         * @param y0 
         * @param x1 终止
         * @param y1 
         * @return double 返回弧度 -PI~PI
         */
        double TwoPointAngle(double x0, double y0, double x1, double y1);

        private:
        //当前控制状态
        EStartRotateControlState m_eControlState;

        //当前机器人目标点位姿
        TBGI_Pose m_tDesPose;

        //目标角度
        double m_dDesAngle;

        //角度偏差是否已经满足
        bool m_bAngleDiffOK;
    };
}
#endif


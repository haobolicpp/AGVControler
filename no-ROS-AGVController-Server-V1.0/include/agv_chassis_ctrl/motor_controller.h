/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 08:32:38
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-07-31 14:56:16
 * @FilePath: /agv_controller/include/agv_chassis_ctrl/motor_controller.h
 * @Description: AGV底盘驱动伺服电机实时控制
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <string.h>
#include <pthread.h>
#include <mqueue.h>

#include "global.h"
#include "co_interface.h"


/**
 * @name: MCStatus
 * @des:  伺服对象访问状态
 * @author: yang.cheng
 * @ver: 1.01
 */
enum class MCStatus
{
    INIT,
    RESET,  //6040 <- 0x86
    ENABLE, //6040 <- 0x2f
    IDLE,
    CYC1,    //60FF <-
    CYC2,    //60
    DISABLE, //6040 <- 0x06
    
    ERROR
};

/**
 * @name: ComStatus
 * @des:  通信状态
 * @author: yang.cheng
 * @ver: 1.01
 */
enum class ComStatus
{
    IDLE,
    SEND,
    ACK,
    ERROR,
};


/**
 * @name: MotorController
 * @des:  类定义
 * @author: yang.cheng
 * @ver: 1.02
 */
class MotorController
{
    public:
        MotorController();
        ~MotorController();
        int init();

    public:
        //CANopen 服务接口使用的变量
        pthread_t thCanSvr;
        mqd_t QCoIf_;
        MCStatus McStatus;
        ComStatus LmStatus, RmStatus;
        int MoterCtrlStart();
        int CoIfMsgHandler(TCoIfMsg *ptCoIfMsg);
        int CoIfMsgHandlerINIT(TCoIfMsg *ptCoIfMsg);
        int CoIfMsgHandlerRESET(TCoIfMsg *ptCoIfMsg);
        int CoIfMsgHandlerENABLE(TCoIfMsg *ptCoIfMsg);
        int CoIfMsgHandlerIDLE(TCoIfMsg *ptCoIfMsg);
        int CoIfMsgHandlerCYC1(TCoIfMsg *ptCoIfMsg);
        int CoIfMsgHandlerCYC2(TCoIfMsg *ptCoIfMsg);
        int CoIfMsgHandlerERROR(TCoIfMsg *ptCoIfMsg);
        int CoIfPeriodCtrl();
        //状态监控
        bool bTransToEnable;
        bool bTransToError;
        
    private:
        int diffEncoder(int encNow, int encLast);
        inline double zeroInterval(double v);

    public:
//静态参数-------------------------------------------
        char busname[16];
        char baudrate[16];
        double wheelDistance;   //轮距 (m)
        double wheelRadius;     //轮半径
        double reductionRatio;  //减速比
        double maxRPM;          //最高转速
        int s32EncoderResolution;//编码器分辨率,即电机转一圈编码器个数
        double rateEncoder; //电机转一个编码，车轮行走的距离（考虑减速比）

//导航控制与遥控控制-----------------------------------
        //当前周期的线速度和角速度的控制量
        double ctrl_v_;
        double ctrl_w_;
        //左轮和右轮速度
        //考虑使用读写锁 因为变量将在上层设定方法中被设定，在下层CANopen服务线程中被读取
        double fAgvLV;
        double fAgvRV;
        volatile int32_t s32AgvLV;
        volatile int32_t s32AgvRV;
        int SetCtrlCmd(double fV, double fW);
        int CalMotorCmd();

//速度控制输出的平滑滤波--------------------------------
        double SF_LvOut_;
        double SF_RvOut_;
        double SF_LvPara_;
        double SF_RvPara_;
        double SF_LvMin_;
        double SF_RvMin_;
        double SF_LvMax_;
        double SF_RvMax_;
        double SmoothFilterLv(double Lv);
        double SmoothFilterRv(double Rv);
//根据轮速编码器计算里程计------------------------------
        int LRPFresh;
        double fAgvLP;         
        double fAgvRP;
        volatile int32_t s32AgvLP;//左轮和右轮位置
        volatile int32_t s32AgvRP;

        int leftEncoderLast; //编码器上个采样周期数据
        int rightEncoderLast;
        int64_t TimeTickLast;
        int64_t GetTimeTick();

        double odom_x_ ;   //累积
        double odom_y_;   
        double odom_theta_;
        double odom_v_;   //当前线速度
        double odom_w_;   //点前角速度
        bool   odom_fresh_;  //如果不是从编码器获取的最新ODOM数据 根据差分计算的速度字段难以反映机器人真实速度
        int CalMotorOdom();  //里程计计算
        int GetOdomMsg(double &x, double &y, double &theta,
            double &v, double &w);
        int GetTwistMsg(double &v, double &w);

};

#endif

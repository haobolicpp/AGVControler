

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <string.h>
#include <pthread.h>
#include <mqueue.h>

#include "canfestival.h"
#include "global.hpp"
#include "co_interface.hpp"


//#include <ros/ros.h>
//#include <nav_msgs/Odometry.h>
//#include <tf2_ros/transform_broadcaster.h>

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


enum class ComStatus
{
    IDLE,
    SEND,
    ACK,
    ERROR,
};

class MotorController
{
    public:
        MotorController();
        ~MotorController();
    
    public:
        int init();
        int setVel(double linearVel, double anglerVel);
        int getOdom(double &linearVel, double &anglerVel, double &posXAcc, double &posYAcc, double &thetaAcc, double dt);

    public:
        //左轮和右轮速度
        //！！todo 考虑使用读写锁 因为变量将在上层设定方法中被设定，在下层CANopen服务线程中被读取
        double fAgvLV;
        double fAgvRV;
        volatile int32_t s32AgvLV;
        volatile int32_t s32AgvRV;

        //左轮和右轮位置
        int LRPFresh;
        double fAgvLP;
        double fAgvRP;
        volatile int32_t s32AgvLP;
        volatile int32_t s32AgvRP;

    public:
        //CANopen 服务接口使用的变量
        pthread_t thCanSvr;
        char busname[16];
        char baudrate[16];

        MCStatus McStatus;
        ComStatus LmStatus, RmStatus;

        mqd_t QCoIf_;
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

        int GetFreshServoPosition(int &LP, int &RP);

    private:
        int diffEncoder(int encNow, int encLast);
        inline double zeroInterval(double v);

    private:
        double wheelDistance;   //轮距 (m)
        double wheelRadius;     //轮半径
        double reductionRatio;  //减速比
        double maxRPM;          //最高转速
        int s32EncoderResolution;//编码器分辨率,即电机转一圈编码器个数

        double m_posXAcc = 0.0;   //累积
        double m_posYAcc = 0.0;   
        double m_thetaAcc = 0.0;

        int leftEncoderLast; //编码器上个采样周期数据
        int rightEncoderLast;

        //初始常量
        double rateEncoder; //电机转一个编码，车轮行走的距离（考虑减速比）
        //double maxPeriodEncoder;  //一个采样周期行进的最大编码器个数
};

#endif

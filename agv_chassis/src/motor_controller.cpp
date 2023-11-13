/*
    底盘运动学参考链接：
    https://www.cxyzjd.com/article/xingdou520/83691951  （轨迹推算不行）
    论文：《一种惯性传感器与编码器相结合的 AGV航迹推算系统》
    自己整理的文档（doc/运动学模型）
*/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <mqueue.h>

#include <ros/ros.h>
#include "global.hpp"
#include "motor_controller.hpp"
#include "co_interface.hpp"


#ifndef PI
#define PI 3.1415926
#endif

//extern CO_Data agv_master_Data;


MotorController::MotorController()
{
    //s32CtrlFreq = 20;

    fAgvLV = 0;
    fAgvRV = 0;
    s32AgvLV = 0;
    s32AgvRV = 0;
    fAgvLP = 0;
    fAgvRP = 0;
    s32AgvLP = 0;
    s32AgvRP = 0;
    LRPFresh = 0;

    McStatus = MCStatus::INIT;
    LmStatus = ComStatus::IDLE;
    RmStatus = ComStatus::IDLE;

    //Parse the CAN paramater from the config
    strcpy(busname, "0");
    strcpy(baudrate, "250K");
}

MotorController::~MotorController()
{
    CoInterfaceDeInit();
}

int MotorController::init()
{
    //Initialize the Canopen Stack's Timer;
    int s32Ret;

    s32Ret = CoInterfaceInit(this);
    if (s32Ret < 0)
    {
        return -1;
    }    
    //读取配置
    //wheelDistance = 0.486;  
    wheelDistance = 0.456;   
    wheelRadius = 0.075;     
    reductionRatio = 20.0;
    s32EncoderResolution = 10000; 
    maxRPM = 0;

    //
    rateEncoder = 2 * PI * wheelRadius / (s32EncoderResolution*reductionRatio);
    //maxPeriodEncoder = 1 / s32CtrlFreq * maxRPM / 60.0 * s32EncoderResolution;
    ROS_INFO("rateEncoder : %.3f", rateEncoder);

    return 0;
}

/**
 * @brief 轮子速度控制
 * 
 * @param linearVel 线速度
 * @param angularVel 角速度
 * @return int 
 */
int MotorController::setVel(double linearVel, double angularVel)
{
    double wd = angularVel * wheelDistance / 2.0; 
    double leftSpeed = linearVel - wd; //当angularVel为正值时，车子逆时针旋转，左轮速度小
    double rightSpeed = linearVel + wd;

    //速度m/s转轮子的rpm（不是电机的rpm）
    double rateV2RPM = 60/(2*PI*wheelRadius);
    double leftRPM = leftSpeed * rateV2RPM;    
    double rightRPM = rightSpeed * rateV2RPM; 
    
    //加减速比进行转换，单位换算关系为 DEC=[(rpm*512*编码器分辨率)/1875],这里rpm要考虑减速比
    //左轮电机安装左右对称，速度取反。
    s32AgvLV = -floor((leftRPM*reductionRatio*512*s32EncoderResolution)/1875.0);
    s32AgvRV = floor((rightRPM*reductionRatio*512*s32EncoderResolution)/1875.0);

    if (s32AgvLV != 0 || s32AgvRV != 0)
    {
        // ROS_INFO("setVel() linearVel: %.3f, angularVel:%.3f, leftRPM(no-reduc):%.3f, rightRPM:%.3f,s32AgvLV:%d,s32AgvLV:%d",
        // linearVel, angularVel, leftRPM, rightRPM, s32AgvLV, s32AgvRV);
    }

    //发送can指令 
    //return CoMotorVelocitySetRequest(this);  
    return 0;
}

/**
 * @brief 计算一个周期dt后里程计信息
 * 
 * @param linearVel 
 * @param anglerVel 
 * @param posXAcc 
 * @param posYAcc 
 * @param thetaAcc 
 * @param dt 单位秒
 * @return int 
 */
int MotorController::getOdom(double &linearVel, double &angularVel, double &posXAcc, double &posYAcc, double &thetaAcc, double dt)
{
    static bool init_run = true;

    if (init_run)
    {
        leftEncoderLast = -s32AgvLP; //左轮电机安装左右对称 取反
        rightEncoderLast = s32AgvRP;
        init_run = false;
        return -1;
    }

    //计算线速度和角速度
    int leftEncoderNow = -s32AgvLP; //左轮电机安装左右对称 取反
    int rightEncoderNow = s32AgvRP;
    double dlc = (double)diffEncoder(leftEncoderNow, leftEncoderLast); //编码器差值，负表示倒着转
    double drc = (double)diffEncoder(rightEncoderNow, rightEncoderLast);
    leftEncoderLast = leftEncoderNow;
    rightEncoderLast = rightEncoderNow;
    double dl = dlc * rateEncoder;  //轮子行进距离（圆弧长）
    double dr = drc * rateEncoder;
    
    double leftSpeed = dl / dt;
    double rightSpeed = dr / dt;

    linearVel = (leftSpeed + rightSpeed) / 2.0;  //结果有可能是0，表示原地旋转
    //angularVel = (leftSpeed - rightSpeed) / wheelDistance; //结果有可能是0，表示直线行驶
    angularVel = (rightSpeed - leftSpeed) / wheelDistance; //注意顺序,右-左，论文有误


    //double angularCentral = angularVel * dt; //通过角速度计算的圆心角
    double angularCentral = (dr - dl) / wheelDistance; //注意顺序,右-左，论文有误
    double dMidArcLen = (dl + dr) / 2.0; //AGV中心位置行进的圆弧长度
    double dMidLineLen = 0; //AGV中心位置行进的直线长度

    //【直接套用余弦定理求直线长】已知r和夹角，余弦定理求边长,如果旋转角度为0，则会有问题
    //dMidLineLen = std::sqrt(2*(1-std::cos(angularCentral))) * dMidArcLen / angularCentral; 
    //【将上式在0度处进行泰勒展开】保留3项，参考论文
    double midl2 = 2*dMidArcLen*dMidArcLen*(0.5 - angularCentral*angularCentral/24.0 + angularCentral*angularCentral*angularCentral*angularCentral/720.0);
    dMidLineLen = std::sqrt(midl2);

    //前进后退判断，分两种情况：1、直线行驶；2、转弯行驶
    //1.直线行驶，角度th为0，前进后退靠dMidLineLen符号控制
    if (std::fabs(dlc-drc) < 200) //实验测得 AGV在直线行驶时 左右轮编码器增联偏差在一个周期内不超过200
    {
        if (drc < 0)
        {
            dMidLineLen = -dMidLineLen;
        }
    }
    //2.拐弯行驶，dMidLineLen不进行操作，靠cos或sin的正负号控制前进后退
    
    double th = m_thetaAcc + angularCentral/2;
    m_posXAcc += dMidLineLen * std::cos(th);
    m_posYAcc += dMidLineLen * std::sin(th);
    m_thetaAcc = angularCentral + m_thetaAcc;
    m_thetaAcc = std::atan2(std::sin(m_thetaAcc), std::cos(m_thetaAcc)); //任意角度转-PI PI范围,nb nb

    posXAcc = m_posXAcc;
    posYAcc = m_posYAcc;
    thetaAcc = m_thetaAcc;
    
    //ROS_INFO("getOdom() linearVel: %.3f, angularVel:%.3f, angularCentral:%.3f, dMidArcLen:%.3f,dMidLineLen:%.3f,posXAcc:%.3f,posYAcc:%.3f,thetaAcc:%.3f",
    //linearVel, angularVel, angularCentral, dMidArcLen, dMidLineLen, posXAcc, posYAcc, thetaAcc);
    //ROS_INFO("leftEncoderNow:%d, leftEncoderLast:%d, dlc:%f,drc:%f, delta:%f", leftEncoderNow, leftEncoderLast, dlc, drc, std::fabs(dlc-drc));

    // static double maxdif = std::fabs(dlc-drc);
    // maxdif = std::max(maxdif, std::fabs(dlc-drc));
    // ROS_INFO("maxdif:%f", maxdif);

    return 0;
}

/**
 * @brief 计算编码器差值
 * 
 * @param encNow 
 * @param encLast 
 * @return int >0:正走，<0：倒着走
 */
int MotorController::diffEncoder(int encNow, int encLast)
{
    /*
    * 
    */
    return (encNow - encLast + 0x100000000) % 0x100000000;
}

double MotorController::zeroInterval(double v)
{
    //0000001 :TODO
    if (v < 0.0000001 && v > -0.00000001)
    {
        v = 0.0;
    }
    return v;
}



/**
 * @brief MotorController异步消息处理方法
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfMsgHandler(TCoIfMsg *ptCoIfMsg)
{
    int s32Ret;

    switch (McStatus)
    {
        case MCStatus::INIT:
            s32Ret = CoIfMsgHandlerINIT(ptCoIfMsg);
        break;

        case MCStatus::RESET:
            s32Ret = CoIfMsgHandlerRESET(ptCoIfMsg);
        break;

        case MCStatus::ENABLE:
            s32Ret = CoIfMsgHandlerENABLE(ptCoIfMsg);
        break;

        case MCStatus::IDLE:
            s32Ret = CoIfMsgHandlerIDLE(ptCoIfMsg);
        break;

        case MCStatus::CYC1:
            s32Ret = CoIfMsgHandlerCYC1(ptCoIfMsg);
        break;

        case MCStatus::CYC2:
            s32Ret = CoIfMsgHandlerCYC2(ptCoIfMsg);
        break;

        case MCStatus::ERROR:
            s32Ret = CoIfMsgHandlerERROR(ptCoIfMsg);
        break;

        default:
        break;
    }

}

/**
 * @brief MotorController异步消息处理方法INIT状态下
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfMsgHandlerINIT(TCoIfMsg *ptCoIfMsg)
{
    int s32Ret;

    if (ptCoIfMsg->T == TC_MC_RESET)
    {
        CoMotorResetRequest(this);
        if (LmStatus == ComStatus::SEND && RmStatus == ComStatus::SEND)
        {
            McStatus = MCStatus::RESET;
        }
        else
        {
            McStatus = MCStatus::ERROR;
        }
    }

    return 0;
}

/**
 * @brief MotorController异步消息处理方法RESET状态下
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfMsgHandlerRESET(TCoIfMsg *ptCoIfMsg)
{
    int s32Ret;

    //状态下的消息处理
    if (ptCoIfMsg->T == TC_WSDO_ACK)
    {
        if (ptCoIfMsg->id == AGV_LMOTOR_ID)
        {
            LmStatus = ComStatus::ACK;
        }
        else if (ptCoIfMsg->id == AGV_RMOTOR_ID)
        {
            RmStatus = ComStatus::ACK;
        }
    }
    else if (ptCoIfMsg->T == TC_SDO_NACK)
    {
        if (ptCoIfMsg->id == AGV_LMOTOR_ID)
        {
            LmStatus = ComStatus::ERROR;
        }
        else if (ptCoIfMsg->id == AGV_RMOTOR_ID)
        {
            RmStatus = ComStatus::ERROR;
        }
        McStatus = MCStatus::ERROR;
    }

    //转移状态至启动电机
    if (LmStatus == ComStatus::ACK && RmStatus == ComStatus::ACK)
    {
        CoMotorEnableRequest(this);
        if (LmStatus == ComStatus::SEND && RmStatus == ComStatus::SEND)
        {
            McStatus = MCStatus::ENABLE;
        }
        else
        {
            McStatus = MCStatus::ERROR;
        }
    }


    return 0;
}



/**
 * @brief MotorController异步消息处理方法ENABLE状态下
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfMsgHandlerENABLE(TCoIfMsg *ptCoIfMsg)
{
    int s32Ret;

    //状态下的消息处理
    if (ptCoIfMsg->T == TC_WSDO_ACK)
    {
        if (ptCoIfMsg->id == AGV_LMOTOR_ID)
        {
            LmStatus = ComStatus::ACK;
        }
        else if (ptCoIfMsg->id == AGV_RMOTOR_ID)
        {
            RmStatus = ComStatus::ACK;
        }
    }
    else if (ptCoIfMsg->T == TC_SDO_NACK)
    {
        if (ptCoIfMsg->id == AGV_LMOTOR_ID)
        {
            LmStatus = ComStatus::ERROR;
        }
        else if (ptCoIfMsg->id == AGV_RMOTOR_ID)
        {
            RmStatus = ComStatus::ERROR;
        }
        McStatus = MCStatus::ERROR;
    }

    //转移状态至周期控制CYC1
    if (LmStatus == ComStatus::ACK && RmStatus == ComStatus::ACK)
    {
        //CYC1状态下 MC周期性的触发读取编码器和下发速度指令
        LmStatus = ComStatus::IDLE;
        LmStatus = ComStatus::IDLE;

        McStatus = MCStatus::IDLE;
    }

    return 0;
}


/**
 * @brief MotorController异步消息处理方法IDLE状态下
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfMsgHandlerIDLE(TCoIfMsg *ptCoIfMsg)
{
    //TODO 响应RESET等消息
    return 0;
}

/**
 * @brief MotorController异步消息处理方法CYC1状态下 等待编码器SDO读取
 * 该状态由IDLE状态在周期控制边界触发读编码器发送请求后转移至
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfMsgHandlerCYC1(TCoIfMsg *ptCoIfMsg)
{
    int s32Ret;

    //获取编码器数据
    if (ptCoIfMsg->T == TC_R4SDO_ACK)
    {
        if (ptCoIfMsg->id == AGV_LMOTOR_ID)
        {
            LmStatus = ComStatus::ACK;
            s32AgvLP = ptCoIfMsg->B.i32;
        }
        else if (ptCoIfMsg->id == AGV_RMOTOR_ID)
        {
            RmStatus = ComStatus::ACK;
            s32AgvRP = ptCoIfMsg->B.i32;
        }
    }
    else if (ptCoIfMsg->T == TC_SDO_NACK)
    {
        if (ptCoIfMsg->id == AGV_LMOTOR_ID)
        {
            LmStatus = ComStatus::ERROR;
        }
        else if (ptCoIfMsg->id == AGV_RMOTOR_ID)
        {
            RmStatus = ComStatus::ERROR;
        }
        McStatus = MCStatus::ERROR;
    }

    //转移状态至周期控制IDLE
    if (LmStatus == ComStatus::ACK && RmStatus == ComStatus::ACK)
    {
        LRPFresh = 1;
        CoMotorVelocitySetRequest(this);
        if (LmStatus == ComStatus::SEND && RmStatus == ComStatus::SEND)
        {
            McStatus = MCStatus::CYC2;
        }
        else
        {
            McStatus = MCStatus::ERROR;
        }
    }

    return 0;
}

/**
 * @brief MotorController异步消息处理方法CYC2状态下 等待速度指令下发完成
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfMsgHandlerCYC2(TCoIfMsg *ptCoIfMsg)
{
    int s32Ret;

    //状态下的消息处理
    if (ptCoIfMsg->T == TC_WSDO_ACK)
    {
        if (ptCoIfMsg->id == AGV_LMOTOR_ID)
        {
            LmStatus = ComStatus::ACK;
        }
        else if (ptCoIfMsg->id == AGV_RMOTOR_ID)
        {
            RmStatus = ComStatus::ACK;
        }
    }
    else if (ptCoIfMsg->T == TC_SDO_NACK)
    {
        if (ptCoIfMsg->id == AGV_LMOTOR_ID)
        {
            LmStatus = ComStatus::ERROR;
        }
        else if (ptCoIfMsg->id == AGV_RMOTOR_ID)
        {
            RmStatus = ComStatus::ERROR;
        }
        McStatus = MCStatus::ERROR;
    }

    //转移状态至周期控制CYC1
    if (LmStatus == ComStatus::ACK && RmStatus == ComStatus::ACK)
    {
        //CYC1状态下 MC周期性的触发读取编码器和下发速度指令
        LmStatus = ComStatus::IDLE;
        LmStatus = ComStatus::IDLE;

        McStatus = MCStatus::IDLE;
    }

    return 0;
}


/**
 * @brief MotorController异步消息处理方法ERROR状态下 等待速度指令下发完成
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfMsgHandlerERROR(TCoIfMsg *ptCoIfMsg)
{
    return 0;
}

/**
 * @brief MotorController控制周期边界的动作
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::CoIfPeriodCtrl()
{
    //todo 各状态的超时处理
    if (McStatus == MCStatus::IDLE)
    {
        CoMotorPostionGetRequest(this);
        if (LmStatus == ComStatus::SEND && RmStatus == ComStatus::SEND)
        {
            McStatus = MCStatus::CYC1;
        }
        else
        {
            McStatus = MCStatus::ERROR;
        }
    }

    return 0;
}


/**
 * @brief MotorController控制周期边界的动作
 * @param ptCoIfMsg 异步消息指针
 * @return int 
 */
int MotorController::MoterCtrlStart()
{
    return CoMotorCtrlStart();        
}

/**
 * @brief 获取最新的编码器数据，如果没有最新的则返回-1
 * @param 
 * @return 0成功 1失败
 */
int MotorController::GetFreshServoPosition(int &LP, int &RP)
{
    if(LRPFresh != 1)
    {
        return -1;
    }

    LP = s32AgvLP;
    RP = s32AgvRP;
    LRPFresh = 0;
    
    return 0;
}
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
#include <math.h>
#include <time.h>

#include "global.h"
#include "motor_controller.h"
#include "co_interface.h"

#ifndef PI
#define PI 3.1415926
#endif


/**
 * @brief MotorController构造
 * @return
 */
MotorController::MotorController()
{
    ctrl_v_ = 0.0;
    ctrl_w_ = 0.0;
    odom_x_ = 0.0;   //累积
    odom_y_ = 0.0;   
    odom_theta_ = 0.0;
    odom_v_ = 0.0;   //当前线速度
    odom_w_ = 0.0;   //点前角速度

    fAgvLV = 0;
    fAgvRV = 0;
    s32AgvLV = 0;
    s32AgvRV = 0;
    fAgvLP = 0;
    fAgvRP = 0;
    s32AgvLP = 0;
    s32AgvRP = 0;
    LRPFresh = 0;

//速度输出平滑滤波------------------------
    SF_LvOut_ = 0;
    SF_RvOut_ = 0;
    SF_LvPara_ = 0.6;
    SF_RvPara_ = 0.6;
    SF_LvMin_ = 1e2; //死区控制 0.2RPM  考虑减速比
    SF_RvMin_ = 1e2; //死区控制 0.2RPM  考虑减速比
    SF_LvMax_ = 3.276e6; //限幅控制 30RPM  考虑减速比 此时最大速度约等于3m/s
    SF_RvMax_ = 3.276e6; //限幅控制 30RPM  考虑减速比 此时最大速度约等于3m/s
    
//控制状态初始化--------------------------
    McStatus = MCStatus::INIT;
    LmStatus = ComStatus::IDLE;
    RmStatus = ComStatus::IDLE;
    bTransToEnable = false;
    bTransToError = false;
//CAN通信参数初始化----------------------
    strcpy(busname, "0");
    strcpy(baudrate, "250K");

}

/**
 * @brief MotorController析构
 * @return
 */
MotorController::~MotorController()
{
    CoInterfaceDeInit();
}

/**
 * @brief MotorController初始化
 * @return -1 失败 0 成功 
 */
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
    //MINIKIVA参数
    // wheelDistance = 0.468;   
    // wheelRadius = 0.08;     
    // reductionRatio = 9;
    // s32EncoderResolution = 10000; 
    // maxRPM = 0;

    wheelDistance = 0.462193;   
    wheelRadius = 0.081;     
    reductionRatio = 9;
    s32EncoderResolution = 10000; 
    maxRPM = 0;

    rateEncoder = 2 * PI * wheelRadius / (s32EncoderResolution*reductionRatio);

    printf("INFO: Mobile Robot Diff Model:\n");
    printf("WheelDistance[%f]-WheelRadius[%f]-ReductionRatio[%f]-EncoderResolution[%d]-RateEncoder[%f]\n", 
    wheelDistance, wheelRadius, reductionRatio, s32EncoderResolution, rateEncoder);

    return 0;
}

/**
 * @name: SetCtrlCmd
 * @des:  设置当前控制律线速度V、角速度W
 * @param {double} fV
 * @param {double} fW
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int MotorController::SetCtrlCmd(double v, double w)
{
    //速度限制与保护------------------------
    if (fabs(v) > 3.0 || fabs(w) > 6.28)
    {
        printf("WARN: Motor Control Velocity Limited!\n");
        ctrl_v_ = 0.0;
        ctrl_w_ = 0.0;
        return -1;
    }
    
    //死区控制------------------------------
    if (fabs(v) < 1e-3)
    {
        ctrl_v_ = 0.0;
    }
    if (fabs(w) < 1e-3)
    {
        ctrl_w_ = 0.0;
    }

    ctrl_v_ = v;
    ctrl_w_ = w;

    return 0;
}

/**
 * @name: CalMotorCmd
 * @des:  根据控制律V、W计算左右轮电机控制指令LV RV
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int MotorController::CalMotorCmd()
{
    //速度超限检查
    if (fabs(ctrl_v_) > 3.0 || fabs(ctrl_w_) > 6.28)
    {
        printf("WARN: Motor Control Velocity Limited!\n");
        ctrl_v_ = 0.0;
        ctrl_w_ = 0.0;
    }

    // printf("ctrl_v[%f]\n", ctrl_v_);

    double wd = ctrl_w_ * wheelDistance / 2.0; 
    double leftSpeed = ctrl_v_ - wd; //当angularVel为正值时，车子逆时针旋转，左轮速度小
    double rightSpeed = ctrl_v_ + wd;

    //速度m/s转轮子的rpm（不是电机的rpm）
    double rateV2RPM = 60/(2*PI*wheelRadius);
    double leftRPM = leftSpeed * rateV2RPM;    
    double rightRPM = rightSpeed * rateV2RPM; 
    
    //加减速比进行转换，单位换算关系为 DEC=[(rpm*512*编码器分辨率)/1875],这里rpm要考虑减速比
    //左轮电机安装左右对称，速度取反。//clark

    //TODO 临界区保护，考虑到效率问题 这里不用保护 一些情况下可能导致本周期左轮和右轮速度相差一个周期
    //CANopen伺服驱动线程频率较高 底盘数据服务线程频率较低。导致CANopen伺服驱动线程多个周期可能执行同一组速度
    //考虑速度插补方法克服较大的速度变化
    //TODO 考虑采用CANopen-PDO数据访问机制以提高运动控制的安全性，在上位机死机丢死控制后，伺服能够进行PDO超时急停
    //TODO 考虑运动模型的不确定性
    fAgvLV = -(leftRPM*reductionRatio*512*s32EncoderResolution)/1875.0;
    fAgvRV = (rightRPM*reductionRatio*512*s32EncoderResolution)/1875.0;
    // s32AgvLV = -floor((leftRPM*reductionRatio*512*s32EncoderResolution)/1875.0);
    // s32AgvRV = floor((rightRPM*reductionRatio*512*s32EncoderResolution)/1875.0);


    return 0;
}


/**
 * @name: GetTimeTick
 * @des: 返回系统Nano Tick
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int64_t MotorController::GetTimeTick()
{
    int64_t s64timeTick;
    struct timespec time_now;
    clock_gettime(CLOCK_REALTIME, &time_now);

    s64timeTick = time_now.tv_sec * 1000000000 + time_now.tv_nsec;
    return s64timeTick;
}
/**
 * @name: CalMotorOdom
 * @des:  里程计计算
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int MotorController::CalMotorOdom()
{
    static bool init_run = true;
    double dt; //时间间隔 单位为秒

    //如果编码器数据未更新
    if (LRPFresh == 0)
    {
        return 0;
    }

    //计算时间间隔 Tick单位为 Nano
    LRPFresh = 0; //消费当前编码器数据

    if (init_run)
    {
        //初始化里程计
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_theta_ = 0.0;
        odom_v_ = 0.0;
        odom_w_ = 0.0;
        odom_fresh_ = false;
        //初始化编码器历史值
        TimeTickLast = GetTimeTick();
        leftEncoderLast = -s32AgvLP; //左轮电机安装左右对称 取反
        rightEncoderLast = s32AgvRP;
        init_run = false;
        return 0;
    }


    int64_t TimeTickNow = GetTimeTick();
    dt = (double)(TimeTickNow - TimeTickLast) / 1e9; //换算时间间隔为 秒
    TimeTickLast = TimeTickNow;
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

    odom_v_ = (leftSpeed + rightSpeed) / 2.0;  //结果有可能是0，表示原地旋转
    odom_w_ = (rightSpeed - leftSpeed) / wheelDistance; //注意顺序,右-左，论文有误

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
    double th = odom_theta_ + angularCentral/2;

    odom_x_ += dMidLineLen * std::cos(th);
    odom_y_ += dMidLineLen * std::sin(th);
    odom_theta_ = angularCentral + odom_theta_;
    odom_theta_ = std::atan2(std::sin(odom_theta_), std::cos(odom_theta_)); //任意角度转-PI PI范围,nb nb
    odom_fresh_ = true;

    return 0;
}

/**
 * @name: GetOdomMsg
 * @des:  提供给外部调用的里程计信息获取接口
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int MotorController::GetOdomMsg(double &x, double &y, double &theta,
            double &v, double &w)
{
    x = odom_x_;
    y = odom_y_;
    theta = odom_theta_;
    v = odom_v_;
    w = odom_w_;

    if (!odom_fresh_) //ODOM已发送
    {
        return -1;
    }
    odom_fresh_ = false;

    return 0;
}
/**
 * @name: GetTwistMsg
 * @des:  提供给外部调用的速度信息获取接口
 * @param {double} &v
 * @param {double} &w
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int MotorController::GetTwistMsg(double &v, double &w)
{
    v = odom_v_;
    w = odom_w_;
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
    return 0;
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

    //转移状态至周期控制IDLE
    if (LmStatus == ComStatus::ACK && RmStatus == ComStatus::ACK)
    {
        //CYC1状态下 MC周期性的触发读取编码器和下发速度指令
        LmStatus = ComStatus::IDLE;
        LmStatus = ComStatus::IDLE;
        McStatus = MCStatus::IDLE;
        
        //标记底盘伺服准备就绪
        bTransToEnable = true;

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
        bTransToEnable = false; //清除使能标志 使能通知为触发模式
        bTransToError = false;
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
    bTransToEnable = false;
    bTransToError = false;
    return CoMotorCtrlStart();        
}

/**
 * @brief 速度平滑滤波器输入
 * @param velnewL 给定的新的线速度
 * @param velnewR 给定的新的角速度
 * @return int 
 */
double MotorController::SmoothFilterLv(double Lv)
{
    SF_LvOut_ = SF_LvPara_ * SF_LvOut_ + (1 - SF_LvPara_) * Lv;

    if (fabs(SF_LvOut_) < SF_LvMin_) //死区控制
    {
        SF_LvOut_ = 0.0;
    }
    else if (SF_LvOut_ > SF_LvMax_) //限幅控制
    {
        SF_LvOut_ = SF_LvMax_;
    }
    else if (SF_LvOut_ < -SF_LvMax_)
    {
        SF_LvOut_ = -SF_LvMax_;
    }

    // printf("LvIn[%f], smooth[%f]\n", Lv, SF_LvOut_);

    return SF_LvOut_;
}
/**
 * @name: 
 * @des: 
 * @param {double} Rv
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
double MotorController::SmoothFilterRv(double Rv)
{
    SF_RvOut_ = SF_RvPara_ * SF_RvOut_ + (1 - SF_RvPara_) * Rv;
    if (fabs(SF_RvOut_) < SF_RvMin_) //死区控制
    {
        SF_RvOut_ = 0.0;
    }
    else if (SF_RvOut_ > SF_RvMax_) //限幅控制
    {
        SF_RvOut_ = SF_RvMax_;
    }
    else if (SF_RvOut_ < -SF_RvMax_)
    {
        SF_RvOut_ = -SF_RvMax_;
    }

    return SF_RvOut_;
}
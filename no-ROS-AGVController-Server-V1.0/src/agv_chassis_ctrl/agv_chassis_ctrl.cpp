/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-10-12 15:12:24
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-10-12 17:09:45
 * @FilePath: /agv_controller/src/agv_chassis_ctrl/agv_chassis_ctrl.cpp
 * @Description: 底盘控制类实现
 */

#include <string>
#include <mqueue.h>
#include <assert.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "tlog.h"
#include "agv_type.h"
#include "agv_cmd.h"
#include "AgvCtrl.h"

#include "agv_chassis_ctrl.h"
#include "motor_controller.h"
#include "TrackingPlanner.h"
#include "TrackingController.h"

#include "SensorConvert.h"
#include "SlamUtil.h"


/**
 * @name: AgvChassisCtrl
 * @des: 构造
 * @param {AgvCtrl} *pt_agv_ctrl
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
AgvChassisCtrl::AgvChassisCtrl(AgvCtrl *pt_agv_ctrl)
{
    pt_agv_ctrl_ = pt_agv_ctrl;
    poAgvTf = pt_agv_ctrl->poAgvTf;

    bHasInited = false;
    bHasStarted = false;
    bRemoteCtrl = false;                  //远程遥控禁止

    //Todo两个线程绑定单核运行 共享64位内存中的一些变量
    s32MotorStateCtrlFreq = AGV_MOTOR_CTRL_FREQ;      //底盘控制频率 50Hz
    s32ChassisCtrlFreq = 20;         //底盘服务频率 20Hz
  
    po_motor_ctrl_ = new MotorController();          //构造执行器
    poTrackingPlanner = new TrackingPlanner();       //构造规划器
    poTrackingController = new TrackingController(); //构造控制器
    poRemoteController = new RemoteController();     //构造遥控器
    remoteV = 0;
    remoteW = 0;

    //默认安全等级为0
    s32SafeLevel_ = 0;
    //控制状态初始化
    s32MissionFrom_ = -1;
    eChassisCtrlState = ChassisCtrlState::INIT;
    fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateInit, 
            this, std::placeholders::_1);
}

/**
 * @name: ~AgvChassisCtrl
 * @des: 析构
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
AgvChassisCtrl::~AgvChassisCtrl()
{
    delete poTrackingPlanner;
    delete poTrackingController;
    delete poRemoteController;
    delete po_motor_ctrl_;
}

/**
 * @name: 底盘控制启动
 * @des: Init
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::Init()
{
    bHasInited = true;

    poTrackingPlanner->Init();
    poTrackingController->Init(AGV_MOTOR_CTRL_FREQ);
    poRemoteController->Init(AGV_MOTOR_CTRL_FREQ, 0.9, 0.9);

    po_motor_ctrl_->init();

    eChassisCtrlState = ChassisCtrlState::READY;
    fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateReady,
            this, std::placeholders::_1);

    //TODO 集成进系统 标准消息定义
    struct mq_attr t_mq_attr = {0};
	t_mq_attr.mq_flags = 0; //Block Mode
	t_mq_attr.mq_msgsize = sizeof(st_robot_msg); //todo intagerate with total project
	t_mq_attr.mq_maxmsg = 8;

    mq_unlink("/q_agv_chassis");
    mode_t omask;
    omask = umask(0); //返回之前的文件权限默认值，同时修改默认的umask数值为0，这样后面设置文件权限的时候，umask值无影响了（创建文件时，文件权限的计算方式是：umask取反 & 要设定的文件权限）
    this->qChassisSvr = mq_open("/q_agv_chassis", O_RDWR | O_CREAT | O_NONBLOCK , (S_IRWXU | S_IRWXG | S_IRWXO) /* 777 其他用户可修改*/,  &t_mq_attr);
    umask(omask);
    if (this->qChassisSvr < 0)
    {
        printf("ERROR: q_agv_chassis create failed with error:%s\n",strerror(errno));
    }
    assert(this->qChassisSvr >= 0);
    printf("INFO: Agv Chassis Inited!\n");

    return 0;
}

/**
 * @name: 底盘控制启动
 * @des: Start
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::Start()
{
    int s32Ret;

    if(!bHasInited || bHasStarted)
    {
        printf("ERROR: Chassis Control Start failed!\n");
        return -1;
    }
    
    bHasStarted = true;
    
    //电机驱动与控制底层模块
    pthread_attr_t pthread_attr;
    struct sched_param sched_param;
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    pthread_attr_setschedparam(&pthread_attr, &sched_param);
    s32Ret = pthread_create(&this->thMotorStateSvr, &pthread_attr, LoopMotorStateSvrStatic, (void *)(this));
    assert(s32Ret == 0);

    //底盘服务顶层模块
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    pthread_attr_setschedparam(&pthread_attr, &sched_param);
    s32Ret = pthread_create(&this->thChassisSvr, &pthread_attr, LoopChassisSvrStatic, (void *)(this));
    assert(s32Ret == 0);

    printf("INFO: Agv Chassis Control Run!\n");
    return 0;
}

/**
 * @brief LoopMotorStateSvrStatic工作线程入口
 * @param arg 当前类指针 
 * @return void
 */
void *AgvChassisCtrl::LoopMotorStateSvrStatic(void *arg)
{
    AgvChassisCtrl *poAgvChassis = (AgvChassisCtrl *)arg;

    //调整线程优先级至实时80
    int ret;
    struct sched_param param;
    int policy;
    ret = pthread_getschedparam(pthread_self(), &policy, &param);
    tlog(TLOG_INFO, "Motor Ctrl Thread Default Policy[%d], Prioity[%d], ret[%d]\n", policy, param.sched_priority, ret);
    param.sched_priority = 80;
    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    pthread_getschedparam(pthread_self(), &policy, &param);
    tlog(TLOG_INFO, "Motor Ctrl Thread SetWith Policy[%d], Prioity[%d], ret[%d]\n", policy, param.sched_priority, ret);

    poAgvChassis->LoopMotorStateSvr();
    return NULL;
}
/**
 * @brief AgvChassisCtrl工作线程入口
 * @param arg 当前类指针 
 * @return void
 */
void *AgvChassisCtrl::LoopChassisSvrStatic(void *arg)
{
    AgvChassisCtrl *poAgvChassis = (AgvChassisCtrl *)arg;

    //调整线程优先级至实时20
    int ret;
    struct sched_param param;
    int policy;
    ret = pthread_getschedparam(pthread_self(), &policy, &param);
    tlog(TLOG_INFO, "Chassis Svr Thread Default Policy[%d], Prioity[%d], ret[%d]\n", policy, param.sched_priority, ret);
    param.sched_priority = 20;
    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    pthread_getschedparam(pthread_self(), &policy, &param);
    tlog(TLOG_INFO, "Chassis Svr Thread SetWith Policy[%d], Prioity[%d], ret[%d]\n", policy, param.sched_priority, ret);

    poAgvChassis->LoopChassisSvr();
    return NULL;
}

/**
 * @brief LoopMotorStateSvr工作线程实体
 * @return int
 */
int AgvChassisCtrl::LoopMotorStateSvr()
{
    int s32Ret;
    TCoIfMsg tCoIfMsg;
    TRobotMsg tAgvMsg = {0};
    
    MotorController *poMC = po_motor_ctrl_;
    struct timeval t_tmptv;
	fd_set t_fd_read_set;
    int s32_fd_max = poMC->QCoIf_ + 1;

    int s32UsFromHz = floor(1000000.0/s32MotorStateCtrlFreq);
    t_tmptv.tv_sec = 0;
    t_tmptv.tv_usec = s32UsFromHz;
    //开启底层控制循环
    poMC->MoterCtrlStart();

    while(bHasStarted)
    {
        FD_ZERO(&t_fd_read_set);
        FD_SET(poMC->QCoIf_, &t_fd_read_set);
        s32Ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, &t_tmptv);
        if (s32Ret < 0)
        {
            printf("ERROR: Agv Motor Control Loop with %s\n", strerror(errno));
            sleep(1);
            continue;
        }
//控制周期边界 触发新的周期--------------------------------------------------
        else if (s32Ret == 0)
        {
            //根据编码器反馈计算里程计数据
            poMC->CalMotorOdom();//提取伺服数据计算里程计和底盘速度
            ChassisOdomPeriod(); //处理里程计数据和底盘速度 更新AGV速度信息
            ChassisCtrlPeriod(); //根据模块状态确定控制律给定 遥控直接给定/导航PID计算
            poMC->CalMotorCmd(); //计算当前周期伺服的执行指令

            //推动采集编码器和下发速度控制的状态机执行
            poMC->CoIfPeriodCtrl();
            //重置定时器
            t_tmptv.tv_sec = 0;
            t_tmptv.tv_usec = s32UsFromHz;
            continue;
        }
//异步SDO响应 伺服的循环访问与控制-------------------------------------------
        if (FD_ISSET(poMC->QCoIf_, &t_fd_read_set))
        {
            memset(&tCoIfMsg, 0, sizeof(TCoIfMsg));
            s32Ret = mq_receive(poMC->QCoIf_, (char *)&tCoIfMsg,
                        sizeof(TCoIfMsg), NULL);
            if (s32Ret < 0)
            {
                printf("ERROR: Agv Motor Control Get Message with %s\n", strerror(errno));
                continue;
            }
            poMC->CoIfMsgHandler(&tCoIfMsg);
            //使能标志 Enable->Idle 
            if (poMC->bTransToEnable)
            {
                ChassisReadyReport();
            }
            //出错标志
            if (poMC->McStatus == MCStatus::ERROR)
            {
                ChassisErrorReport();
            }

        }

    }
    return s32Ret;
}
/**
 * @brief LoopChassisSvr工作线程实体
 * @return int
 */
int AgvChassisCtrl::LoopChassisSvr()
{
    int s32Ret;
    TRobotMsg tAgvMsg = {0};
    
    struct timeval t_tmptv;
	fd_set t_fd_read_set;
    int s32_fd_max = qChassisSvr + 1;

    int s32UsFromHz = floor(1000000.0/s32ChassisCtrlFreq);

    while(bHasStarted)
    {
        FD_ZERO(&t_fd_read_set);
        FD_SET(qChassisSvr, &t_fd_read_set);
        s32Ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, &t_tmptv);
        if (s32Ret < 0)
        {
            printf("ERROR: Agv Chassis Control Loop with %s\n", strerror(errno));
            sleep(1);
            continue;
        }
        else if(s32Ret == 0)
        {
            t_tmptv.tv_sec = 0;
            t_tmptv.tv_usec = s32UsFromHz;
            fChassisCtrlStateFunc(NULL); //时间驱动
        }
        //手动/自动状态切换
        else if (FD_ISSET(qChassisSvr, &t_fd_read_set))
        {
            memset(&tAgvMsg, 0, sizeof(TRobotMsg));
            s32Ret = mq_receive(qChassisSvr, (char *)&tAgvMsg,
                        sizeof(TRobotMsg), NULL);
            if (s32Ret < 0)
            {
                printf("ERROR: Agv Chassis Control Get Msg with %s\n", strerror(errno));
                continue;
            }
            AsyncMsgHandler(&tAgvMsg);
            RobotMsgRelease(&tAgvMsg);
        }
    }

    return s32Ret;
}


/**
 * @brief AsyncMsgHandler异步消息处理
 * @param ptAgvMsg
 * @return void
 */
int AgvChassisCtrl::AsyncMsgHandler(TRobotMsg *ptAgvMsg)
{
    //自动手动模式切换
    int s32Ret;
    int s32SafeLevel;
    uint8_t *ptAgvMsgBody;
//各状态通用消息处理----------------------------------------------------------------
    //1 出错消息
    if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
        ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_ERROR)
    {
        eChassisCtrlState = ChassisCtrlState::ERROR;
        fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateError, 
            this, std::placeholders::_1);
        return 0;
    }
    //2 安全等级
    else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
        ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_SAFETY )
    {
        s32SafeLevel = s32SafeLevel_;
        ptAgvMsgBody = RobotMsgGetBody(ptAgvMsg);
        s32SafeLevel_ = *(int *)(ptAgvMsgBody);
        printf("INFO: AGV Safety Level Changed [%d]->[%d]\n", s32SafeLevel, s32SafeLevel_);
        //不要返回 任然需要驱动状态执行
    }

//状态特定消息处理
    s32Ret = fChassisCtrlStateFunc(ptAgvMsg);
    return s32Ret;
}

/**
 * @name: AsyncMsgPost
 * @des: 
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::AsyncMsgPost(TRobotMsg *ptAgvMsg)
{
    int ret;
    ret = mq_send(qChassisSvr, (char *)ptAgvMsg, sizeof(TRobotMsg), 0);
    if (ret < 0)
    {
        RobotMsgRelease(ptAgvMsg);
        printf("ERROR: Chassis Control Msg Post With %d\n",errno);
        return -1;
    }

    return 0;
}


/**
 * @name: ChassisCtrlStateInit
 * @des:  ChassisCtrl 在INIT状态下的行为
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisCtrlStateInit(TRobotMsg *ptAgvMsg)
{
    if (ptAgvMsg == NULL) //时间驱动
    {
        return 0;
    }

    return 0;
}
/**
 * @name: ChassisCtrlStateInit
 * @des: ChassisCtrl 在READY状态下的行为
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisCtrlStateReady(TRobotMsg *ptAgvMsg)
{
    if (ptAgvMsg == NULL) //时间驱动
    {
        return 0;
    }

    else //事件驱动
    {
        //出错处理
        if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_ERROR)
        {
            eChassisCtrlState = ChassisCtrlState::ERROR;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateError, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [READY]->[ERROR]\n");
            return 0;
        }
        //在RESET行为后进入该状态
        //TODO 检查通信和电机状态以判断是否进入STOP状态
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_READY &&
            ptAgvMsg->t_msg_header.u16_src == AGV_CHASSIS_MODULE)
        {
            eChassisCtrlState = ChassisCtrlState::STOP;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateStop, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [READY]->[STOP]\n");
            return 0;
        }
    }



    return 0;
}
/**
 * @name: ChassisCtrlStateStop
 * @des: 伺服使能后进入该状态 可以进行控制命令下发与执行
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisCtrlStateStop(TRobotMsg *ptAgvMsg)
{
    int s32Ret;
    TRobotMsg tAgvMsg;
    TRobotMsgHeader tAgvMsgHeader;
    int s32MsgBodyOffset;
    //解析消息使用
    uint8_t *pMsgBody;
    TAgvTwist2D *ptAgvTwist2D;
    uint8_t u8Remote;
    //构造发送的消息使用
    uint8_t *ptAgvMsgBody;

    if (ptAgvMsg == NULL) //时间驱动
    {
        return 0;
    }

    else
    {
        if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_ERROR)
        {
            eChassisCtrlState = ChassisCtrlState::ERROR;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateError, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [RUN]->[ERROR]\n");
            return 0;
        }

        //0 远程遥控请求-----------------------------------------------------------
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_MODE_CAHNGE)
        {
            u8Remote = ptAgvMsg->t_msg_body.sbody[0];
            u8Remote == 0 ? bRemoteCtrl = false : bRemoteCtrl = true;

            //使用远程控制
            printf("INFO: Chassis Control [Get Remote Request %d]\n", u8Remote);
            poRemoteController->Reset();
            remoteV = 0.0;
            remoteW = 0.0;

            return 0;
        }
        //1 处理遥控命令-----------------------------------------------------------
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
        ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_VEL )
        {
            pMsgBody =  RobotMsgGetBody(ptAgvMsg);
            if (pMsgBody != NULL && bRemoteCtrl) //允许远程遥控
            {
                ptAgvTwist2D = (TAgvTwist2D *)pMsgBody;
                //设置当前控制量
                remoteV = ptAgvTwist2D->v;
                remoteW = ptAgvTwist2D->w;
                // printf("Get Remote Vel V[%f] W[%f]\n", remoteV, remoteW);
            }
            else
            {
                //Error pMsgBody is NULL
                remoteV = 0.0;
                remoteW = 0.0;
            }

            return 0;
        }
        //2 处理导航任务 导航任务触发进入RUN状态--------------------------------------
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
        ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_RUN )
        {
            TAgvPose2D tPoseOrg;
            TAgvPose2D tPoseEnd;
            TAgvMotionNode tCurNode;
            int s32LinearPlanTotal;

            //初始和控制速度为0
            po_motor_ctrl_->SetCtrlCmd(0.0, 0.0);
            //TODO 根据消息设置跟踪任务
            pMsgBody =  RobotMsgGetBody(ptAgvMsg);
            memcpy(&tPoseEnd, pMsgBody, sizeof(TAgvPose2D)); //获取目标位姿
            //TODO 获取AGV当前位姿
            //GetCurrentAGVPose() 
            s32Ret = poAgvTf->GetGlobalPose2D(tPoseOrg);
            if (s32Ret < 0)
            {
                //错误管理
                return -1;
            }
            //重置规划器和控制器
            poTrackingPlanner->Reset();
            poTrackingController->Reset(1.0/AGV_MOTOR_CTRL_FREQ);

            //根据当前轨迹跟踪任务执行规划
            s32Ret = poTrackingPlanner->LinearPlan(&tPoseOrg, &tPoseEnd, 
            1.0/AGV_MOTOR_CTRL_FREQ, //规划的周期
            0.5,                   //期望速度
            0.25,                   //期望加速度
            &s32LinearPlanTotal);  //返回的规划点数量
            // s32Ret = poTrackingPlanner->CyclePlanTest(&tPoseOrg, &tPoseEnd,
            //         1/AGV_MOTOR_CTRL_FREQ, 0.2, 0.2,
            //         &s32LinearPlanTotal);
            if (s32Ret < 0)
            {
                //错误管理
                printf("ERROR: Tracking Planner Linear Plan Error!\n");
                return -1;
            }
            //发送规划结果给CLIENT 构造路径规划消息 并发送至通信模块
            tAgvMsgHeader.u16_type = AGV_CMD_T_CTRL_GLOBAL_PATH;
            tAgvMsgHeader.u16_src = 0;
            tAgvMsgHeader.u16_dest = 0;
            //BODY CONTENT--------POINT_TOTAL-----------POINT_INFO------------
            tAgvMsgHeader.s32_len = 4 + 2 * sizeof(float) * s32LinearPlanTotal;
            ptAgvMsgBody = (uint8_t*)malloc(tAgvMsgHeader.s32_len);
            s32MsgBodyOffset = 0;
            memcpy(ptAgvMsgBody, &s32LinearPlanTotal, 4);
            s32MsgBodyOffset += 4;
            for (int i = 0; i < s32LinearPlanTotal; i++)
            {
                float fx = (float)(poTrackingPlanner->ptRefPlanNode[i].tPose2D.x);
                float fy = (float)(poTrackingPlanner->ptRefPlanNode[i].tPose2D.y);
                // printf("INFO: Tracking Plan Linear Point List:");
                // printf("fx[%.6f]-fy[%.6f]\n", fx, fy);
                memcpy((ptAgvMsgBody + s32MsgBodyOffset), &fx, 4);
                s32MsgBodyOffset += 4;
                memcpy((ptAgvMsgBody + s32MsgBodyOffset), &fy, 4);
                s32MsgBodyOffset += 4;
            }
            //TODO 优化通信命令请求的 TRIGER-ACTION 架构
            RobotMsgInit(&tAgvMsg, tAgvMsgHeader, ptAgvMsgBody);
            s32Ret = mq_send(ptAgvMsg->t_msg_header.u32_sq,  //sq 保存消息源 当前Client连接的消息队列服务
                (char *)(&tAgvMsg), sizeof(TRobotMsg), 0);
            if (s32Ret < 0)
            {
                //错误管理
                printf("ERROR: Tracking Planner Report To Client Error [Global Plan]!\n");
                return -1;
            }
            free(ptAgvMsgBody);
            //Send

            //开始记录数据
            StartLogOdom();

            //根据当前规划启动PID轨迹跟踪控制器
            tCurNode.tPose2D = tPoseOrg;
            tCurNode.tTwist2D.v = 0.0;
            tCurNode.tTwist2D.w = 0.0;
            s32Ret = poTrackingController->Start(poTrackingPlanner->ptRefPlanNode, 
            s32LinearPlanTotal, &tCurNode);
            
            if (s32Ret < 0)
            {
                //错误管理
                printf("ERROR: Tracking Control Start Error!\n");
                return -1;
            }
            
            //保存跟踪任务现场数据
            s32MissionFrom_ = ptAgvMsg->t_msg_header.u32_sq;
            //切换状态为轨迹跟踪RUN状态
            eChassisCtrlState = ChassisCtrlState::RUN;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateRun, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [STOP]->[RUN]\n");
        }
    }

    return 0;
}
/**
 * @name: 
 * @des: 
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisCtrlStateRun(TRobotMsg *ptAgvMsg)
{
    //导航任务执行中
    int s32Ret;
    TRobotMsg tAgvMsg;
    TRobotMsgHeader tAgvMsgHeader;
    uint8_t *ptAgvMsgBody;
    int s32MsgBodyOffset;
    const uint8_t *pMsgBody;
    TAgvTwist2D *ptAgvTwist2D;
    uint8_t u8Notification = 0; //到达目标位姿完成

    if (ptAgvMsg == NULL) //时间驱动
    {
        return 0;
    }

    else
    {
        //0 出错处理-----------------------------------------------------------
        if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_ERROR)
        {
            eChassisCtrlState = ChassisCtrlState::ERROR;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateError, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [RUN]->[ERROR]\n");
            return 0;
        }
        //1 路径跟踪任务完成 STOP命令来至底层伺服执行服务端
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_STOP &&
            ptAgvMsg->t_msg_header.u16_src == AGV_CHASSIS_MODULE)
        {
            //发送通知给任务请求的CLIENT
            tAgvMsgHeader.u16_src = 0x10; //最高位可以表示内部或外部消息
            tAgvMsgHeader.u16_dest = 1;
            tAgvMsgHeader.u16_class = AGV_CMD_C_CTRL;
            tAgvMsgHeader.u16_type = AGV_CMD_T_CTRL_SET_STAION | 0x8000;
            tAgvMsgHeader.u32_sq = 0;
            tAgvMsgHeader.s32_len = 1;
            RobotMsgInit(&tAgvMsg, tAgvMsgHeader, &u8Notification);
            //通知源请求端目标到达
            s32Ret = mq_send(s32MissionFrom_, (char *)&tAgvMsg, sizeof(TRobotMsg), 0);
            if (s32Ret < 0)
            {
                //错误管理
                printf("ERROR: Tracking Planner Report To Client Error [Target Achieve]!\n");
            }
            //结束记录数据
            StopLogOdom();
            eChassisCtrlState = ChassisCtrlState::STOP;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateStop, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [RUN]->[STOP]\n");


            return 0;
        }
        //2 路径跟踪任务暂停 STOP命令来至主控模块
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_STOP &&
            ptAgvMsg->t_msg_header.u16_src == AGV_CTRL_MODULE)
        {
            eChassisCtrlState = ChassisCtrlState::STOP;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateStop, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [RUN]->[STOP]\n");
            return 0;
        }

        //3 因监测到障碍物 导航任务挂起
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_SAFETY )
        {
            ptAgvMsgBody = RobotMsgGetBody(ptAgvMsg);
            assert(ptAgvMsgBody != NULL);
            s32SafeLevel_ = *(int *)(ptAgvMsgBody);
            if (s32SafeLevel_ != 0)
            {
                eChassisCtrlState = ChassisCtrlState::SUSPEND;
                fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateSuspend, 
                    this, std::placeholders::_1);
                printf("INFO: Chassis Control State [RUN]->[SUSPEND]\n");
            }
            return 0;
        }

        else
        {
            //本状态忽略的消息
        }
    }

    return 0;
}

/**
 * @name: 
 * @des: 
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisCtrlStateSuspend(TRobotMsg *ptAgvMsg)
{
    int s32Ret;
    uint8_t *ptAgvMsgBody;

    if (ptAgvMsg == NULL) //时间驱动
    {
        s32StateCount_ = s32StateCount_ + 1;
        if(s32StateCount_ >= s32StateDelay_) //状态延时到达
        {
            //TODO 重新规划？
            if (s32SafeLevel_ == 0) //重新判断是否继续挂起
            {
                eChassisCtrlState = ChassisCtrlState::RUN;
                fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateRun, 
                    this, std::placeholders::_1);
                printf("INFO: Chassis Control State [SUSPEND]->[RUN]\n");
                return 0;
            }
            else
            {
                s32StateDelay_ = 40; //延时2S后恢复
                s32StateCount_ = 0;
            }
        }

        return 0;
    }

    else
    {
        //0 出错处理-----------------------------------------------------------
        if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_ERROR)
        {
            eChassisCtrlState = ChassisCtrlState::ERROR;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateError, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [SUSPEND]->[ERROR]\n");
            return 0;
        }
        //2 恢复运行-----------------------------------------------------------
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_SAFETY)
        {
            ptAgvMsgBody = RobotMsgGetBody(ptAgvMsg);
            assert(ptAgvMsgBody != NULL);
            s32SafeLevel_ = *(int *)(ptAgvMsgBody);

            s32StateDelay_ = 40; //延时2S后恢复
            s32StateCount_ = 0;
        }
        //3 取消导航
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_STOP &&
            ptAgvMsg->t_msg_header.u16_src == AGV_CTRL_MODULE)
        {
            poTrackingPlanner->Reset();
            poTrackingController->Reset(1.0/AGV_MOTOR_CTRL_FREQ);
            eChassisCtrlState = ChassisCtrlState::STOP;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateStop, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [RUN]->[STOP]\n");
            return 0;
        }
        else
        {
            //忽略的消息
        }
    }

    //3 TODO 状态超时---------------------------------------------------------
}
/**
 * @name: 
 * @des: 
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisCtrlStateError(TRobotMsg *ptAgvMsg)
{
    //底层出错
    if (ptAgvMsg == NULL) //时间驱动
    {
        return 0;
    }
    else
    {
        if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_RESET)
        {
            //TODO执行RESET过程
        }

        //3 取消导航
        else if(ptAgvMsg->t_msg_header.u16_class == AGV_CHASSIS_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CHASSIS_T_STOP &&
            ptAgvMsg->t_msg_header.u16_src == AGV_CTRL_MODULE)
        {
            poTrackingPlanner->Reset();
            poTrackingController->Reset(1.0/AGV_MOTOR_CTRL_FREQ);
            eChassisCtrlState = ChassisCtrlState::STOP;
            fChassisCtrlStateFunc = std::bind(&AgvChassisCtrl::ChassisCtrlStateStop, 
                this, std::placeholders::_1);
            printf("INFO: Chassis Control State [RUN]->[STOP]\n");
            return 0;
        }
    }

    return 0;
}
/**
 * @name: 
 * @des: 
 * @param {TRobotMsg} *ptAgvMsg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisCtrlStateReset(TRobotMsg *ptAgvMsg)
{
    if (ptAgvMsg == NULL) //时间驱动
    {
        return 0;
    }

    return 0;
}


/**
 * @name: IsIdle
 * @des:  判断是否处于空闲状态
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
bool AgvChassisCtrl::IsIdle()
{
    return eChassisCtrlState == ChassisCtrlState::STOP;
}


/**
 * @name: ChassisCtrlPeriod
 * @des:  在实时控制周期中的行为 各状态下控制量给定方法
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisCtrlPeriod()
{
    int s32Ret;

    TRobotMsgHeader tAgvMsgHeader;
    TRobotMsg tAgvMsg, tAgvMsg2;

    double f64CurV, f64CurW; //当前速度
    TAgvPose2D tCurPose;
    TAgvMotionNode tCurNode, tRefNode;
    TAgvTwist2D tOutTwist;
    TAgvTwist2D tRemoteTwist;

    MotorController *poMC = po_motor_ctrl_;

    poAgvTf->GetGlobalPose2D(tCurPose); //由SLAM模块获取位姿信息/位姿预测信息
    poMC->GetTwistMsg(f64CurV, f64CurW); //由伺服控制模块获取速度信息
    tCurNode.tPose2D = tCurPose;         //构建当前位姿点结构
    tCurNode.tTwist2D.v = f64CurV;
    tCurNode.tTwist2D.w = f64CurW;

//Debug Log Pose-----------------------------------------------
    TLinuxTime tTimeNow;
    double fTimeNowMs;
    Rigid3d r3Amcl;
    this->poAgvTf->GetTransform("world", "base_link", r3Amcl);
    TAgvPose2D tAmclPose2D = ToAgvPose2D(r3Amcl);
//Debug Log Pose-----------------------------------------------

    //调用PID控制 计算控制量
    if (eChassisCtrlState == ChassisCtrlState::RUN)
    {
        s32Ret = poTrackingController->fTrakingStateFunc(&tCurNode, &tOutTwist, &tRefNode);
//Debug Log Pose-----------------------------------------------
        // if (bLogOdom)
        // {
        //     LinuxTimeNow(&tTimeNow);
        //     fTimeNowMs = (tTimeNow.tv_sec * 1e9 + tTimeNow.tv_nsec * 1.0)/1e6;
        //     fprintf(pLogOdom, "%f, Ref, %f, %f, %f, Carto, %f, %f, %f, Amcl, %f, %f, %f, Twist, %f, %f, Ctrl, %f, %f\n",
        //     fTimeNowMs,
        //     tRefNode.tPose2D.x, tRefNode.tPose2D.y, tRefNode.tPose2D.phi,
        //     tCurNode.tPose2D.x, tCurNode.tPose2D.y, tCurNode.tPose2D.phi,
        //     tAmclPose2D.x, tAmclPose2D.y, tAmclPose2D.phi,
        //     tCurNode.tTwist2D.v, tCurNode.tTwist2D.w,
        //     tOutTwist.v, tOutTwist.w);
        // }
//Debug Log Pose-----------------------------------------------

        if (s32Ret < 0) //跟踪过程出错 通知底盘服务模块出错(因为这里运行在伺服执行模块中)
        {   
            tAgvMsgHeader.u16_src = AGV_CHASSIS_MODULE;
            tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
            tAgvMsgHeader.u16_type = AGV_CHASSIS_T_ERROR;
            tAgvMsgHeader.s32_len = 0;
            RobotMsgInit(&tAgvMsg, tAgvMsgHeader, NULL);
            this->AsyncMsgPost(&tAgvMsg);  //发送通知消息
            poMC->SetCtrlCmd(0.0, 0.0);
        }
        else if(s32Ret == 1)  //跟踪过程结束 通知底盘服务路径跟踪任务完成
        {
            tAgvMsgHeader.u16_src = AGV_CHASSIS_MODULE;
            tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
            tAgvMsgHeader.u16_type = AGV_CHASSIS_T_STOP;
            tAgvMsgHeader.s32_len = 0;
            RobotMsgInit(&tAgvMsg, tAgvMsgHeader, NULL);
            this->AsyncMsgPost(&tAgvMsg);  //发送通知消息
            poMC->SetCtrlCmd(0.0, 0.0); 

            
            //调试退化 发送消息到SLAM以记录每一帧的退化情况
            TRobotMsg tAgvMsg2 = {0};
            tAgvMsgHeader.u16_class = AGV_LOG_C_CTRL;
            tAgvMsgHeader.u16_type = AGV_LOG_T_STOP;
            tAgvMsgHeader.s32_len = 0;
            RobotMsgInit(&tAgvMsg2, tAgvMsgHeader, NULL);
            pt_agv_ctrl_->poAgvSlammer->AsyncMsgPost(&tAgvMsg2);
            // Rigid3d r3Carto, r3Amcl;
            // poAgvTf->GetTransform("world", "base_link", r3Amcl);
            // poAgvTf->GetTransform("map", "base_link", r3Carto);
            // TAgvPose2D tAmcl2D = ToAgvPose2D(r3Amcl);
            // TAgvPose2D tCarto2D = ToAgvPose2D(r3Carto);
            // printf("WARN: Goal Reach AMCL-X[%f]-Y[%f]-PHI[%f]\n", tAmcl2D.x, tAmcl2D.y, tAmcl2D.phi);
            // printf("WARN: Goal Reach CART-X[%f]-Y[%f]-PHI[%f]\n", tCarto2D.x, tCarto2D.y, tCarto2D.phi);
        }

        else if(s32Ret == 2) //轨迹跟踪末端到达状态返回 这里预留调试使用
        {
            poMC->SetCtrlCmd(0.0, 0.0); 
        }

        else //路径跟踪控制正常执行 TODO定期汇报执行进度
        {
            poMC->SetCtrlCmd(tOutTwist.v, tOutTwist.w);
        }
    }

    //遥控情况下使用平滑控制器计算速度
    else if (eChassisCtrlState == ChassisCtrlState::STOP)
    {
        tRemoteTwist.v = remoteV;
        tRemoteTwist.w = remoteW;
        s32Ret = poRemoteController->RemoteCtrlPeriod(&tRemoteTwist, &tOutTwist);
        if (s32Ret == 0) //计算结果有效
        {
            poMC->SetCtrlCmd(tOutTwist.v, tOutTwist.w);
        }
        else
        {
            poMC->SetCtrlCmd(0.0, 0.0); 
        }

    }
    //挂起状态
    else if (eChassisCtrlState == ChassisCtrlState::SUSPEND)
    {
        poMC->SetCtrlCmd(0.0, 0.0);
    }

    //错误状态下始终输出控制量为0
    else if (eChassisCtrlState == ChassisCtrlState::ERROR)
    {
        poMC->SetCtrlCmd(0.0, 0.0);
    }

    else
    {
        poMC->SetCtrlCmd(0.0, 0.0);
    }

    return 0;
}

/**
 * @brief 里程计消息的周期处理
 * @param msg Twist
 * @return 
 */
double timeOdomDiff_ = 0;
double timeOdomPre_ = 0;

int AgvChassisCtrl::ChassisOdomPeriod()
{
    int s32Ret;
    double x, y, theta, v, w;
    MotorController *poMC = po_motor_ctrl_;

    //设置全局信息的速度字段
    poMC->GetTwistMsg(v, w);
    pt_agv_ctrl_->SetBaseInfoAgvVel(v, w);

    //IMPORTANT 不再依赖底盘驱动发布odom->baselink的变换 由SLAM模块统一发布所有位姿数据！！
    s32Ret = poMC->GetOdomMsg(x, y, theta, v, w);
    if (s32Ret < 0)
    {
        return -1; //里程计未更新 则不发布里程计
    }

    TLinuxTime tTimeNow; 
    LinuxTimeNow(&tTimeNow);
    double timeOdomNow = (tTimeNow.tv_sec * 1e9 + tTimeNow.tv_nsec)/1e6;
    timeOdomDiff_ = timeOdomNow - timeOdomPre_;
    timeOdomPre_ = timeOdomNow;
    // printf("INFO: OdomMessage Generate with stime[%f]\n", timeOdomDiff_);

    // 发布里程计数据至需要的模块
    TLinuxTime tLinuxTime;
    TAgvTimeStamp tAgvTimeStamp;
    TAgvOdomMsg tAgvOdomMsg;
    const Eigen::AngleAxisd yaw_angle(theta, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quaternion(yaw_angle);

    LinuxTimeNow(&tLinuxTime);
    tAgvTimeStamp.sec = tLinuxTime.tv_sec;
    tAgvTimeStamp.nsec = tLinuxTime.tv_nsec;
    tAgvOdomMsg.header.stamp = tAgvTimeStamp;
    snprintf(tAgvOdomMsg.header.frame_id, FRAME_IDS_MAX, "odom");
    snprintf(tAgvOdomMsg.child_frame_id, FRAME_IDS_MAX, "base_link");
    // // tAgvOdomMsg.header.frame_id = "odom";
    // // tAgvOdomMsg.child_frame_id = "base_link";
    tAgvOdomMsg.pose.position.x = x;
    tAgvOdomMsg.pose.position.y = y;
    tAgvOdomMsg.pose.position.z = 0.0;
    tAgvOdomMsg.pose.orientation.w = quaternion.w();
    tAgvOdomMsg.pose.orientation.x = quaternion.x();
    tAgvOdomMsg.pose.orientation.y = quaternion.y();
    tAgvOdomMsg.pose.orientation.z = quaternion.z();

    // printf("WARN: Odom Raw X[%f], Y[%f]\n", tAgvOdomMsg.pose.position.x, tAgvOdomMsg.pose.position.y);

    tAgvOdomMsg.twist.linear.x = v;
    tAgvOdomMsg.twist.linear.y = 0.0;
    tAgvOdomMsg.twist.linear.z = 0.0;
    // //printf("OdomInfo:[%f]\n",linear_speed);

    tAgvOdomMsg.twist.angular.x = 0.0;
    tAgvOdomMsg.twist.angular.y = 0.0;
    tAgvOdomMsg.twist.angular.z = w;

    poAgvTf->AddOdomToExtrapolator(tAgvOdomMsg);
    //TODO其他模块使用ODOM消息
    //消息体
    //发送消息至SLAM模块
    TRobotMsgHeader tAgvMsgHeader;
    TRobotMsg tAgvMsg;
    tAgvMsgHeader.u16_class = AGV_SLAM_C_SENSOR;
    tAgvMsgHeader.u16_type = AGV_SLAM_T_SENSOR_ODOM;
    tAgvMsgHeader.s32_len = sizeof(TAgvOdomMsg);
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, (uint8_t *)&tAgvOdomMsg);
    s32Ret = pt_agv_ctrl_->poAgvSlammer->AsyncMsgPost(&tAgvMsg);
    if (s32Ret < 0)
    {
        printf("ERROR: Odom Post To Slam Failed\n");
    }

    //发送消息至ROS调试端
    TRobotMsgHeader tRosMsgHeader;
    TRobotMsg tRosMsg;
    tRosMsgHeader.u16_class = AGV_ROS_C_CTRL;
    tRosMsgHeader.u16_type = AGV_ROS_T_ODOM_INFO;
    tRosMsgHeader.s32_len = sizeof(TAgvOdomMsg);
    RobotMsgInit(&tRosMsg, tRosMsgHeader, (uint8_t *)&tAgvOdomMsg);
    pt_agv_ctrl_->poAgvRosSvr_->AsyncMsgPost(&tRosMsg);

    return 0;
}



/**
 * @name: ChassisReadyReport
 * @des:  底盘准备就绪通知
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisReadyReport()
{
    TRobotMsgHeader tAgvMsgHeader;
    TRobotMsg tAgvMsg;

    tAgvMsgHeader.u16_src = AGV_CHASSIS_MODULE;
    tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_CHASSIS_T_READY;
    tAgvMsgHeader.s32_len = 0;
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, NULL);
    this->AsyncMsgPost(&tAgvMsg);  //发送通知消息

    return 0;
}

/**
 * @name: ChassisErrorReport
 * @des:  底盘出错通知
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::ChassisErrorReport()
{
    TRobotMsgHeader tAgvMsgHeader;
    TRobotMsg tAgvMsg;

    tAgvMsgHeader.u16_src = AGV_CHASSIS_MODULE;
    tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_CHASSIS_T_ERROR;
    tAgvMsgHeader.s32_len = 0;
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, NULL);
    this->AsyncMsgPost(&tAgvMsg);  //发送通知消息

    return 0;
}



/**
 * @name: StartLogOdom
 * @des:  开始记录数据
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::StartLogOdom()
{
    // char dirLogOdom[128];
    // char timestr[256];
    // time_t t = time(0);
    // strftime(timestr, 256, "%Y%m%d%H%M%S", localtime(&t));
    // sprintf(dirLogOdom, "/home/%s/record/cpose_record%s.txt", getlogin(), timestr);
    // pLogOdom = fopen(dirLogOdom, "w+");
    // bLogOdom = true;

    return 0;
}

/**
 * @name: StopLogOdom
 * @des:  结束记录数据
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvChassisCtrl::StopLogOdom()
{
    // if (bLogOdom)
    // {
    //     bLogOdom = false;
    //     fclose(pLogOdom);
    // }
    return 0;
}
#include <stdlib.h>
#include <string.h>
#include <mqueue.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <boost/thread/recursive_mutex.hpp>

#include "agv_type.h"
#include "agv_cmd.h"
#include "agv_log.h"
#include "agv_map_ctrl.h"
#include "agv_tcp_svr.h"
#include "agv_tcp_ctrl.h"

#include "AgvCtrl.h"

#include "agv_cmd.h"
#include "agv_map_ctrl.h"
#include "agv_chassis_ctrl.h"
#include "AgvSickScanDriver.h"
#include "SlamUtil.h"


/**
 * @brief AsyncMsgHandler异步消息处理
 * @param ptAgvMsg
 * @return void
 */
AgvCtrl::AgvCtrl(/* args */)
{
    //初始化全局资源
    lockBaseInfo_ = new boost::recursive_mutex();
    lockMapInfo_ = new boost::recursive_mutex();
    //系统接口变量
    s32AgvCtrlFreq = 50;
    bHasInited = false;
    bHasStarted = false;

    ctrlState = E_AGV_CTRL_STATE::INIT;

    //底盘遥控相关
    mani_watch_dog = 0; //默认发布周期为40ms 则超时周期为200ms 待调整
    mani_linear_vel = 0;
    mani_angular_vel = 0;
    mani_linear_vel_def = 0.35; //默认遥控线速度
    mani_angular_vel_def = 0.5; //默认遥控角速度
}

/**
 * @brief AsyncMsgHandler异步消息处理
 * @param ptAgvMsg
 * @return void
 */
AgvCtrl::~AgvCtrl()
{
    delete lockBaseInfo_;
    delete lockMapInfo_;
}

/**
 * @brief AsyncMsgHandler异步消息处理
 * @param ptAgvMsg
 * @return void
 */
int AgvCtrl::Init()
{
    int s32Ret;

    //加载基本配置
    s32Ret = read_agv_config_from_json(this);
    if (s32Ret != 0)
    {
        return -1;
    }

    //初始化地图管理模块
    poAgvMapCtrl = new CAGVMapCtrl(this);
    if (0 != poAgvMapCtrl->Init())
    {
        return -1;
    }

    //初始化tf
    poAgvTf = new AgvTf();
    if (0 != poAgvTf->Init())
    {
        return -1;
    }

    //初始化SLAM模块
    poAgvSlammer = new AgvSlammer(this);
    if (0 != poAgvSlammer->Init())
    {
        return -1;
    }

    // poAgvRvdCtrl = new AgvRvdCtrl(this, poAgvTf);
    // if (0 != poAgvRvdCtrl->Init())
    // {
    //     return -1;
    // }

    //初始化底盘
    poAgvChassisCtrl = new AgvChassisCtrl(this);
    if (0 != poAgvChassisCtrl->Init())
    {
        return -1;
    }

    //初始化传感器模块
    poAgvSickScanDriver = new AgvSickScanDriver(this);
    if (0 != poAgvSickScanDriver->Init())
    {
        return -1;
    }

    poAgvRosSvr_ = new AgvRosSvr(this);


    //初始化网络广播服务模块
    s32Ret = udp_svr_ctrl_init(this);
    if (s32Ret != 0)
    {
        return -1;
    }

    //初始化TCP网络连接管理
    s32Ret = robot_tcp_svr_init(&t_agv_tcp_svr , 6, this);
    if (s32Ret != 0)
    {
        return -1;
    }

     //TODO 集成进系统 标准消息定义
    struct mq_attr t_mq_attr = {0};
	t_mq_attr.mq_flags = 0; //Block Mode
	t_mq_attr.mq_msgsize = sizeof(TRobotMsg); //todo intagerate with total project
	t_mq_attr.mq_maxmsg = 8;

    mq_unlink("/q_agv_ctrl");
    mode_t omask;
    omask = umask(0); //返回之前的文件权限默认值，同时修改默认的umask数值为0，这样后面设置文件权限的时候，umask值无影响了（创建文件时，文件权限的计算方式是：umask取反 & 要设定的文件权限）
    this->qAgvCtrlSvr = mq_open("/q_agv_ctrl", O_RDWR | O_CREAT | O_NONBLOCK , (S_IRWXU | S_IRWXG | S_IRWXO) /* 777 其他用户可修改*/,  &t_mq_attr);
    umask(omask);
    if (this->qAgvCtrlSvr < 0)
    {
        printf("ERROR: q_agv_ctrl create failed with error:%s\n",strerror(errno));
    }
    assert(this->qAgvCtrlSvr >= 0);

    bHasInited = true;
    printf("INFO: AgvCtrl Inited!\n");

    return 0;
}

/**
 * @brief AsyncMsgHandler异步消息处理
 * @param 
 * @return void
 */
int AgvCtrl::Start()
{
    int s32Ret;
    
    //AgvSlammer模块
    //AgvMapAssembler 移动到SLAM模块内部
    s32Ret = poAgvSlammer->Start();
    if (s32Ret != 0)
    {
        return -1;
    }

    //AGV底盘驱动模块
    s32Ret = poAgvChassisCtrl->Start();
    if (s32Ret != 0)
    {
        return -1;
    }
    //传感器管理模块
    s32Ret = poAgvSickScanDriver->Start();
    if (s32Ret != 0)
    {
        return -1;
    }
    
    // //视觉定位与对接模块
    // s32Ret = poAgvRvdCtrl->Start();
    // if (s32Ret != 0)
    // {
    //     return -1;
    // }

    //通信模块
    s32Ret = agv_udp_svr_run(this);
    if (s32Ret != 0)
    {
        return -1;
    }
    s32Ret = robot_tcp_svr_run(&t_agv_tcp_svr);
    if (s32Ret != 0)
    {
        return -1;
    }

    //主框架启动
    if(!bHasInited || bHasStarted)
    {
        printf("ERROR: AgvCtrl Start failed!\n");
        return -1;
    }
    bHasStarted = true;
    ctrlState = E_AGV_CTRL_STATE::NAVI;

    pthread_attr_t pthread_attr;
    struct sched_param sched_param;
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 30;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);
    s32Ret = pthread_create(&this->thAgvCtrlSvr, &pthread_attr, LoopAgvCtrlSvrStatic, (void *)(this));
    assert(s32Ret >= 0);
    
    printf("INFO: AgvCtrl Run!\n");

    return 0;
}

/**
 * @brief LoopAgvCtrlSvrStatic工作线程入口
 * @param arg 当前类指针 
 * @return void
 */
void *AgvCtrl::LoopAgvCtrlSvrStatic(void *arg)
{
    AgvCtrl *poAgvCtrl = (AgvCtrl *)arg;
    poAgvCtrl->LoopAgvCtrlSvr();
    return NULL;
}

/**
 * @brief LoopAgvCtrlSvr工作线程实体
 * @param 
 * @return int
 */
int AgvCtrl::LoopAgvCtrlSvr()
{
    int s32Ret;
    TRobotMsg tAgvMsg = {0};
    
    struct timeval t_tmptv;
	fd_set t_fd_read_set;
    int s32_fd_max = qAgvCtrlSvr + 1;
    int s32UsFromHz = floor(1000000.0/s32AgvCtrlFreq);
    t_tmptv.tv_sec = 0;
    t_tmptv.tv_usec = s32UsFromHz;

    while(bHasStarted)
    {
        FD_ZERO(&t_fd_read_set);
        FD_SET(qAgvCtrlSvr, &t_fd_read_set);
        s32Ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, &t_tmptv);
        if (s32Ret < 0)
        {
            printf("ERROR: Agv Ctrl Loop Error %s\n", strerror(errno));
            sleep(1);
            continue;
        }
        //周期事件处理
        else if (s32Ret == 0)
        {
            t_tmptv.tv_sec = 0;
            t_tmptv.tv_usec = s32UsFromHz;
            AgvCtrlRoutine();
            continue;
        }
        //异步SDO响应 接收到SDO
        if (FD_ISSET(qAgvCtrlSvr, &t_fd_read_set))
        {
            memset(&tAgvMsg, 0, sizeof(tAgvMsg));
            s32Ret = mq_receive(qAgvCtrlSvr, (char *)&tAgvMsg,
                        sizeof(tAgvMsg), NULL);
            if (s32Ret < 0)
            {
                printf("ERROR: Agv Ctrl Recv Msg: %s", strerror(errno));
                continue;
            }
            AsyncMsgHandler(&tAgvMsg);
            RobotMsgRelease(&tAgvMsg);
        }

    }

    return s32Ret;
}

/**
 * @brief 异步消息处理函数
 * @param ptAgvMsg 异步消息指针
 * @return 0 成功 -1失败
 */
int AgvCtrl::AsyncMsgHandler(const TRobotMsg *ptAgvMsg)
{
    return 0;
}


/**
 * @brief 设置AGV基本信息 可重入
 * @param tBaseInfo
 * @return 0 成功 -1失败
 */
int AgvCtrl::SetBaseInfo(TBaseInfo &tBaseInfo)
{
    boost::unique_lock<boost::recursive_mutex> lock(*lockBaseInfo_);
    tBaseInfo_ = tBaseInfo;
    return 0;
}

/**
 * @brief 设置AGV全局位姿 可重入
 * @param X X
 * @param Y Y
 * @param A Theta
 * @return 0 成功 -1失败
 */
int AgvCtrl::SetBaseInfoAgvPos(double X, double Y, double A)
{
    boost::unique_lock<boost::recursive_mutex> lock(*lockBaseInfo_);
    tBaseInfo_.tPt.dX = X;
    tBaseInfo_.tPt.dY = Y;
    tBaseInfo_.dAngle = A;
    return 0;
}

/**
 * @brief 设置AGV全局位姿 可重入
 * @param Lv 线速度
 * @param Av 角速度
 * @return 0 成功 -1失败
 */
int AgvCtrl::SetBaseInfoAgvVel(double Lv, double Av)
{
    //boost::unique_lock<boost::recursive_mutex> lock(*lockBaseInfo_);
    tBaseInfo_.dLineSpeed = Lv;
    tBaseInfo_.dAngularSpeed = Av;
    return 0;
}

/**
 * @brief 获取AGV基本信息 可重入
 * @param tBaseInfo
 * @return 0 成功 -1失败
 */
int AgvCtrl::GetBaseInfo(TBaseInfo &tBaseInfo)
{
    boost::unique_lock<boost::recursive_mutex> lock(*lockBaseInfo_);
    tBaseInfo = tBaseInfo_;
    return 0;
}

/**
 * @brief 获取AGV当前速度
 * @param Lv 当前线速度返回
 * @param Av 当前角速度返回
 * @return 0 成功 -1失败
 */
int AgvCtrl::GetBaseInfoAgvVel(double &Lv, double &Av)
{
    //boost::unique_lock<boost::recursive_mutex> lock(*lockBaseInfo_);
    Lv = tBaseInfo_.dLineSpeed;
    Av = tBaseInfo_.dAngularSpeed;
    return 0;
}

/**
 * @brief 更新当前地图信息  地图保存与更新效率待优化 vector到char*转化
 * @param tMapInfo
 * @return 0 成功 -1失败
 */
int AgvCtrl::SetMapInfo(const TOccupancyGrid &tOccupancyGrid)
{
    int i;
    int map_size = 0;

    map_size = tOccupancyGrid.height * tOccupancyGrid.width;
    assert(map_size < (2000 * 2000)); //todo

    boost::unique_lock<boost::recursive_mutex> lock(*lockMapInfo_);
    tMapInfo_.dResolution = tOccupancyGrid.resolution;
    tMapInfo_.iHeight = tOccupancyGrid.height;
    tMapInfo_.iWidth = tOccupancyGrid.width;
    tMapInfo_.dOriginXOffset = tOccupancyGrid.origin.position.x;
    tMapInfo_.dOriginYOffset = tOccupancyGrid.origin.position.y;
    memcpy(tMapInfo_.pCostMapData, tOccupancyGrid.data.data(), map_size);

    MapValid_ = true;
    return 0;
}
/**
 * @name: SetMapInfo
 * @des:  设置全局地图信息 栅格地图+拓扑地图
 * @param {TMapInfo} &tMapInfo
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvCtrl::SetMapInfo(const TMapInfo &tMapInfo)
{
    boost::unique_lock<boost::recursive_mutex> lock(*lockMapInfo_);
    tMapInfo_ = tMapInfo;
    return 0;
}
/**
 * @name: GetMapInfo
 * @des:  获取全局地图信息
 * @param {TMapInfo} &tMapInfo
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvCtrl::GetMapInfo(TMapInfo &tMapInfo)
{
    boost::unique_lock<boost::recursive_mutex> lock(*lockMapInfo_);
    tMapInfo = tMapInfo_;
    return 0;
}

/**
 * @name: GetMapInfo
 * @des:  获取全局地图信息
 * @param {TOccupancyGrid} &tOccupancyGrid
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvCtrl::GetMapInfo(TOccupancyGrid &tOccupancyGrid)
{
    int i;
    int map_size = 0;
    
    boost::unique_lock<boost::recursive_mutex> lock(*lockMapInfo_);
    tOccupancyGrid.resolution = tMapInfo_.dResolution;

    tOccupancyGrid.height = tMapInfo_.iHeight;
    tOccupancyGrid.width = tMapInfo_.iWidth;
    tOccupancyGrid.origin.position.x = tMapInfo_.dOriginXOffset;
    tOccupancyGrid.origin.position.y = tMapInfo_.dOriginYOffset;

    map_size = tMapInfo_.iHeight * tMapInfo_.iWidth;
    assert(map_size < (2000 * 2000)); //todo
    tOccupancyGrid.data.resize(map_size);
    for (i = 0; i < map_size; i++)
    {
        tOccupancyGrid.data[i] = tMapInfo_.pCostMapData[i];
    }

    return 0;
}

/**
 * @brief AgvCtrlRoutine 周期事件处理方法
 * @param 
 * @return int
 */
int AgvCtrl::UpdateBaseInfo()
{
    int s32Ret;

    Rigid3d transform;

    //TODO 统一调用接口
    // poAgvTf->tf_map2odom_.TrimTfBuffer();

    s32Ret = poAgvTf->GetTransform("map", "base_link", transform);
    if (s32Ret != 0)
    {
        return -1;
    }
    
    TBaseInfo tBaseInfo;
    GetBaseInfo(tBaseInfo);
    tBaseInfo.tPt.dX = transform.translation().x();
    tBaseInfo.tPt.dY = transform.translation().y();
    tBaseInfo.dAngle = GetYaw(transform.rotation());
    //tBaseInfo_ = tBaseInfo;
    SetBaseInfo(tBaseInfo);

    CheckPose2DViolentChange();

    return 0;
}

/**
 * @brief AgvCtrlRoutine 周期事件处理方法
 * @param 
 * @return int
 */
int AgvCtrl::AgvCtrlRoutine()
{
    int s32_ret;

    st_robot_tcp_svr *pt_tcp_svr = &t_agv_tcp_svr;
    st_tcp_connect_info *pt_connect_info;
    st_tcp_period_event * pt_event;

    UpdateBaseInfo();

    //mutex work protect---遍历连接的工作队列
    pthread_mutex_lock(&pt_tcp_svr->mutex_connect);
    LIST_FOREACH(pt_connect_info, &pt_tcp_svr->lh_connect_work, node)
    {
        pthread_mutex_lock(&pt_connect_info->mutex_event);
        LIST_FOREACH(pt_event, &pt_connect_info->lh_event, node)
        {
            s32_ret = AgvCtrlProcess(pt_event, pt_connect_info);
        }
        pthread_mutex_unlock(&pt_connect_info->mutex_event);
    }
    pthread_mutex_unlock(&pt_tcp_svr->mutex_connect);

///////////////////////////////////////////////////////////////////
    //发送消息至ROS调试端
    // uint8_t pPoseBuf[2 * sizeof(TAgvPose2D)];
    // TAgvPose2D *pPoseStr = (TAgvPose2D *)pPoseBuf;
    // //Local Pose
    // TAgvPose2D tLocalPose2D = {0, 0, 0};
    // memcpy(pPoseStr, &tLocalPose2D, sizeof(TAgvPose2D));
    // //Global Pose
    // Rigid3d r3M2O;

    // TAgvPose2D tMap2Odom2D = {5 , 5,  PI/6};
    // pPoseStr = pPoseStr + 1;
    // memcpy(pPoseStr, &tMap2Odom2D, sizeof(TAgvPose2D));
    // //构造消息
    // TRobotMsgHeader tRosMsgHeader;
    // TRobotMsg tRosMsg;
    // tRosMsgHeader.u16_class = AGV_ROS_C_CTRL;
    // tRosMsgHeader.u16_type = AGV_ROS_T_POSE_INFO;
    // tRosMsgHeader.s32_len = 2 * sizeof(TAgvPose2D);
    // RobotMsgInit(&tRosMsg, tRosMsgHeader, pPoseBuf);
    // poAgvRosSvr_->AsyncMsgPost(&tRosMsg);

///////////////////////////////////////////////////////////////////

    return 0;
}

/**
 * @brief AgvCtrlRoutine 周期事件处理方法
 * @param 
 * @return int
 */
int AgvCtrl::AgvCtrlProcess(st_tcp_period_event *pt_event, 
               st_tcp_connect_info *pt_connect_info)
{
    int s32_ret = 0;

    switch (pt_event->e_type)
    {
        case AGV_EVENT_SUB_BASE_INFO:
            s32_ret = EventSubBaseInfoProcess(pt_event, pt_connect_info);
        break;

        case AGV_EVENT_SUB_MAP_INFO:
            s32_ret = EventSubMapInfoProcess(pt_event, pt_connect_info);
        break;

        case AGV_EVENT_PUB_VEL_CMD:
            s32_ret = EventPubVelCmdProcess(pt_event, pt_connect_info);
        break;


        default: break;
    }

    return s32_ret;
}



/**
 * @brief EventSubBaseInfoProcess 周期性汇报AGV基础信息
 * @param 
 * @return int
 */
int AgvCtrl::EventSubBaseInfoProcess(st_tcp_period_event *pt_event, 
               st_tcp_connect_info *pt_connect_info)
{
    int s32_ret;
    int offset;
    TRobotMsg t_agv_msg = {0};

    if (++(pt_event->count)  < pt_event->period)
    {
        return 0;
    }
    pt_event->count = 0;

    t_agv_msg.t_msg_header.u16_src = 0x10;
    t_agv_msg.t_msg_header.u16_dest = 1;
    t_agv_msg.t_msg_header.u16_class = 0;
    t_agv_msg.t_msg_header.u16_type = 2;
    t_agv_msg.t_msg_header.u32_sq = 0;

    TBaseInfo tBaseInfo;
    GetBaseInfo(tBaseInfo);

    offset = 0;
    //Battery
    t_agv_msg.t_msg_body.sbody[offset] = tBaseInfo.iElectricity;
    offset = offset + 1;
    //CPU
    t_agv_msg.t_msg_body.sbody[offset] = tBaseInfo.iCPU;
    offset = offset + 1;
    //MEM
    t_agv_msg.t_msg_body.sbody[offset] = tBaseInfo.iRAM;
    offset = offset + 1;
    //RES
    t_agv_msg.t_msg_body.sbody[offset] = 0; //res byte
    offset = offset + 1;

    //CONF
    float fConfidence = (float)tBaseInfo.dConfidence;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fConfidence, sizeof(float));
    offset = offset + sizeof(float);
    //L-Vel
    float fLineSpeed = (float)tBaseInfo.dLineSpeed;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fLineSpeed, sizeof(float));
    offset = offset + sizeof(float);
    //A-Vel
    float fAngularSpeed = (float)tBaseInfo.dAngularSpeed;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fAngularSpeed, sizeof(float));
    offset = offset + sizeof(float);
    //POSE-X
    float fX = (float)tBaseInfo.tPt.dX;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fX, sizeof(float));
    offset = offset + sizeof(float);
    //POSE-Y
    float fY = (float)tBaseInfo.tPt.dY;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fY, sizeof(float));
    offset = offset + sizeof(float);
    //POSE-Angle
    float fAngle = (float)tBaseInfo.dAngle;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fAngle, sizeof(float));
    offset = offset + sizeof(float);

    t_agv_msg.t_msg_header.s32_len = offset;
    s32_ret = mq_send(pt_connect_info->q_svr_handler, (char *)&t_agv_msg,
            sizeof(TRobotMsg), 0);
    
    return s32_ret;
}


/**
 * @brief EventSubMapInfoProcess 周期性汇报当前扫图地图信息
 * @param 
 * @return int
 */
int AgvCtrl::EventSubMapInfoProcess(st_tcp_period_event *pt_event, 
               st_tcp_connect_info *pt_connect_info)
{
    int s32_ret;
    int offset;
    TRobotMsg t_agv_msg = {0};

    int map_size;
    int msg_body_size;

    if (++(pt_event->count)  < pt_event->period)
    {
        return 0;
    }

    pt_event->count = 0;

    t_agv_msg.t_msg_header.u16_src = 0x10;
    t_agv_msg.t_msg_header.u16_dest = 1;
    t_agv_msg.t_msg_header.u16_class = AGV_CMD_C_FILE;
    t_agv_msg.t_msg_header.u16_type = AGV_CMD_T_MAP_REPORT;
    t_agv_msg.t_msg_header.u32_sq = 0;

    //nomap just return
    if (MapValid_ == false)
    {
        printf("map invalid!\n");
        return 0;
    }
    
    //TODO 加锁处理？
    map_size = tMapInfo_.iHeight * tMapInfo_.iWidth;

    printf("map size %d\n", map_size);

    msg_body_size = 20 + map_size;
    assert(msg_body_size >= 32);
    t_agv_msg.res_flag = 1;
    t_agv_msg.t_msg_body.lbody = (uint8_t *)malloc(msg_body_size);
    if (t_agv_msg.t_msg_body.lbody == NULL)
    {
        return -1;
    }

    offset = 0;
    uint8_t *pbody = t_agv_msg.t_msg_body.lbody;
  
    //Resolution
    float fResolution = (float)tMapInfo_.dResolution;
    memcpy(pbody, &fResolution, sizeof(float));
    pbody = pbody + sizeof(float);
    offset = offset + sizeof(float);
    //Map H
    memcpy(pbody, &tMapInfo_.iHeight, sizeof(int));
    pbody = pbody + sizeof(int);
    offset = offset + sizeof(int);
    //Map W
    memcpy(pbody, &tMapInfo_.iWidth, sizeof(int));
    pbody = pbody + sizeof(int);
    offset = offset + sizeof(int);

    float fOriginXOffset = (float)tMapInfo_.dOriginXOffset;
    memcpy(pbody, &fOriginXOffset, sizeof(float));
    pbody = pbody + sizeof(float);
    offset = offset + sizeof(float);

    float fOriginYOffset = (float)tMapInfo_.dOriginYOffset;
    memcpy(pbody, &fOriginYOffset, sizeof(float));
    pbody = pbody + sizeof(float);
    offset = offset + sizeof(float);

    memcpy(pbody, tMapInfo_.pCostMapData, map_size);
    offset = offset + map_size;

    t_agv_msg.t_msg_header.s32_len = offset;
    s32_ret = mq_send(pt_connect_info->q_svr_handler, (char *)&t_agv_msg,
            sizeof(TRobotMsg), 0);

    if (s32_ret < 0)
    {
        printf("WARN: Agv Map report lost %d", errno);
        return -1;
    }
    return 0;
}


/**
 * @brief EventPubVelCmdProcess AGV控制结构周期周期发布遥控指令
 * @param 
 * @return int
 */
int AgvCtrl::EventPubVelCmdProcess(st_tcp_period_event *pt_event, 
               st_tcp_connect_info *pt_connect_info)
{
    int s32_ret;
    int offset;
    TRobotMsg t_agv_msg = {0};

    if (++(pt_event->count)  < pt_event->period)
    {
        return 0;
    }
    pt_event->count = 0;

    //上位机通信不稳定 规定事件内未能收到有效的遥控速度指令
    //上位机控制有效 
    //WARNING: 狗 线速度 角速度均为4字节自然对齐 多线程操作理论上不会破坏单一数据
    //这里为了保证效率 不予以加锁
    if (mani_watch_dog == 0)
    {
        PubManiCmdVel(0, 0);
    }
    else
    {
        mani_watch_dog--;
        PubManiCmdVel(mani_linear_vel, mani_angular_vel);
    }
    
    return 0;
}


/**
 * @brief 建图与定位切换
 * @param arg 建图->定位 true  定位->建图 false
 * @return int
 */
int AgvCtrl::CallChangeModeSvr(bool arg)
{
    int ret;
    TRobotMsgHeader tAgvMsgHeader = {0};
    TRobotMsg tAgvMsg = {0};

    if(arg && ctrlState == E_AGV_CTRL_STATE::SLAM)//建图->定位 true
    {
        //设置当前为SLAM2NAVI状态 等待地图最终优化完成
        //ctrlState = E_AGV_CTRL_STATE::SLAM2NAVI;
        ctrlState = E_AGV_CTRL_STATE::NAVI;

        //发送消息通知SLAM模块结束建图
        tAgvMsgHeader.u16_class = AGV_CMD_C_CTRL;
        tAgvMsgHeader.u16_type = AGV_CMD_T_LOCALIZATION_RUN;
        tAgvMsgHeader.s32_len = 0;
        RobotMsgInit(&tAgvMsg, tAgvMsgHeader, NULL);
        poAgvSlammer->AsyncMsgPost(&tAgvMsg);
        poAgvSlammer->pAgvMapAssembler_->AsyncMsgPost(&tAgvMsg);
    }
    else if(!arg && ctrlState == E_AGV_CTRL_STATE::NAVI)//定位->建图 false
    {
        //设置当前为NAVI2NAVI状态 等待SLAM资源准备完毕进入SLAM状态
        ctrlState = E_AGV_CTRL_STATE::SLAM;

        tAgvMsgHeader.u16_class = AGV_CMD_C_CTRL;
        tAgvMsgHeader.u16_type = AGV_CMD_T_SLAM_RUN;
        tAgvMsgHeader.s32_len = 0;
        RobotMsgInit(&tAgvMsg, tAgvMsgHeader, NULL);
        poAgvSlammer->AsyncMsgPost(&tAgvMsg);
        poAgvSlammer->pAgvMapAssembler_->AsyncMsgPost(&tAgvMsg);
    }

    return 0;
}

/**
 * @name: CallMoveToTarget
 * @des:  调用PID方法实现的路径跟踪请求 
 * TODO导航请求要能描述跟踪任务 路径形态 等 以使用PID控制跟踪贝塞尔曲线或圆弧
 * 目前仅使用直线导航任务
 * @param {st_tcp_connect_info} *ptConnectInfo
 * @param {TAgvPose2D} tTargetPose
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.02
 */
int AgvCtrl::CallMoveToTarget(st_tcp_connect_info *ptConnectInfo, TAgvPose2D tTargetPose)
{
    int s32Ret;
    TRobotMsgHeader tAgvMsgHeader = {0};
    TRobotMsg tAgvMsg = {0};

    //不满足导航条件
    if (ctrlState != E_AGV_CTRL_STATE::NAVI || !poAgvChassisCtrl->IsIdle())
    {
        printf("ERROR: Navigation Condition Not Satisfied!\n");
        return -1;
    }

    //下发导航任务给底盘控制模块
    tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
    tAgvMsgHeader.u16_src = AGV_COMM_MODULE;
    tAgvMsgHeader.u16_type = AGV_CHASSIS_T_RUN;
    tAgvMsgHeader.u32_sq = ptConnectInfo->q_svr_handler; //AGV-PLanner模块需要记录命令的源
    tAgvMsgHeader.s32_len = sizeof(TAgvPose2D);
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, (uint8_t *)&tTargetPose);

    poAgvChassisCtrl->AsyncMsgPost(&tAgvMsg);

//调试退化 发送消息到SLAM以记录每一帧的退化情况
    TRobotMsg tAgvMsg2 = {0};
    tAgvMsgHeader.u16_class = AGV_LOG_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_LOG_T_START;
    tAgvMsgHeader.s32_len = 0;
    RobotMsgInit(&tAgvMsg2, tAgvMsgHeader, NULL);
    poAgvSlammer->AsyncMsgPost(&tAgvMsg2);


    return 0;
}
/**
 * @name: CallCancalTarget
 * @des:  撤销当前导航任务
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.02
 */
int AgvCtrl::CallCancalTarget()
{
    int s32Ret;
    TRobotMsgHeader tAgvMsgHeader = {0};
    TRobotMsg tAgvMsg = {0};

    tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_CHASSIS_T_STOP;
    tAgvMsgHeader.u16_src = AGV_CTRL_MODULE;
    tAgvMsgHeader.s32_len = 0;
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, NULL);

    //通知Planner模块AGV停止
    poAgvChassisCtrl->AsyncMsgPost(&tAgvMsg);


//调试退化 发送消息到SLAM以记录每一帧的退化情况
    TRobotMsg tAgvMsg2 = {0};
    tAgvMsgHeader.u16_class = AGV_LOG_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_LOG_T_STOP;
    tAgvMsgHeader.s32_len = 0;
    RobotMsgInit(&tAgvMsg2, tAgvMsgHeader, NULL);
    poAgvSlammer->AsyncMsgPost(&tAgvMsg2);

    return 0;
}


/**
 * @brief 切换底盘状态 遥控/自主导航
 * @version V1.0
 * @param 
 * @return 0 成功 -1失败
 */
bool AgvCtrl::RequestChassisMode(uint8_t mode_req)
{
    TRobotMsg tAgvMsg = {0};
    TRobotMsgHeader tAgvMsgHeader = {0};
    uint8_t u8Mode = mode_req;
    //消息体
    tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_CHASSIS_T_MODE_CAHNGE;
    tAgvMsgHeader.s32_len = sizeof(uint8_t);
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, (uint8_t *)&u8Mode);
    //发送消息
    poAgvChassisCtrl->AsyncMsgPost(&tAgvMsg);

    return true;
}

/**
 * @brief 当前遥控速度发送至底盘模块
 * @version V1.0
 * @param 
 * @return 0 成功 -1失败
 */
int AgvCtrl::PubManiCmdVel(float linear_vel, float angular_vel)
{
    int s32Ret;
    TRobotMsg tAgvMsg = {0};
    TRobotMsgHeader tAgvMsgHeader = {0};

    //针对2D空间线速度和角速度 用TAgvTwist2D结构 减少消息创建的开销
    TAgvTwist2D tAgvTwist2D;
    tAgvTwist2D.v = linear_vel;
    tAgvTwist2D.w = angular_vel;
    //消息体
    tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
    tAgvMsgHeader.u16_type = AGV_CHASSIS_T_VEL;
    tAgvMsgHeader.s32_len = sizeof(TAgvTwist2D);
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, (uint8_t *)&tAgvTwist2D);
    poAgvChassisCtrl->AsyncMsgPost(&tAgvMsg);

    return 0;
}

/**
 * @brief 通信稳定的情况下 看门口复位
 * @version V1.0
 * @param 
 * @return 0 成功 -1失败
 */
void AgvCtrl::ManiWatchDogReset()
{
    mani_watch_dog = 10;
}

/**
 * @brief 通信稳定的情况下 看门口复位
 * @version V1.0
 * @param 
 * @return 0 成功 -1失败
 */
void AgvCtrl::SetChassisVel(float linear_vel, float angular_vel)
{
    mani_linear_vel = linear_vel;
    mani_angular_vel = angular_vel;
}



/**
 * @name: RvdImageRequest
 * @des:  申请靶标取图
 * @param {st_tcp_connect_info} *ptConnectInfo
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvCtrl::RvdImageRequest(st_tcp_connect_info *ptConnectInfo)
{
    int i;
    int s32Ret;
    TRobotMsgHeader tAgvMsgHeader = {0};
    TRobotMsg tAgvMsg = {0};

    tAgvMsgHeader.u16_class = AGV_CMD_C_FILE;
    tAgvMsgHeader.u16_type = AGV_CMD_T_RVD_IMAGE_REQ;
    tAgvMsgHeader.u32_sq = ptConnectInfo->q_svr_handler; //AGV-PLanner模块需要记录命令的源
    tAgvMsgHeader.s32_len = 0;
    RobotMsgInit(&tAgvMsg, tAgvMsgHeader, NULL);
    poAgvRvdCtrl->AsyncMsgPost(&tAgvMsg);

    return 0;
}


/**
 * @name: CheckPose2DViolentChange
 * @des:  在系统更新周期中监测和追溯位置的剧变(Debug)
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int AgvCtrl::CheckPose2DViolentChange()
{
    // int s32Ret;
    // Rigid3d r3M2O, r3M2OTrans;
    // Rigid2d r2M2O;
    // Rigid3d r3O2B, r3O2BTrans;
    // Rigid2d r2O2B;
    // TAgvPose2D tCheckPose2D;
    // TAgvPose2D tM2OTrans;
    // TAgvPose2D tO2BTrans;
    // double diffX, diffY, diffPhi;
    // TBaseInfo tBaseInfo;

    // GetBaseInfo(tBaseInfo);
    // tCheckPose2D.x = tBaseInfo.tPt.dX;
    // tCheckPose2D.y = tBaseInfo.tPt.dY;
    // tCheckPose2D.phi = tBaseInfo.dAngle;

    // diffX = fabs(tCheckPose2D.x - tCheckPose2DPre_.x);
    // diffY = fabs(tCheckPose2D.y - tCheckPose2DPre_.y);
    // diffPhi = tCheckPose2D.phi - tCheckPose2DPre_.phi;
    // if (diffPhi > PI) diffPhi = diffPhi - 2*PI;
    // else if (diffPhi < -PI) diffPhi = diffPhi + 2*PI;
    // diffPhi = fabs(diffPhi);

    // s32Ret = poAgvTf->GetTransform("map", "odom", r3M2O);
    // if (s32Ret != 0)
    // {
    //     return -1;
    // }
    // s32Ret = poAgvTf->GetTransform("odom", "base_link", r3M2O);
    // if (s32Ret != 0)
    // {
    //     return -1;
    // }
    // //位置发生跳变
    // if (diffX > 0.5 || diffY > 0.5 || diffPhi > PI/4)
    // {
    //     r3M2OTrans = r3M2OPre_.inverse() * r3M2O;
    //     r3O2BTrans = r3O2BPre_.inverse() * r3O2B;
    //     tM2OTrans = ToAgvPose2D(r3M2OTrans);
    //     tO2BTrans = ToAgvPose2D(r3O2BTrans);
    //     tlog(TLOG_WARN, "Agv Positon Get Violent Changes-------------\n");
    //     tlog(TLOG_WARN, "Agv Pose Previous Period X[%f]-Y[%f]-Phi[%f]\n", 
    //     tCheckPose2DPre_.x, tCheckPose2DPre_.y, tCheckPose2DPre_.phi);
    //     tlog(TLOG_WARN, "Agv Pose Current Period X[%f]-Y[%f]-Phi[%f]\n", 
    //     tCheckPose2D.x, tCheckPose2D.y, tCheckPose2D.phi);
    //     tlog(TLOG_WARN, "With Map To Odom Change X[%f]-Y[%f]-Phi[%f]\n", 
    //     tM2OTrans.x, tM2OTrans.y, tM2OTrans.phi);
    //     tlog(TLOG_WARN, "With Odom To Base Change X[%f]-Y[%f]-Phi[%f]\n", 
    //     tO2BTrans.x, tO2BTrans.y, tO2BTrans.phi);
    // }

    // //更新周期数据
    // tCheckPose2DPre_ = tCheckPose2D;
    // r3M2OPre_ = r3M2O;
    // r3O2BPre_ = r3O2B;


    return 0;
}






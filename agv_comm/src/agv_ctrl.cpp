#include <stdlib.h>
#include <string.h>

#include "agv_comm/agv_type.h"
#include "agv_comm/agv_ctrl.hpp"
#include "agv_comm/agv_cmd.hpp"
#include "agv_comm/agv_tcp_ctrl.hpp"
#include "agv_comm/agv_ros_ctrl.hpp"
#include "agv_comm/agv_map_ctrl.h"

int event_sub_base_info_process(st_agv_event *pt_event, 
               st_tcp_connect_info *pt_connect_info, 
               CAgvRosCtrl *po_ros_ctrl);

int event_sub_map_info_process(st_agv_event *pt_event, 
               st_tcp_connect_info *pt_connect_info, 
               CAgvRosCtrl *po_ros_ctrl);

int event_pub_vel_cmd_process(st_agv_event *pt_event, 
               st_tcp_connect_info *pt_connect_info, 
               CAgvRosCtrl *po_ros_ctrl);

double Point2LineDis(TPointd tPt, TPointd tLineStartPt, TPointd tLineEndPt, TPointd &tIntersectionPt);

/******************************************************************************
* 函数名称: agv_ctrl_init()
* 作 用 域: 
* 功能描述: 初始化AGV控制结构
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int agv_ctrl_init(st_agv_ctrl *pt_agv_ctrl, ros::NodeHandle *nh)
{
    int s32_ret;

    s32_ret = read_agv_config_from_json(pt_agv_ctrl);
    if (s32_ret != 0)
    {
        return -1;
    }

    pt_agv_ctrl->po_ros_ctrl = new CAgvRosCtrl(nh, pt_agv_ctrl);
    if (0 != pt_agv_ctrl->po_ros_ctrl->Init())
    {
        return -1;
    }
    pt_agv_ctrl->po_ros_ctrl->SetupSubscription();

    s32_ret = udp_svr_ctrl_init(pt_agv_ctrl);
    if (s32_ret != 0)
    {
        return -1;
    }

    s32_ret = tcp_connect_ctrl_init(pt_agv_ctrl, AGV_CLIENT_MAX);
    if (s32_ret != 0)
    {
        return -1;
    }

    return 0;
}

/******************************************************************************
* 函数名称: agv_ctrl_run()
* 作 用 域: 
* 功能描述: 系统启动
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int agv_ctrl_run(st_agv_ctrl *pt_agv_ctrl)
{
    int s32_ret;

    s32_ret = agv_udp_svr_run(pt_agv_ctrl);
    if (s32_ret != 0)
    {
        return -1;
    }

    s32_ret = agv_tcp_svr_run(pt_agv_ctrl);
    if (s32_ret != 0)
    {
        return -1;
    }

    return 0;
}

/******************************************************************************
* 函数名称: agv_ctrl_routine()
* 作 用 域: 
* 功能描述: AGV控制结构周期事务处理
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int agv_ctrl_routine(st_agv_ctrl *pt_agv_ctrl)
{
    int s32_ret;

    //st_agv_ros_ctrl *pt_ros_ctrl;
    CAgvRosCtrl *po_ros_ctrl;
    st_tcp_connect_ctrl *pt_connect_ctrl;
    st_tcp_connect_info *pt_connect_info;
    st_agv_event * pt_event;

    if (pt_agv_ctrl == NULL)
    {
        return -1;
    }

    pt_agv_ctrl->po_ros_ctrl->agvPosTransform();

    po_ros_ctrl = pt_agv_ctrl->po_ros_ctrl;
    pt_connect_ctrl = &pt_agv_ctrl->t_connect_ctrl;

    //mutex work protect---遍历连接的工作队列
    pthread_mutex_lock(&pt_connect_ctrl->mutex_work);
    LIST_FOREACH(pt_connect_info, &pt_connect_ctrl->lh_connect_work, node_ctrl)
    {
        pthread_mutex_lock(&pt_connect_info->mutex_event);
        LIST_FOREACH(pt_event, &pt_connect_info->lh_event, node)
        {
            s32_ret = agv_ctrl_process(pt_event, pt_connect_info, po_ros_ctrl);
        }
        pthread_mutex_unlock(&pt_connect_info->mutex_event);
    }
    pthread_mutex_unlock(&pt_connect_ctrl->mutex_work);


    return 0;
}


/******************************************************************************
* 函数名称: agv_ctrl_process()
* 作 用 域: 
* 功能描述: AGV控制结构周期事务处理
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int agv_ctrl_process(st_agv_event *pt_event, 
               st_tcp_connect_info *pt_connect_info, 
               CAgvRosCtrl *po_ros_ctrl)
{
    int s32_ret = 0;

    switch (pt_event->e_type)
    {
        case AGV_EVENT_SUB_BASE_INFO:
            s32_ret = event_sub_base_info_process(pt_event, pt_connect_info, po_ros_ctrl);
        break;

        case AGV_EVENT_SUB_MAP_INFO:
            s32_ret = event_sub_map_info_process(pt_event, pt_connect_info, po_ros_ctrl);
        break;

        case AGV_EVENT_PUB_VEL_CMD:
            s32_ret = event_pub_vel_cmd_process(pt_event, pt_connect_info, po_ros_ctrl);
        break;


        defualt: break;
    }


    
    return s32_ret;
}


/******************************************************************************
* 函数名称: event_sub_base_info_process()
* 作 用 域: 
* 功能描述: AGV控制结构周期事务处理
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int event_sub_base_info_process(st_agv_event *pt_event, 
               st_tcp_connect_info *pt_connect_info, 
               CAgvRosCtrl *po_ros_ctrl)
{
    int s32_ret;
    int offset;
    st_agv_msg t_agv_msg = {0};

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

    //pthread_rwlock_rdlock(&po_ros_ctrl->rwlock_base);
    po_ros_ctrl->RLockBase();

    offset = 0;

    //Battery
    t_agv_msg.t_msg_body.sbody[offset] = po_ros_ctrl->tBaseInfo.iElectricity;
    offset = offset + 1;
    //CPU
    t_agv_msg.t_msg_body.sbody[offset] = po_ros_ctrl->tBaseInfo.iCPU;
    offset = offset + 1;
    //MEM
    t_agv_msg.t_msg_body.sbody[offset] = po_ros_ctrl->tBaseInfo.iRAM;
    offset = offset + 1;
    //RES
    t_agv_msg.t_msg_body.sbody[offset] = 0; //res byte
    offset = offset + 1;

    //CONF
    float fConfidence = (float)po_ros_ctrl->tBaseInfo.dConfidence;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fConfidence, sizeof(float));
    offset = offset + sizeof(float);
    //L-Vel
    float fLineSpeed = (float)po_ros_ctrl->tBaseInfo.dLineSpeed;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fLineSpeed, sizeof(float));
    offset = offset + sizeof(float);
    //A-Vel
    float fAngularSpeed = (float)po_ros_ctrl->tBaseInfo.dAngularSpeed;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fAngularSpeed, sizeof(float));
    offset = offset + sizeof(float);
    //POSE-X
    float fX = (float)po_ros_ctrl->tBaseInfo.tPt.dX;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fX, sizeof(float));
    offset = offset + sizeof(float);
    //POSE-Y
    float fY = (float)po_ros_ctrl->tBaseInfo.tPt.dY;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fY, sizeof(float));
    offset = offset + sizeof(float);
    //POSE-Angle
    float fAngle = (float)po_ros_ctrl->tBaseInfo.dAngle;
    memcpy(&t_agv_msg.t_msg_body.sbody[offset], &fAngle, sizeof(float));
    offset = offset + sizeof(float);
    //pthread_rwlock_unlock(&po_ros_ctrl->rwlock_base);
    po_ros_ctrl->ULockBase();

    //-----测试------//
    #if 0
    TStation tStart = {0};
    TStation tEnd = {0};
    TPointd tInter,tStartW,tEndW;
    pt_connect_info->po_ros_ctrl->m_pAGVMapCtrl->test_get_start_end_station(tStart, tEnd);
    pt_connect_info->po_ros_ctrl->m_pAGVMapCtrl->MapToWorld(tStart.tPt.dX, tStart.tPt.dY, tStartW.dX, tStartW.dY);
    pt_connect_info->po_ros_ctrl->m_pAGVMapCtrl->MapToWorld(tEnd.tPt.dX, tEnd.tPt.dY, tEndW.dX, tEndW.dY);
    double d = Point2LineDis(TPointd{(double)fX, (double)fY}, tStartW, tEndW, tInter);
    printf("start %d, end %d dis：%.3f\n", tStart.iStationID, tEnd.iStationID, d);
    #endif

    t_agv_msg.t_msg_header.s32_len = offset;
    s32_ret = mq_send(pt_connect_info->q_svr_handler, (char *)&t_agv_msg,
            sizeof(st_agv_msg), 0);

    return s32_ret;
}



/******************************************************************************
* 函数名称: event_sub_map_info_process()
* 作 用 域: 
* 功能描述: AGV控制结构周期事务处理
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int event_sub_map_info_process(st_agv_event *pt_event, 
               st_tcp_connect_info *pt_connect_info, 
               CAgvRosCtrl *po_ros_ctrl)
{
    int s32_ret;
    int offset;
    st_agv_msg t_agv_msg = {0};

    int map_size;
    int msg_body_size;

    if (++(pt_event->count)  < pt_event->period)
    {
        return 0;
    }
    pt_event->count = 0;

    t_agv_msg.t_msg_header.u16_src = 0x10;
    t_agv_msg.t_msg_header.u16_dest = 1;
    t_agv_msg.t_msg_header.u16_class = 1;
    t_agv_msg.t_msg_header.u16_type = 3;
    t_agv_msg.t_msg_header.u32_sq = 0;

    //nomap just return
    if (po_ros_ctrl->MapValid == false)
    {
        return 0;
    }
    
    //pthread_rwlock_rdlock(&po_ros_ctrl->rwlock_base);


    //计算地图数据整体大小并分配内存给消息的数据指针
    po_ros_ctrl->RLockMap();
    map_size = po_ros_ctrl->tMapInfo.iHeight * po_ros_ctrl->tMapInfo.iWidth;
    po_ros_ctrl->ULockMap();

    msg_body_size = 20 + map_size;
    assert(msg_body_size > 32);
    t_agv_msg.t_msg_body.lbody = (uint8_t *)malloc(msg_body_size);
    if (t_agv_msg.t_msg_body.lbody == NULL)
    {
        return -1;
    }

    offset = 0;
    uint8_t *pbody = t_agv_msg.t_msg_body.lbody;
  
    po_ros_ctrl->RLockMap();
    //Resolution
    float fResolution = (float)po_ros_ctrl->tMapInfo.dResolution;
    memcpy(pbody, &fResolution, sizeof(float));
    pbody = pbody + sizeof(float);
    offset = offset + sizeof(float);
    //Map H
    memcpy(pbody, &po_ros_ctrl->tMapInfo.iHeight, sizeof(int));
    pbody = pbody + sizeof(int);
    offset = offset + sizeof(int);
    //Map W
    memcpy(pbody, &po_ros_ctrl->tMapInfo.iWidth, sizeof(int));
    pbody = pbody + sizeof(int);
    offset = offset + sizeof(int);

    float fOriginXOffset = (float)po_ros_ctrl->tMapInfo.dOriginXOffset;
    memcpy(pbody, &fOriginXOffset, sizeof(float));
    pbody = pbody + sizeof(float);
    offset = offset + sizeof(float);

    float fOriginYOffset = (float)po_ros_ctrl->tMapInfo.dOriginYOffset;
    memcpy(pbody, &fOriginYOffset, sizeof(float));
    pbody = pbody + sizeof(float);
    offset = offset + sizeof(float);

    memcpy(pbody, po_ros_ctrl->tMapInfo.pCostMapData, map_size);
    offset = offset + map_size;

    //pthread_rwlock_unlock(&po_ros_ctrl->rwlock_base);
    po_ros_ctrl->ULockMap();

    t_agv_msg.t_msg_header.s32_len = offset;

    s32_ret = mq_send(pt_connect_info->q_svr_handler, (char *)&t_agv_msg,
            sizeof(st_agv_msg), 0);

    return s32_ret;
}



/******************************************************************************
* 函数名称: event_pub_vel_cmd_process()
* 作 用 域: 
* 功能描述: AGV控制结构周期周期发布遥控指令
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int event_pub_vel_cmd_process(st_agv_event *pt_event, 
               st_tcp_connect_info *pt_connect_info, 
               CAgvRosCtrl *po_ros_ctrl)
{
    int s32_ret;
    int offset;
    st_agv_msg t_agv_msg = {0};

    if (++(pt_event->count)  < pt_event->period)
    {
        return 0;
    }
    pt_event->count = 0;

    //上位机通信不稳定 规定事件内未能收到有效的遥控速度指令
    //上位机控制有效 
    //WARNING: 狗 线速度 角速度均为4字节自然对齐 多线程操作理论上不会破坏单一数据
    //这里为了保证效率 不予以加锁
    if (po_ros_ctrl->mani_watch_dog == 0)
    {
        po_ros_ctrl->PubManiCmdVel(0, 0);
    }
    else
    {
        po_ros_ctrl->mani_watch_dog--;
        po_ros_ctrl->PubManiCmdVel(po_ros_ctrl->mani_linear_vel, po_ros_ctrl->mani_angular_vel);
    }
    
    
    return 0;
}

double Point2LineDis(TPointd tPt, TPointd tLineStartPt, TPointd tLineEndPt, TPointd &tIntersectionPt)
{
    //向量a的坐标表示
    TPointd A;
    A.dX = tPt.dX - tLineStartPt.dX;
    A.dY = tPt.dY - tLineStartPt.dY;
    //向量b的坐标表示
    TPointd B;
    B.dX = tLineEndPt.dX - tLineStartPt.dX;
    B.dY = tLineEndPt.dY - tLineStartPt.dY;
    if(0 == B.dX && 0 == B.dY)
    {
        return 0;
    }
    //有公式可得向量c的模长|c| = a.b/|b|
    //a.b = x1x2 + y1y2
    //向量c的坐标表示  c = tIntersectionPt - tLineStartPt = ((a.b)/|b|^2).b
    TPointd C;
    C.dX = ((A.dX * B.dX + A.dY * B.dY ) / (double)(B.dX*B.dX + B.dY * B.dY)) * B.dX;
    C.dY = ((A.dX * B.dX + A.dY * B.dY ) / (double)(B.dX*B.dX + B.dY * B.dY)) * B.dY;
    tIntersectionPt.dX = C.dX + tLineStartPt.dX ;
    tIntersectionPt.dY = C.dY + tLineStartPt.dY;
    //由向量 e = a - c 再取e的模即可得到点到直线的距离
    double dDistance = sqrt((A.dX -C.dX) * (A.dX -C.dX) + (A.dY -C.dY) * (A.dY -C.dY));
    return dDistance;
}
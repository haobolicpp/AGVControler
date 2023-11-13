

#include <string.h>
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "agv_chassis/CtrlMode.h"

#include "cartographer_ros_msgs/ChangeMode.h"
#include "cartographer_ros_msgs/StopPubMap.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "agv_comm/agv_cmd.hpp"
#include "agv_comm/agv_type.h"
#include "agv_comm/agv_ctrl.hpp"
#include "agv_comm/agv_ros_ctrl.hpp"
#include "agv_comm/agv_map_ctrl.h"




/******************************************************************************
* 函数名称: CAgvRosCtrl()
* 作 用 域: Global
* 功能描述: ROS控制类构造
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
CAgvRosCtrl::CAgvRosCtrl(ros::NodeHandle *nh, st_agv_ctrl *pt_agv_ctrl) : ros_nh(nh)
{
    memset(&tBaseInfo, 0, sizeof(TBaseInfo));
    memset(&tMapInfo, 0, sizeof(TMapInfo));

    pthread_rwlock_init(&rwlock_base, NULL);
    pthread_rwlock_init(&rwlock_map, NULL);

    tf_listener = new tf::TransformListener(ros::Duration(10));
    MapValid = false;

    e_state = E_AGV_CTRL_STATE::E_AGV_CTRL_INIT;

    //默认的遥控操作参数
    mani_watch_dog = 0; //默认发布周期为40ms 则超时周期为200ms 待调整
    mani_linear_vel = 0;
    mani_angular_vel = 0;
    mani_linear_vel_def = 0.35; //默认遥控线速度
    mani_angular_vel_def = 0.5; //默认遥控角速度
    rosClientChassisMode = nh->serviceClient<agv_chassis::CtrlMode>("agv_chassis_mode");
    m_pubManiVel = ros_nh->advertise<geometry_msgs::Twist>("mani_cmd_vel", 1, true);

    m_pt_agv_ctrl = pt_agv_ctrl;
    m_pAGVMapCtrl = new CAGVMapCtrl(pt_agv_ctrl);

    rosClientCM = nh->serviceClient<cartographer_ros_msgs::ChangeMode>("ChangeMode");
    rosClientSP = nh->serviceClient<cartographer_ros_msgs::StopPubMap>("StopPubMap");

    pMoveBaseAc = new MoveBaseClient("move_base", true);
    m_pMoveBase_GlobalPathAc = new MoveBaseClient_GlobalPath("move_base_global_path", true);
    MoveBaseState = 0;
    ptMoveCmdFrom = NULL;
}

/******************************************************************************
* 函数名称: ~CAgvRosCtrl()
* 作 用 域: Global
* 功能描述: ROS控制类析构
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
CAgvRosCtrl::~CAgvRosCtrl()
{
    free(tMapInfo.pCostMapData);
    memset(&tBaseInfo, 0, sizeof(TBaseInfo));
    memset(&tMapInfo, 0, sizeof(TMapInfo));

    pthread_rwlock_destroy(&rwlock_base);
    pthread_rwlock_destroy(&rwlock_map);

    delete tf_listener;
    delete m_pAGVMapCtrl;
    delete pMoveBaseAc;
    delete m_pMoveBase_GlobalPathAc;
}

int CAgvRosCtrl::Init()
{
    m_pubGridMap = ros_nh->advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    m_pubGlobalPath = ros_nh->advertise<nav_msgs::Path>("agv_global_path", 1, true);

    if (0 != m_pAGVMapCtrl->init())
    {
        return -1;
    }

    PublishGridmap();

	ROS_INFO("AGV-COMM waiting for the move_base action server");
    //pMoveBaseAc->waitForServer(ros::Duration(60));
    m_pMoveBase_GlobalPathAc->waitForServer(ros::Duration(60));
	ROS_INFO("AGV-COMM connected to move base server");

    ChangeStateToNavi();

    return 0;
}

/******************************************************************************
* 函数名称: 
* 作 用 域: Global
* 功能描述: 发布地图给move_base用
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: bob.li
* 完成日期: 2021年9月8日
******************************************************************************/
void CAgvRosCtrl::PublishGridmap()
{
    //发布地图grid
    m_pubGridMap.publish( m_pAGVMapCtrl->get_grid_map());
}


/******************************************************************************
* 函数名称: SetupSubscription()
* 作 用 域: Global
* 功能描述: 设置订阅话题
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::SetupSubscription()
{
       //ros_sub_base_info = ros_nh->subscribe("turtle1/pose", 100, &CAgvRosCtrl::agvPosSubCB, this);
       ros_sub_map = ros_nh->subscribe("map", 10, &CAgvRosCtrl::agvMapSubCB, this);
}


/******************************************************************************
* 函数名称: agvPosSubCB()
* 作 用 域: Global
* 功能描述: 设置订阅话题
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::agvPosSubCB()
{
    return;
}


/******************************************************************************
* 函数名称: agvPosSubCB()
* 作 用 域: Global
* 功能描述: 设置订阅话题
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::agvMapSubCB(const nav_msgs::OccupancyGridConstPtr& msg)
{
    int map_size;

    printf("Received a %d X %d map @ %.3f m/pix\n",
           msg->info.width,
           msg->info.height,
           msg->info.resolution);

    //pthread_rwlock_wrlock(&rwlock_map);
    WLockMap();

    tMapInfo.dResolution = msg->info.resolution;
    tMapInfo.iWidth = msg->info.width;
    tMapInfo.iHeight = msg->info.height;
    tMapInfo.dOriginXOffset = msg->info.origin.position.x;
    tMapInfo.dOriginYOffset = msg->info.origin.position.y;

    map_size = msg->info.width * msg->info.height;
    tMapInfo.pCostMapData = (uint8_t *)realloc(tMapInfo.pCostMapData, map_size);
    assert(tMapInfo.pCostMapData != NULL);
    memcpy(tMapInfo.pCostMapData, msg->data.data(), map_size);
    MapValid = true;
  
    ULockMap();

}

/******************************************************************************
* 函数名称: agvPosTransform()
* 作 用 域: Global
* 功能描述: 转换位置坐标
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int CAgvRosCtrl::agvPosTransform()
{
    geometry_msgs::PoseStamped pose_baselink;
    geometry_msgs::PoseStamped pose_map;

    pose_baselink.header.frame_id = "base_link";
    pose_baselink.header.stamp = ros::Time();
    //we'll just use the most recent transform available for our simple example
    pose_baselink.pose.position.x = 0.0;
    pose_baselink.pose.position.y = 0.0;
    pose_baselink.pose.position.z = 0.0;
    pose_baselink.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

    if (!tf_listener->canTransform("map", "base_link", ros::Time()))
    {
        return -1;
    }

    try{
        tf_listener->transformPose("map", pose_baselink, pose_map);
    }
    catch( tf::TransformException ex)
    {
        ROS_WARN("transfrom exception : %s",ex.what());
        return -1;
    }
    //just an arbitrary point in space
    tf::Quaternion quat;
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::quaternionMsgToTF(pose_map.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    tBaseInfo.tPt.dX = pose_map.pose.position.x;
    tBaseInfo.tPt.dY = pose_map.pose.position.y;
    tBaseInfo.dAngle = yaw;

    //ROS_INFO("tBaseInfo : x[%f]-y[%f]-a[%f] ",tBaseInfo.tPt.dX, tBaseInfo.tPt.dY, tBaseInfo.dAngle);
}




/******************************************************************************
* 函数名称: CallChangeModeSvr()
* 作 用 域: Global
* 功能描述: 通知SLAM模块切换模式-Cartographer节点且换模式 建图/定位
* 输入参数: true 建图->定位   false 定位->建图
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int CAgvRosCtrl::CallChangeModeSvr(bool arg)
{
    int ret;

    cartographer_ros_msgs::ChangeMode svrCmIns;
    svrCmIns.request.pure_localization = arg;

    ret = rosClientCM.call(svrCmIns);

    if (0 != ret)
    {
        return svrCmIns.response.status.code;
    }

    else
    {
        return -1;
    }


}

/******************************************************************************
* 函数名称: CallStopPubMapSvr()
* 作 用 域: Global
* 功能描述: 通知SLAM模块切换模式-停止新地图发布
* 输入参数: true 建图->定位 停止发布  false 定位->建图 启动发布
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int CAgvRosCtrl::CallStopPubMapSvr(bool arg)
{
    cartographer_ros_msgs::StopPubMap svrSpIns;
    svrSpIns.request.pure_localization = arg;

    if (0 != rosClientSP.call(svrSpIns))
    {
        return svrSpIns.response.status.code;
    }

    else
    {
        return -1;
    }
}

/******************************************************************************
* 函数名称: CallMoveToTarget()
* 作 用 域: Global
* 功能描述: 申请MoveBase驱动AGV到目标位姿
* 输入参数: x y omega
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void DoneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr & result,
        CAgvRosCtrl *pRosCtrl)
{
    int ret;
    st_agv_msg t_agv_msg = {0};
    ROS_INFO("MoveBase Goal Arrived!");

    //生成异步消息通知对应的连接线程以告知Client端 目标到达
    t_agv_msg.t_msg_header.u16_src = 0x10;
    t_agv_msg.t_msg_header.u16_dest = 1;
    t_agv_msg.t_msg_header.u16_class = AGV_CMD_C_CTRL;
    t_agv_msg.t_msg_header.u16_type = AGV_CMD_T_CTRL_SET_STAION | 0x8000;
    t_agv_msg.t_msg_header.u32_sq = 0;
    t_agv_msg.t_msg_header.s32_len = 1;

    t_agv_msg.t_msg_body.sbody[0] = 0;

    ret = mq_send(pRosCtrl->ptMoveCmdFrom->q_svr_handler, (char *)&t_agv_msg,
        sizeof(st_agv_msg), 0);
    
    pRosCtrl->MoveBaseState = 0;

}

void ActiveCb()
{
    ROS_INFO("MoveBase Goal Actived!\n");
}
 
void FeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    //ROS_INFO("MoveBase CurrentPose-x[%f]-y[%f]-w[%f]", 
    //feedback->base_position.pose.position.x,
    //feedback->base_position.pose.position.y,
    //feedback->base_position.pose.orientation.w);
}

/******************************************************************************
* 函数名称: CallMoveToTarget()
* 作 用 域: Global
* 功能描述: move_base到达目标回调
* 输入参数: x y omega
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: bob.li
* 完成日期: 2021年9月17日
******************************************************************************/
void DoneGlobalPathCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBase_GlobalPathResultConstPtr & result,
        CAgvRosCtrl *pRosCtrl)
{
    int ret;
    st_agv_msg t_agv_msg = {0};
    ROS_INFO("MoveBase Goal Arrived!");

    //生成异步消息通知对应的连接线程以告知Client端 目标到达
    t_agv_msg.t_msg_header.u16_src = 0x10;
    t_agv_msg.t_msg_header.u16_dest = 1;
    t_agv_msg.t_msg_header.u16_class = AGV_CMD_C_CTRL;
    t_agv_msg.t_msg_header.u16_type = AGV_CMD_T_CTRL_SET_STAION | 0x8000;
    t_agv_msg.t_msg_header.u32_sq = 0;
    t_agv_msg.t_msg_header.s32_len = 1;

    t_agv_msg.t_msg_body.sbody[0] = 0;

    ret = mq_send(pRosCtrl->ptMoveCmdFrom->q_svr_handler, (char *)&t_agv_msg,
        sizeof(st_agv_msg), 0);

    pRosCtrl->MoveBaseState = 0;

}
 
void FeedbackGlobalPathCb(const move_base_msgs::MoveBase_GlobalPathFeedbackConstPtr& feedback)
{
    ROS_INFO("FeedbackGlobalPathCb");
    // feedback->base_position.pose.position.x,
    // feedback->base_position.pose.position.y,
    // feedback->base_position.pose.orientation.w);
}

int CAgvRosCtrl::CallMoveToTarget(st_tcp_connect_info *ptConnectInfo, double x, double y, double w)
{
    //当前存在目标点任务 或 系统处于建图模式下 请求无效
    if (MoveBaseState != 0 || e_state != E_AGV_CTRL_STATE::E_AGV_CTRL_NAVI)
    {
        return -1;
    }

    MoveBaseState = 1;
    ptMoveCmdFrom = ptConnectInfo;

    move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
    tf::Quaternion qt = tf::createQuaternionFromYaw(w);
	goal.target_pose.pose.orientation.w = qt.getW();
    goal.target_pose.pose.orientation.x = qt.getX();
    goal.target_pose.pose.orientation.y = qt.getY();
    goal.target_pose.pose.orientation.z = qt.getZ();
	pMoveBaseAc->sendGoal(goal, boost::bind(&DoneCb, _1, _2, this), 
                    &ActiveCb, &FeedbackCb);
    return 0;
}

/******************************************************************************
* 函数名称: CallMoveToTarget()
* 作 用 域: Global
* 功能描述: move_base到达目标回调
* 输入参数: dRotateAngle：行走之前，需要旋转到的角度
            dEndAngle:终点朝向角
            global_path：全局路径
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: bob.li
* 完成日期: 2021年9月17日
******************************************************************************/
int CAgvRosCtrl::CallMoveToTarget(st_tcp_connect_info *ptConnectInfo, double dRotateAngle, double dEndAngle,std::vector<TPointd>& global_path)
{
    //当前存在目标点任务 或 系统处于建图模式下 请求无效
    if (MoveBaseState != 0 || e_state != E_AGV_CTRL_STATE::E_AGV_CTRL_NAVI)
    {
        printf("agv_comm state err!MoveBaseState:%d\n", MoveBaseState);
        return -1;
    }

    MoveBaseState = 1;
    ptMoveCmdFrom = ptConnectInfo;

    move_base_msgs::MoveBase_GlobalPathGoal goal;

    //终点赋值
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = global_path.rbegin()->dX;
	goal.target_pose.pose.position.y = global_path.rbegin()->dY;
    tf::Quaternion qt = tf::createQuaternionFromYaw(dEndAngle);
	goal.target_pose.pose.orientation.w = qt.getW();
    goal.target_pose.pose.orientation.x = qt.getX();
    goal.target_pose.pose.orientation.y = qt.getY();
    goal.target_pose.pose.orientation.z = qt.getZ();
    
    //路径赋值
    goal.global_path.header.frame_id = "map";
    goal.global_path.header.stamp = ros::Time::now();
    for (auto &pt : global_path)
    {
        geometry_msgs::PoseStamped pos;
        pos.header.frame_id = "map";
        pos.header.stamp = ros::Time::now();
        pos.pose.position.x = pt.dX;
        pos.pose.position.y = pt.dY;

        goal.global_path.poses.push_back(pos);
    }

    //起点初始旋转角度
    goal.start_angle = dRotateAngle;

    //终点位姿修改
    auto endPoint = goal.global_path.poses.rbegin();
    endPoint->pose.orientation = goal.target_pose.pose.orientation;

	m_pMoveBase_GlobalPathAc->sendGoal(goal, boost::bind(&DoneGlobalPathCb, _1, _2, this), 
                    &ActiveCb, &FeedbackGlobalPathCb);

    //发布路径
    m_pubGlobalPath.publish(goal.global_path);
    return 0;
}


/******************************************************************************
* 函数名称: ()
* 作 用 域: Global
* 功能描述: 
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
 int CAgvRosCtrl::CallCancalTarget()
 {
    if (MoveBaseState != 0)
    {
        //pMoveBaseAc->cancelGoal();
        m_pMoveBase_GlobalPathAc->cancelGoal();
        MoveBaseState = 0;
    }

    return 0;
 }


/******************************************************************************
* 函数名称: ChangeStateToSlam
* 作 用 域: Global
* 功能描述: 
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::ChangeStateToSlam()
{
    e_state = E_AGV_CTRL_STATE::E_AGV_CTRL_SLAM;
    return;
}

/******************************************************************************
* 函数名称: ChangeStateToNavi
* 作 用 域: Global
* 功能描述: 
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::ChangeStateToNavi()
{
    e_state = E_AGV_CTRL_STATE::E_AGV_CTRL_NAVI;
    return;
}

/******************************************************************************
* 函数名称: ChangeStateToError
* 作 用 域: Global
* 功能描述: 
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::ChangeStateToError()
{
    e_state = E_AGV_CTRL_STATE::E_AGV_CTRL_ERROR;
    return;    
}


/******************************************************************************
* 函数名称: RequestChassisMode
* 作 用 域: Global
* 功能描述: 切换底盘状态
* 输入参数: mode_req = 0 底盘自动导航状态 mode_req = 1 底盘受遥控状态
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
bool CAgvRosCtrl::RequestChassisMode(uint8_t mode_req)
{
    agv_chassis::CtrlMode srv;
    srv.request.mode_req = mode_req;
    return rosClientChassisMode.call(srv);
}

/******************************************************************************
* 函数名称: RequestChassisMode
* 作 用 域: Global
* 功能描述: 切换底盘状态
* 输入参数: mode_req = 0 底盘自动导航状态 mode_req = 1 底盘受遥控状态
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::PubManiCmdVel(float linear_vel, float angular_vel)
{
    geometry_msgs::Twist twist;
    twist.linear.x = linear_vel;  // m/s
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angular_vel; // rad/s

    m_pubManiVel.publish(twist);

}


/******************************************************************************
* 函数名称: ManiWatchDogReset
* 作 用 域: Global
* 功能描述: 复位遥控连接的看门狗
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::ManiWatchDogReset()
{
    mani_watch_dog = 10;
}

/******************************************************************************
* 函数名称: SetChassisVel
* 作 用 域: Global
* 功能描述: 复位遥控连接的看门狗
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void CAgvRosCtrl::SetChassisVel(float linear_vel, float angular_vel)
{
    mani_linear_vel = linear_vel;
    mani_angular_vel = angular_vel;
}
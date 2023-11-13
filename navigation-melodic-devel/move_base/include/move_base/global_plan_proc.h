#ifndef GLOBAL_PLAN_PROC_H
#define GLOBAL_PLAN_PROC_H
#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"
#include <move_base_msgs/MoveBase_GlobalPathAction.h>

namespace move_base {
//接收全局路径和终点位姿并调度AGV行走
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBase_GlobalPathAction> MoveBaseAction_GlobalPathServer;
//新的状态定义
enum MoveBaseStateNew
{
    Free,        //空闲
    Controlling, //控制行走
};

class MoveBase;
class CGlobalPlanProc
{
public:
    CGlobalPlanProc(MoveBase* ptr);
    ~CGlobalPlanProc(){};

    //接收全局路径的回调
    void executeGlobalPathCb(const move_base_msgs::MoveBase_GlobalPathGoalConstPtr& move_base_goal);

    //全局路径临时存储，注意路径最后一个点是终点，包括位姿
    std::vector<geometry_msgs::PoseStamped> m_globalPath;
    MoveBase* m_pMoveBase;
    //全局规划action
    MoveBaseAction_GlobalPathServer* m_pActionGPServer;
    //新的状态控制
    MoveBaseStateNew m_stateNew;

};
};

#endif
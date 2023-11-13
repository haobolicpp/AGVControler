#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "move_base/global_plan_proc.h"
namespace move_base {
CGlobalPlanProc::CGlobalPlanProc(MoveBase* ptr)
{
    m_pMoveBase = ptr;
    m_stateNew = Free;
    m_pActionGPServer = new MoveBaseAction_GlobalPathServer(ros::NodeHandle(), "move_base_global_path", boost::bind(&CGlobalPlanProc::executeGlobalPathCb, this, _1), false);
    m_pActionGPServer->start();
}

void CGlobalPlanProc::executeGlobalPathCb(const move_base_msgs::MoveBase_GlobalPathGoalConstPtr& move_base_goal)
{
    ROS_INFO("executeGlobalPathCb: desangle:%f",move_base_goal->start_angle);
    //暂存路径，
    m_globalPath = move_base_goal->global_path.poses;

    //初始朝向角
    m_pMoveBase->m_startRotateCtrl.InitDesAngle(move_base_goal->start_angle); 

    if(!m_pMoveBase->isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      //m_pActionGPServer->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = m_pMoveBase->goalToGlobalFrame(move_base_goal->target_pose);

    m_pMoveBase->publishZeroVelocity();
    //we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(m_pMoveBase->planner_mutex_);
    m_pMoveBase->planner_goal_ = goal;
    m_pMoveBase->runPlanner_ = true;
    m_pMoveBase->planner_cond_.notify_one();
    lock.unlock();

    m_pMoveBase->current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(m_pMoveBase->controller_frequency_);
    if(m_pMoveBase->shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      m_pMoveBase->planner_costmap_ros_->start();
      m_pMoveBase->controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    m_pMoveBase->last_valid_control_ = ros::Time::now();
    m_pMoveBase->last_valid_plan_ = ros::Time::now();
    m_pMoveBase->last_oscillation_reset_ = ros::Time::now();
    m_pMoveBase->planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {
      if(m_pMoveBase->c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", m_pMoveBase->controller_frequency_);
        r = ros::Rate(m_pMoveBase->controller_frequency_);
        m_pMoveBase->c_freq_change_ = false;
      }

      if(m_pActionGPServer->isPreemptRequested()){
        if(m_pActionGPServer->isNewGoalAvailable()){
        //   //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
        //   move_base_msgs::MoveBaseGoal new_goal = *m_pActionGPServer->acceptNewGoal();

        //   if(!m_pMoveBase->isQuaternionValid(new_goal.target_pose.pose.orientation)){
        //     m_pActionGPServer->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
        //     return;
        //   }

        //   goal = m_pMoveBase->goalToGlobalFrame(new_goal.target_pose);

        //   //we'll make sure that we reset our state for the next execution cycle
        //   m_pMoveBase->recovery_index_ = 0;
        //   m_pMoveBase->state_ = PLANNING;

        //   //we have a new goal so make sure the planner is awake
        //   lock.lock();
        //   m_pMoveBase->planner_goal_ = goal;
        //   m_pMoveBase->runPlanner_ = true;
        //   m_pMoveBase->planner_cond_.notify_one();
        //   lock.unlock();

        //   //publish the goal point to the visualizer
        //   ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
        //   m_pMoveBase->current_goal_pub_.publish(goal);

        //   //make sure to reset our timeouts and counters
        //   m_pMoveBase->last_valid_control_ = ros::Time::now();
        //   m_pMoveBase->last_valid_plan_ = ros::Time::now();
        //   m_pMoveBase->last_oscillation_reset_ = ros::Time::now();
        //   m_pMoveBase->planning_retries_ = 0;
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          m_pMoveBase->resetState();

          //notify the ActionServer that we've successfully preempted
          //ROS_INFO("move_base","Move base preempting the current goal");
          m_pActionGPServer->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != m_pMoveBase->planner_costmap_ros_->getGlobalFrameID()){
        goal = m_pMoveBase->goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        m_pMoveBase->recovery_index_ = 0;
        m_pMoveBase->state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        m_pMoveBase->planner_goal_ = goal;
        m_pMoveBase->runPlanner_ = true;
        m_pMoveBase->planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        m_pMoveBase->current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        m_pMoveBase->last_valid_control_ = ros::Time::now();
        m_pMoveBase->last_valid_plan_ = ros::Time::now();
        m_pMoveBase->last_oscillation_reset_ = ros::Time::now();
        m_pMoveBase->planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      bool done = m_pMoveBase->executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      if(done)
      {
          ROS_INFO("m_pMoveBase->executeCycle done!");
          return;
      }       

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      //ROS_INFO("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / m_pMoveBase->controller_frequency_) && m_pMoveBase->state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", m_pMoveBase->controller_frequency_, r.cycleTime().toSec());
    }

    ROS_INFO("action server over!");

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    m_pMoveBase->runPlanner_ = true;
    m_pMoveBase->planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    //m_pActionGPServer->setAborted(move_base_msgs::movebase(), "Aborting on the goal because the node has been killed");
    return;
}

};
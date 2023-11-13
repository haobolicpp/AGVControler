

#ifndef CHASSIS_MESSAGE_HPP
#define CHASSIS_MESSAGE_HPP

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include "motor_controller.hpp"

#include "agv_chassis/CtrlMode.h"

enum class CtrlMode
{
    AUTO,
    MANI
};

class ChassisMessage
{
public:
    ChassisMessage(ros::NodeHandle *nh);
    
    ros::NodeHandle *nh_;

    std::string odom_frame_;
    std::string base_frame_;
    bool pub_odom_tf = true;

    CtrlMode ctrl_mode;
    MotorController motor_ctrl_;

    void SetupSubscription();
    void PublishOdometryToROS();

    int ctrlFreq;
    int SetCtrlFreq(int s32CtrlFreq);
    void Run();

private:

    geometry_msgs::Twist current_twist_;
    ros::ServiceServer mode_service;
    ros::Publisher odom_publisher_;
    ros::Subscriber motion_cmd_subscriber_;
    ros::Subscriber manipulate_cmd_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Time last_time_;
    ros::Time current_time_;



    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void ManTwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    //bool ModeReqCallback(agv_chassis::CtrlMode::Request &req, 
    //                agv_chassis::CtrlMode::Response &resp);
};


#endif /* SCOUT_MESSENGER_HPP */

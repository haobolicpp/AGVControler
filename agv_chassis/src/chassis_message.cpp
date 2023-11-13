
#include <chassis_message.hpp>
#include <tf/transform_datatypes.h>

/**
 * @brief 模式切换服务回调函数的声明
 * @param msg Twist
 * @return 
 */
bool ModeReqCallback(agv_chassis::CtrlMode::Request &req, 
                    agv_chassis::CtrlMode::Response &resp,
                    ChassisMessage *poCM);


/**
 * @brief 初始化ROS接口功能
 * @param msg Twist
 * @return 
 */
ChassisMessage::ChassisMessage(ros::NodeHandle *nh) : nh_(nh)
{

    odom_frame_ = "odom";
    base_frame_ = "base_link";

    ctrlFreq = 50;
    ctrl_mode = CtrlMode::AUTO;

    motor_ctrl_.init();

}

/**
 * @brief 初始化ROS接口功能
 * @param msg Twist
 * @return 
 */
void ChassisMessage::SetupSubscription()
{
    // Mode Service
    mode_service = nh_->advertiseService<agv_chassis::CtrlMode::Request, agv_chassis::CtrlMode::Response>
    ("/agv_chassis_mode", boost::bind(&ModeReqCallback, _1, _2 ,this));
    // odometry publisher
    odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_frame_, 50);
    // cmd subscriber
    motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>("/cmd_vel", 5, &ChassisMessage::TwistCmdCallback, this); //不启用平滑包则订阅“cmd_vel”
    manipulate_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>("/mani_cmd_vel", 5, &ChassisMessage::ManTwistCmdCallback, this); 
    //start motor control
    //motor_ctrl_.MoterCtrlStart();
    
}

/**
 * @brief cmd_vel订阅的回调函数，在自动导航模式下由MoveBase发布cmd_vel
 * @param msg Twist
 * @return 
 */
void ChassisMessage::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    //解析VEL_CMD 根据运动模型计算左右轮的速度 生成指令通过CANopen-SDO下发！！！
    if(ctrl_mode == CtrlMode::AUTO)
    {
        motor_ctrl_.setVel(msg->linear.x, msg->angular.z);
    }
    // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
}

/**
 * @brief cmd_vel订阅的回调函数，在手动遥控模式下由AgvComm节点发布cmd_vel
 * @param msg Twist
 * @return 
 */
void ChassisMessage::ManTwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    //解析VEL_CMD 根据运动模型计算左右轮的速度 生成指令通过CANopen-SDO下发！！！
    if(ctrl_mode == CtrlMode::MANI)
    {
        motor_ctrl_.setVel(msg->linear.x, msg->angular.z);
    }
    // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
}

/**
 * @brief 切换控制模式 导航自动控制/人工遥控
 * @param 
 * @return 
 */
bool ModeReqCallback(agv_chassis::CtrlMode::Request &req, 
                    agv_chassis::CtrlMode::Response &resp,
                    ChassisMessage *poCM)
{
    bool ret = false;
    resp.mode_resp = req.mode_req;

    if (req.mode_req == 0)
    {
        poCM->ctrl_mode = CtrlMode::AUTO;
        poCM->motor_ctrl_.setVel(0, 0);
        ret = true;
    }
    else if (req.mode_req == 1)
    {
        poCM->ctrl_mode = CtrlMode::MANI;
        poCM->motor_ctrl_.setVel(0, 0);
        ret = true;
    }

    return ret;
}

/**
 * @brief 发布里程计信息
 * @param msg Twist
 * @return 
 */
void ChassisMessage::PublishOdometryToROS()
{
    int ret;
    double dt;
    double position_x_, position_y_, theta_ = 0.0;
    double linear_speed_, angular_speed_ = 0.0;

    static bool init_run = true;
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    if (motor_ctrl_.LRPFresh == 0)
    {
        return;
    }

    else if (motor_ctrl_.LRPFresh == 1)
    {
        current_time_ = ros::Time::now();
        dt = (current_time_ - last_time_).toSec();
        ret = motor_ctrl_.getOdom(linear_speed_, angular_speed_,position_x_, position_y_, theta_, dt);
        motor_ctrl_.LRPFresh = 0;
        if (ret != 0)
        {
            return;
        }
    }

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
    // publish tf transformation
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

//--------------cartograher delete it---------------
    if (this->pub_odom_tf)
    {
        tf_broadcaster_.sendTransform(tf_msg);
    }
//-------------------------------------------------

    // publish odometry and tf messages
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;


    odom_publisher_.publish(odom_msg);

    last_time_ = current_time_;
}



/**
 * @brief 设定底盘控制周期
 * @param s32CtrlFreq 控制周期【0，50】Hz 默认20Hz
 * @return 
 */
int ChassisMessage::SetCtrlFreq(int s32CtrlFreq)
{
    if (s32CtrlFreq <= 0 || s32CtrlFreq >50)
    {
        return -1;
    }

    ctrlFreq = s32CtrlFreq;
}

/**
 * @brief ChassisMessage主循环
 * @param 
 * @return 
 */
void ChassisMessage::Run()
{
    int s32Ret;
    TCoIfMsg tCoIfMsg;

    MotorController *poMC = &(motor_ctrl_);
    struct timeval t_tmptv;
	fd_set t_fd_read_set;
    int s32_fd_max = poMC->QCoIf_ + 1;

    int s32UsFromHz;
    s32UsFromHz = floor(1000000.0/ctrlFreq);

    //开启底层控制循环
    poMC->MoterCtrlStart();

    while(ros::ok())
    {
        t_tmptv.tv_sec = 0;
        t_tmptv.tv_usec = s32UsFromHz;
        FD_ZERO(&t_fd_read_set);
        FD_SET(poMC->QCoIf_, &t_fd_read_set);
        s32Ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, &t_tmptv);
        if (s32Ret < 0)
        {
            printf("chassis run with err %s\n", strerror(errno));
            sleep(1);
            continue;
        }

        //控制周期边界 触发SDO速度指令发送 推动状态机执行
        else if (s32Ret == 0)
        {
            //printf("Control Period Here...\n");
            //整合最新的速度命令
            ros::spinOnce();
            poMC->CoIfPeriodCtrl();
            continue;
        }

        //异步SDO响应 接收到SDO
        if (FD_ISSET(poMC->QCoIf_, &t_fd_read_set))
        {
            memset(&tCoIfMsg, 0, sizeof(TCoIfMsg));

            s32Ret = mq_receive(poMC->QCoIf_, (char *)&tCoIfMsg,
                        sizeof(TCoIfMsg), NULL);
            if (s32Ret < 0)
            {
                printf("QCoIf Recv Error: %s", strerror(errno));
                continue;
            }

            poMC->CoIfMsgHandler(&tCoIfMsg);
            //获取实时编码器数据并发布
            if (poMC->LRPFresh == 1)
            {
                this->PublishOdometryToROS();
            }

        }

    }
}
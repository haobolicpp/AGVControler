#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "agv_comm/sys_queue.h"
#include "agv_comm/agv_type.h"
#include "agv_comm/agv_ctrl.hpp"
#include "agv_comm/agv_cmd.hpp"
#include "agv_comm/agv_udp_ctrl.hpp"
#include "agv_comm/agv_tcp_ctrl.hpp"



int main(int argc, char **argv)

{
    int s32_ret;
    st_agv_ctrl g_t_agv_ctrl;

    //system("rm -rf /dev/agv-tcp-conn-*");

    ros::init(argc, argv, "agv_comm_node");
    ros::NodeHandle nh;

    //初始化用于AGV通信与控制的数据结构
    s32_ret = agv_ctrl_init(&g_t_agv_ctrl, &nh);
    if (s32_ret != 0)
    {
        return -1;
    }
    //系统启动
    s32_ret = agv_ctrl_run(&g_t_agv_ctrl);
    if (s32_ret != 0)
    {
        return -1;
    }


    sleep(1);

    ros::Rate loopRate(50);
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;

        ss << "hello world" ;
        msg.data = ss.str();

        agv_ctrl_routine(&g_t_agv_ctrl);
        //agv_file_svr_post_msg(t_agv_msg);
        //ROS_INFO("%s", msg.data.c_str());
        //base_info_pub.publish(msg);
        ros::spinOnce();
        loopRate.sleep();

    }
    

    return 0;
}
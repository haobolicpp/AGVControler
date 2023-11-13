#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ros/ros.h"
#include <chassis_message.hpp>

int main(int argc, char **argv)

{
    int s32Ret;
    int s32CtrlFrq = 20;

    ros::init(argc, argv, "agv_chassis");
    ros::NodeHandle nh;

    ChassisMessage messager(&nh);

    sleep(1);
    printf("ROS CYC START!\n");
    messager.SetupSubscription();
    sleep(1);

    messager.Run();
    //publish robot state at 50Hz while listening to twist commands
    #if 0
    ros::Rate rate(20); // 50Hz
    while (true)
    {
        messager.PublishOdometryToROS();
        ros::spinOnce();
        rate.sleep();
    }
    #endif

    return 0;
}
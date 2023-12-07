#ifndef AGV_MSG_H
#define AGV_MSG_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "agv_type.h"

//-------------COMMON HEADER----------------
#define AGV_FRAME_DILIMTER_LEN 4
#define AGV_FRAME_H_0 0xff
#define AGV_FRAME_H_1 0xff
#define AGV_FRAME_H_2 0xff
#define AGV_FRAME_H_3 0xff
#define AGV_COMMON_HEADER_LEN 20
#define AGV_COMMON_HEADER_OFFSET_SRC 4
#define AGV_COMMON_HEADER_OFFSET_DEST 6
#define AGV_COMMON_HEADER_OFFSET_SQ 8
#define AGV_COMMON_HEADER_OFFSET_C 12
#define AGV_COMMON_HEADER_OFFSET_T 14
#define AGV_COMMON_HEADER_OFFSET_PLEN 16

//-------------GLOBAL DEF-------------------
#define AGV_CMD_MAX 16*1024*1024 - AGV_COMMON_HEADER_LEN

#define AGV_CMD_S_BODY_LEN 512
//todo

#define AGV_CTRL_MODULE 0                    //主控模块ID
//-------------CTRL CLASS-------------------
#define AGV_COMM_MODULE 1                    //通信模块ID
#define AGV_CMD_C_UDP 0                      //广播命令扫描以扫描子网内的下位控制器
#define AGV_CMD_T_UDP_BROADCAST 1 
//todo

#define AGV_CMD_C_CTRL 0                     //控制类
#define AGV_CMD_T_CTRL_MANI_WORD 3           //用户遥控命令
#define AGV_CMD_T_CTRL_SET_STAION 6          //执行导航目标任务
#define AGV_CMD_T_CTRL_CANCEL_MOVE 7         //取消本次导航任务
#define AGV_CMD_T_CTRL_GLOBAL_PATH 9         //返回规划的路径至Client
#define AGV_CMD_T_CTRL_MANI_REQ 11           //用户遥控启动申请

#define AGV_CMD_T_TRAJECTORY_START 12        //内部 轨迹跟踪模块 与T=6配合使用
#define AGV_CMD_T_TRAJECTORY_ACHIEVE 13      //内部 轨迹跟踪完成
#define AGV_CMD_T_TRAJECTORY_STOP 14         //内部 轨迹跟踪撤销 与T=7配合使用

//定位\建图
#define AGV_CMD_T_LOCALIZATION_RUN 15        //内部 通知SLAM模块 转换定位模式 
#define AGV_CMD_T_SLAM_RUN 16                //内部 通知SLAM模块 转换建图模式

//-------------MAP CLASS--------------------
#define AGV_CMD_C_FILE 1                     //文件操作类
#define AGV_CMD_T_MAP_ACK 0                  //预留
#define AGV_CMD_T_MAP_START_SLAM 1           //请求建图命令
#define AGV_CMD_T_MAP_STOP_SLAM 2            //结束建图命令
#define AGV_CMD_T_MAP_REPORT 3
#define AGV_CMD_T_MAP_DOWNLOAD_REQ 4         //下载地图命令

#define AGV_CMD_T_RVD_IMAGE_REQ  13            //申请靶标取图
#define AGV_CMD_T_RVD_IMAGE_RESP  14            //响应取图返回给CLIENT
#define AGV_CMD_T_RVD_POSE_RESP  15            //靶标位姿返回给CLIENT


//------------SENSOR CLASS---------------------
#define AGV_SLAM_MODULE   2                    //SLAM模块ID
#define AGV_SLAM_C_SENSOR 2
#define AGV_SLAM_T_SENSOR_LASER 0
#define AGV_SLAM_T_SENSOR_ODOM 1
#define AGV_SLAM_T_SENSOR_IMU 2
#define AGV_SLAM_T_SENSOR_MAP 3

//-----------AGV CHASSIS -------------------
#define AGV_CHASSIS_MODULE        3           //底盘控制模块ID
#define AGV_CHASSIS_C_CTRL        3
#define AGV_CHASSIS_T_VEL         0
#define AGV_CHASSIS_T_MODE_CAHNGE 1
#define AGV_CHASSIS_T_RUN         2
#define AGV_CHASSIS_T_STOP        3
#define AGV_CHASSIS_T_ERROR       4
#define AGV_CHASSIS_T_RESET       5
#define AGV_CHASSIS_T_READY       6
#define AGV_CHASSIS_T_SAFETY      7

//-----------AGV ROS DEBUG_-----------------
#define AGV_ROS_C_CTRL            8
#define AGV_ROS_T_POSE_INFO       0
#define AGV_ROS_T_ODOM_INFO       1
#define AGV_ROS_T_GLOBAL_MAP      2
#define AGV_ROS_T_AMCL_PARTICLES  3
#define AGV_ROS_T_LOCAL_MAP       4
#define AGV_ROS_T_LASER_POINTS    5


//----------AGV LOG REQ--------------
#define AGV_LOG_C_CTRL            9
#define AGV_LOG_T_START           1
#define AGV_LOG_T_STOP            2


//线程通信
//-------------MSG Struct------------------
#pragma pack (1)
typedef struct robot_msg_header
{
    //uint32_t u32_delimiter;
    uint16_t u16_src;
    uint16_t u16_dest;
    uint32_t u32_sq;
    uint16_t u16_class;
    uint16_t u16_type;
    int32_t s32_len;

}st_robot_msg_header;
#pragma pack ()


typedef struct robot_msg
{
    st_robot_msg_header t_msg_header;

    union ut_msg_body
    {
        uint8_t sbody[AGV_CMD_S_BODY_LEN];
        uint8_t *lbody;
        
    }t_msg_body;

    uint8_t res_flag;
}st_robot_msg;


int robot_msg_init(st_robot_msg *pt_msg, st_robot_msg_header t_msg_header, uint8_t *pbody);
int robot_msg_release(st_robot_msg *pt_msg);
uint8_t* robot_msg_get_body(st_robot_msg *pt_msg);


//C++ 风格
typedef struct robot_msg_header TRobotMsgHeader;
typedef struct robot_msg TRobotMsg;
#define RobotMsgInit robot_msg_init
#define RobotMsgRelease robot_msg_release
#define RobotMsgGetBody robot_msg_get_body

#endif // AGV_MSG_H
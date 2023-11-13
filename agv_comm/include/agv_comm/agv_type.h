#ifndef AGV_TYPE_H
#define AGV_TYPE_H

#include <stdio.h>
#include <stdint.h>

//坐标
typedef struct TPointd{
    double dX; //X坐标
    double dY; //Y坐标
}TPointd;
typedef struct TPoint{
    int iX; //X坐标
    int iY; //Y坐标
}TPoint;

#ifndef PI
#define PI 3.1415926
#endif


typedef enum
{
    AGV_EVENT_SUB_BASE_INFO = 1,
    AGV_EVENT_SUB_MAP_INFO = 2,
    AGV_EVENT_SUB_PATH_INFO = 3,
    AGV_EVENT_PUB_VEL_CMD = 4,
    AGV_EVENT_CALL = 5,
    AGV_EVENT_MAX
}E_AGV_EVENT_TYPE;


typedef struct udp_svr_ctrl st_udp_svr_ctrl;

typedef struct agv_msg_header st_agv_msg_header;
typedef struct agv_msg st_agv_msg;
typedef struct agv_cmd_handler st_agv_cmd_handler;

typedef struct tcp_connect_config st_tcp_connect_config;
typedef struct tcp_connect_info st_tcp_connect_info;
typedef struct tcp_connect_ctrl st_tcp_connect_ctrl;

typedef struct agv_base_info st_agv_base_info;
typedef struct agv_path_info st_agv_path_info;
typedef struct agv_map_info st_agv_map_info;
//typedef struct agv_ros_ctrl st_agv_ros_ctrl;

typedef struct agv_event st_agv_event;

typedef struct agv_ctrl st_agv_ctrl;

typedef struct agv_base_config st_agv_base_config;

#endif // AGV_TYPE_H
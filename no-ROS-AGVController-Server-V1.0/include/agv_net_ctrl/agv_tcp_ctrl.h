/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-28 15:15:49
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 10:42:32
 * @FilePath: /agv_ros/src/agv_ros_node/include/agv_ros_node/agv_tcp_ctrl.h
 * @Description: 
 */
#ifndef AGV_TCP_CTRL_H
#define AGV_TCP_CTRL_H

#include <stdint.h>
#include <pthread.h>
#include <mqueue.h>
#include <netinet/in.h>

#include "agv_type.h"
#include "sys_queue.h"
#include "agv_net_cfg.h"


//TCP 连接控制结构声明-----------------------------------------------------
typedef struct tcp_cmd_handler       st_tcp_cmd_handler;
typedef struct tcp_cmd_handler       st_udp_cmd_handler;
typedef struct tcp_connect_config    st_tcp_connect_config;
typedef struct tcp_connect_info      st_tcp_connect_info;
typedef struct tcp_connect_ctrl      st_tcp_connect_ctrl;
typedef struct tcp_period_event      st_tcp_period_event;

struct tcp_connect_config
{
    int connect_type;
    int stream_buffer_size;
    LIST_HEAD(, tcp_cmd_handler) lh_cmd_handler;
};

struct tcp_period_event
{
    E_TCP_EVENT_TYPE e_type;
    int period;
    int count;
    LIST_ENTRY(tcp_period_event) node;
};

struct tcp_connect_info
{
    int index;
    char name[64];

    int s32_sock_connect;
    pthread_t t_svr_handler;
    mqd_t q_svr_handler;

    st_tcp_connect_config *pt_config;

    pthread_mutex_t mutex_event;
    LIST_HEAD(, tcp_period_event) lh_event;

    LIST_ENTRY(tcp_connect_info) node;

    void *pRoot; //自定义结构
};

//连接结构初始化
int tcp_connect_info_init(st_tcp_connect_info *pt_info, const char *pt_name, void *pRoot);

//命令解析管理
st_tcp_connect_config *tcp_connect_config_create(E_TCP_CONNECT_TYPE e_svr_type);
int tcp_connect_config_delete(st_tcp_connect_config *pt_config);
int tcp_connect_config_regist_handler(st_tcp_connect_config *pt_config, st_tcp_cmd_handler *pt_handler);

//周期事件管理
int tcp_connect_regist_event(st_tcp_connect_info *pt_info, E_TCP_EVENT_TYPE e_type, int period);
int tcp_connect_remove_event(st_tcp_connect_info *pt_info, E_TCP_EVENT_TYPE e_type);

//连接至服务和创建本地通信线程
int tcp_connect_to_server(st_tcp_connect_info *pt_info, const char *svr_ip, int svr_port);
int tcp_connect_instance_create(st_tcp_connect_info *pt_info, void *(*client_body)(void *));


//------------------------环形队列方法---------------------------------------------------------
typedef enum
{
    E_STREAM_INIT = 0,
    E_STREAM_R_NOTCOMPLETE = 0x01,
    E_STREAM_R_COMPLETE = 0x02,
    E_STREAM_ERROR = 0x3,
    E_STREAM_MAX
}E_STERAM_BUFF_STATE;

typedef struct
{
    int size;
    int size_1;
    int r;
    int w;
    uint8_t *alloc;
    int cmd_total;
    int cmd_remaining;
    E_STERAM_BUFF_STATE state;
}st_stream_buff;

int tcp_stream_buff_init(st_stream_buff *st_stream_buff, uint8_t *p8_alloc, int s32_size);

st_stream_buff *tcp_stream_buff_create(int s32_size);
int tcp_stream_buff_delete(st_stream_buff *pt_stream_buff);

int tcp_stream_buff_in(st_stream_buff *st_stream_buff, uint8_t *p8_in, int s32_size);
int tcp_stream_buff_check(st_stream_buff *st_stream_buff, int *s32_cmd_total_size);
int tcp_stream_buff_out(st_stream_buff *st_stream_buff, uint8_t *p8_out, int s32_size);
int tcp_stream_buff_filled(st_stream_buff *st_stream_buff);

bool is_2_power(int number);


//------------------------命令处理节点--------------------------------------------

typedef int (*tcp_cmd_func)(void *data, 
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);

struct tcp_cmd_handler
{
    char name[32];
    uint16_t c;
    uint16_t t;
    tcp_cmd_func cmd_func;
    LIST_ENTRY(tcp_cmd_handler) node;
};

st_tcp_cmd_handler *agv_cmd_handler_create(const char *name, 
        uint16_t c, uint16_t t, tcp_cmd_func cmd_func);

int tcp_cmd_handler_delete(st_tcp_cmd_handler *pt_handler);
#endif // AGV_TCP_CTRL_H

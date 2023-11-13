#ifndef AGV_TCP_CTRL_H
#define AGV_TCP_CTRL_H

#include <stdint.h>
#include <pthread.h>
#include <mqueue.h>
#include <netinet/in.h>

#include <agv_comm/agv_type.h>
#include <agv_comm/sys_queue.h>
#include <agv_comm/agv_ros_ctrl.hpp>



typedef enum
{
    E_AGV_SVR_BASE = 1,
    E_AGV_SVR_FILE,
    E_AGV_SVR_MAX
}E_AGV_SVR_TYPE;

//TCP连接配置结构
struct tcp_connect_config
{
    int connect_type;
    int stream_buffer_size;
    LIST_HEAD(, agv_cmd_handler) lh_cmd_handler;
};

//TCP连接需要处理的事件列表结构
struct agv_event
{
    E_AGV_EVENT_TYPE e_type;
    int period;
    int count;
    //st_tcp_connect_info *p_connect;
    LIST_ENTRY(agv_event) node;
};
//TCP 连接控制实例结构
struct tcp_connect_info
{
    int index;
    char name[32];

    int s32_sock_connect;
    pthread_t t_svr_handler;
    mqd_t q_svr_handler;

    st_tcp_connect_ctrl *pt_connect_ctrl;
    
    CAgvRosCtrl *po_ros_ctrl;
    st_tcp_connect_config *pt_config;

    LIST_ENTRY(tcp_connect_info) node_ctrl;

    pthread_mutex_t mutex_event;
    LIST_HEAD(, agv_event) lh_event;
};

//TCP 连接控制主结构
struct tcp_connect_ctrl
{
    int s32_connect_max;
    int s32_connect_current;

    pthread_mutex_t mutex_pool;
    LIST_HEAD(, tcp_connect_info) lh_connect_pool;

    pthread_mutex_t mutex_work;
    LIST_HEAD(, tcp_connect_info) lh_connect_work;

};



int tcp_connect_ctrl_init(st_agv_ctrl *pt_ctrl, int connect_max);

int tcp_connect_info_init(st_tcp_connect_info *pt_info, int index);

st_tcp_connect_info *tcp_connect_info_add(st_tcp_connect_ctrl *pt_ctrl, 
        int s32_sock_connect, st_tcp_connect_config *t_config);

int tcp_connect_info_remove(st_tcp_connect_info *pt_info);
void tcp_connect_ctrl_watch(st_tcp_connect_ctrl *pt_ctrl);

int tcp_connect_ctrl_pub(st_agv_msg t_agv_msg);

st_tcp_connect_config *tcp_connect_config_create(E_AGV_SVR_TYPE e_svr_type);
int tcp_connect_config_delete(st_tcp_connect_config *pt_config);

int tcp_connect_config_regist_handler(st_tcp_connect_config *pt_config, st_agv_cmd_handler *pt_handler);

int tcp_connect_regist_event(st_tcp_connect_info *pt_info, E_AGV_EVENT_TYPE e_type, int period);
int tcp_connect_remove_event(st_tcp_connect_info *pt_info, E_AGV_EVENT_TYPE e_type);

int create_tcp_svr_instance(st_tcp_connect_info *pt_tcp_connect_info, 
                void *(*svr_body)(void *));

extern int agv_tcp_svr_run(st_agv_ctrl *pt_agv_ctrl);
//------------------------stream function------------------

#endif // AGV_TCP_CTRL_H

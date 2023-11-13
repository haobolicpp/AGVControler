#include <stdio.h>
#include <pthread.h>
#include <mqueue.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/stat.h>

#include "ros/ros.h"
#include "agv_comm/agv_type.h"
#include "agv_comm/agv_cmd.hpp"
#include "agv_comm/agv_ctrl.hpp"
#include "agv_comm/agv_tcp_ctrl.hpp"
#include "agv_comm/agv_tcp_svr.hpp"
#include "agv_comm/agv_file_svr.hpp"
#include "agv_comm/agv_base_svr.hpp"


static pthread_t thread_handler_tcp_svr;
void *thread_arg_tcp_svr = NULL;
void *thread_body_tcp_svr(void *arg);

/******************************************************************************
* 函数名称: agv_tcp_svr_init()
* 作 用 域: 
* 功能描述: TCP服务簇初始化
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int agv_tcp_svr_run(st_agv_ctrl *pt_agv_ctrl)
{
    int s32_ret;
    int status;
    pthread_attr_t pthread_attr;
    struct sched_param sched_param;

    if (pt_agv_ctrl == NULL)
    {
        return -1;
    }
    st_tcp_connect_ctrl *pt_connect_ctrl = &pt_agv_ctrl->t_connect_ctrl;
    //Create the thread for comm_svr
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 10;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);

    status = pthread_create(&thread_handler_tcp_svr,
                    &pthread_attr, thread_body_tcp_svr, (void *)(pt_connect_ctrl) );
    if (status != 0)
    {
        printf("tcp_ctrl thread create failed (%d)\n", status);
        return -1;
    }

    return 0;
}


/******************************************************************************
* 函数名称: thread_body_tcp_svr()
* 作 用 域: 
* 功能描述: 监听TCP连接并进行管理的线程
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void *thread_body_tcp_svr(void *arg)
{
    int s32_ret;

    int s32_sock_client;
    int s32_sock_file_svr;
    int s32_sock_ctrl_svr; //only maintain one connection here
    struct sockaddr_in t_svr_addr;
    struct sockaddr_in t_cli_addr;
    socklen_t t_cli_addr_len = sizeof(socklen_t);

    st_tcp_connect_info *pt_connect_info;
    st_tcp_connect_config *pt_connect_config;
//---------------------------Get Control Object-----------------------
    st_tcp_connect_ctrl *pt_connect_ctrl = (st_tcp_connect_ctrl *)arg;

//---------------------------CREATE SOCK------------------------------
    s32_sock_file_svr = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s32_sock_file_svr < 0)
    {
        printf("s32_sock_file_svr socket error with no.%d\n", errno);
        return NULL;
    }

    s32_sock_ctrl_svr = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s32_sock_ctrl_svr < 0)
    {
        printf("s32_sock_ctrl_svr socket error with no.%d\n", errno);
        return NULL;
    }

//----------------------------SET SOCK OPT-----------------------------
    int optval = 1;
    s32_ret = setsockopt(s32_sock_file_svr, SOL_SOCKET, SO_REUSEADDR,
                    (const char *)&optval, sizeof(optval));
    if (s32_ret < 0)
    {
        printf("thread_body_tcp_svr set socket opt error with no.%d\n", errno);
        close(s32_sock_file_svr);
        return NULL;
    }

    s32_ret = setsockopt(s32_sock_ctrl_svr, SOL_SOCKET, SO_REUSEADDR,
                    (const char *)&optval, sizeof(optval));
    if (s32_ret < 0)
    {
        printf("thread_body_tcp_svr set socket opt error with no.%d\n", errno);
        close(s32_sock_ctrl_svr);
        return NULL;
    }

//--------------------------BIND SOCK----------------------------------------
    t_svr_addr.sin_family = AF_INET;
    t_svr_addr.sin_port = htons(AGV_TCP_PORT_DATA);
    t_svr_addr.sin_addr.s_addr = htonl(INADDR_ANY); //Wifi IP OR HOST NAME
    s32_ret = bind(s32_sock_file_svr, (struct sockaddr *)&t_svr_addr, 
                sizeof(t_svr_addr));

    if (s32_ret < 0)
    {
        printf("s32_sock_tcp_svr 19200 bind failed with err %d\n", errno);
        close(s32_sock_file_svr);
        return NULL;
    }

    t_svr_addr.sin_family = AF_INET;
    t_svr_addr.sin_port = htons(AGV_TCP_PORT_CTRL);
    t_svr_addr.sin_addr.s_addr = htonl(INADDR_ANY); //Wifi IP OR HOST NAME
    s32_ret = bind(s32_sock_ctrl_svr, (struct sockaddr *)&t_svr_addr, 
                sizeof(t_svr_addr));

    if (s32_ret < 0)
    {
        printf("s32_sock_tcp_svr 19201 bind failed with err %d\n", errno);
        close(s32_sock_ctrl_svr);
        return NULL;
    }

//---------------------------------LISTEN------------------------------
    s32_ret = listen(s32_sock_file_svr, 5);
    if (s32_ret < 0)
    {
        printf("s32_sock_tcp_svr listen failed with err %d\n", errno);
        close(s32_sock_file_svr);
        return NULL;
    }

    s32_ret = listen(s32_sock_ctrl_svr, 5);
    if (s32_ret < 0)
    {
        printf("s32_sock_tcp_svr listen failed with err %d\n", errno);
        close(s32_sock_ctrl_svr);
        return NULL;
    }

    printf("All Listen Socket initialized!\n");
//---------------------------------ACCEPT------------------------------

    int s32_fd_max = 0;
	fd_set t_fd_read_set;
    s32_fd_max = s32_sock_file_svr > s32_sock_ctrl_svr ? 
    (s32_sock_file_svr + 1) : (s32_sock_ctrl_svr + 1);

    while(1)
    {
        FD_ZERO(&t_fd_read_set);
        FD_SET(s32_sock_file_svr, &t_fd_read_set);
        FD_SET(s32_sock_ctrl_svr, &t_fd_read_set);
        s32_ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, NULL);
        if (s32_ret < 0)
        {
            printf("agv_tcp_svr select with err %d\n", errno);
            continue;
        }

        if (FD_ISSET(s32_sock_file_svr, &t_fd_read_set))
        {
            s32_sock_client = accept(s32_sock_file_svr, 
                    (struct sockaddr*)&t_cli_addr, &t_cli_addr_len);
            if(s32_sock_client < 0)
                continue;
            
            pt_connect_config = tcp_connect_config_create(E_AGV_SVR_FILE);
            pt_connect_info = tcp_connect_info_add(pt_connect_ctrl, 
                        s32_sock_client, pt_connect_config);
            if (pt_connect_info == NULL)
            {
                printf("tcp connect limited\n");
                close(s32_sock_client);
                continue;
            }
            tcp_connect_ctrl_watch(pt_connect_ctrl);

            s32_ret = create_tcp_svr_instance(pt_connect_info, thread_tcp_svr_instance);
            assert(s32_ret == 0);
            printf("tcp instance file svr created!\n");

        }

        if (FD_ISSET(s32_sock_ctrl_svr, &t_fd_read_set))
        {
            s32_sock_client = accept(s32_sock_ctrl_svr, 
                    (struct sockaddr*)&t_cli_addr, &t_cli_addr_len);
            if(s32_sock_client < 0)
                continue;
            
            pt_connect_config = tcp_connect_config_create(E_AGV_SVR_BASE);
            pt_connect_info = tcp_connect_info_add(pt_connect_ctrl, s32_sock_client, pt_connect_config);
            if (pt_connect_info == NULL)
            {
                printf("tcp connect limited\n");
                close(s32_sock_client);
                continue;
            }
            
            s32_ret = tcp_connect_regist_event(pt_connect_info, AGV_EVENT_SUB_BASE_INFO, 10);
            assert(s32_ret == 0);
            tcp_connect_ctrl_watch(pt_connect_ctrl);

            s32_ret = create_tcp_svr_instance(pt_connect_info, thread_tcp_svr_instance);
            printf("tcp instance file svr created!\n");
        }

    }

    return NULL;
}


/******************************************************************************
* 函数名称: create_tcp_svr_instance()
* 作 用 域: 
* 功能描述: 创建一个连接服务实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int create_tcp_svr_instance(st_tcp_connect_info *pt_tcp_connect_info, 
                void *(*svr_body)(void *))
{
    int s32_ret;

    pthread_attr_t pthread_attr;
    struct sched_param sched_param;

    //Create the thread for comm_svr
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 10;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);

    s32_ret = pthread_create(&pt_tcp_connect_info->t_svr_handler,
                &pthread_attr, svr_body, (void *)(pt_tcp_connect_info) );
    if (s32_ret != 0)
    {
        printf("create_tcp_svr_instance create failed (%d)\n", s32_ret);
        return -1;
    }

}



/******************************************************************************
* 函数名称: tcp_connect_ctrl_init()
* 作 用 域: 
* 功能描述: 连接控制结构初始化
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int tcp_connect_ctrl_init(st_agv_ctrl *pt_agv_ctrl, int connect_max)
{
    int i;
    int s32_ret;

    st_tcp_connect_ctrl *pt_connect_ctrl;
    st_tcp_connect_info *pt_connect_info;

    if (pt_agv_ctrl == NULL || connect_max <= 0 || connect_max > 8)
    {
        return -1;
    }

    pt_connect_ctrl = &pt_agv_ctrl->t_connect_ctrl;

    //创建用于连接池的锁
    s32_ret = pthread_mutex_init(&pt_connect_ctrl->mutex_pool, NULL);
    if (s32_ret != 0)
    {
        return -1;
    }

    //创建用于工作队列的锁
    s32_ret = pthread_mutex_init(&pt_connect_ctrl->mutex_work, NULL);
    if (s32_ret != 0)
    {
        return -1;
    }

    //初始化连接池和工作队列链表
    LIST_INIT(&pt_connect_ctrl->lh_connect_pool);
    LIST_INIT(&pt_connect_ctrl->lh_connect_work);

    //初始化连接池
    for(i = 0; i < connect_max; i++)
    {
        pt_connect_info = (st_tcp_connect_info *)malloc(sizeof(st_tcp_connect_info));
        if(pt_connect_info == NULL)
        {
            return -1;
        }

        //初始化一个连接 用于消息队列的名字 唯一性保证
        s32_ret = tcp_connect_info_init(pt_connect_info, i);
        if(s32_ret != 0)
        {
            return -1;
        }
        //使本连接获知当前连接控制结构的指针 用于后期归还至连接池
        pt_connect_info->pt_connect_ctrl = pt_connect_ctrl;
        pt_connect_info->po_ros_ctrl = pt_agv_ctrl->po_ros_ctrl;

        //将连接放入连接池
        LIST_INSERT_HEAD(&pt_connect_ctrl->lh_connect_pool, pt_connect_info, node_ctrl);
    }

    return 0;


}

/******************************************************************************
* 函数名称: tcp_connect_info_init()
* 作 用 域: 
* 功能描述: 初始化一个连接 初始化一些静态资源
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int tcp_connect_info_init(st_tcp_connect_info *pt_info, int index)
{
    int s32_ret;

    if (pt_info == NULL)
    {
        return -1;
    }
    pt_info->index = index;
    snprintf(pt_info->name, 32, "/agv-tcp-conn-%d", index);

    pt_info->s32_sock_connect = -1;
    pt_info->pt_config =NULL;
    //创建用于事件队列的多线程锁
    s32_ret = pthread_mutex_init(&pt_info->mutex_event, NULL);
    if (s32_ret != 0)
    {
        return -1;
    }
    LIST_INIT(&pt_info->lh_event);

    return 0;
}

/******************************************************************************
* 函数名称: tcp_connect_info_create()
* 作 用 域: 
* 功能描述: 连接控制结构初始化 创建一些动态资源
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
st_tcp_connect_info *tcp_connect_info_add(st_tcp_connect_ctrl *pt_ctrl, 
        int s32_sock_connect, st_tcp_connect_config *pt_config)
{

    st_tcp_connect_info *pt_connect_info = NULL;
    
    if (LIST_EMPTY(&pt_ctrl->lh_connect_pool))
    {
        return NULL;
    }


    //在连接池中获取一个连接元素
    pthread_mutex_lock(&pt_ctrl->mutex_pool);
    pt_connect_info = LIST_FIRST(&pt_ctrl->lh_connect_pool);
    LIST_REMOVE(pt_connect_info, node_ctrl);
    pthread_mutex_unlock(&pt_ctrl->mutex_pool);

    //设置相关属性
    pt_connect_info->s32_sock_connect = s32_sock_connect;
    pt_connect_info->pt_config = pt_config;
    //打开消息队列
    struct mq_attr t_mq_attr = {0};
	t_mq_attr.mq_flags = 0; //Block Mode
	t_mq_attr.mq_msgsize = sizeof(st_agv_msg);
	t_mq_attr.mq_maxmsg = 8;

    mq_unlink(pt_connect_info->name);

    mode_t omask;
    omask = umask(0); //返回之前的文件权限默认值，同时修改默认的umask数值为0，这样后面设置文件权限的时候，umask值无影响了（创建文件时，文件权限的计算方式是：umask取反 & 要设定的文件权限）
    pt_connect_info->q_svr_handler = mq_open(pt_connect_info->name, 
                O_RDWR | O_CREAT | O_NONBLOCK , (S_IRWXU | S_IRWXG | S_IRWXO) /* 777 其他用户可修改*/,  &t_mq_attr);
    umask(omask);
    if (-1 == pt_connect_info->q_svr_handler)
    {
        printf("mq_open error:%d\n",errno);
    }
    assert(pt_connect_info->q_svr_handler >= 0);

    //插入工作队列
    pthread_mutex_lock(&pt_ctrl->mutex_work);
    LIST_INSERT_HEAD(&pt_ctrl->lh_connect_work, pt_connect_info, node_ctrl);
    pthread_mutex_unlock(&pt_ctrl->mutex_work);


    return pt_connect_info;
}

/******************************************************************************
* 函数名称: tcp_connect_info_remove()
* 作 用 域: 
* 功能描述: 处理失效的连接，并归还其至连接池
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int tcp_connect_info_remove(st_tcp_connect_info *pt_connect_info)
{
    if (pt_connect_info == NULL)
    {
        return -1;
    }

    st_tcp_connect_ctrl *pt_ctrl = pt_connect_info->pt_connect_ctrl;

    if (pt_ctrl == NULL)
    {
        return -1;
    }

    //移除工作队列
    pthread_mutex_lock(&pt_ctrl->mutex_work);
    LIST_REMOVE(pt_connect_info, node_ctrl);
    pthread_mutex_unlock(&pt_ctrl->mutex_work);

//清理动态资源 //unlink q?------------------------------------------------------------
    mq_close(pt_connect_info->q_svr_handler);
    close(pt_connect_info->s32_sock_connect);
    pt_connect_info->s32_sock_connect = -1;
    tcp_connect_config_delete(pt_connect_info->pt_config);
    //释放事件队列----------------------------------------
    st_agv_event *pt_event, *pt_event_tmp;
    pthread_mutex_lock(&pt_connect_info->mutex_event);
    LIST_FOREACH_SAFE(pt_event, &pt_connect_info->lh_event, node, pt_event_tmp)
    {
        LIST_REMOVE(pt_event, node);
        free(pt_event);
        //agv_event_delete(pt_event)
    }
    pthread_mutex_unlock(&pt_connect_info->mutex_event);
//清理动态资源------------------------------------------------------------------------    

    pthread_mutex_lock(&pt_ctrl->mutex_pool);
    LIST_INSERT_HEAD(&pt_ctrl->lh_connect_pool, pt_connect_info, node_ctrl);
    pthread_mutex_unlock(&pt_ctrl->mutex_pool);

    return 0;
}



/******************************************************************************
* 函数名称: tcp_connect_ctrl_watch()
* 作 用 域: 
* 功能描述: 查看连接资源
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void tcp_connect_ctrl_watch(st_tcp_connect_ctrl *pt_ctrl)
{
    int i;
    st_tcp_connect_info *pt_connect_info;
    
    i = 0;
    printf("TCP-CONNECT-CTRL-POOL:---------------------\n");
    LIST_FOREACH(pt_connect_info, &pt_ctrl->lh_connect_pool, node_ctrl)
    {
        printf("[%.2d]-%s\n", i, pt_connect_info->name);
        i++;
    }

    i = 0;
    printf("TCP-CONNECT-CTRL-WORK:---------------------\n");
    LIST_FOREACH(pt_connect_info, &pt_ctrl->lh_connect_work, node_ctrl)
    {
        printf("[%.2d]-%s\n", i, pt_connect_info->name);
        i++;
    }
}



/******************************************************************************
* 函数名称: tcp_connect_ctrl_watch()
* 作 用 域: 
* 功能描述: 查看连接资源
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int tcp_connect_ctrl_pub(st_agv_msg t_agv_msg)
{
    return 0;
}


/******************************************************************************
* 函数名称: tcp_connect_config_create()
* 作 用 域: 
* 功能描述: 创建对应实例的服务配置结构实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
st_tcp_connect_config *tcp_connect_config_create(E_AGV_SVR_TYPE e_svr_type)
{
    st_tcp_connect_config *pt_config;
    
    if (e_svr_type == E_AGV_SVR_BASE)
    {
        pt_config = base_svr_config_create();
    }
    else if (e_svr_type == E_AGV_SVR_FILE)
    {
        pt_config = file_svr_config_create();
    }
    else
    {
        return NULL;
    }
}

/******************************************************************************
* 函数名称: tcp_connect_config_delete()
* 作 用 域: 
* 功能描述: 销毁对应实例的服务配置结构实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int tcp_connect_config_delete(st_tcp_connect_config *pt_config)
{
    int s32_ret;
    st_agv_cmd_handler *pt_handler;
    st_agv_cmd_handler *pt_handler_tmp;
    
    if (pt_config == NULL)
    {
        return -1;
    }
    
    //Delete and Free The Cmd Handler
    LIST_FOREACH_SAFE(pt_handler, &pt_config->lh_cmd_handler, node, pt_handler_tmp)
    {
        LIST_REMOVE(pt_handler, node);
        agv_cmd_handler_delete(pt_handler);
    }
    

    return s32_ret;
}

/******************************************************************************
* 函数名称: tcp_connect_config_regist_handler()
* 作 用 域: 
* 功能描述: 向TCP连接的实例内注册AGV命令处理实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int tcp_connect_config_regist_handler(st_tcp_connect_config *pt_config, st_agv_cmd_handler *pt_handler)
{
    if (pt_config == NULL || pt_handler ==NULL)
    {
        return -1;
    }

    LIST_INSERT_HEAD(&pt_config->lh_cmd_handler, pt_handler, node);

    return 0;

}


/******************************************************************************
* 函数名称: tcp_connect_regist_event()
* 作 用 域: 
* 功能描述: 向TCP连接的实例内注册AGV命令处理实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int tcp_connect_regist_event(st_tcp_connect_info *pt_info, E_AGV_EVENT_TYPE e_type, int period)
{
    st_agv_event *pt_event;

    if (pt_info == NULL)
    {
        return -1;
    }

//if the event exist, just modified it-----------------------------
    pthread_mutex_lock(&pt_info->mutex_event);
    LIST_FOREACH(pt_event, &pt_info->lh_event, node)
    {
        if (pt_event->e_type == e_type)
        {
            pt_event->period = period;
            pt_event->count = 0;
        }
    }
    pthread_mutex_unlock(&pt_info->mutex_event);
//if the event not exist, create and add it-------------------------
    pt_event = (st_agv_event *)malloc(sizeof(st_agv_event));
    if (pt_event == NULL)
    {
        return -1;
    }
    pt_event->e_type = e_type;
    pt_event->period = period;
    pt_event->count = 0;
    LIST_INSERT_HEAD(&pt_info->lh_event, pt_event, node);

    return 0;
}

/******************************************************************************
* 函数名称: tcp_connect_remove_event()
* 作 用 域: 
* 功能描述: 向TCP连接的实例内注册AGV命令处理实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int tcp_connect_remove_event(st_tcp_connect_info *pt_info, E_AGV_EVENT_TYPE e_type)
{
    st_agv_event *pt_event, *pt_event_tmp;

    if (pt_info == NULL)
    {
        return -1;
    }

//if the event exist, just modified it-----------------------------
    if (0 == pthread_mutex_lock(&pt_info->mutex_event))
    {
        LIST_FOREACH_SAFE(pt_event, &pt_info->lh_event, node, pt_event_tmp)
        {
            if (pt_event->e_type == e_type)
            {
                LIST_REMOVE(pt_event, node);
                free(pt_event);
            }
        }
    }
    else
    {
        return -1;
    }
    pthread_mutex_unlock(&pt_info->mutex_event);

    return 0;
}
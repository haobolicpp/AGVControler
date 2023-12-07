/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-08 17:04:19
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-13 10:05:12
 * @FilePath: /agv_controller/src/agv_net_ctrl/agv_tcp_svr.cpp
 * @Description: TCP Server实现
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <mqueue.h>
#include <errno.h>
#include <net/if.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h> 
#include <sys/stat.h> 

#include "sys_queue.h"
#include "agv_net_cfg.h"
#include "agv_tcp_ctrl.h"
#include "agv_tcp_inst.h"
#include "agv_tcp_svr.h"

#include "agv_base_svr.h"
#include "agv_file_svr.h"
#include "agv_ros_svr.h"

#define ROBOT_TCP_CONNECT_MAX 6
/**
 * @name: thread_agv_tcp_svr
 * @des:  定制的AGV TCP服务端
 * @param {void} *arg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
#include "AgvCtrl.h"
#include "AgvRosSvr.h"
void *thread_agv_tcp_svr(void *arg);
void robot_tcp_svr_watch(st_robot_tcp_svr *pt_tcp_svr);
/**
 * @name: robot_tcp_svr_init
 * @des:  tcp
 * @param {st_robot_tcp_svr} *pt_tcp_svr
 * @param {int} connect_max
 * @param {void} *pRoot
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int robot_tcp_svr_init(st_robot_tcp_svr *pt_tcp_svr, int connect_max, void *pRoot)
{
    int i;
    int ret;

    st_tcp_connect_info *pt_connect_info;

    if (connect_max <= 0 || connect_max > ROBOT_TCP_CONNECT_MAX)
    {
        printf("ERROR: TCP Server Initialize With Connect Limit!\n");
        return -1;
    }

    //创建用于连接池的锁
    ret = pthread_mutex_init(&pt_tcp_svr->mutex_connect, NULL);
    assert(ret == 0);
    //创建用于工作队列的锁
    // ret = pthread_mutex_init(&pt_tcp_svr->mutex_work, NULL);
    // assert(ret == 0);

    //初始化连接池和工作队列链表
    LIST_INIT(&pt_tcp_svr->lh_connect_pool);
    LIST_INIT(&pt_tcp_svr->lh_connect_work);
    pt_tcp_svr->pRoot = pRoot;
    char pt_connect_name[64] = {0};
    //初始化连接池
    for(i = 0; i < connect_max; i++)
    {
        pt_connect_info = (st_tcp_connect_info *)malloc(sizeof(st_tcp_connect_info));
        if(pt_connect_info == NULL)
        {
            return -1;
        }
        memset(pt_connect_name, 0, 64);
        snprintf(pt_connect_name, 64, "robot_tcp_svr_q_%d", i);
        //初始化一个连接 用于消息队列的名字 唯一性保证
        ret = tcp_connect_info_init(pt_connect_info, pt_connect_name, pRoot);
        if(ret != 0)
        {
            return -1;
        }

        //将连接放入连接池
        LIST_INSERT_HEAD(&pt_tcp_svr->lh_connect_pool, pt_connect_info, node);
    }

    return 0;
}

/**
 * @name: robot_tcp_svr_run
 * @des:  TCP服务启动
 * @param {st_robot_tcp_svr} *pt_tcp_svr
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int robot_tcp_svr_run(st_robot_tcp_svr *pt_tcp_svr)
{
    int ret;

    pthread_attr_t pthread_attr;
    struct sched_param sched_param;

    assert(pt_tcp_svr != NULL);

    //创建TCP服务端监听线程
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 10;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);
    ret = pthread_create(&pt_tcp_svr->th_tcp_svr,
                    &pthread_attr, thread_agv_tcp_svr, (void *)(pt_tcp_svr) );
    assert(ret == 0);

    return 0;
}


/**
 * @name: robot_tcp_svr_add
 * @des:  服务端增加一个工作连接
 * @param {st_tcp_connect_ctrl} *pt_ctrl
 * @param {int} sock_connect
 * @param {st_tcp_connect_config} *pt_config
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
st_tcp_connect_info *robot_tcp_svr_add(st_robot_tcp_svr *pt_tcp_svr, 
    int sock_connect, st_tcp_connect_config *pt_config)
{
    assert(pt_tcp_svr != NULL);
    assert(pt_config  != NULL);

    st_tcp_connect_info *pt_connect_info = NULL;
    
    if (LIST_EMPTY(&pt_tcp_svr->lh_connect_pool))
    {
        return NULL;
    }

    //在连接池中获取一个连接元素
    pthread_mutex_lock(&pt_tcp_svr->mutex_connect);
    pt_connect_info = LIST_FIRST(&pt_tcp_svr->lh_connect_pool);
    LIST_REMOVE(pt_connect_info, node);
    pthread_mutex_unlock(&pt_tcp_svr->mutex_connect);

    //设置相关属性
    pt_connect_info->s32_sock_connect = sock_connect;
    pt_connect_info->pt_config = pt_config;

    //定义消息队列属性
    struct mq_attr t_mq_attr = {0};
	t_mq_attr.mq_flags = 0; //Block Mode
	t_mq_attr.mq_msgsize = sizeof(st_robot_msg);
	t_mq_attr.mq_maxmsg = 8;

    //创建消息队列
    mq_unlink(pt_connect_info->name);
    //返回之前的文件权限默认值，同时修改默认的umask数值为0，这样后面设置文件权限的时候，
    //umask值无影响了（创建文件时，文件权限的计算方式是：umask取反 & 要设定的文件权限）
    mode_t omask;
    omask = umask(0); 
    pt_connect_info->q_svr_handler = mq_open(pt_connect_info->name, 
                O_RDWR | O_CREAT | O_NONBLOCK , (S_IRWXU | S_IRWXG | S_IRWXO),  &t_mq_attr);
    umask(omask);
    if (pt_connect_info->q_svr_handler < 0)
    {
        printf("ERROR: TCP Connect INFO Q Create Failed: %s\n", strerror(errno));
        pthread_mutex_lock(&pt_tcp_svr->mutex_connect);
        LIST_INSERT_HEAD(&pt_tcp_svr->lh_connect_pool, pt_connect_info, node);
        pthread_mutex_unlock(&pt_tcp_svr->mutex_connect);
        return NULL;
    }

    //插入工作队列
    pthread_mutex_lock(&pt_tcp_svr->mutex_connect);
    LIST_INSERT_HEAD(&pt_tcp_svr->lh_connect_work, pt_connect_info, node);
    pthread_mutex_unlock(&pt_tcp_svr->mutex_connect);


    return pt_connect_info;
}

/**
 * @name: 
 * @des: 
 * @param {st_tcp_connect_info} *pt_connect_info
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int robot_tcp_svr_remove(st_robot_tcp_svr *pt_tcp_svr, st_tcp_connect_info *pt_connect_info)
{
    if (pt_tcp_svr == NULL || pt_connect_info == NULL)
    {
        return -1;
    }

    //移除工作队列
    pthread_mutex_lock(&pt_tcp_svr->mutex_connect);
    LIST_REMOVE(pt_connect_info, node);
    pthread_mutex_unlock(&pt_tcp_svr->mutex_connect);

    //清理动态资源---------------------------------------- 
    mq_close(pt_connect_info->q_svr_handler);
    close(pt_connect_info->s32_sock_connect);
    pt_connect_info->s32_sock_connect = -1;
    tcp_connect_config_delete(pt_connect_info->pt_config);
    //释放事件队列----------------------------------------
    tcp_period_event *pt_event, *pt_event_tmp;
    pthread_mutex_lock(&pt_connect_info->mutex_event);
    LIST_FOREACH_SAFE(pt_event, &pt_connect_info->lh_event, node, pt_event_tmp)
    {
        LIST_REMOVE(pt_event, node);
        free(pt_event);
    }
    pthread_mutex_unlock(&pt_connect_info->mutex_event);

    
    pthread_mutex_lock(&pt_tcp_svr->mutex_connect);
    LIST_INSERT_HEAD(&pt_tcp_svr->lh_connect_pool, pt_connect_info, node);
    pthread_mutex_unlock(&pt_tcp_svr->mutex_connect);

    return 0;
}

/**
 * @name: robot_tcp_svr_check
 * @des:  检查失效连接并回收资源
 * @param {st_robot_tcp_svr} *pt_tcp_svr
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int robot_tcp_svr_check(st_robot_tcp_svr *pt_tcp_svr)
{
    int flag = 0;

    if (pt_tcp_svr == NULL )
    {
        return -1;
    }

    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *>(pt_tcp_svr->pRoot);
    st_tcp_connect_info *pt_connect_info, *pt_connect_info_tmp;
    tcp_period_event *pt_event, *pt_event_tmp;

    pthread_mutex_lock(&pt_tcp_svr->mutex_connect);
    LIST_FOREACH_SAFE(pt_connect_info, &pt_tcp_svr->lh_connect_work, node, pt_connect_info_tmp)
    {
        if (pt_connect_info->s32_sock_connect < 0) //失效连接
        {
            flag = 1;
            LIST_REMOVE(pt_connect_info, node);

            if (pt_connect_info->pt_config->connect_type == E_AGV_SVR_ROS)
            {
                poAgvCtrl->poAgvRosSvr_->DeInit();
            }

        //清理服务配置与消息队列---------------------------------------- 
            mq_close(pt_connect_info->q_svr_handler);
            tcp_connect_config_delete(pt_connect_info->pt_config);
        //清理周期事件队列---------------------------------------------
            pthread_mutex_lock(&pt_connect_info->mutex_event);
            LIST_FOREACH_SAFE(pt_event, &pt_connect_info->lh_event, node, pt_event_tmp)
            {
                LIST_REMOVE(pt_event, node);
                free(pt_event);
            }
            pthread_mutex_unlock(&pt_connect_info->mutex_event);
        //归还连接资源------------------------------------------------
            LIST_INSERT_HEAD(&pt_tcp_svr->lh_connect_pool, pt_connect_info, node);
        }
    }
    pthread_mutex_unlock(&pt_tcp_svr->mutex_connect);

    if (flag != 0)
    {
        robot_tcp_svr_watch(pt_tcp_svr);
    }

    return 0;
}
/**
 * @name: robot_tcp_svr_watch
 * @des: 
 * @param {st_tcp_connect_ctrl} *pt_ctrl
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
void robot_tcp_svr_watch(st_robot_tcp_svr *pt_tcp_svr)
{
    int i;
    st_tcp_connect_info *pt_connect_info;
    
    i = 0;
    printf("INFO: Robot Tcp Server Connect [Idle]----->\n");
    LIST_FOREACH(pt_connect_info, &pt_tcp_svr->lh_connect_pool, node)
    {
        printf("[%.2d]-%s\n", i, pt_connect_info->name);
        i++;
    }

    i = 0;
    printf("INFO: Robot Tcp Server Connect [Work]----->\n");
    LIST_FOREACH(pt_connect_info, &pt_tcp_svr->lh_connect_work, node)
    {
        printf("[%.2d]-%s\n", i, pt_connect_info->name);
        i++;
    }
}
/**
 * @name: thread_agv_tcp_svr
 * @des:  自定义AGV通信服务端
 * @param {void} *arg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
void *thread_agv_tcp_svr(void *arg)
{
    int ret;

    int sock_client;

    int sock_file_svr;
    int sock_ctrl_svr; //only maintain one connection here
    int sock_ros_svr;

    struct sockaddr_in t_svr_addr;
    struct sockaddr_in t_cli_addr;
    socklen_t t_cli_addr_len = sizeof(socklen_t);

    st_tcp_connect_info *pt_connect_info;
    st_tcp_connect_config *pt_connect_config;

    st_robot_tcp_svr *pt_tcp_svr = (st_robot_tcp_svr *)arg;
    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *>(pt_tcp_svr->pRoot);

//---------------------------CREATE SOCK------------------------------
    sock_file_svr = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    assert(sock_file_svr > 0);

    sock_ctrl_svr = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    assert(sock_ctrl_svr > 0);

    sock_ros_svr =  socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    assert(sock_ros_svr  > 0);
//----------------------------SET SOCK OPT-----------------------------
    int optval = 1;
    ret = setsockopt(sock_file_svr, SOL_SOCKET, SO_REUSEADDR,
        (const char *)&optval, sizeof(optval));
    assert(ret == 0);

    ret = setsockopt(sock_ctrl_svr, SOL_SOCKET, SO_REUSEADDR,
        (const char *)&optval, sizeof(optval));
    assert(ret == 0);

    ret = setsockopt(sock_ros_svr, SOL_SOCKET, SO_REUSEADDR,
        (const char *)&optval, sizeof(optval));
    assert(ret == 0);

//--------------------------BIND SOCK----------------------------------------
    //----------------------------BIND DATA PORT-----------------------------
    t_svr_addr.sin_family = AF_INET;
    t_svr_addr.sin_port = htons(AGV_TCP_PORT_FILE);
    t_svr_addr.sin_addr.s_addr = htonl(INADDR_ANY); 
    ret = bind(sock_file_svr, (struct sockaddr *)&t_svr_addr, 
                sizeof(t_svr_addr));
    assert(ret == 0);
    //----------------------------BIND CTRL PORT-----------------------------
    t_svr_addr.sin_family = AF_INET;
    t_svr_addr.sin_port = htons(AGV_TCP_PORT_BASE);
    t_svr_addr.sin_addr.s_addr = htonl(INADDR_ANY); //Wifi IP OR HOST NAME
    ret = bind(sock_ctrl_svr, (struct sockaddr *)&t_svr_addr, 
                sizeof(t_svr_addr));
    assert(ret == 0);
    //----------------------------BIND ROS PORT-----------------------------
    t_svr_addr.sin_family = AF_INET;
    t_svr_addr.sin_port = htons(AGV_TCP_PORT_ROS);
    t_svr_addr.sin_addr.s_addr = htonl(INADDR_ANY); //Wifi IP OR HOST NAME
    ret = bind(sock_ros_svr, (struct sockaddr *)&t_svr_addr, 
                sizeof(t_svr_addr));
    assert(ret == 0);
//---------------------------------LISTEN------------------------------
    ret = listen(sock_file_svr, 2);
    assert(ret == 0);
    ret = listen(sock_ctrl_svr, 2);
    assert(ret == 0);
    ret = listen(sock_ros_svr,  2);
    assert(ret == 0);
    printf("INFO: TCP Server All Socket initialized!\n");
//---------------------------------ACCEPT------------------------------
    int s32_fd_max = 0;
	fd_set t_fd_read_set;
    s32_fd_max = sock_file_svr > sock_ctrl_svr ? sock_file_svr : sock_ctrl_svr;
    s32_fd_max = s32_fd_max > sock_ros_svr ? s32_fd_max : sock_ros_svr;
    s32_fd_max = s32_fd_max + 1;

    struct timeval t_tmptv;
    t_tmptv.tv_sec = 1;
    t_tmptv.tv_usec = 0;

    while(1)
    {
        FD_ZERO(&t_fd_read_set);
        FD_SET(sock_file_svr, &t_fd_read_set);
        FD_SET(sock_ctrl_svr, &t_fd_read_set);
        FD_SET(sock_ros_svr,  &t_fd_read_set);

        ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, &t_tmptv);
        if (ret < 0)
        {
            printf("ERROR: TCP Server Select with err %d\n", errno);
            continue;
        }
        //周期事件处理
        else if (ret == 0)
        {
            t_tmptv.tv_sec = 1;
            t_tmptv.tv_usec = 0;
            robot_tcp_svr_check(pt_tcp_svr);
            continue;
        }
        //大数据通道实例
        else if (FD_ISSET(sock_file_svr, &t_fd_read_set))
        {
            sock_client = accept(sock_file_svr, (struct sockaddr*)&t_cli_addr, &t_cli_addr_len);
            if(sock_client < 0)
                continue;
            
            pt_connect_config = file_svr_config_create();
            pt_connect_info = robot_tcp_svr_add(pt_tcp_svr, sock_client, pt_connect_config);
            if (pt_connect_info == NULL)
            {
                printf("ERROR: TCP Server Connect Limited\n");
                close(sock_client);
                continue;
            }
            robot_tcp_svr_watch(pt_tcp_svr);//打印连接资源
            ret = tcp_connect_instance_create(pt_connect_info);
            assert(ret == 0);
            printf("INFO: TCP File Client Connect With IP-[%s]!\n", inet_ntoa(t_cli_addr.sin_addr));

        }
        //控制命令传输实例
        else if (FD_ISSET(sock_ctrl_svr, &t_fd_read_set))
        {
            sock_client = accept(sock_ctrl_svr, (struct sockaddr*)&t_cli_addr, &t_cli_addr_len);
            if(sock_client < 0)
                continue;
            
            pt_connect_config = base_svr_config_create();
            pt_connect_info = robot_tcp_svr_add(pt_tcp_svr, sock_client, pt_connect_config);
            if (pt_connect_info == NULL)
            {
                printf("ERROR: TCP Server Connect Limited\n");
                close(sock_client);
                continue;
            }
            //周期传输AGV状态信息到上位机
            ret = tcp_connect_regist_event(pt_connect_info, AGV_EVENT_SUB_BASE_INFO, 10);
            assert(ret == 0);
            robot_tcp_svr_watch(pt_tcp_svr); //打印连接资源
            ret = tcp_connect_instance_create(pt_connect_info);
            assert(ret == 0);
            printf("INFO: TCP Ctrl Client Connect With IP-[%s]!\n", inet_ntoa(t_cli_addr.sin_addr));
        }
        //ROS调试端通信实例
        else if (FD_ISSET(sock_ros_svr, &t_fd_read_set))
        {
            sock_client = accept(sock_ros_svr, (struct sockaddr*)&t_cli_addr, &t_cli_addr_len);
            if(sock_client < 0)
                continue;
            //连接实例为ROS-SVR
            pt_connect_config = ros_svr_config_create();
            pt_connect_info = robot_tcp_svr_add(pt_tcp_svr, sock_client, pt_connect_config);
            if (pt_connect_info == NULL)
            {
                printf("ERROR: TCP Server Connect Limited\n");
                close(sock_client);
                continue;
            }
            //连接限制唯一
            if (poAgvCtrl->poAgvRosSvr_->bRosConnected)
            {
                printf("Warn: TCP ROS Server Connect limited!\n");
                robot_tcp_svr_remove(pt_tcp_svr, pt_connect_info);
                close(sock_client);
                continue;
            }
            robot_tcp_svr_watch(pt_tcp_svr); //打印连接资源
            ret = tcp_connect_instance_create(pt_connect_info);
            assert(ret == 0);
            //初始化ROS调试接口
            poAgvCtrl->poAgvRosSvr_->Init(pt_connect_info);
            printf("INFO: TCP ROS Client Connect With IP-[%s]!\n", inet_ntoa(t_cli_addr.sin_addr));
        }

    }

    return NULL;
}




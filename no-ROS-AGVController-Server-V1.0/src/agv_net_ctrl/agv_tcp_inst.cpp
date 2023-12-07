/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-11 13:50:42
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 16:26:26
 * @FilePath: /agv_controller/src/agv_net_ctrl/agv_tcp_inst.cpp
 * @Description: TCP连接实例 线程处理流程泛化
 */


#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <mqueue.h>
#include <errno.h>
#include <assert.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "sys_queue.h"
#include "agv_type.h"
#include "agv_cmd.h"

#include "agv_tcp_ctrl.h"
#include "agv_tcp_inst.h"


static void *thread_tcp_instance(void *arg);

static int tcp_stream_buff_send(st_stream_buff *pt_send_stream_buff, 
            st_tcp_connect_info *pt_info, uint8_t *p8_tcp_buff, int s32_tcp_buff_size);

static int tcp_thread_msg_process(st_robot_msg t_agv_msg, uint8_t *file_cmd, int *file_cmd_len);


/**
 * @name: tcp_client_instance_create
 * @des:  创建客户端线程实例
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_connect_instance_create(st_tcp_connect_info *pt_info)
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

    s32_ret = pthread_create(&pt_info->t_svr_handler,
                &pthread_attr, thread_tcp_instance, (void *)(pt_info) );
    if (s32_ret != 0)
    {
        printf("ERROR: TCP Connect Instance Create Failed With [%d]\n", s32_ret);
        return -1;
    }
    return 0;
}

/**
 * @name: thread_tcp_instance
 * @des:  tcp连接 服务线程
 * @param {void} *arg
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
void *thread_tcp_instance(void *arg)
{
    int s32_ret;
    int s32_bytes;
    int s32_cmd_ret;

    uint8_t p8_recv_buff[AGV_TCP_BUFF_SIZE];
    uint8_t p8_send_buff[AGV_TCP_BUFF_SIZE];

    st_robot_msg t_agv_msg;
    st_robot_msg_header *pt_agv_msg_head;

    st_tcp_cmd_handler *pt_handler;

    int cmd_recv_len;
    int cmd_send_len;
    //获取套接字 和 消息队列

    st_tcp_connect_info *pt_tcp_connect_info = (st_tcp_connect_info *)arg;
    int s32_sock_connect = pt_tcp_connect_info->s32_sock_connect;
    mqd_t q_svr_handler = pt_tcp_connect_info->q_svr_handler;
    st_tcp_connect_config *pt_config = pt_tcp_connect_info->pt_config;

    int s32_stream_buff_size = pt_tcp_connect_info->pt_config->stream_buffer_size;
    int s32_cmd_buff_size = s32_stream_buff_size/2;

    printf("INFO: TCP Stream Buffer Size: %d\n", s32_stream_buff_size);

    //创建用于命令解析的缓存
    uint8_t *pt_recv_cmd = (uint8_t *)malloc(s32_cmd_buff_size);
    assert(pt_recv_cmd != NULL);
    uint8_t *pt_send_cmd = (uint8_t *)malloc(s32_cmd_buff_size);
    assert(pt_send_cmd != NULL);

    //创建用于接收的环形队列
    st_stream_buff *pt_recv_stream_buff = tcp_stream_buff_create(s32_stream_buff_size);
    assert(pt_recv_stream_buff != NULL);

    //创建用于发送的环形队列
    st_stream_buff *pt_send_stream_buff = tcp_stream_buff_create(s32_stream_buff_size);
    assert(pt_send_stream_buff != NULL);

    //设置套接字属性为非阻塞
    int opts = fcntl(s32_sock_connect, F_GETFL);
    s32_ret = fcntl(s32_sock_connect, F_SETFL, opts | O_NONBLOCK);
    assert(s32_ret == 0);

    #ifdef AGV_FILE_SVR_SCOKET_OPT
    int opt=1; 
    s32_ret=setsockopt(s32_sock_connect, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(int));
    printf("TCP_NODELAY : %d -%s\n", s32_ret, strerror(errno));

    socklen_t opt_len;
    int send_buff_size;
    opt_len = sizeof(send_buff_size);
    s32_ret = getsockopt(s32_sock_connect, SOL_SOCKET, SO_SNDBUF,
                    &send_buff_size, &opt_len);
    printf("send_buff_size : %d\n", send_buff_size);


    send_buff_size = 500;
    s32_ret = setsockopt(s32_sock_connect, SOL_SOCKET, SO_SNDBUF,
                &send_buff_size, opt_len);
    if (s32_ret < 0)
    {
        printf("setsockopt err %s\n", strerror(errno));
    }
    printf("setsockopt send buff successed %d\n", send_buff_size);
    #endif

    struct timeval t_tmptv;
	fd_set t_fd_read_set;
    //fd_set t_fd_write_set;
    int s32_fd_max = s32_sock_connect > q_svr_handler 
        ? (s32_sock_connect + 1) : (q_svr_handler + 1);

    while(1)
    {
        FD_ZERO(&t_fd_read_set);
        FD_SET(s32_sock_connect, &t_fd_read_set);
        FD_SET(q_svr_handler, &t_fd_read_set);

        s32_ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, &t_tmptv);
//Select 错误--------------------------------------------------------------------------
        if (s32_ret < 0)
        {
            printf("Warn: TCP Instance Slecet: %s\n", strerror(errno));
            sleep(1);
            continue;
        }
//select 超时----------------------------------------------------------------------------
        else if (s32_ret == 0)
        {
            t_tmptv.tv_sec = 0;
            t_tmptv.tv_usec = 1000000;
            continue;
        }
//select 套接字接收数据-----------------------------------------------------------------------------
        if (FD_ISSET(s32_sock_connect, &t_fd_read_set))
        {
            s32_bytes = recv(s32_sock_connect, p8_recv_buff, AGV_TCP_BUFF_SIZE, 0);
    //remote close---------------------------------
            if (s32_bytes == 0)
            {
                printf("Warn: Remote Host TCP Closed...\n");
                break;
            }
    //communication error!--------------------------
            else if(s32_bytes < 0)
            {
                printf("Warn: TCP Receive Byte Failed: %s\n", strerror(errno));
                break;
            }
    //valid frame received!-------------------------
            else
            {
                //----------For Debug-----------------------------------------------
                //printf("file_svr recv %d bytes\n", s32_bytes);
                tcp_stream_buff_in(pt_recv_stream_buff, p8_recv_buff, s32_bytes);
                while(tcp_stream_buff_check(pt_recv_stream_buff, &cmd_recv_len) 
                        == E_STREAM_R_COMPLETE) //valid
                {
                    tcp_stream_buff_out(pt_recv_stream_buff, pt_recv_cmd, cmd_recv_len);
                    pt_agv_msg_head = (st_robot_msg_header *) (pt_recv_cmd + AGV_FRAME_DILIMTER_LEN);
                    //----------For Debug-----------------------------------------------
                    //printf("file_svr cmd parsed buff_r[%d]--class[%u], plen[%u]--\n",
                    //pt_recv_stream_buff->r, pt_agv_msg_head->u16_class, pt_agv_msg_head->s32_len);
                    //----------For Debug-----------------------------------------------
                //----------------------PROCESS THE CMD-----------------------------------
                    LIST_FOREACH(pt_handler, &pt_config->lh_cmd_handler, node)
                    {
                        if (pt_agv_msg_head->u16_class == pt_handler->c
                            && pt_agv_msg_head->u16_type == pt_handler->t)
                        {
                            
                            s32_cmd_ret = pt_handler->cmd_func(pt_tcp_connect_info, 
                                pt_recv_cmd, cmd_recv_len, pt_send_cmd, &cmd_send_len);
                            if (cmd_send_len > 0)
                            {
                                tcp_stream_buff_in(pt_send_stream_buff, 
                                    pt_send_cmd, cmd_send_len);
                            }
                            
                            break;
                        }
                    }
                //----------------------PROCESS THE CMD-----------------------------------
                }
                //Execute The Real Send, if the tcp buffer is full, wait select for Subsequent send
                tcp_stream_buff_send(pt_send_stream_buff, 
                    pt_tcp_connect_info, p8_send_buff, AGV_TCP_BUFF_SIZE);
                if (pt_send_stream_buff->state == E_STREAM_ERROR)
                {
                    printf("ERROR: TCP Instance Stream Buffer Send!\n");
                    break;
                }
            } //Recv Socket Process
        }//Recv FD

//消息队列处理-----------------------------------------------------------
        if (FD_ISSET(q_svr_handler, &t_fd_read_set))
        {
            memset(&t_agv_msg, 0, sizeof(st_robot_msg));

            s32_ret = mq_receive(q_svr_handler, (char *)&t_agv_msg,
                        sizeof(st_robot_msg), NULL);
            if (s32_ret < 0)
            {
                printf("Warn: TCP Instance Q Receive %d\n", errno);
                continue;
            }

            //根据消息生成待发送的数据
            s32_ret = tcp_thread_msg_process(t_agv_msg, pt_send_cmd, &cmd_send_len);
            if (s32_ret < 0)
            {
                printf("Warn: TCP Instance Message Process C[%d]-T[%d]!\n", 
                    t_agv_msg.t_msg_header.u16_class, t_agv_msg.t_msg_header.u16_type);
                continue;
            }

            //数据进入环形buffer
            s32_bytes = tcp_stream_buff_in(pt_send_stream_buff, pt_send_cmd, cmd_send_len);
            if (s32_bytes != cmd_send_len)
            {
                printf("ERROR: TCP Instance Stream Buffer Overflow!\n");
                continue;
            }

            //触发环形buffer的数据发送
            tcp_stream_buff_send(pt_send_stream_buff, 
                    pt_tcp_connect_info, p8_send_buff, AGV_TCP_BUFF_SIZE);
            if (pt_send_stream_buff->state == E_STREAM_ERROR)
            {
                printf("ERROR: TCP Instance Stream Buffer Send!\n");
                break;
            }
        }//MSG Q RECIEVE PROCESS


    }//while(ok)

//释放资源
    close(pt_tcp_connect_info->s32_sock_connect);
    pt_tcp_connect_info->s32_sock_connect = -1;

    free(pt_recv_cmd);
    free(pt_send_cmd);
    tcp_stream_buff_delete(pt_recv_stream_buff);
    tcp_stream_buff_delete(pt_send_stream_buff);
    return NULL;
}



/**
 * @name: tcp_stream_buff_send
 * @des:  环形队列数据通过tcp套接字发送
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_stream_buff_send(st_stream_buff *pt_send_stream_buff, 
                    st_tcp_connect_info *pt_info, uint8_t *p8_tcp_buff, int s32_tcp_buff_size)
{
    int tcp_sock = pt_info->s32_sock_connect;
    int send_expect = 0;
    int send_actual = 0;
    int send_remain = 0;
    int send_offset = 0;

    while (0 != send_remain || 0 != tcp_stream_buff_filled(pt_send_stream_buff))
    {
        if (0 != send_remain)
        {
            send_expect = send_remain;
            send_actual = send(tcp_sock, p8_tcp_buff + send_offset, send_expect, 0);
            //printf("type %d remain send %d\n",pt_info->pt_config->connect_type, send_actual);
            if (send_actual < 0)
            {
                if (errno == EAGAIN)
                {
                    usleep(1000);
                    continue;
                }
                else  //Socket Error
                {
                    pt_send_stream_buff->state = E_STREAM_ERROR;
                    break;
                }
            }
            else if (send_actual == 0) //Disconnect
            {
                pt_send_stream_buff->state = E_STREAM_ERROR;
                break;
            }
            else if (send_actual == send_expect)  //a block has send
            {
                send_remain = 0;
                send_offset = 0;
            }
            else if (send_actual < send_expect)
            {
                usleep(1000);
                send_remain = send_expect - send_actual;
                send_offset = send_offset + send_actual;
            }
        }
        else
        {
            send_expect = tcp_stream_buff_out(pt_send_stream_buff, p8_tcp_buff, s32_tcp_buff_size);
            send_actual = send(tcp_sock, p8_tcp_buff, send_expect, 0);
            //printf("type %d normal send %d\n",pt_info->pt_config->connect_type, send_actual);
            if (send_actual < 0)
            {
                if (errno == EAGAIN)
                {
                    usleep(1000);
                    continue;
                }
                else  //Socket Error
                {
                    pt_send_stream_buff->state = E_STREAM_ERROR;
                    break;
                }
            }
            else if (send_actual == 0) //Disconnect
            {
                pt_send_stream_buff->state = E_STREAM_ERROR;
                break;
            }
            else if (send_actual == send_expect)  //a block has send
            {
                send_remain = 0;
                send_offset = 0;
            }
            else if (send_actual < send_expect)
            {
                usleep(1000); //just wait a moment
                send_remain = send_expect - send_actual;
                send_offset = send_actual;
            }
        }
    }

    return 0;
}

/**
 * @name: 
 * @des: 
 * @param {st_robot_msg} t_agv_msg
 * @param {uint8_t} *file_cmd
 * @param {int} *file_cmd_len
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_thread_msg_process(st_robot_msg t_agv_msg, uint8_t *file_cmd, int *file_cmd_len)
{
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    uint8_t *pt_msg_body;

    if (file_cmd == NULL || file_cmd_len == NULL)
    {
        return -1;
    }
    
    cmd_body_len = t_agv_msg.t_msg_header.s32_len;
    if (cmd_body_len < 0)
    {
        return -1;
    }

    //Delimiter Add
    cmd_offset = 0;
    file_cmd[cmd_offset] = AGV_FRAME_H_0;
    cmd_offset = cmd_offset + 1;
    file_cmd[cmd_offset] = AGV_FRAME_H_1;
    cmd_offset = cmd_offset + 1;
    file_cmd[cmd_offset] = AGV_FRAME_H_2;
    cmd_offset = cmd_offset + 1;
    file_cmd[cmd_offset] = AGV_FRAME_H_3;

    //Header Add
    cmd_offset = cmd_offset + 1;
    memcpy((file_cmd + cmd_offset), &t_agv_msg.t_msg_header, sizeof(st_robot_msg_header));

    //Body Add
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);
    pt_msg_body = robot_msg_get_body(&t_agv_msg);
    if (pt_msg_body != NULL)
    {
        memcpy((file_cmd + cmd_offset), pt_msg_body, cmd_body_len);
    }
    robot_msg_release(&t_agv_msg);
    
    cmd_total_len = sizeof(st_robot_msg_header) + cmd_body_len;
    *file_cmd_len = cmd_total_len + AGV_FRAME_DILIMTER_LEN;//ADD the Delimiter Lenth

    return 0;
}
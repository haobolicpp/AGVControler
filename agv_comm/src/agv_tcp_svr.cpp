#include <stdio.h>
#include <time.h>
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
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "ros/ros.h"

#include "agv_comm/sys_queue.h"
#include "agv_comm/agv_type.h"
#include "agv_comm/agv_cmd.hpp"
#include "agv_comm/agv_tcp_ctrl.hpp"
#include "agv_comm/agv_tcp_svr.hpp"




//Stream Send, just put the data in circultaed buffer for send
#if 0
static int agv_stream_buff_tcp_send(st_stream_buff *pt_send_stream_buff, 
                    int tcp_sock, uint8_t *p8_tcp_buff, int s32_tcp_buff_size);
#endif

int agv_stream_buff_tcp_send(st_stream_buff *pt_send_stream_buff, 
                    st_tcp_connect_info *pt_info, uint8_t *p8_tcp_buff, int s32_tcp_buff_size);

//agv msg process
static int agv_file_svr_procss_msg(st_agv_msg t_agv_msg, uint8_t *file_cmd, int *file_cmd_len);

/******************************************************************************
* 函数名称: thread_body_file_svr()
* 作 用 域: Global
* 功能描述: 文件服务主线程
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
void *thread_tcp_svr_instance(void *arg)
{
    int s32_ret;
    int s32_bytes;
    int s32_cmd_ret;

    uint8_t p8_recv_buff[AGV_FILE_TCPBUFF_SIZE];
    uint8_t p8_send_buff[AGV_FILE_TCPBUFF_SIZE];

    st_agv_msg t_agv_msg;
    st_agv_msg_header *pt_agv_msg_head;
    st_agv_cmd_handler *pt_handler;

    int cmd_recv_len;
    int cmd_send_len;
    //获取套接字 和 消息队列

    st_tcp_connect_info *pt_tcp_connect_info = (st_tcp_connect_info *)arg;
    int s32_sock_connect = pt_tcp_connect_info->s32_sock_connect;
    mqd_t q_svr_handler = pt_tcp_connect_info->q_svr_handler;
    st_tcp_connect_config *pt_config = pt_tcp_connect_info->pt_config;

    int s32_stream_buff_size = pt_tcp_connect_info->pt_config->stream_buffer_size;
    int s32_cmd_buff_size = s32_stream_buff_size/2;

    printf("s32_stream_buff_size : %d\n", s32_stream_buff_size);

    //创建用于命令解析的缓存
    uint8_t *pt_recv_cmd = (uint8_t *)malloc(s32_cmd_buff_size);
    uint8_t *pt_send_cmd = (uint8_t *)malloc(s32_cmd_buff_size);
    if (pt_recv_cmd == NULL || pt_send_cmd == NULL)
    {
        tcp_connect_info_remove(pt_tcp_connect_info);
        return NULL;
    }


    //创建用于接收的环形队列
    st_stream_buff *pt_recv_stream_buff = agv_stream_buff_create(s32_stream_buff_size);
    if (pt_recv_stream_buff == NULL)
    {
        free(pt_recv_cmd);
        free(pt_send_cmd);
        tcp_connect_info_remove(pt_tcp_connect_info);
        return NULL;
    }
    //创建用于发送的环形队列
    st_stream_buff *pt_send_stream_buff = agv_stream_buff_create(s32_stream_buff_size);
    if (pt_send_stream_buff == NULL)
    {
        free(pt_recv_cmd);
        free(pt_send_cmd);
        agv_stream_buff_delete(pt_recv_stream_buff);
        tcp_connect_info_remove(pt_tcp_connect_info);
        return NULL;
    }

    //设置套接字属性为非阻塞
    int opts = fcntl(s32_sock_connect, F_GETFL);
    if(opts < 0 || fcntl(s32_sock_connect, F_SETFL, opts | O_NONBLOCK) < 0)
    {
        free(pt_recv_cmd);
        free(pt_send_cmd);
        agv_stream_buff_delete(pt_recv_stream_buff);
        tcp_connect_info_remove(pt_tcp_connect_info);
        printf("file svr set non-block with error %s\n", strerror(errno));
        return NULL;
    }

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

    while(ros::ok())
    {
        t_tmptv.tv_sec = 0;
        t_tmptv.tv_usec = 1000000;
        FD_ZERO(&t_fd_read_set);
        FD_SET(s32_sock_connect, &t_fd_read_set);
        FD_SET(q_svr_handler, &t_fd_read_set);

        s32_ret = select(s32_fd_max, &t_fd_read_set, NULL, NULL, &t_tmptv);
//select with error--------------------------------------------------------------------------
        if (s32_ret < 0)
        {
            printf("file_svr select with err %s\n", strerror(errno));
            sleep(1);
            continue;
        }
//select time out----------------------------------------------------------------------------
        else if (s32_ret == 0)
        {
            continue;
        }
//select byte receive-----------------------------------------------------------------------------
        if (FD_ISSET(s32_sock_connect, &t_fd_read_set))
        {
            s32_bytes = recv(s32_sock_connect, p8_recv_buff, AGV_FILE_TCPBUFF_SIZE, 0);
            //remote close---------------------------------
            if (s32_bytes == 0)
            {
                printf("remote host close the connection!\n");
                break;
            }
            //communication error!--------------------------
            else if(s32_bytes < 0)
            {
                printf("file_svr recv err: %s\n", strerror(errno));
                break;
            }
            //valid frame received!-------------------------
            else
            {
                //printf("file_svr recv %d bytes\n", s32_bytes);
                agv_stream_buff_in(pt_recv_stream_buff, p8_recv_buff, s32_bytes);
                while(agv_stream_buff_check(pt_recv_stream_buff, &cmd_recv_len) == 2) //valid
                {
                    agv_stream_buff_out(pt_recv_stream_buff, pt_recv_cmd, cmd_recv_len);

                    //----------For Debug-----------------------------------------------
                    pt_agv_msg_head = (st_agv_msg_header *) (pt_recv_cmd + AGV_FRAME_DILIMTER_LEN);
                   
                    //printf("file_svr cmd parsed buff_r[%d]--class[%u], plen[%u]--\n",
                    //    pt_recv_stream_buff->r, pt_agv_msg_head->u16_class, pt_agv_msg_head->s32_len);
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
                                agv_stream_buff_in(pt_send_stream_buff, 
                                    pt_send_cmd, cmd_send_len);
                            }
                            
                            break;
                        }
                    }
                //----------------------PROCESS THE CMD-----------------------------------
                }
                //Execute The Real Send, if the tcp buffer is full, wait select for Subsequent send
                agv_stream_buff_tcp_send(pt_send_stream_buff, 
                    pt_tcp_connect_info, p8_send_buff, AGV_FILE_TCPBUFF_SIZE);
                if (pt_send_stream_buff->state == E_STREAM_ERROR)
                {
                    break;
                }
            } //Recv Socket Process
        }//Recv FD

//MSG Q RECIEVE PROCESS-------------------------------------------------
        if (FD_ISSET(q_svr_handler, &t_fd_read_set))
        {
            memset(&t_agv_msg, 0, sizeof(st_agv_msg));

            s32_ret = mq_receive(q_svr_handler, (char *)&t_agv_msg,
                        sizeof(st_agv_msg), NULL);
            if (s32_ret < 0)
            {
                printf("tcp svr q recieve with err %d\n", errno);
                continue;
            }

            //根据消息生成待发送的数据
            s32_ret = agv_file_svr_procss_msg(t_agv_msg, pt_send_cmd, &cmd_send_len);
            if (s32_ret < 0)
            {
                printf("tcp svr produce agv cmd failed!\n");
                continue;
            }

            if (t_agv_msg.t_msg_header.u16_class == 1 && t_agv_msg.t_msg_header.u16_type == 3)
            {
                printf("map msg period send!\n");
            }

            //数据进入环形buffer
            s32_bytes = agv_stream_buff_in(pt_send_stream_buff, pt_send_cmd, cmd_send_len);
            if (s32_bytes != cmd_send_len)
            {
                printf("stream buffer full!\n");
                continue;
            }

            //触发环形buffer的数据发送
            agv_stream_buff_tcp_send(pt_send_stream_buff, 
                    pt_tcp_connect_info, p8_send_buff, AGV_FILE_TCPBUFF_SIZE);
            if (pt_send_stream_buff->state == E_STREAM_ERROR)
            {
                break;
            }


        }//MSG Q RECIEVE PROCESS


    }//while(ok)

    free(pt_recv_cmd);
    free(pt_send_cmd);
    agv_stream_buff_delete(pt_recv_stream_buff);
    agv_stream_buff_delete(pt_send_stream_buff);
    tcp_connect_info_remove(pt_tcp_connect_info);
    tcp_connect_ctrl_watch(pt_tcp_connect_info->pt_connect_ctrl);
    return NULL;
}



/******************************************************************************
* 函数名称: agv_stream_buff_tcp_send()
* 作 用 域: Local
* 功能描述: 发送数据进入环形缓冲区并发送
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int agv_stream_buff_tcp_send(st_stream_buff *pt_send_stream_buff, 
                    st_tcp_connect_info *pt_info, uint8_t *p8_tcp_buff, int s32_tcp_buff_size)
{
    int tcp_sock = pt_info->s32_sock_connect;
    int send_expect = 0;
    int send_actual = 0;
    int send_remain = 0;
    int send_offset = 0;

    while (0 != send_remain || 0 != agv_stream_buff_filled(pt_send_stream_buff))
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
            send_expect = agv_stream_buff_out(pt_send_stream_buff, p8_tcp_buff, s32_tcp_buff_size);
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

/******************************************************************************
* 函数名称: 文件服务线程处理异步消息
* 作 用 域: Local
* 功能描述: 
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int agv_file_svr_procss_msg(st_agv_msg t_agv_msg, uint8_t *file_cmd, int *file_cmd_len)
{
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
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
    memcpy((file_cmd + cmd_offset), &t_agv_msg.t_msg_header, sizeof(st_agv_msg_header));

    //Body Add
    cmd_offset = cmd_offset + sizeof(st_agv_msg_header);
    if (cmd_body_len <= AGV_CMD_S_BODY_LEN)
    {
        memcpy((file_cmd + cmd_offset), &t_agv_msg.t_msg_body.sbody, cmd_body_len);
    }
    else
    {
        memcpy((file_cmd + cmd_offset), t_agv_msg.t_msg_body.lbody, cmd_body_len);
        if (t_agv_msg.t_msg_body.src_flag == 0)
        {
            free(t_agv_msg.t_msg_body.lbody); //todo 地图数据有可能小于32个字节吗？
        }
    }
    
    cmd_total_len = sizeof(st_agv_msg_header) + cmd_body_len;
    *file_cmd_len = cmd_total_len + AGV_FRAME_DILIMTER_LEN;//ADD the Delimiter Lenth

    return 0;
}



/******************************************************************************
* 函数名称: agv_stream_buff_init()
* 作 用 域: Global
* 功能描述: 环形缓冲区初始化
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int agv_stream_buff_init(st_stream_buff *pt_stream_buff, uint8_t *p8_alloc, int s32_size)
{
    if (s32_size < 32 || p8_alloc == NULL)
    {
        return -1;
    }

    //fast mod algorithm request the buffer size is power 2 
    if(!is_2_power(s32_size))
    {
        return -1;
    }


    pt_stream_buff->r = 0;
    //pt_stream_buff->r_t = 0;
    pt_stream_buff->w = 0;
    pt_stream_buff->size = s32_size;
    pt_stream_buff->size_1 = s32_size - 1;
    pt_stream_buff->alloc = p8_alloc;

    pt_stream_buff->state = E_STREAM_INIT;

    return 0;
}

/******************************************************************************
* 函数名称: agv_stream_buff_create()
* 作 用 域: Global
* 功能描述: 动态构造环形缓冲区
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: ！NULL-Success    NULL-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年9月5日
******************************************************************************/
st_stream_buff *agv_stream_buff_create(int s32_size)
{
    uint8_t *p8_alloc;
    st_stream_buff *pt_stream_buff;
    
    if(!is_2_power(s32_size))
    {
        return NULL;
    }

    p8_alloc = (uint8_t *)malloc(s32_size);
    if (p8_alloc == NULL)
    {
        return NULL;
    }
    
    pt_stream_buff = (st_stream_buff *)malloc(sizeof(st_stream_buff));

    if (pt_stream_buff == NULL)
    {
        return NULL;
    }

    pt_stream_buff->r = 0;
    //pt_stream_buff->r_t = 0;
    pt_stream_buff->w = 0;
    pt_stream_buff->size = s32_size;
    pt_stream_buff->size_1 = s32_size - 1;
    pt_stream_buff->alloc = p8_alloc;
    pt_stream_buff->state = E_STREAM_INIT;

    return pt_stream_buff;

}

/******************************************************************************
* 函数名称: agv_stream_buff_create()
* 作 用 域: Global
* 功能描述: 动态析构环形缓冲区
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success    -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年9月5日
******************************************************************************/
int agv_stream_buff_delete(st_stream_buff *pt_stream_buff)
{
    if (pt_stream_buff != NULL)
    {    
        if (pt_stream_buff->alloc != NULL)
        {
            free(pt_stream_buff->alloc);
        }
        
        free(pt_stream_buff); 
        pt_stream_buff = NULL;
    }
    return 0;
}

/******************************************************************************
* 函数名称: agv_stream_buff_in()
* 作 用 域: Global
* 功能描述: 数据入环形缓冲区
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int agv_stream_buff_in(st_stream_buff *pt_stream_buff, uint8_t *p8_in, int s32_size)
{
    int i;
    int w_t;
    int filled;
    int remain;

    //for circulate copy
    int part1;
    int part2;

    //
    filled = agv_stream_buff_filled(pt_stream_buff);
    remain = pt_stream_buff->size - filled;

    if (pt_stream_buff == NULL)
    {
        return -1;
    }
    if (p8_in == NULL || s32_size == 0 || remain < s32_size)
    {
        return 0;
    }

    w_t = pt_stream_buff->w + s32_size & pt_stream_buff->size_1;
    //not ciculated!
    if (w_t > pt_stream_buff->w)
    {
        memcpy(&pt_stream_buff->alloc[pt_stream_buff->w], p8_in, s32_size);
        pt_stream_buff->w  = (pt_stream_buff->w + s32_size) & pt_stream_buff->size_1;
    }
    //ciculated!
    else
    {
        part1 = pt_stream_buff->size - pt_stream_buff->w;
        part2 = s32_size - part1;

        //out part1
        memcpy(&pt_stream_buff->alloc[pt_stream_buff->w], p8_in, part1);
        pt_stream_buff->w = (pt_stream_buff->w + part1) & pt_stream_buff->size_1;

        //out part2
        if (part2 != 0)
        {
            memcpy(&pt_stream_buff->alloc[pt_stream_buff->w], (p8_in + part1), part2);
            pt_stream_buff->w = (pt_stream_buff->w + part2) & pt_stream_buff->size_1;
        }
    }

    return s32_size;
}

/******************************************************************************
* 函数名称: agv_stream_buff_check()
* 作 用 域: Global
* 功能描述: 监测环形缓冲区中完整的数据帧
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int agv_stream_buff_check(st_stream_buff *pt_stream_buff, int *s32_cmd_total_size)
{
    int r_t;
    uint32_t u32_plen;
    uint8_t u8_delimiter[4] = {0};
    uint8_t u8_plen_element[4] = {0};

    if (pt_stream_buff == NULL)
    {
        return -1;
    }

    //slide and find the common header

    if (pt_stream_buff->state == E_STREAM_INIT)
    {
        while(agv_stream_buff_filled(pt_stream_buff) >= AGV_COMMON_HEADER_LEN && 
                    pt_stream_buff->state == E_STREAM_INIT)
        {
            r_t = pt_stream_buff->r;
            u8_delimiter[0] = pt_stream_buff->alloc[r_t];
            r_t = (r_t + 1) & pt_stream_buff->size_1;
            u8_delimiter[1] = pt_stream_buff->alloc[r_t];
            r_t = (r_t + 1) & pt_stream_buff->size_1;
            u8_delimiter[2] = pt_stream_buff->alloc[r_t];
            r_t = (r_t + 1) & pt_stream_buff->size_1;
            u8_delimiter[3] = pt_stream_buff->alloc[r_t];

            //get the plen
            r_t = (pt_stream_buff->r + AGV_COMMON_HEADER_OFFSET_PLEN) & pt_stream_buff->size_1;
            u8_plen_element[0] = pt_stream_buff->alloc[r_t];
            r_t = (r_t + 1) & pt_stream_buff->size_1;
            u8_plen_element[1] = pt_stream_buff->alloc[r_t];
            r_t = (r_t + 1) & pt_stream_buff->size_1;
            u8_plen_element[2] = pt_stream_buff->alloc[r_t];
            r_t = (r_t + 1) & pt_stream_buff->size_1;
            u8_plen_element[3] = pt_stream_buff->alloc[r_t];

            u32_plen = u8_plen_element[0]  
                        | u8_plen_element[1] << 8
                        | u8_plen_element[2] << 16
                        | u8_plen_element[3]<< 24;

            //agv_cmd_get_plen(&pt_stream_buff->alloc[pt_stream_buff->r], &u32_plen);
            //find a common header

            if (u8_delimiter[0] == AGV_FRAME_H_0
                && u8_delimiter[1] == AGV_FRAME_H_1
                && u8_delimiter[2] == AGV_FRAME_H_2
                && u8_delimiter[3] == AGV_FRAME_H_3
                && u32_plen >= 0 && u32_plen <= AGV_CMD_MAX)
            {
                pt_stream_buff->cmd_total = AGV_COMMON_HEADER_LEN + u32_plen;
                pt_stream_buff->cmd_remaining = u32_plen;
                pt_stream_buff->state = E_STREAM_R_NOTCOMPLETE;
            }

            //invalid and go on slide find a valid command!
            else
            {
                pt_stream_buff->r = (pt_stream_buff->r + 1) & pt_stream_buff->size_1;
            }
        }
    }

    //common header complated
    if (pt_stream_buff->state == E_STREAM_R_NOTCOMPLETE)
    {
        if (agv_stream_buff_filled(pt_stream_buff) >= pt_stream_buff->cmd_total)
        {
            *s32_cmd_total_size = pt_stream_buff->cmd_total;
            pt_stream_buff->state = E_STREAM_R_COMPLETE;
        }

        else
        {
            pt_stream_buff->cmd_remaining = pt_stream_buff->cmd_total 
                            - agv_stream_buff_filled(pt_stream_buff);
        }
    }

    return pt_stream_buff->state;
}



/******************************************************************************
* 函数名称: agv_stream_buff_out()
* 作 用 域: Global
* 功能描述: 数据出环形缓冲区
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int agv_stream_buff_out(st_stream_buff *pt_stream_buff, uint8_t *p8_out, int s32_size)
{
    int i;
    int r_t;
    int s32_filled;
    int s32_out_size;

    //for circulate copy
    int part1;
    int part2;

    //the max out bytes is current filled
    s32_filled = agv_stream_buff_filled(pt_stream_buff);
    s32_out_size = s32_filled < s32_size ? s32_filled : s32_size;

    if (s32_out_size == 0)
    {
        return 0;
    }
    
    //no out buffer give, show just move the read pos
    if (p8_out == NULL)
    {
        pt_stream_buff->r = (pt_stream_buff->r + s32_out_size) & pt_stream_buff->size_1;
        pt_stream_buff->state = E_STREAM_INIT;
        return s32_out_size;
    }

    //buff circulate?
    r_t = (pt_stream_buff->r + s32_out_size) & pt_stream_buff->size_1;
    //not circulate
    if (r_t > pt_stream_buff->r)
    {
        memcpy(p8_out, &pt_stream_buff->alloc[pt_stream_buff->r], s32_out_size);
        pt_stream_buff->r = r_t;
    }
    //circulated
    else
    {
        //calculate part1 and part2, look out that part 2 maybe 0!
        part1 = pt_stream_buff->size - pt_stream_buff->r;
        part2 = s32_out_size - part1;

        //out part1
        memcpy(p8_out, &pt_stream_buff->alloc[pt_stream_buff->r], part1);
        pt_stream_buff->r = (pt_stream_buff->r + part1) & pt_stream_buff->size_1;

        //out part2
        if (part2 != 0)
        {
            memcpy((p8_out + part1), &pt_stream_buff->alloc[pt_stream_buff->r], part2);
            pt_stream_buff->r = (pt_stream_buff->r + part2) & pt_stream_buff->size_1;
        }
    }

    pt_stream_buff->state = E_STREAM_INIT;

    return s32_out_size;
}


/******************************************************************************
* 函数名称: agv_stream_buff_filled()
* 作 用 域: Global
* 功能描述: 探测缓冲区有效数据的数量
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int agv_stream_buff_filled(st_stream_buff *st_stream_buff)
{
    //empty
    if (st_stream_buff->r == st_stream_buff->w)
    {
        return 0;
    }

    //not cycled
    else if (st_stream_buff->w > st_stream_buff->r)
    {
        return st_stream_buff->w - st_stream_buff->r;
    }

    //cycled
    else
    {
        return st_stream_buff->size - 
                (st_stream_buff->r - st_stream_buff->w);
    }

}


//-------------------CMD UTILS--------------------------------

/******************************************************************************
* 函数名称: is_2_power()
* 作 用 域: Global
* 功能描述: 判断一个数是否是2的整数幂
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
bool is_2_power(int number)
{
    int j = 1;
    while (number>j) 
    {
        j<<=1;
    }
    return j==number?true:false;
}





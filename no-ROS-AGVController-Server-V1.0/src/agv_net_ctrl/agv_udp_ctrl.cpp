
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

#include "sys_queue.h"
#include "agv_type.h"
#include "agv_cmd.h"
#include "AgvCtrl.h"
#include "agv_udp_ctrl.h"

//#define AGV_UDP_DEBUG_SET

static void *thread_body_udp_svr(void *arg);
static int client_broadcast_req(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);

/******************************************************************************
* 函数名称: udp_svr_ctrl_init()
* 作 用 域: Global
* 功能描述: 初始化UDP服务
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int udp_svr_ctrl_init(AgvCtrl *pt_agv_ctrl)
{
    st_udp_svr_ctrl *pt_udp_svr_ctrl;
    st_tcp_cmd_handler *pt_handler;
    
    if (pt_agv_ctrl == NULL)
    {
        return -1;
    }
    
    pt_udp_svr_ctrl = &pt_agv_ctrl->t_udp_ctrl;
    LIST_INIT(&pt_udp_svr_ctrl->lh_cmd_handler);

//todo ADD the cmd process element
//-------------------------regist client_broadcast_requset--------------------
    pt_handler = agv_cmd_handler_create("client_broadcast_requset", 
        AGV_CMD_C_UDP, 
        AGV_CMD_T_UDP_BROADCAST,
        client_broadcast_req);
    udp_svr_regist_handler(pt_udp_svr_ctrl, pt_handler);


    return 0;
}


/******************************************************************************
* 函数名称: agv_udp_svr_run()
* 作 用 域: Global
* 功能描述: 运行AGV控制器UDP服务
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int agv_udp_svr_run(AgvCtrl *pt_agv_ctrl)
{
    int s32_ret;
    struct mq_attr t_mq_attr = {0};
    pthread_attr_t pthread_attr;
    struct sched_param sched_param;

    if (pt_agv_ctrl == NULL)
    {
        return -1;
    }

    //Create the thread for comm_svr
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 20;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);

    s32_ret = pthread_create(&pt_agv_ctrl->t_udp_ctrl.thread_handler_udp_svr,
                    &pthread_attr, thread_body_udp_svr, (void *)pt_agv_ctrl);
    if (s32_ret != 0)
    {
        printf("comm_svr_thread create failed (%d)\n", s32_ret);
        return -1;
    }
    return 0;


}

/******************************************************************************
* 函数名称: thread_body_comm_svr()
* 作 用 域: Global
* 功能描述: communication server thread body, 
            recieve msg from the socket or transmit msg to the scoket
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
void *thread_body_udp_svr(void *arg)
{
    int s32_ret;
    int s32_cmd_ret;
    
    int sock_udp_svr;
    struct sockaddr_in t_svr_addr;
    struct sockaddr_in t_cli_addr;
    socklen_t u32_addr_len = sizeof(struct sockaddr_in);
    int32_t cmd_recv_len;
    int32_t cmd_send_len;
    uint8_t pt_recv_cmd[AGV_UDP_BUFF_SIZE] = {0};
    uint8_t pt_send_cmd[AGV_UDP_BUFF_SIZE] = {0};

    st_robot_msg t_agv_msg = {0};
    AgvCtrl *pt_agv_ctrl = (AgvCtrl *)arg;

    st_udp_svr_ctrl *pt_udp_svr_ctrl = &pt_agv_ctrl->t_udp_ctrl;
    st_tcp_cmd_handler *pt_handler;
    st_robot_msg_header *pt_agv_msg_head;

	//Create Socket
    sock_udp_svr = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_udp_svr < 0)
    {
        printf("sock_udp_svr create failed\n");
        return NULL;
    }

    //Bind Socket
    t_svr_addr.sin_family = AF_INET;
    t_svr_addr.sin_port = htons(AGV_UDP_PORT);
    t_svr_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock_udp_svr, (struct sockaddr *)&t_svr_addr, sizeof(t_svr_addr)) < 0)
    {
        printf("sock_udp_comm_svr bind failed\n");
        return NULL;
    }

    //Main Server Loop
    while(1)
    {
        cmd_recv_len = recvfrom(sock_udp_svr, pt_recv_cmd, AGV_UDP_BUFF_SIZE, 0,
                            (struct sockaddr *)&t_cli_addr, &u32_addr_len);
        if (cmd_recv_len < 0)
        {
            printf("Net socket Recv failed with errno %s!\n", strerror(errno));
            continue;
        }

        //for debug
#ifdef AGV_UDP_DEBUG_SET
        int i;
        printf("AGV-UDP-SVR R[%d]:", cmd_recv_len);
        for(i = 0; i < cmd_recv_len; i++)
        {
            printf("%.2x  ",pt_recv_cmd[i]);
        }
        printf("\n");
#endif
    //帧合法性判断
        if ( pt_recv_cmd[0] == AGV_FRAME_H_0 
            && pt_recv_cmd[1] == AGV_FRAME_H_1
            && pt_recv_cmd[2] == AGV_FRAME_H_2
            && pt_recv_cmd[1] == AGV_FRAME_H_3
            && cmd_recv_len >= AGV_COMMON_HEADER_LEN)
        {
            pt_agv_msg_head = 
                (st_robot_msg_header *)(pt_recv_cmd + AGV_FRAME_DILIMTER_LEN);

            LIST_FOREACH(pt_handler, &pt_udp_svr_ctrl->lh_cmd_handler, node)
            {
                if (pt_agv_msg_head->u16_class == pt_handler->c 
                        && pt_agv_msg_head->u16_type == pt_handler->t)
                {
                    s32_cmd_ret = pt_handler->cmd_func(pt_agv_ctrl, 
                        pt_recv_cmd, cmd_recv_len, pt_send_cmd, &cmd_send_len);
                    if (cmd_send_len > 0)
                    {
                        s32_ret = sendto(sock_udp_svr, pt_send_cmd, cmd_send_len, 0,
                                        (struct sockaddr *)&t_cli_addr, u32_addr_len);

                    }
                    break;
                }
            }

        }


    }


    return NULL;
}


/******************************************************************************
* 函数名称: thread_body_comm_svr()
* 作 用 域: Global
* 功能描述: communication server thread body, 
            recieve msg from the socket or transmit msg to the scoket
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int udp_svr_regist_handler(st_udp_svr_ctrl *pt_udp_svr_ctrl, st_udp_cmd_handler *pt_handler)
{
    if (pt_udp_svr_ctrl == NULL || pt_handler ==NULL)
    {
        return -1;
    }

    LIST_INSERT_HEAD(&pt_udp_svr_ctrl->lh_cmd_handler, pt_handler, node);

    return 0;
}




/******************************************************************************
* 函数名称: thread_body_comm_svr()
* 作 用 域: Global
* 功能描述: communication server thread body, 
            recieve msg from the socket or transmit msg to the scoket
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值: 0-Success -1-Failed
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2019年9月5日
******************************************************************************/
int client_broadcast_req(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    AgvCtrl *pt_agv_ctrl = (AgvCtrl *)data; 
    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);

    printf("HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);

    //ADD HEADER
    cmd_offset = 0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_1;
    cmd_out[cmd_offset++] = AGV_FRAME_H_2;
    cmd_out[cmd_offset++] = AGV_FRAME_H_3;

    pt_msg_header->u16_dest = 1;

    pt_msg_header->u16_src = 0x10;

    pt_msg_header->u16_type = pt_msg_header->u16_type | 0x8000;
    pt_msg_header->s32_len = 4;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_robot_msg_header));
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);

    //body set
    cmd_out[cmd_offset++] = AGV_CTRL_M1_VERSION;
    cmd_out[cmd_offset++] = AGV_CTRL_S1_VERSION;
    cmd_out[cmd_offset++] = AGV_CTRL_S2_VERSION;
    cmd_out[cmd_offset++] = 0;

     *olen = cmd_offset;

     return 0;

}



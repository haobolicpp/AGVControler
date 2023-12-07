#include <stdio.h>
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
#include <arpa/inet.h>
#include <sys/stat.h>

#include "sys_queue.h"
#include "agv_net_cfg.h"
#include "agv_type.h"
#include "agv_cmd.h"
#include "agv_tcp_ctrl.h"
#include "agv_tcp_inst.h"

/**
 * @name: tcp_connect_info_init
 * @des: tcp连接结构初始化
 * @param {st_tcp_connect_info} *pt_info
 * @param {void} *pRoot
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_connect_info_init(st_tcp_connect_info *pt_info, const char *pt_name, void *pRoot)
{
    int ret;

    if (pt_info == NULL)
    {
        return -1;
    }

    //连接结构初始化
    pt_info->s32_sock_connect = -1;
    pt_info->pt_config =NULL;
    pt_info->pRoot = pRoot;
    LIST_INIT(&pt_info->lh_event);

    //定义消息队列名称
    assert(pt_name != NULL);
    assert(pt_name[0] != '/');
    snprintf(pt_info->name, 64, "/%s", pt_name);

    //创建用于事件队列的多线程锁
    ret = pthread_mutex_init(&pt_info->mutex_event, NULL);
    assert(ret == 0);

    return 0;
}

/**
 * @name: tcp_connect_to_server
 * @des:  连接服务端
 * @param {st_tcp_connect_info} *pt_info
 * @param {char} *svr_ip
 * @param {int} svr_port
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_connect_to_server(st_tcp_connect_info *pt_info, const char *svr_ip, int svr_port)
{
    // int ret;

    // if (pt_info == NULL || svr_ip == NULL )
    // {
    //     return -1;
    // }

    // pt_info->s32_sock_connect = socket(AF_INET, SOCK_STREAM, 0);
    // if (pt_info->s32_sock_connect < 0)
    // {
    //     printf("ERROR: Connect To TCP Server %s\n", strerror(errno));
    //     return -1;
    // }

    // //目标服务端地址与端口
    // struct sockaddr_in svr_addr;
    // memset(&svr_addr, 0, sizeof(svr_addr));
    // svr_addr.sin_family = AF_INET;
    // svr_addr.sin_port = htons(AGV_TCP_PORT_ROS);
    // svr_addr.sin_addr.s_addr = inet_addr(AGV_TCP_ADDR_ROS);

    // //连接至目标服务
    // ret = connect(pt_info->s32_sock_connect, 
    //     (struct sockaddr *)&svr_addr, sizeof(svr_addr));

    // if (ret < 0)
    // {
    //     printf("ERROR: Connect To TCP Server %s\n", strerror(errno));
    //     return -1;
    // }

    // printf("INFO: Connect To TCP Server Success\n");
    return 0;
}

/**
 * @name: tcp_connect_config_create
 * @des:  创建一个数据解析实例
 * @param {E_AGV_SVR_TYPE} e_svr_type
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
st_tcp_connect_config *tcp_connect_config_create(E_TCP_CONNECT_TYPE e_svr_type)
{
    // st_tcp_connect_config *pt_config;
    
    // if (e_svr_type == E_AGV_ROS_CLIENT)
    // {
    //     pt_config = ros_client_config_create();
    //     assert(pt_config != NULL);
    // }
    // else
    // {
    //     return NULL;
    // }
    // return pt_config;
}
/**
 * @name: tcp_connect_config_delete
 * @des:  销毁连接配置实例
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_connect_config_delete(st_tcp_connect_config *pt_config)
{
    int s32_ret;
    st_tcp_cmd_handler *pt_handler;
    st_tcp_cmd_handler *pt_handler_tmp;
    
    if (pt_config == NULL)
    {
        return -1;
    }
    
    //Delete and Free The Cmd Handler
    LIST_FOREACH_SAFE(pt_handler, &pt_config->lh_cmd_handler, node, pt_handler_tmp)
    {
        LIST_REMOVE(pt_handler, node);
        tcp_cmd_handler_delete(pt_handler);
    }
    

    return s32_ret;
}

/**
 * @name: 
 * @des: 
 * @param {st_tcp_connect_config} *pt_config
 * @param {st_tcp_cmd_handler} *pt_handler
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_connect_config_regist_handler(st_tcp_connect_config *pt_config, st_tcp_cmd_handler *pt_handler)
{
    if (pt_config == NULL || pt_handler ==NULL)
    {
        return -1;
    }

    LIST_INSERT_HEAD(&pt_config->lh_cmd_handler, pt_handler, node);

    return 0;

}


/**
 * @name: 
 * @des: 
 * @param {st_tcp_connect_info} *pt_info
 * @param {E_AGV_EVENT_TYPE} e_type
 * @param {int} period
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_connect_regist_event(st_tcp_connect_info *pt_info, E_TCP_EVENT_TYPE e_type, int period)
{
    st_tcp_period_event *pt_event;

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
    pt_event = (st_tcp_period_event *)malloc(sizeof(st_tcp_period_event));
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

/**
 * @name: tcp_connect_remove_event
 * @des:  移除一个周期事件
 * @param {st_tcp_connect_info} *pt_info
 * @param {E_TCP_EVENT_TYPE} e_type
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_connect_remove_event(st_tcp_connect_info *pt_info, E_TCP_EVENT_TYPE e_type)
{
    st_tcp_period_event *pt_event, *pt_event_tmp;

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




/**
 * @name: tcp_stream_buff_init
 * @des:  环形队列初始化
 * @param {st_stream_buff} *pt_stream_buff
 * @param {uint8_t} *p8_alloc
 * @param {int} s32_size
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_stream_buff_init(st_stream_buff *pt_stream_buff, uint8_t *p8_alloc, int s32_size)
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
    pt_stream_buff->w = 0;
    pt_stream_buff->size = s32_size;
    pt_stream_buff->size_1 = s32_size - 1;
    pt_stream_buff->alloc = p8_alloc;

    pt_stream_buff->state = E_STREAM_INIT;

    return 0;
}

/**
 * @name: tcp_stream_buff_create
 * @des:  环形队列创建
 * @param {int} s32_size
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
st_stream_buff *tcp_stream_buff_create(int s32_size)
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
    pt_stream_buff->w = 0;
    pt_stream_buff->size = s32_size;
    pt_stream_buff->size_1 = s32_size - 1;
    pt_stream_buff->alloc = p8_alloc;
    pt_stream_buff->state = E_STREAM_INIT;

    return pt_stream_buff;

}

/**
 * @name: tcp_stream_buff_delete
 * @des:  环形队列销毁
 * @param {st_stream_buff} *pt_stream_buff
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_stream_buff_delete(st_stream_buff *pt_stream_buff)
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

/**
 * @name: tcp_stream_buff_in
 * @des:  数据进入环形队列
 * @param {st_stream_buff} *pt_stream_buff
 * @param {uint8_t} *p8_in
 * @param {int} s32_size
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_stream_buff_in(st_stream_buff *pt_stream_buff, uint8_t *p8_in, int s32_size)
{
    int i;
    int w_t;
    int filled;
    int remain;

    //for circulate copy
    int part1;
    int part2;

    //
    filled = tcp_stream_buff_filled(pt_stream_buff);
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

/**
 * @name: tcp_stream_buff_check
 * @des:  按协议格式从环形队列检出命令
 * @param {st_stream_buff} *pt_stream_buff
 * @param {int} *s32_cmd_total_size
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_stream_buff_check(st_stream_buff *pt_stream_buff, int *s32_cmd_total_size)
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
        while(tcp_stream_buff_filled(pt_stream_buff) >= AGV_COMMON_HEADER_LEN && 
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
        if (tcp_stream_buff_filled(pt_stream_buff) >= pt_stream_buff->cmd_total)
        {
            *s32_cmd_total_size = pt_stream_buff->cmd_total;
            pt_stream_buff->state = E_STREAM_R_COMPLETE;
        }

        else
        {
            pt_stream_buff->cmd_remaining = pt_stream_buff->cmd_total 
                            - tcp_stream_buff_filled(pt_stream_buff);
        }
    }

    return pt_stream_buff->state;
}


/**
 * @name: tcp_stream_buff_out
 * @des:  命令帧出环形队列
 * @param {st_stream_buff} *pt_stream_buff
 * @param {uint8_t} *p8_out
 * @param {int} s32_size
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_stream_buff_out(st_stream_buff *pt_stream_buff, uint8_t *p8_out, int s32_size)
{
    int i;
    int r_t;
    int s32_filled;
    int s32_out_size;

    //for circulate copy
    int part1;
    int part2;

    //the max out bytes is current filled
    s32_filled = tcp_stream_buff_filled(pt_stream_buff);
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


/**
 * @name: tcp_stream_buff_filled
 * @des:  返回环形队列数据剩余
 * @param {st_stream_buff} *st_stream_buff
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_stream_buff_filled(st_stream_buff *st_stream_buff)
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


/**
 * @name: is_2_power
 * @des:  2整幂判断
 * @param {int} number
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
bool is_2_power(int number)
{
    int j = 1;
    while (number>j) 
    {
        j<<=1;
    }
    return j==number?true:false;
}

/**
 * @name: agv_cmd_handler_create
 * @des:  创建一个命令解析句柄
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
st_tcp_cmd_handler *agv_cmd_handler_create(const char *name, 
        uint16_t c, uint16_t t, tcp_cmd_func cmd_func)
{
    st_tcp_cmd_handler *pt_handler = 
        (st_tcp_cmd_handler *)malloc(sizeof(st_tcp_cmd_handler));

    if (pt_handler == NULL)
    {
        return NULL;
    }

    strcpy(pt_handler->name, name);
    pt_handler->c = c;
    pt_handler->t = t;
    pt_handler->cmd_func = cmd_func;

    return pt_handler;

}

/**
 * @name: tcp_cmd_handler_delete
 * @des:  销毁一个命令解析句柄
 * @param {st_tcp_cmd_handler} *pt_handler
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int tcp_cmd_handler_delete(st_tcp_cmd_handler *pt_handler)
{
    if (pt_handler != NULL)
    {
        free(pt_handler);
    }

    return 0;
}
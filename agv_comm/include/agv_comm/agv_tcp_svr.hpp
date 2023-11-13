
#ifndef AGV_TCP_SVR_H
#define AGV_TCP_SVR_H

#include "agv_comm/agv_tcp_svr.hpp"

//For TCP send and recieve buffer
#define AGV_FILE_TCPBUFF_SIZE 8192

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

int agv_stream_buff_init(st_stream_buff *st_stream_buff, uint8_t *p8_alloc, int s32_size);

st_stream_buff *agv_stream_buff_create(int s32_size);
int agv_stream_buff_delete(st_stream_buff *pt_stream_buff);

int agv_stream_buff_in(st_stream_buff *st_stream_buff, uint8_t *p8_in, int s32_size);
int agv_stream_buff_check(st_stream_buff *st_stream_buff, int *s32_cmd_total_size);
int agv_stream_buff_out(st_stream_buff *st_stream_buff, uint8_t *p8_out, int s32_size);
int agv_stream_buff_filled(st_stream_buff *st_stream_buff);

//考虑长帧长时间未收全导致队列长时间挂起，需要周期性监测用来刷新缓冲区
int agv_stream_buff_fresh(st_stream_buff *st_stream_buff);

//-------------------------command utils--------------------------------------------------------
bool is_2_power(int number);
int agv_cmd_get_sq(const uint8_t *p8_cmd, uint32_t *p32_sq);
int agv_cmd_get_plen(const uint8_t *p8_cmd, uint32_t *p32_plen);

//-------------------------command utils--------------------------------------------------------
void *thread_tcp_svr_instance(void *arg);

#endif // AGV_TCP_SVR_H






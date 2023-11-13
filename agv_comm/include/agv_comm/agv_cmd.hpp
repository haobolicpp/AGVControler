#ifndef AGV_MSG_H
#define AGV_MSG_H

#include <stdint.h>
#include <netinet/in.h>

#include "agv_comm/sys_queue.h"
#include "agv_comm/agv_type.h"

//-------------COMMON HEADER----------------
#define AGV_FRAME_DILIMTER_LEN 4
#define AGV_FRAME_H_0 0xff
#define AGV_FRAME_H_1 0xff
#define AGV_FRAME_H_2 0xff
#define AGV_FRAME_H_3 0xff
#define AGV_COMMON_HEADER_LEN 20
#define AGV_COMMON_HEADER_OFFSET_SRC 4
#define AGV_COMMON_HEADER_OFFSET_DEST 6
#define AGV_COMMON_HEADER_OFFSET_SQ 8
#define AGV_COMMON_HEADER_OFFSET_C 12
#define AGV_COMMON_HEADER_OFFSET_T 14
#define AGV_COMMON_HEADER_OFFSET_PLEN 16

//-------------GLOBAL DEF-------------------
#define AGV_CMD_MAX 16*1024*1024 - AGV_COMMON_HEADER_LEN
#define AGV_CMD_S_BODY_LEN 32
//todo

//-------------CTRL CLASS-------------------
#define AGV_CMD_C_UDP 0
#define AGV_CMD_T_UDP_BROADCAST 1 
//todo

#define AGV_CMD_C_CTRL 0
#define AGV_CMD_T_CTRL_MANI_WORD 3
#define AGV_CMD_T_CTRL_SET_STAION 6
#define AGV_CMD_T_CTRL_CANCEL_MOVE 7
#define AGV_CMD_T_CTRL_GLOBAL_PATH 9
#define AGV_CMD_T_CTRL_MANI_REQ 11


//-------------MAP CLASS--------------------
#define AGV_CMD_C_MAP 1
#define AGV_CMD_T_MAP_ACK 0
#define AGV_CMD_T_MAP_START_SLAM 1
#define AGV_CMD_T_MAP_STOP_SLAM 2

#define AGV_CMD_T_MAP_DOWNLOAD_REQ 4
//todo


//-------------MSG Header Struct------------------
#pragma pack (1)
struct agv_msg_header
{
    //uint32_t u32_delimiter;
    uint16_t u16_src;
    uint16_t u16_dest;
    uint32_t u32_sq;
    uint16_t u16_class;
    uint16_t u16_type;
    int32_t s32_len;

};
#pragma pack ()


//-------------MSG Header Struct------------------
struct agv_msg
{
    st_agv_msg_header t_msg_header;

    union ut_msg_body
    {
        uint8_t sbody[AGV_CMD_S_BODY_LEN];
        uint8_t *lbody;
        uint8_t src_flag;
    }t_msg_body;
    
    //private for msg send back
    //struct sockaddr_in t_cli_addr;
    //st_tcp_connect_info *pt_connect_info;

};

//-------------Agv Cmd Process Node------------------


typedef  int (*agv_cmd_func)(void *data, 
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);


struct agv_cmd_handler
{
    char name[32];

    uint16_t c;
    uint16_t t;
    agv_cmd_func cmd_func;

    LIST_ENTRY(agv_cmd_handler) node;

};


st_agv_cmd_handler *agv_cmd_handler_create(const char *name, 
        uint16_t c, uint16_t t, agv_cmd_func cmd_func);

int agv_cmd_handler_delete(st_agv_cmd_handler *pt_handler);






//todo-----------------------------------------------------------
int agv_comm_msg_to_frame(st_agv_msg *pt_agv_comm_msg, 
        struct sockaddr_in *pt_cli_addr,
        uint8_t *p_frame, int *p_frame_len);

int agv_comm_frame_to_msg(st_agv_msg *pt_agv_comm_msg, 
        struct sockaddr_in *pt_cli_addr,
        uint8_t *p_frame, int frame_len);

extern int agv_comm_svr_post_msg(st_agv_msg t_agv_msg);
extern int agv_msg_svr_post_msg(st_agv_msg t_agv_msg); 





#endif // AGV_MSG_H
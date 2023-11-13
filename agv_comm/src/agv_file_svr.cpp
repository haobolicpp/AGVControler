
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <agv_comm/agv_type.h>
#include <agv_comm/agv_cmd.hpp>
#include <agv_comm/agv_file_svr.hpp>
#include <agv_comm/agv_tcp_ctrl.hpp>
#include <agv_comm/agv_map_ctrl.h>


#define FILE_SVR_PARA_VALID     \
    if (pt_connect_info == NULL \
        || cmd_in == NULL       \
        || ilen < AGV_COMMON_HEADER_LEN) \
    {                           \
        return -1;              \
    }


int file_map_download_req(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);
int file_svr_start_slam(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);
int file_svr_stop_slam(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);


/******************************************************************************
* 函数名称: file_svr_config_create()
* 作 用 域: 
* 功能描述: 创建文件服务配置结构实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
st_tcp_connect_config * file_svr_config_create()
{
    int s32_ret;
    
    st_tcp_connect_config *pt_config = 
           (st_tcp_connect_config *) malloc(sizeof(st_tcp_connect_config));
    
    if (pt_config == NULL)
    {
        return NULL;
    }

    pt_config->connect_type = E_AGV_SVR_FILE;
    pt_config->stream_buffer_size = 2 * AGV_FILE_MAX_SIZE;
    
    LIST_INIT(&pt_config->lh_cmd_handler);
    st_agv_cmd_handler *pt_handler;

//todo ADD the cmd process element
//-------------------------register map download req--------------------
    pt_handler = agv_cmd_handler_create("file_map_download_req", 
        AGV_CMD_C_MAP, 
        AGV_CMD_T_MAP_DOWNLOAD_REQ,
        file_map_download_req);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    pt_handler = agv_cmd_handler_create("file_svr_start_slam", 
        AGV_CMD_C_MAP, 
        AGV_CMD_T_MAP_START_SLAM,
        file_svr_start_slam);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    pt_handler = agv_cmd_handler_create("file_svr_stop_slam", 
        AGV_CMD_C_MAP, 
        AGV_CMD_T_MAP_STOP_SLAM,
        file_svr_stop_slam);
    tcp_connect_config_regist_handler(pt_config, pt_handler);
//-------------------------register map download req--------------------


    return pt_config;
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
int file_svr_config_delete(st_tcp_connect_config *pt_config)
{
    if (pt_config != NULL)
    {
        free(pt_config);
        pt_config = NULL;
    }
    return 0;
}


/******************************************************************************
* 函数名称: file_svr_map_requset()
* 作 用 域: 
* 功能描述: 地图申请响应
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int file_map_download_req(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;

    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    st_agv_msg_header *pt_msg_header = (st_agv_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);

    printf("HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);

    //存文件
    pt_connect_info->po_ros_ctrl->m_pAGVMapCtrl->save_map(cmd_in+sizeof(st_agv_msg_header)+AGV_FRAME_DILIMTER_LEN);

    //发布地图
    pt_connect_info->po_ros_ctrl->PublishGridmap();

    //ADD HEADER
    cmd_offset = 0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_1;
    cmd_out[cmd_offset++] = AGV_FRAME_H_2;
    cmd_out[cmd_offset++] = AGV_FRAME_H_3;

    pt_msg_header->u16_type = pt_msg_header->u16_type | 0x8000;
    pt_msg_header->s32_len = 0;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_agv_msg_header));
    cmd_offset = cmd_offset + sizeof(st_agv_msg_header);

     *olen = cmd_offset;

     return 0;

}



/******************************************************************************
* 函数名称: file_svr_start_slam()
* 作 用 域: 
* 功能描述: 建图请求，后续将持续上报地图
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int file_svr_start_slam(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    st_agv_msg_header *pt_msg_header = (st_agv_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    
    ret = tcp_connect_regist_event(pt_connect_info, AGV_EVENT_SUB_MAP_INFO, 100);
    if (ret != 0)
    {
        *olen = 0;
        return -1;
    }

    pt_connect_info->po_ros_ctrl->CallChangeModeSvr(false);
    pt_connect_info->po_ros_ctrl->CallStopPubMapSvr(false);
    usleep(10000);
    pt_connect_info->po_ros_ctrl->ChangeStateToSlam();

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

    pt_msg_header->u16_type = pt_msg_header->u16_type | 0x8000;
    pt_msg_header->u16_src = 0x10;
    pt_msg_header->u16_dest = 0x1;
    pt_msg_header->s32_len = 0;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_agv_msg_header));
    cmd_offset = cmd_offset + sizeof(st_agv_msg_header);

     *olen = cmd_offset;

     return 0;

}


/******************************************************************************
* 函数名称: file_svr_stop_slam()
* 作 用 域: 
* 功能描述: 结束建图请求，最后上传一次地图相关数据
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int file_svr_stop_slam(void *data,
            uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    char file_name[256] = {0};
    st_agv_msg t_agv_msg = {0};


    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    st_agv_msg_header *pt_msg_header = (st_agv_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    CAgvRosCtrl *po_ros_ctrl = pt_connect_info->po_ros_ctrl;


    printf("HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);


    *olen = 0;
    ret = tcp_connect_remove_event(pt_connect_info, AGV_EVENT_SUB_MAP_INFO);
    if (ret != 0)return -1;

//clear the msg queue for prohabit the history map data send by tcp!!
//the agv client dynimc change the ring buffer's size 
    while(1)
    {
        ret = mq_receive(pt_connect_info->q_svr_handler, (char *)&t_agv_msg,
                        sizeof(st_agv_msg), NULL);
        printf("msg_q clearing!!\n");
        if (ret == 0)
        {
            if (t_agv_msg.t_msg_body.src_flag == 0)
            {
                free(t_agv_msg.t_msg_body.lbody); //todo 地图数据有可能小于32个字节吗？
            }
        }
        else
        {
            break;
        }
    }


    ret = pt_connect_info->po_ros_ctrl->CallChangeModeSvr(true);
    ret = pt_connect_info->po_ros_ctrl->CallStopPubMapSvr(true);
    usleep(10000);
    pt_connect_info->po_ros_ctrl->ChangeStateToNavi();
    //服务调用完成后，在当前用户的Downloads文件下保存地图相关的三个文件
    //这里仅用pbstream 其他地图信息由最后一次发布的/map中获得（等效yaml pgm）
    //构建pbstream 文件名称
    int pbstream_size;
    char *pbstream_buff;
    sprintf(file_name, "/home/%s/Downloads/Default.pbstream", getlogin());
    FILE *f_pbstream = fopen(file_name,"rb");
    if (f_pbstream == NULL)
    {
        printf("cannot open pbstream-file\n");
        return -1;
    }
    
    fseek(f_pbstream,0,SEEK_END);
    pbstream_size=ftell(f_pbstream);
    pbstream_buff=(char *) malloc(pbstream_size);
    if (pbstream_buff == NULL)
    {
        printf("error: resource limited!\n");
        return -1;
    }

    fseek(f_pbstream, 0, SEEK_SET);
    ret = fread(pbstream_buff, 1, pbstream_size, f_pbstream);

    if (ret != pbstream_size) 
    {
        free(pbstream_buff);
        printf("error: pbstream read error\n");
        return -1;
    }

    //ADD HEADER
    cmd_offset = 0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_1;
    cmd_out[cmd_offset++] = AGV_FRAME_H_2;
    cmd_out[cmd_offset++] = AGV_FRAME_H_3;

    pt_msg_header->u16_type = pt_msg_header->u16_type | 0x8000;
    pt_msg_header->u16_src = 0x10;
    pt_msg_header->u16_dest = 0x1;
    pt_msg_header->s32_len = 0;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_agv_msg_header));
    cmd_offset = cmd_offset + sizeof(st_agv_msg_header);

    po_ros_ctrl->RLockMap();
    //Resolution
    float fResolution = (float)po_ros_ctrl->tMapInfo.dResolution;
    memcpy((cmd_out + cmd_offset), &fResolution, sizeof(float));
    cmd_offset = cmd_offset + sizeof(float);
    //Map H
    memcpy((cmd_out + cmd_offset), &po_ros_ctrl->tMapInfo.iHeight, sizeof(int));
    cmd_offset = cmd_offset + sizeof(int);
    //Map W
    memcpy((cmd_out + cmd_offset), &po_ros_ctrl->tMapInfo.iWidth, sizeof(int));
    cmd_offset = cmd_offset + sizeof(int);
    //Ox
    float fOriginXOffset = (float)po_ros_ctrl->tMapInfo.dOriginXOffset;
    memcpy((cmd_out + cmd_offset), &fOriginXOffset, sizeof(float));
    cmd_offset = cmd_offset + sizeof(float);
    //Oy
    float fOriginYOffset = (float)po_ros_ctrl->tMapInfo.dOriginYOffset;
    memcpy((cmd_out + cmd_offset), &fOriginYOffset, sizeof(float));
    cmd_offset = cmd_offset + sizeof(float);
    //MapBody
    int map_size = po_ros_ctrl->tMapInfo.iHeight * po_ros_ctrl->tMapInfo.iWidth;
    if (map_size > 0)
    {
        memcpy((cmd_out + cmd_offset), po_ros_ctrl->tMapInfo.pCostMapData, map_size);
        cmd_offset = cmd_offset + map_size;
    }
    po_ros_ctrl->ULockMap();

    //pbstream size
    memcpy((cmd_out + cmd_offset), &pbstream_size, sizeof(int));
    cmd_offset = cmd_offset + sizeof(int);
    //pbstream body
    memcpy((cmd_out + cmd_offset), pbstream_buff, pbstream_size);
    cmd_offset = cmd_offset + pbstream_size;
    
    //total body len
    cmd_body_len = cmd_offset - sizeof(st_agv_msg_header) - 4;
    memcpy((cmd_out + AGV_COMMON_HEADER_OFFSET_PLEN), &cmd_body_len, sizeof(int));

    *olen = cmd_offset;

    free(pbstream_buff);

    printf("send back pbstream!!\n");
    fflush(stdout);

    return 0;
}
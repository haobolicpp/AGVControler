
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "agv_type.h"
#include "agv_cmd.h"
#include "agv_file_svr.h"
#include "agv_tcp_ctrl.h"
#include "agv_map_ctrl.h"
#include "AgvCtrl.h"


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
int file_svr_rvd_image_req(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen);

/**
 * @name: file_svr_config_create
 * @des:  文件传输服务配置
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
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
    st_tcp_cmd_handler *pt_handler;

//todo ADD the cmd process element
//-------------------------register map download req--------------------
    pt_handler = agv_cmd_handler_create("file_map_download_req", 
        AGV_CMD_C_FILE, 
        AGV_CMD_T_MAP_DOWNLOAD_REQ,
        file_map_download_req);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    pt_handler = agv_cmd_handler_create("file_svr_start_slam", 
        AGV_CMD_C_FILE, 
        AGV_CMD_T_MAP_START_SLAM,
        file_svr_start_slam);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    pt_handler = agv_cmd_handler_create("file_svr_stop_slam", 
        AGV_CMD_C_FILE, 
        AGV_CMD_T_MAP_STOP_SLAM,
        file_svr_stop_slam);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

    pt_handler = agv_cmd_handler_create("file_svr_rvd_image_req", 
        AGV_CMD_C_FILE, 
        AGV_CMD_T_RVD_IMAGE_REQ,
        file_svr_rvd_image_req);
    tcp_connect_config_regist_handler(pt_config, pt_handler);

//todo------------------register map download req--------------------

    return pt_config;
}


/**
 * @name: file_svr_config_delete
 * @des:  文件传输服务销毁
 * @param {st_tcp_connect_config} *pt_config
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int file_svr_config_delete(st_tcp_connect_config *pt_config)
{
    if (pt_config != NULL)
    {
        free(pt_config);
        pt_config = NULL;
    }
    return 0;
}


/**
 * @name: file_map_download_req
 * @des: 
 * @param {void} *data
 * @param {uint8_t} *cmd_in
 * @param {int} ilen
 * @param {uint8_t} *cmd_out
 * @param {int} *olen
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int file_map_download_req(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;

    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *> (pt_connect_info->pRoot);
    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);

    printf("INFO: TCP File Server Get HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);

    //存文件
    poAgvCtrl->poAgvMapCtrl->save_map(cmd_in + sizeof(st_robot_msg_header) + AGV_FRAME_DILIMTER_LEN);

    //ADD HEADER
    cmd_offset = 0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_0;
    cmd_out[cmd_offset++] = AGV_FRAME_H_1;
    cmd_out[cmd_offset++] = AGV_FRAME_H_2;
    cmd_out[cmd_offset++] = AGV_FRAME_H_3;

    pt_msg_header->u16_type = pt_msg_header->u16_type | 0x8000;
    pt_msg_header->s32_len = 0;

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_robot_msg_header));
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);

     *olen = cmd_offset;

     return 0;

}

/**
 * @name: file_svr_start_slam
 * @des:  启动建图命令
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int file_svr_start_slam(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *> (pt_connect_info->pRoot);
    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    
    printf("INFO: TCP File Server Get HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);

    ret = tcp_connect_regist_event(pt_connect_info, AGV_EVENT_SUB_MAP_INFO, 250);
    if (ret != 0)
    {
        *olen = 0;
        return -1;
    }

    //TODO做状态机
    poAgvCtrl->CallChangeModeSvr(false);
    
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

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_robot_msg_header));
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);

     *olen = cmd_offset;

     return 0;

}


/**
 * @name: file_svr_stop_slam
 * @des:  结束建图命令
 * @param {void} *data
 * @param {uint8_t} *cmd_in
 * @param {int} ilen
 * @param {uint8_t} *cmd_out
 * @param {int} *olen
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int file_svr_stop_slam(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    char file_name[256] = {0};
    st_robot_msg t_agv_msg = {0};

    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *> (pt_connect_info->pRoot);
    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);

    printf("INFO: TCP File Server Get HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);


    *olen = 0;
    ret = tcp_connect_remove_event(pt_connect_info, AGV_EVENT_SUB_MAP_INFO);
    if (ret != 0)return -1;

//clear the msg queue for prohabit the history map data send by tcp!!
//the agv client dynimc change the ring buffer's size 


    ret = poAgvCtrl->CallChangeModeSvr(true);
    // ret = pt_agv_ctrl->po_ros_ctrl->CallStopPubMapSvr(true);
    //todo 等待优化完成！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
    while(!poAgvCtrl->poAgvSlammer->MapOptimizationDone)
    {
        //获取地图优化进度
        double process = poAgvCtrl->poAgvSlammer->ReportProgress();
        sleep(1);
    }
    // usleep(10000);

    //pt_agv_ctrl->po_ros_ctrl->ChangeStateToNavi();
    //服务调用完成后，在当前用户的Downloads文件下保存地图相关的三个文件
    //这里仅用pbstream 其他地图信息由最后一次发布的/map中获得（等效yaml pgm）
    //构建pbstream 文件名称
    int pbstream_size;
    char *pbstream_buff;
    sprintf(file_name, "/home/%s/Downloads/Default.pbstream", getlogin());
    FILE *f_pbstream = fopen(file_name,"rb");
    if (f_pbstream == NULL)
    {
        printf("ERROR: Cannot Open Map Pbstream File!\n");
        return -1;
    }
    
    fseek(f_pbstream,0,SEEK_END);
    pbstream_size=ftell(f_pbstream);
    pbstream_buff=(char *) malloc(pbstream_size);
    if (pbstream_buff == NULL)
    {
        printf("ERROR: Resource Limited When Read Map Pbstream!\n");
        return -1;
    }

    fseek(f_pbstream, 0, SEEK_SET);
    ret = fread(pbstream_buff, 1, pbstream_size, f_pbstream);

    if (ret != pbstream_size) 
    {
        free(pbstream_buff);
        printf("ERROR: Map Pbstream Read Error\n");
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

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_robot_msg_header));
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);

    {
        boost::unique_lock<boost::recursive_mutex> lock(*poAgvCtrl->lockMapInfo_);//LOCK
        float fResolution = (float)poAgvCtrl->tMapInfo_.dResolution;
        memcpy((cmd_out + cmd_offset), &fResolution, sizeof(float));
        cmd_offset = cmd_offset + sizeof(float);
        //Map H
        memcpy((cmd_out + cmd_offset), &poAgvCtrl->tMapInfo_.iHeight, sizeof(int));
        cmd_offset = cmd_offset + sizeof(int);
        //Map W
        memcpy((cmd_out + cmd_offset), &poAgvCtrl->tMapInfo_.iWidth, sizeof(int));
        cmd_offset = cmd_offset + sizeof(int);
        //Ox
        float fOriginXOffset = (float)poAgvCtrl->tMapInfo_.dOriginXOffset;
        memcpy((cmd_out + cmd_offset), &fOriginXOffset, sizeof(float));
        cmd_offset = cmd_offset + sizeof(float);
        //Oy
        float fOriginYOffset = (float)poAgvCtrl->tMapInfo_.dOriginYOffset;
        memcpy((cmd_out + cmd_offset), &fOriginYOffset, sizeof(float));
        cmd_offset = cmd_offset + sizeof(float);
        //MapBody
        int map_size = poAgvCtrl->tMapInfo_.iHeight * poAgvCtrl->tMapInfo_.iWidth;
        if (map_size > 0)
        {
            memcpy((cmd_out + cmd_offset), poAgvCtrl->tMapInfo_.pCostMapData, map_size);
            cmd_offset = cmd_offset + map_size;
        }
    }
    
    //pbstream size
    memcpy((cmd_out + cmd_offset), &pbstream_size, sizeof(int));
    cmd_offset = cmd_offset + sizeof(int);
    //pbstream body
    memcpy((cmd_out + cmd_offset), pbstream_buff, pbstream_size);
    cmd_offset = cmd_offset + pbstream_size;
    
    //total body len
    cmd_body_len = cmd_offset - sizeof(st_robot_msg_header) - 4;
    memcpy((cmd_out + AGV_COMMON_HEADER_OFFSET_PLEN), &cmd_body_len, sizeof(int));

    *olen = cmd_offset;

    free(pbstream_buff);

    printf("INFO: Tcp Server Send Back Map pbstream...\n");
    fflush(stdout);

    return 0;
}


/**
 * @name: Client 申请获取靶标位姿原始图
 * @des: 
 * @param {void} *data
 * @param {uint8_t} *cmd_in
 * @param {int} ilen
 * @param {uint8_t} *cmd_out
 * @param {int} *olen
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int file_svr_rvd_image_req(void *data, uint8_t *cmd_in, int ilen, uint8_t *cmd_out, int *olen)
{
    int ret;
    int cmd_total_len;
    int cmd_body_len;
    int cmd_offset;
    
    st_tcp_connect_info *pt_connect_info = (st_tcp_connect_info *)data;
    AgvCtrl *poAgvCtrl = static_cast<AgvCtrl *> (pt_connect_info->pRoot);
    st_robot_msg_header *pt_msg_header = (st_robot_msg_header *)(cmd_in + AGV_FRAME_DILIMTER_LEN);
    printf("INFO: TCP File Server Get HANDLER-C[%.2x]-T[%.2x]-PLEN[%.2d]\n", 
            pt_msg_header->u16_class,
            pt_msg_header->u16_type,
            pt_msg_header->s32_len);
    
    poAgvCtrl->RvdImageRequest(pt_connect_info);
    
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

    memcpy((cmd_out + cmd_offset), pt_msg_header, sizeof(st_robot_msg_header));
    cmd_offset = cmd_offset + sizeof(st_robot_msg_header);

     *olen = cmd_offset;

     return 0;
}
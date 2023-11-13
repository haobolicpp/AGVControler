#ifndef AGV_CONFIG_H
#define AGV_CONFIG_H
#include "agv_comm/agv_type.h"

//agv base config
struct agv_base_config
{
    char chMapName[256]; //map name
    char chMapMD5[16]; //map md5
};

//agv other config


/**
 * @brief 
 * 
 * @param pt_agv_ctrl 
 * @return int 
 */
int read_agv_config_from_json(st_agv_ctrl *pt_agv_ctrl);

/**
 * @brief 
 * 
 * @param pt_agv_ctrl 
 * @return int 
 */
int write_agv_config_to_json(st_agv_ctrl *pt_agv_ctrl);

/**
     * @brief 递归创建目录
     * 
     * @param pstr_dir 
     * @return int 
     */
int create_dir_recursive(const char *pstr_dir);

#endif // AGV_CONFIG_H
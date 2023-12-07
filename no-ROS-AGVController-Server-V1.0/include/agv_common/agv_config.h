/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-30 09:02:01
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 11:55:24
 * @FilePath: /agv_controller/include/agv_common/agv_config.h
 * @Description: 
 */
#ifndef AGV_CONFIG_H
#define AGV_CONFIG_H

#include "agv_type.h"


#define AGV_CTRL_M1_VERSION 1
#define AGV_CTRL_S1_VERSION 1
#define AGV_CTRL_S2_VERSION 2


//agv base config
typedef struct agv_base_config
{
    char chMapName[256]; //map name
    char chMapMD5[16]; //map md5
}st_agv_base_config;

//agv other config


/**
 * @brief 
 * 
 * @param pt_agv_ctrl 
 * @return int 
 */
int read_agv_config_from_json(AgvCtrl *pt_agv_ctrl);

/**
 * @brief 
 * 
 * @param pt_agv_ctrl 
 * @return int 
 */
int write_agv_config_to_json(AgvCtrl *pt_agv_ctrl);

/**
     * @brief 递归创建目录
     * 
     * @param pstr_dir 
     * @return int 
     */
int create_dir_recursive(const char *pstr_dir);

#endif // AGV_CONFIG_H
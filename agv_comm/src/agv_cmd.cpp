#ifndef AGV_CMD_H
#define AGV_CMD_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "agv_comm/agv_cmd.hpp"

/******************************************************************************
* 函数名称: agv_cmd_handler_create()
* 作 用 域: 
* 功能描述: 创建一个AGV命令处理实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
st_agv_cmd_handler *agv_cmd_handler_create(const char *name, 
        uint16_t c, uint16_t t, agv_cmd_func cmd_func)
{
    st_agv_cmd_handler *pt_handler = 
        (st_agv_cmd_handler *)malloc(sizeof(st_agv_cmd_handler));

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

/******************************************************************************
* 函数名称: agv_cmd_handler_delete()
* 作 用 域: 
* 功能描述: 销毁一个AGV命令处理实例
* 输入参数: 
* 输出参数: 
* 全局变量: 
* 返 回 值:
*
* 版     本: Ver 1.0
* 作     者: yang.cheng
* 完成日期: 2021年6月5日
******************************************************************************/
int agv_cmd_handler_delete(st_agv_cmd_handler *pt_handler)
{
    if (pt_handler != NULL)
    {
        free(pt_handler);
    }

    return 0;
}


#endif // AGV_CMD_H




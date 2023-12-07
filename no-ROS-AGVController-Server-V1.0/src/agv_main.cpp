/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-15 17:07:31
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-10-12 17:13:27
 * @FilePath: /agv_controller/src/agv_main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <tlog.h>

#include "AgvCtrl.h"
/**
 * @name: main
 * @des: 
 * @param {int} argc
 * @param {char} *
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int main(int argc, char **argv)
{
    int s32Ret;
    char log_file[128] = {0};

    //系统LOG初始化启动
    sprintf(log_file, "/home/%s/log/agv_ctrl.log", getlogin());
    s32Ret = tlog_init(log_file, 1024 * 1024 * 32, 10, 0, 0);
    tlog_setlogscreen(1);
    if (s32Ret < 0)
    {
        printf("ERROR: Agv Controller Log System Start Failed!\n");
        return 0;
    }
    tlog(TLOG_INFO, "Bgi Agv Controller Start Up!\n");

    //系统各模块初始化与启动
    AgvCtrl *m_AgvCtrl = new AgvCtrl();
    m_AgvCtrl->Init();
    m_AgvCtrl->Start();

    //初始化用于AGV通信与控制的数据结构
    while(1)
    {
        sleep(1);
    }

    return 0;
}

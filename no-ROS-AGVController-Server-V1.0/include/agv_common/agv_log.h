/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-02 10:50:59
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-02 11:02:26
 * @FilePath: /agv_controller/include/agv_common/agv_log.h
 * @Description: Log接口函数
 */


#ifndef AGV_LOG_H
#define AGV_LOG_H

#include "tlog.h"
#include "agv_type.h"

int LogAgvPose2D(TAgvPose2D tAgvPose2D);

#endif // AGV_LOG_H
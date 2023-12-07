/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-02 10:50:43
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-08-02 11:08:51
 * @FilePath: /agv_controller/src/agv_common/agv_log.cpp
 * @Description: Log接口函数
 */


#include "tlog.h"
#include "agv_type.h"

/**
 * @name: 
 * @des:  记录一个AGV 2D位姿
 * @param {TAgvPose2D} tAgvPose2D
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int LogAgvPose2D(TAgvPose2D tAgvPose2D)
{
    tlog(TLOG_INFO, "AgvPose2D X[%f]-Y[%f]-Phi[%f]\n", 
        tAgvPose2D.x, tAgvPose2D.y, tAgvPose2D.phi);

    return 0;
}

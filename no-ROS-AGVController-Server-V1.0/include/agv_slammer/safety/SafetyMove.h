/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-16 15:31:35
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-05 15:53:06
 * @FilePath: /agv_controller/include/agv_slammer/safety/SafetyMove.h
 * @Description: 基于局部地图的安全监测实现
 */
#ifndef SAFETYMOVE_H
#define SAFETYMOVE_H


#include "agv_type.h"
#include "AgvTf.h"
#include "costmap_2d.h"
#include "agv_type.h"


class SafetyMove
{
private:
    /* data */
public:

    double RangeMax_;
    double RangeMin_;
    //激光雷达监测建立局部地图
    float fMapDetectSizeX_;
    float fMapDetectSizeY_;
    float fMapDetectResolution_;
    costmap_2d::Costmap2D *pMapDetect_;

    //定义安全区域地图
    float fMapSafetySizeX_;
    float fMapSafetySizeY_;
    float fMapSafetyResolution_;
    costmap_2d::Costmap2D *pMapSafety_;
    float centerXMapSafety_; //安全监测区域相对于BaskLink存在一个变换 由当前速度决定安全监测区域的位姿
    float centerYMapSafety_;
    float centerThetaMapSaftey_;
    int s32SafteyLevel_;

    //参数
    float laserRangeMax_;
    bool laserVisit_;
    int laserSampler_;
    int laserSamplerSet_;

    AgvTf *poAgvTf_;
    TAgvPose2D tCurLocalPose_;
    Rigid3d base2safety_;

public:
    SafetyMove(AgvTf *poAgvTf);
    ~SafetyMove();

public:
    int Init();
    int Update(TAgvPose2D tCurLocalPose);
    int PointCloudProcess(TAgvPointCloud2 tPointCloudInOdom);

    int LaserScanToPointCloud(const TAgvLaserScanMsg &tLaserScanMsgIn, const int stepOver,
        TAgvPose2D tO2L,TAgvPointCloud2 &tPointCloudOut);
    bool SafetyControlCompute(int &s32CurSafetyLevel);

    int ConvertCostMapToGrid(costmap_2d::Costmap2D *ptCostMap, TOccupancyGrid &tGridMap);

};

#endif // SAFETYMOVE_H




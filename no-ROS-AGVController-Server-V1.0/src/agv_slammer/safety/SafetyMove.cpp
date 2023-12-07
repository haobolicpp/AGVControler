/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-08-16 15:32:21
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-13 13:32:32
 * @FilePath: /agv_controller/src/agv_slammer/safety/SafetyMove.cpp
 * @Description: 用于安全策略的局部地图处理方法
 */

#include "agv_type.h"
#include "AgvTf.h"
#include "costmap_2d.h"
#include "SafetyMove.h"

using namespace costmap_2d;
// using costmap_2d::NO_INFORMATION;
// using costmap_2d::LETHAL_OBSTACLE;
// using costmap_2d::FREE_SPACE;

/**
 * @brief SafetyMove构造
 * @param poAgvTf 全局坐标树
 * @return
 */
SafetyMove::SafetyMove(AgvTf *poAgvTf)
{
    //全局坐标树访问关联
    poAgvTf_ = poAgvTf;

    RangeMax_ = 3.0;
    RangeMin_ = 0.0;
    //激光雷达监测建立局部地图 默认赋值
    fMapDetectSizeX_ = 3.0;
    fMapDetectSizeY_ = 3.0;
    fMapDetectResolution_ = 0.05;
    pMapDetect_ = new costmap_2d::Costmap2D();

    //定义安全区域地图 默认赋值
    fMapSafetySizeX_ = 1.0;
    fMapSafetySizeY_ = 1.0;
    fMapSafetyResolution_ = 0.1;
    pMapSafety_ = new costmap_2d::Costmap2D(); //不需要栅格内容扩展
    s32SafteyLevel_ = 0;

    laserRangeMax_ = 8;
    laserVisit_ = false;
    laserSampler_ = 0;
    laserSamplerSet_ = 3;
}

/**
 * @brief SafetyMove析构
 * @param 
 * @return
 */
SafetyMove::~SafetyMove()
{
    delete pMapDetect_;
    delete pMapSafety_;
}

/**
 * @brief SafetyMove用户初始化接口
 * @param 
 * @return
 */
int SafetyMove::Init()
{
    //0 安全监测区域到BaseLink的变换
    double thetaMapSafety = 0.0;
    double xMapSafety = 0.5;
    double yMapSafety = 0.0;
    //1 确定BaseLink到安全区域坐标系的变换的变换-------------------------------------
    const Eigen::AngleAxisd yaw_angle(thetaMapSafety, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quaternion(yaw_angle);
    base2safety_ = Rigid3d({xMapSafety, yMapSafety, 0.0}, quaternion);
    //初始化相关地图
    pMapDetect_->resizeMap((unsigned int)fMapDetectSizeX_/fMapDetectResolution_, (unsigned int)(fMapDetectSizeY_/fMapDetectResolution_), fMapDetectResolution_, 0, 0);
    pMapSafety_->resizeMap((unsigned int)fMapSafetySizeX_/fMapDetectResolution_, (unsigned int)(fMapSafetySizeY_/fMapDetectResolution_), fMapSafetyResolution_, 0, 0);
    return 0;
}

/**
 * @brief SafetyMove局部地图更新
 * @param 
 * @return
 */
int SafetyMove::Update(TAgvPose2D tCurLocalPose)
{
    //1-获取当前时刻AGV在Odom下的位姿
    tCurLocalPose_.x = tCurLocalPose.x;
    tCurLocalPose_.y = tCurLocalPose.y;
    tCurLocalPose_.phi = tCurLocalPose.phi;

    //根据位姿更新动态局部地图的起始点
    double new_origin_x = tCurLocalPose_.x - pMapDetect_->getSizeInMetersX() / 2;
    double new_origin_y = tCurLocalPose_.y - pMapDetect_->getSizeInMetersY() / 2;
    pMapDetect_->updateOrigin(new_origin_x, new_origin_y);

    //置安全功能启用标志为true 防止地图原点未更新导致地图更新出现问题
    laserVisit_ = true;
    //2 安全区域坐标系原点作为地图中心点重置当前安全监测区域地图-----------------------
    double xOriginSafety = 0 - pMapSafety_->getSizeInMetersX() / 2;
    double yOriginSafety = 0 - pMapSafety_->getSizeInMetersY() / 2;
    pMapSafety_->resizeMap((unsigned int)(fMapSafetySizeX_/fMapSafetyResolution_), 
    (unsigned int)(fMapSafetySizeY_/fMapSafetyResolution_), 
    fMapSafetyResolution_, xOriginSafety, yOriginSafety);
    //3 拷贝局部监测图数据到安全区域-------------------------------------------------
    int isFailed;
    int min_i, min_j, max_i, max_j; //获取Safty地图栅格索引范围
    unsigned int mx, my; //
    double wx, wy;
    unsigned char costInDetect;
    ::Rigid3d::Vector point3DSafety, point3DOdom;
    Rigid3d odom2base;
    Rigid3d odom2safety;
    isFailed = poAgvTf_->GetTransform("odom", "base_link", odom2base);
    if (isFailed != 0)
    {
        return -1;
    }
    odom2safety = odom2base * base2safety_;

    pMapSafety_->worldToMapEnforceBounds(-1e30, -1e30, min_i, min_j);
    pMapSafety_->worldToMapEnforceBounds(1e30, 1e30, max_i, max_j);
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        pMapSafety_->mapToWorld(i, j, wx, wy);
        // Transform from safety_frame_ to odom_frame_
        point3DSafety << wx, wy, 0;
        point3DOdom = odom2safety * point3DSafety;
        // Set master_grid with cell from map
        if (pMapDetect_->worldToMap(point3DOdom(0), point3DOdom(1), mx, my))
        {
            costInDetect = pMapDetect_->getCost(mx, my);
            pMapSafety_->setCost(i, j, costInDetect);
        }
        else
        {   //超出边界为探测到
            pMapSafety_->setCost(i, j, NO_INFORMATION);
        }
      }
    }

    // SafetyControlCompute();

    return 0;
}

/**
 * @name: PointCloudProcess
 * @des:  安全模块处理当前点云数据
 * @param {TAgvPointCloud2} tPointCloudInOdom
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int SafetyMove::PointCloudProcess(TAgvPointCloud2 tPointCloudInOdom)
{    
    int s32Ret;
    Rigid3d sensor_to_odom;
    poAgvTf_->GetTransform("odom", "laser", sensor_to_odom);

    //点云起点
    double ox = sensor_to_odom.translation().x();
    double oy = sensor_to_odom.translation().y();
    unsigned int x0, y0;
    //地图起始结束点
    double origin_x = pMapDetect_->origin_x_, origin_y = pMapDetect_->origin_y_;
    double map_end_x = origin_x + pMapDetect_->size_x_ * pMapDetect_->resolution_;
    double map_end_y = origin_y + pMapDetect_->size_y_ * pMapDetect_->resolution_;

    if (!pMapDetect_->worldToMap(ox, oy, x0, y0))
    {
        return -1;
    }
    //点云处理
    for(unsigned int i = 0; i < tPointCloudInOdom.points.size(); i++)
    {
        double wx = tPointCloudInOdom.points[i].x;
        double wy = tPointCloudInOdom.points[i].y; 

        double a = wx - ox;
        double b = wy - oy;

        // the minimum value to raytrace from is the origin
        if (wx < origin_x)
        {
            double t = (origin_x - ox) / a;
            wx = origin_x;
            wy = oy + b * t;
        }
        if (wy < origin_y)
        {
            double t = (origin_y - oy) / b;
            wx = ox + a * t;
            wy = origin_y;
        }

        // the maximum value to raytrace to is the end of the map
        if (wx > map_end_x)
        {
            double t = (map_end_x - ox) / a;
            wx = map_end_x - .001;
            wy = oy + b * t;
        }
        if (wy > map_end_y)
        {
            double t = (map_end_y - oy) / b;
            wx = ox + a * t;
            wy = map_end_y - .001;
        }

        // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
        unsigned int x1, y1;

        if (!pMapDetect_->worldToMap(wx, wy, x1, y1))
        {
            continue;
        }
        unsigned int cell_raytrace_range = (unsigned int)(max(0.0, ceil(laserRangeMax_ / pMapDetect_->resolution_)));
        Costmap2D::MarkCell marker(pMapDetect_->costmap_, FREE_SPACE);
        pMapDetect_->raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);
        int offset = y1 * pMapDetect_->size_x_ + x1;
        pMapDetect_->costmap_[offset] = LETHAL_OBSTACLE;
    }
    return 0;
}

/**
 * @name: SafetyControlCompute
 * @des: 
 * @param {int} &s32CurSafetyLevel 返回安全等级
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
bool SafetyMove::SafetyControlCompute(int &s32CurSafetyLevel)
{
    unsigned int u32CellId;
    unsigned int u32CellTotal;
    unsigned int u32CellHalf;
    unsigned char u8CellCostMax;

    bool bSafetyLevelChanged = false;
    int s32SafteyLevel = 0;
    //SafetyMap 栅格数量
    u32CellTotal = pMapSafety_->getSizeInCellsX() * pMapSafety_->getSizeInCellsY();
    u32CellHalf = 0.5 * u32CellTotal;

    //计算近距离障碍物占用概率
    u8CellCostMax = 0;
    for (u32CellId = 0; u32CellId < u32CellHalf; u32CellId++)
    {
        if (pMapSafety_->costmap_[u32CellId] > u8CellCostMax)
        {
            u8CellCostMax = pMapSafety_->costmap_[u32CellId];
        }
    }
    if (u8CellCostMax >= 90)
    {
        s32SafteyLevel = 2;
    }
    else
    {
        //计算远距离障碍物占用概率
        u8CellCostMax = 0;
        for (u32CellId = u32CellHalf; u32CellId < u32CellTotal; u32CellId++)
        {
            if (pMapSafety_->costmap_[u32CellId] > u8CellCostMax)
            {
                u8CellCostMax = pMapSafety_->costmap_[u32CellId];
            }
        }
        if (u8CellCostMax >= 90)
        {
            s32SafteyLevel = 1;
        }
    }
    //当前危险等级发生变换
    if (s32SafteyLevel_ != s32SafteyLevel)
    {
        bSafetyLevelChanged = true;
        s32SafteyLevel_ = s32SafteyLevel;
        s32CurSafetyLevel = s32SafteyLevel;
    }
    else
    {
        bSafetyLevelChanged = false;
        s32CurSafetyLevel = s32SafteyLevel;
    }
    
    return bSafetyLevelChanged;
}

/**
 * @name: LaserScanToPointCloud
 * @des:  转换雷达帧到点云 以雷达为原点
 * @param {TAgvLaserScanMsg} &tLaserScanMsgIn
 * @param {int} stepOver  间隔采样的间隔值
 * @param {TAgvPointCloud2} &tPointCloudOut
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int SafetyMove::LaserScanToPointCloud(const TAgvLaserScanMsg &tLaserScanMsgIn, 
        int stepOver, TAgvPose2D tO2L,
        TAgvPointCloud2 &tPointCloudOut)
{
    int s32Ret;
    int i;
    float angle = tLaserScanMsgIn.angle_min;
    float range;

    if (stepOver <= 0) 
    {
        stepOver = 1;
    }

    Eigen::Matrix3d MatO2L;
    Eigen::Vector3d PointInLaser;
    Eigen::Vector3d PointInOdom;
    MatO2L(0, 0) = cos(tO2L.phi);
    MatO2L(0, 1) = -sin(tO2L.phi);
    MatO2L(0, 2) = tO2L.x;
    MatO2L(1, 0) = sin(tO2L.phi);
    MatO2L(1, 1) = cos(tO2L.phi);
    MatO2L(1, 2) = tO2L.y;
    MatO2L(2, 0) = 0;
    MatO2L(2, 1) = 0;
    MatO2L(2, 2) = 1;


    tPointCloudOut.points.resize(0);

    for (i = 0; i < tLaserScanMsgIn.ranges_size; i++)
    {
        if(0 == (i % stepOver))
        {
            range = tLaserScanMsgIn.ranges[i];
            const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
            Eigen::Vector3f point;
            if( range <= RangeMax_)
            {
                point = rotation * (range * Eigen::Vector3f::UnitX());
            }
            else
            {
                point = rotation * (RangeMax_ * Eigen::Vector3f::UnitX());
            }
                const Eigen::Vector3d point_cast = point.cast<double>(); 
                PointInLaser = {point_cast(0), point_cast(1), 1};
                PointInOdom = MatO2L * PointInLaser;

                tPointCloudOut.points.push_back({PointInOdom(0), PointInOdom(1), 0});
        }
        angle += tLaserScanMsgIn.angle_increment;
    }

    return 0;
}


/**
 * @name: ConvertCostMapToGrid
 * @des:  转换CostMap到GridMap
 * @param {Costmap2D} *ptCostMap
 * @param {TOccupancyGrid} &tGridMap
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int SafetyMove::ConvertCostMapToGrid(costmap_2d::Costmap2D *ptCostMap, TOccupancyGrid &tGridMap)
{
    if (ptCostMap == NULL)
    {
        return -1;
    }

    memset(tGridMap.header.frame_id, 0, FRAME_IDS_MAX);
    snprintf(tGridMap.header.frame_id, FRAME_IDS_MAX, "odom");

    tGridMap.resolution = ptCostMap->getResolution();
    tGridMap.width = ptCostMap->getSizeInCellsX();
    tGridMap.height = ptCostMap->getSizeInCellsY();

    tGridMap.origin.position.x = ptCostMap->getOriginX();
    tGridMap.origin.position.y = ptCostMap->getOriginY();

    int mapSize = tGridMap.width * tGridMap.height;

    tGridMap.data.resize(0);
    for(int i = 0; i < mapSize; i++)
    {
        tGridMap.data.push_back(ptCostMap->costmap_[i]);
    }

    return 0;
}
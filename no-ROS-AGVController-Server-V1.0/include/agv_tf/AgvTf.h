/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 08:32:38
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-18 10:53:52
 * @FilePath: /agv_controller/include/agv_tf/AgvTf.h
 * @Description: 坐标转换服务
 */

#ifndef AGVTF_H
#define AGVTF_H

#include <string>
#include <mutex>
#include <thread>
#include <deque>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/pose_extrapolator.h"

#include "time.h"
#include "agv_type.h"
//自定义刚体变换描述命名空间
using namespace std;
using namespace cartographer::transform;
using namespace cartographer::mapping;

typedef struct timespec TLinuxTime;
//Transform 基本单元类定义
class AgvTfEntry
{
public:
    AgvTfEntry(string parent, string child, bool isStatic);
    ~AgvTfEntry();

public:
    string parent_;
    string child_;

    Rigid3d transform_;
    long int timestamp_;
    bool isStatic_;
    mutex mutex_;

    std::deque<Rigid3d> tf_buffer_;
public:
    int Set(Rigid3d transform);
    int Set(Rigid3d transform, long int timestamp);
    int SetTfBuffer(Rigid3d transform);
    int Get(Rigid3d &transform);
    int Get(Rigid3d &transform, long int &timestamp);
    int TrimTfBuffer();

};


//Transform 坐标系变换信息维护
class AgvTf
{
public:
    /* data */
    AgvTfEntry tf_world2map_;
    AgvTfEntry tf_map2odom_;
    AgvTfEntry tf_odom2base_;
    AgvTfEntry tf_base2laser_;
public:
    //外层外推器 用于PID控制的高频率实时位姿预测
    mutex mutexExpl_;
    PoseExtrapolator *ptExpl_;

public:
    AgvTf(/* args */);
    ~AgvTf();

public:
    int Init();

    int SetTransform(const string &source_frame_id, const string &target_frame_id, Rigid3d transform);
    int SetTransform(const string &source_frame_id, const string &target_frame_id, Rigid3d transform, bool is_buffer);

    int GetTransform(const string &source_frame_id, const string &target_frame_id, Rigid3d &transform);
    void AddMapToOdomPose(Rigid3d &transform);
    void TrimMapToOdomPose();
    
    int ResetExtrapolator();
    int AddPoseToExtrapolator(Rigid3d r3Pose);
    int AddOdomToExtrapolator(TAgvOdomMsg tOdomMsg);

    int GetGlobalPose3D(Rigid3d &r3Pose);
    int GetGlobalPose2D(TAgvPose2D &tAgvPose);

};

/**
 * @name: ToLinuxTime
 * @des:  Slam时间转Linux时间
 * @param {Time} time
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
TLinuxTime ToLinuxTime(::cartographer::common::Time time);
/**
 * @name: FromLinuxTime
 * @des: Linux时间转SLAM时间
 * @param {TLinuxTime} time
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
::cartographer::common::Time FromLinuxTime(TLinuxTime time);

/**
 * @name: LinuxTimeNow
 * @des: 当前系统时间
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
int LinuxTimeNow(TLinuxTime *ptLinuxTime);


#endif // AGVTF_H
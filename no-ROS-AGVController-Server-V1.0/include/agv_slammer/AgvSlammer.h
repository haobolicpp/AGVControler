/*********************************************************************
 * AgvSlammer Class Definiation
 * 模块主体
 * Copyright: BGI
 * Author: yang.cheng
 *********************************************************************/
#ifndef AGVSLAMMER_H
#define AGVSLAMMER_H

#include <stdint.h>
#include <pthread.h>
#include <mqueue.h>
#include <sys/time.h>

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/pose_extrapolator.h"

#include "agv_type.h"
#include "agv_cmd.h"

#include "AgvTf.h"
#include "SlamPara.h"
#include "SensorConvert.h"
#include "SensorBrige.h"
#include "AgvMapBuilder.h"
#include "AgvMapAssembler.h"
#include "AmclTracker.h"
#include "SafetyMove.h"

//SLAM模块的运行状态
enum class AgvSlammerState
{
    INIT,
    LOCALIZATION, //基于已有图的定位模式
    SLAM,       //扫图模式
    ERROR
};


//SLAM模块类定义
class AgvSlammer
{
private:
    mqd_t qSlamSvr;
    pthread_t thSlamSvr;
    /* data */
public:
    AgvCtrl *pt_agv_ctrl_;
    int s32CtrlFreq;
    bool bHasInited;
    bool bHasStarted;
    AgvSlammerState eState;
    static void *LoopSlamSvrStatic(void *arg);
    int LoopSlamSvr();
    //Todo AgvCtrl 指针包含
    // AgvTf *g_pAgvTf_;
    //定义slam模块消息
    TAgvOdomMsg *ptAgvOdomMsg;
    TAgvImuMsg *ptAgvImuMsg;
    TAgvLaserScanMsg *ptAgvLaserScanMsg;
    //消息处理和控制流程实现
public:
    int AsyncMsgPost(TRobotMsg *ptAgvMsg);
    int AsyncMsgHandler(TRobotMsg *ptAgvMsg);

    AgvSlammer(AgvCtrl *pt_agv_ctrl);
    ~AgvSlammer();

public:
    //Maybe Wasted
    //所有过程均在同一个线程中执行
    absl::Mutex mutex_;
    std::unique_ptr<AgvMapBuilder> upAgvMapBuilder_ GUARDED_BY(mutex_);
    std::unique_ptr<AgvMapAssembler> pAgvMapAssembler_ GUARDED_BY(mutex_);//地图发布器
    // AgvMapBuilder *upAgvMapBuilder_;
    // AgvMapAssembler *pAgvMapAssembler_;
    std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_; //运动预测器
    std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
    //用户配置
    NodeOptions node_options_;
    TrajectoryOptions trajectory_options_;
    //判断地图是否优化完成
    bool MapOptimizationDone;

    //根据配置文件获取传感器配置集
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
    ComputeExpectedSensorIds(const TrajectoryOptions& options) const;

    bool ValidateTrajectoryOptions(const TrajectoryOptions& options);

    int AddTrajectory(const TrajectoryOptions& options);
    void StartTrajectory(const TrajectoryOptions& options);

    //外推器和采样器
    void AddExtrapolator(const int trajectory_id, const TrajectoryOptions& options);
    void AddSensorSamplers(const int trajectory_id, const TrajectoryOptions& options);
    //加载地图数据
    void LoadState(const std::string& state_filename, const bool load_frozen_state);
    double ReportProgress();

public:
    int Init();
    int Start();
    //定位模式函数
    int iLocalization();
    //建图模式函数
    int iMapping();

    TMapInfo tMapInfo;
    //粒子辅助纠偏器
    TAgvLaserScanMsg tCurLaserScan;
    AmclTracker *poAmclTracker_;

    //当前安全等级
    int s32SafetyLevel_;
    SafetyMove *poSafetyMove_;
    double timeDiffMax_ = 0;

    FILE *pLogSlam;
    bool bLogSlam;
    bool bStartLog = false;
    double timePre = 0.0;

};




#endif // AGVSLAMMER_H
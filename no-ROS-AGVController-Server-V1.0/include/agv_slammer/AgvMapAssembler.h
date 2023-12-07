/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-04-10 08:32:38
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-09-12 16:43:46
 * @FilePath: /agv_controller/include/agv_slammer/AgvMapAssembler.h
 * @Description: 地图合成与发布类
 */
#ifndef AGVMAPASSEMBLER_H
#define AGVMAPASSEMBLER_H

#include <stdint.h>
#include <pthread.h>
#include <mqueue.h>
#include <sys/time.h>
#include <map>

#include "absl/synchronization/mutex.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"

#include "AgvTf.h"
#include "agv_type.h"
#include "agv_cmd.h"


using ::cartographer::mapping::SubmapId;
using ::cartographer::io::SubmapSlice;
//地图模块的运行状态
enum class AgvMapAssemblerState
{
    INIT,
    NOMAPPUB, //定位模式不发送地图
    MAPPUB,   //建图模式发送地图
    ERROR
};

class AgvMapAssembler
{
private:
    mqd_t qMapAssemblerSvr;
    pthread_t thMapAssemblerSvr;
public:
    AgvCtrl *pt_agv_ctrl_;
    bool bHasInited;
    bool bHasStarted;
    AgvMapAssemblerState eState;
    int s32CtrlFreq;
    int tickCount_;//计数器，每计数100次发布地图一次
    static void *LoopMapAssemblerStatic(void *arg);
    int LoopMapAssemblerSvr();
    // cartographer::transform::Rigid3d local_to_map_;
    Rigid3d local_to_map_;
    AgvTf *g_pAgvTf_;

public:
    int AsyncMsgPost(TRobotMsg *ptAgvMsg);
    int AsyncMsgHandler(TRobotMsg *ptAgvMsg);
    AgvMapAssembler(AgvCtrl *pt_agv_ctrl, AgvTf *g_pAgvTf);
    ~AgvMapAssembler();

    AgvMapAssembler(const AgvMapAssembler&) = delete;
    AgvMapAssembler& operator=(const AgvMapAssembler&) = delete;
public:
    //Maybe Wasted
    //所有过程均在同一个线程中执行
    absl::Mutex mutex_;
    std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
    std::string last_frame_id_;
    TAgvTimeStamp last_timestamp_;
    //处理子图列表
    std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(const ::cartographer::mapping::SubmapId& submap_id);
    void HandleSubmapList(TSubmapList *pTSubmapList);
    // 绘制地图
    // 发布地图
    void DrawAndPublish();
    // std::unique_ptr<nav_msgs::OccupancyGrid> CreateOccupancyGridMsg(
    //     const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    //     const double resolution, const std::string& frame_id,
    //     const TAgvTimeStamp& time);
    TOccupancyGrid CreateOccupancyGridMsg(
            const cartographer::io::PaintSubmapSlicesResult& painted_slices,
            const double resolution, const std::string& frame_id,
            const TAgvTimeStamp& time);

public:
    int Init();
    int Start();
    void *LoopSlamSvrStatic(void *arg);
    int LoopSlamSvr();
};




#endif // AGVMAPASSEMBLER_H
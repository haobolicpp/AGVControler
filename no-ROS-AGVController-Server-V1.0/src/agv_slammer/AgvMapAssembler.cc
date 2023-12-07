/*
 * @Author: yang.cheng yang.cheng@bgirobot.com
 * @Date: 2023-09-08 17:04:19
 * @LastEditors: yang.cheng yang.cheng@bgirobot.com
 * @LastEditTime: 2023-10-12 17:06:52
 * @FilePath: /agv_controller/src/agv_slammer/AgvMapAssembler.cc
 * @Description: 地图合成与发布类
 */

#include <pthread.h>
#include <mqueue.h>
#include <assert.h>
#include <mqueue.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "agv_type.h"
#include "agv_cmd.h"
#include "tlog.h"

#include "AgvMapAssembler.h"
#include "gflags/gflags.h"
#include "SlamUtil.h"
#include "AgvCtrl.h"
#include "AgvTf.h"
#include "SensorConvert.h"

#define SLAM_MAP_MAX_SIZE 16 * 1024 * 1024
uint8_t *occu_grid_map_buffer = (uint8_t *)malloc(SLAM_MAP_MAX_SIZE);//1Mb

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");

AgvMapAssembler::AgvMapAssembler(AgvCtrl *pt_agv_ctrl, AgvTf *g_pAgvTf)
{
    bHasInited = false;
    bHasStarted = false;
    eState = AgvMapAssemblerState::INIT;
    // s32CtrlFreq = 100;
    s32CtrlFreq = 20;
    tickCount_ = 0;
    pt_agv_ctrl_ = pt_agv_ctrl;
    g_pAgvTf_ = g_pAgvTf;
}

AgvMapAssembler::~AgvMapAssembler()
{
}
/**
 * @brief AgvMapAssembler模块初始化
 * @param 
 * @return 
 */
int AgvMapAssembler::Init()
{
    int s32ret;

    eState = AgvMapAssemblerState::NOMAPPUB;
    //TODO 集成进系统 标准消息定义
    struct mq_attr t_mq_attr = {0};
    t_mq_attr.mq_flags = 0; //Block Mode
    t_mq_attr.mq_msgsize = sizeof(st_robot_msg); //todo intagerate with total project
    //t_mq_attr.mq_msgsize = 32;
    t_mq_attr.mq_maxmsg = 8;

    mq_unlink("/q_agv_map_assembler");
    mode_t omask;
    omask = umask(0); //返回之前的文件权限默认值，同时修改默认的umask数值为0，这样后面设置文件权限的时候，umask值无影响了（创建文件时，文件权限的计算方式是：umask取反 & 要设定的文件权限）
    this->qMapAssemblerSvr = mq_open("/q_agv_map_assembler", O_RDWR | O_CREAT | O_NONBLOCK , (S_IRWXU | S_IRWXG | S_IRWXO) /* 777 其他用户可修改*/,  &t_mq_attr);
    umask(omask);
    if (this->qMapAssemblerSvr < 0)
    {
        printf("ERROR: q_agv_map_assembler create failed with error:%s\n",strerror(errno));
    }
    assert(this->qMapAssemblerSvr >= 0);

    printf("INFO: AgvMapAssembler Inited!\n");

    bHasInited = true;
    return 0;
}

/**
 * @brief AgvMapAssembler模块运行
 * @param 
 * @return 
 */
int AgvMapAssembler::Start()
{
    int s32Ret;

    if(!bHasInited || bHasStarted)
    {
        printf("ERROR: AgvMapAssembler Start failed!\n");
        return -1;
    }
    bHasStarted = true;
    
    pthread_attr_t pthread_attr;
    struct sched_param sched_param;
    pthread_attr_init(&pthread_attr);
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_OTHER);
    sched_param.sched_priority = 10;//优先级设多少？
    pthread_attr_setschedparam(&pthread_attr, &sched_param);

    s32Ret = pthread_create(&this->thMapAssemblerSvr, &pthread_attr, LoopMapAssemblerStatic, (void *)(this));
    assert(s32Ret >= 0);

    printf("AgvMapAssembler Run!\n");
    return 0;
}

/**
 * @brief AgvMapAssembler工作线程入口
 * @param arg 当前类指针 
 * @return void
 */
void *AgvMapAssembler::LoopMapAssemblerStatic(void *arg)
{
    int ret;
    struct sched_param param;
    int policy;

    ret = pthread_getschedparam(pthread_self(), &policy, &param);
    tlog(TLOG_INFO, "Map Thread Default Policy[%d], Prioity[%d], ret[%d]\n", policy, param.sched_priority, ret);
    param.sched_priority = 10;
    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    pthread_getschedparam(pthread_self(), &policy, &param);
    tlog(TLOG_INFO, "Map Thread SetWith Policy[%d], Prioity[%d], ret[%d]\n", policy, param.sched_priority, ret);
    
    AgvMapAssembler *poAgvMapAssembler = (AgvMapAssembler *)arg;
    poAgvMapAssembler->LoopMapAssemblerSvr();
    
    return NULL;
}

/**
 * @brief AgvMapAssembler工作线程实体
 * @return 
 */
int AgvMapAssembler::LoopMapAssemblerSvr()
{
    int s32Ret;
    struct timeval t_tmptv;
    fd_set tFdReadSet;
    TRobotMsg tAgvMsg;
    TSubmapList tSubmapList = {0};
    int trajectory_id;

    int s32FdSetMax = qMapAssemblerSvr + 1;
    int usFromCtrlFreq = floor(1000000.0/s32CtrlFreq);
    
    while(bHasStarted)
    {
        t_tmptv.tv_sec = 0;
        t_tmptv.tv_usec = usFromCtrlFreq; 
        FD_ZERO(&tFdReadSet);
        FD_SET(qMapAssemblerSvr, &tFdReadSet);
        s32Ret = select(s32FdSetMax, &tFdReadSet, NULL, NULL, &t_tmptv);
        if (s32Ret < 0)
        {
            printf("ERROR: Agv Map Assembler Loop with %s\n", strerror(errno));
            sleep(1);
            continue;
        }
        //消息处理，定位模式停止地图发布,切换状态
        else if (FD_ISSET(qMapAssemblerSvr, &tFdReadSet))
        {
            memset(&tAgvMsg, 0, sizeof(TRobotMsg));

            s32Ret = mq_receive(qMapAssemblerSvr, (char *)&tAgvMsg,
                        sizeof(TRobotMsg), NULL);
            if (s32Ret < 0)
            {
                printf("ERROR: Agv Map Assembler Get Msg Failed %s\n", strerror(errno));
                continue;
            }
            s32Ret = AsyncMsgHandler(&tAgvMsg);
            RobotMsgRelease(&tAgvMsg);
        }
        //todo周期性发送地图和全局位姿
        else
        {
          absl::MutexLock lock(&pt_agv_ctrl_->poAgvSlammer->mutex_);
          //周期性发布全局位姿0.01s
          trajectory_id = pt_agv_ctrl_->poAgvSlammer->upAgvMapBuilder_
                            ->map_builder_->num_trajectory_builders() - 1;
          local_to_map_ = pt_agv_ctrl_->poAgvSlammer->upAgvMapBuilder_
                            ->getLocalToGlobal(trajectory_id);

          g_pAgvTf_->SetTransform("map", "odom", local_to_map_);
          
          tickCount_++;
          if(eState == AgvMapAssemblerState::NOMAPPUB)//无地图发布
          {
            continue;
            //return s32Ret;
          }
          else if(eState == AgvMapAssemblerState::MAPPUB && tickCount_ >= 100)//发布地图 1s
          {
            //获取子图列表
            pt_agv_ctrl_->poAgvSlammer->upAgvMapBuilder_->getSubmapList(&tSubmapList);
            //处理子图列表
            HandleSubmapList(&tSubmapList);
            //绘制发布地图
            DrawAndPublish();
            tickCount_ = 0;
          }
        }
        //printf("AgvMapAssembler Loop Body!\n");
    }

    return s32Ret;
}




/**
 * @brief HandleSubmapList处理子图列表
 * @param 
 * @return 
 */
void AgvMapAssembler::HandleSubmapList(TSubmapList *pTSubmapList)
{
    absl::MutexLock locker(&mutex_);

  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto& submap_iter : pTSubmapList->submap) {
    const SubmapId id{submap_iter.trajectory_id, submap_iter.submap_index};
    submap_ids_to_delete.erase(id);
    if ((submap_iter.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_iter.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_iter.pose);
    submap_slice.metadata_version = submap_iter.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_iter.submap_version) {
      continue;
    }

    auto fetched_textures = FetchSubmapTextures(id);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }
    // Delete all submaps that didn't appear in the message.
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = pTSubmapList->header.stamp;
  last_frame_id_ = pTSubmapList->header.frame_id;
} 
/**
 * @brief FetchSubmapTextures获取子图纹理
 * @param 
 * @return 
 */
std::unique_ptr<::cartographer::io::SubmapTextures> AgvMapAssembler::FetchSubmapTextures(const ::cartographer::mapping::SubmapId& submap_id) 
{
    TSubmapQuery tSubmapQuery = {0};
    pt_agv_ctrl_->poAgvSlammer->upAgvMapBuilder_->HandleSubmapQuery(submap_id, &tSubmapQuery);

  if (tSubmapQuery.textures.empty()) {
    return nullptr;
  }
  auto response = absl::make_unique<::cartographer::io::SubmapTextures>();
  response->version = tSubmapQuery.submap_version;
  for (const auto& texture : tSubmapQuery.textures) {
    const std::string compressed_cells(texture.cells.begin(),
                                       texture.cells.end());
    response->textures.emplace_back(::cartographer::io::SubmapTexture{
        ::cartographer::io::UnpackTextureData(compressed_cells, texture.width,
                                              texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }
  return response;
}
/**
 * @brief DrawAndPublish绘制发布地图
 * @param 
 * @return 
 */
void AgvMapAssembler::DrawAndPublish()
{
  absl::MutexLock locker(&mutex_);

  if (submap_slices_.empty() || last_frame_id_.empty())
  {
    return;
  }

  auto painted_slices = PaintSubmapSlices(submap_slices_, FLAGS_resolution);

  TOccupancyGrid msg = CreateOccupancyGridMsg(
      painted_slices, FLAGS_resolution, last_frame_id_, last_timestamp_);

  pt_agv_ctrl_->SetMapInfo(msg);

  return;
}

/**
 * @brief 模块功能请求 一般被其他模块调用
 * @param 
 * @return 0 成功 -1失败
 */
int AgvMapAssembler::AsyncMsgPost(TRobotMsg *ptAgvMsg)
{
    int ret;
    ret = mq_send(qMapAssemblerSvr, (char *)ptAgvMsg, sizeof(TRobotMsg), 0);
    if (ret < 0)
    {
        RobotMsgRelease(ptAgvMsg);
        printf("AgvMapAssembler Post [%d]\n",errno);
        return -1;
    }
    return 0;
}
/**
 * @brief 异步消息处理
 * @param 
 * @return 0 成功 -1失败
 */
int AgvMapAssembler::AsyncMsgHandler(TRobotMsg *ptAgvMsg)
{
    //建图->定位,不发布地图
    if(ptAgvMsg->t_msg_header.u16_class == AGV_CMD_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CMD_T_LOCALIZATION_RUN)
    {
      this->eState = AgvMapAssemblerState::NOMAPPUB;
    }
    //发布地图
    else if(ptAgvMsg->t_msg_header.u16_class == AGV_CMD_C_CTRL && 
            ptAgvMsg->t_msg_header.u16_type == AGV_CMD_T_SLAM_RUN)
    {
      this->eState = AgvMapAssemblerState::MAPPUB;
    }

    return 0;
}

/**
 * @brief 创建栅格地图
 * @param 
 * @return 
 */
TOccupancyGrid AgvMapAssembler::CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const TAgvTimeStamp& time) {
  // TOccupancyGrid occupancy_grid = {0};
  TOccupancyGrid occupancy_grid;
  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height = cairo_image_surface_get_height(painted_slices.surface.get());
  occupancy_grid.header.stamp = time;
  snprintf(occupancy_grid.header.frame_id, FRAME_IDS_MAX, "map");
  // occupancy_grid.header.frame_id = frame_id;
  occupancy_grid.map_load_time = time;
  occupancy_grid.resolution = resolution;
  occupancy_grid.width = width;
  occupancy_grid.height = height;
  occupancy_grid.origin.position.x =
      -painted_slices.origin.x() * resolution;
  occupancy_grid.origin.position.y =
      (-height + painted_slices.origin.y()) * resolution;
  occupancy_grid.origin.position.z = 0.;
  occupancy_grid.origin.orientation.w = 1.;
  occupancy_grid.origin.orientation.x = 0.;
  occupancy_grid.origin.orientation.y = 0.;
  occupancy_grid.origin.orientation.z = 0.;

  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid.data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value =
          observed == 0
              ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      occupancy_grid.data.push_back(value);
    }
  }

  return occupancy_grid;
}
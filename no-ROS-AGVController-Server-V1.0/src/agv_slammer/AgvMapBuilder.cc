
#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

#include "agv_type.h"
#include "AgvCtrl.h"
#include "agv_cmd.h"
#include "AgvTf.h"
#include "SlamPara.h"
#include "SensorBrige.h"
#include "AgvMapBuilder.h"
#include "AmclTracker.h"

#include <chrono>
/**
 * @brief AgvMapBuilder 构造
 * @param node_options 用户主配置
 * @param map_builder Carto 建图类对象
 * @param pAgvTf 系统坐标树管理对象
 * @return
 */
AgvMapBuilder::AgvMapBuilder(
    const NodeOptions& node_options,
    bool isPureLoc,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    AgvCtrl *poAgvCtrl,
    AgvTf *poAgvTf):
    node_options_(node_options),
    isPureLoc_(isPureLoc),
    map_builder_(std::move(map_builder)),
    poAgvCtrl_(poAgvCtrl),
    poAgvTf_(poAgvTf)
{

    bGetLocalToGloable = false;

    // if (isPureLoc_)
    // {
    //     map_builder_->pose_graph()->SetGlobalSlamOptimizationCallback
    //     (
    //         [this](const std::map<int, cartographer::mapping::SubmapId>& last_optimized_submap_ids,
    //         const std::map<int, cartographer::mapping::NodeId>& last_optimized_node_ids)
    //         {
    //             OnGlobalSlamOptimizations(last_optimized_submap_ids, last_optimized_node_ids);
    //         }
    //     );
    // }

    //DEBUG
    // char dirLogPose[128];
    // char timestr[256];
    // time_t t = time(0);
    // strftime(timestr, 256, "%Y%m%d%H%M", localtime(&t));
    // sprintf(dirLogPose, "/home/%s/Downloads/pose_record%s.txt", getlogin(), timestr);
    // pLogPose = fopen(dirLogPose, "w+");
    // bLogPose = true;

}
/**
 * @brief AgvMapBuilder 析构
 * @param node_options 用户主配置
 * @param map_builder Carto 建图类对象
 * @param pAgvTf 系统坐标树管理对象
 * @return
 */
AgvMapBuilder::~AgvMapBuilder()
{

}


/**
 * @brief 加载已有pbstream数据
 * @param state_filename pbstream完整路径
 * @param load_frozen_state 冻结已有的Map(traj)数据，后端不再优化它们。基于已有图的定位要求冻结它们
 * @return void
 */
void AgvMapBuilder::LoadState(const std::string& state_filename, bool load_frozen_state)
{
    absl::MutexLock lock(&mutex_);
    const std::string suffix = ".pbstream";
    CHECK_EQ(state_filename.substr(
                std::max<int>(state_filename.size() - suffix.size(), 0)),
            suffix)
        << "The file containing the state to be loaded must be a "
            ".pbstream file.";
    LOG(INFO) << "Loading saved state '" << state_filename << "'...";
    cartographer::io::ProtoStreamReader stream(state_filename);
    map_builder_->LoadState(&stream, load_frozen_state);
}


/**
 * @brief 前端SLAM结果获取的回调函数
 * @param trajectory_id 前端SLAM执行的轨迹序号
 * @param time 前端完成SLAM计算的时间戳
 * @param local_pose AGV当前在odom下的位姿
 * @return 
 */
void AgvMapBuilder::OnLocalSlamResult(const int trajectory_id,
    const ::cartographer::common::Time time,
    const ::cartographer::transform::Rigid3d local_pose,
    ::cartographer::sensor::RangeData range_data_in_local,
    std::vector<double> eigen_value,
    std::vector<double> eigen_vector)
{
    std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data =
        std::make_shared<LocalTrajectoryData::LocalSlamData>(
            LocalTrajectoryData::LocalSlamData
            {
                time, 
                local_pose, 
                std::move(range_data_in_local)
            });
{
  absl::MutexLock lock(&mutex_);
  local_slam_data_[trajectory_id] = std::move(local_slam_data);
}

    int s32Ret;
    Rigid3d r3O2B = local_slam_data_[trajectory_id]->local_pose;
    poAgvTf_->SetTransform("odom", "base_link", r3O2B);
    poAgvTf_->AddPoseToExtrapolator(r3O2B);

    // TLinuxTime tTimeNow;
    // LinuxTimeNow(&tTimeNow);
    // double timeNow = (tTimeNow.tv_sec * 1e9 + tTimeNow.tv_nsec * 1.0) / 1e6;
    // double timeDiff;

    // if (poAgvCtrl_->poAgvSlammer->bLogSlam && bStartLog)
    // {
    //     timeDiff = timeNow - timePre;
    //     timePre = timeNow;
    //     fprintf(poAgvCtrl_->poAgvSlammer->pLogSlam, "2-----------,%f\n", timeDiff);
    //     printf("2-----------,%f\n", timeDiff);
    // }

    // if (!bStartLog)
    // {
    //     timePre = timeNow;
    //     bStartLog = true;
    // }



//     AmclTracker *poAmclTracker = poAgvCtrl_->poAgvSlammer->poAmclTracker_;
//     SafetyMove *poSafetyMove = poAgvCtrl_->poAgvSlammer->poSafetyMove_;
//     TLinuxTime tTimeOrg, tTimeEnd; //时间统计
//     double timeDiff;

// //发送消息至ROS调试端----------------------------------------------
//     uint8_t pPoseBuf[2 * sizeof(TAgvPose2D)];
//     TAgvPose2D *pPoseStr = (TAgvPose2D *)pPoseBuf;
//     //Local Pose
//     TAgvPose2D tLocalPose2D = ToAgvPose2D(r3O2B);
//     memcpy(pPoseStr, &tLocalPose2D, sizeof(TAgvPose2D));
//     //Global Pose
//     Rigid3d r3M2O;
//     poAgvTf_->GetTransform("map", "odom", r3M2O);
//     TAgvPose2D tMap2Odom2D = ToAgvPose2D(r3M2O);
//     pPoseStr = pPoseStr + 1;
//     memcpy(pPoseStr, &tMap2Odom2D, sizeof(TAgvPose2D));
//     //构造消息
//     TRobotMsgHeader tRosMsgHeader;
//     TRobotMsg tRosMsg;
//     tRosMsgHeader.u16_class = AGV_ROS_C_CTRL;
//     tRosMsgHeader.u16_type = AGV_ROS_T_POSE_INFO;
//     tRosMsgHeader.s32_len = 2 * sizeof(TAgvPose2D);
//     RobotMsgInit(&tRosMsg, tRosMsgHeader, pPoseBuf);
//     poAgvCtrl_->poAgvRosSvr_->AsyncMsgPost(&tRosMsg);

//     Rigid3d r3M2B = r3M2O * r3O2B;
//     TAgvPose2D tPoseM2b = ToAgvPose2D(r3M2B);
// //本地记录文件，记录退化情况----------------------------------------------
//     if (eigen_value.size() > 0)
//     {
//         int degeneration_flag = 0;
//         double degeneration = fabs(eigen_value[2]/eigen_value[0]);
//         if (degeneration > 120.0)
//         {
//             degeneration_flag = 1;
//         }

//         if (poAgvCtrl_->poAgvSlammer->bLogSlam)
//         {
//             fprintf(poAgvCtrl_->poAgvSlammer->pLogSlam, "df,%d,%5f,%5f,%5f,%5f\n",
//             degeneration_flag, degeneration, tPoseM2b.x, tPoseM2b.y, tPoseM2b.phi);
//         }
//     }

    //AMCL辅助定位器处理当前位姿和雷达数据
    // if (isPureLoc_)
    // {

    //     Rigid3d r3CartoPose, r3AmclPose, r3CartoDrift;
    //     LinuxTimeNow(&tTimeOrg);
    //     s32Ret = poAmclTracker->LaserScanProcess(r3O2B, &poAgvCtrl_->poAgvSlammer->tCurLaserScan, r3AmclPose);
    //     if (s32Ret == 0) //行驶里程符合滤波条件
    //     {        
    //         LinuxTimeNow(&tTimeEnd);
    //         timeDiff = ((tTimeEnd.tv_sec * 1e9 + tTimeEnd.tv_nsec * 1.0) - 
    //         (tTimeOrg.tv_sec * 1e9 + tTimeOrg.tv_nsec * 1.0)) / 1e6;
    //         poAgvTf_->GetTransform("map", "base_link", r3CartoPose);
    //         r3CartoDrift = r3AmclPose * r3CartoPose.inverse();
    //         poAgvTf_->SetTransform("world", "map", r3CartoDrift); //AMCL与CARTO定位偏移量
    //         TAgvPose2D TDrift2D = ToAgvPose2D(r3CartoDrift);
    //         printf("INFO: AMCL TO CARTO Drift-X[%f]-Y[%f]-Phi[%f] Consume %f ms\n",
    //         TDrift2D.x, TDrift2D.y, TDrift2D.phi, timeDiff);
    //     }
    // }
    // static int s_SendPriod = 0;

    //局部地图构建与运行安全监测
    // if (isPureLoc_)
    // {
    //     TRobotMsg tAgvMsg;
    //     TRobotMsgHeader tAgvMsgHeader;
    //     TAgvPointCloud2 tPointCloudInOdom; //转换点云数据的格式
    //     TAgvPosition3D tPointTemp;
    //     Rigid3d r3O2L;
    //     TAgvPose2D tPoseO2L;
    //     poAgvTf_->GetTransform("odom", "laser", r3O2L);
    //     tPoseO2L = ToAgvPose2D(r3O2L);

    //     LinuxTimeNow(&tTimeOrg);
    //     poSafetyMove->LaserScanToPointCloud(poAgvCtrl_->poAgvSlammer->tCurLaserScan, 
    //         4, tPoseO2L, tPointCloudInOdom);
    //     // tPointCloudInOdom.points.clear();
    //     // int s32PointCloudSize = 
    //     //     local_slam_data_[trajectory_id]->range_data_in_local.returns.size();
    //     // for (int i = 0; i < s32PointCloudSize; i++)
    //     // {
    //     //     tPointTemp.x = 
    //     //         local_slam_data_[trajectory_id]->range_data_in_local.returns[i].position.x();
    //     //     tPointTemp.y = 
    //     //         local_slam_data_[trajectory_id]->range_data_in_local.returns[i].position.y();
    //     //     tPointTemp.z = 
    //     //         local_slam_data_[trajectory_id]->range_data_in_local.returns[i].position.z();
    //     //     tPointCloudInOdom.points.push_back(tPointTemp);
    //     // }
    //     //处理本帧点云
    //     poSafetyMove->PointCloudProcess(tPointCloudInOdom);
    //     //更新局部地图
    //     TAgvPose2D TCurLocalPose = ToAgvPose2D(r3O2B);
    //     poSafetyMove->Update(TCurLocalPose);
    //     //计算安全等级 判断是否出现障碍物
    //     if (poSafetyMove->SafetyControlCompute(poAgvCtrl_->poAgvSlammer->s32SafetyLevel_))
    //     {
    //         tAgvMsgHeader.u16_class = AGV_CHASSIS_C_CTRL;
    //         tAgvMsgHeader.u16_type = AGV_CHASSIS_T_SAFETY;
    //         tAgvMsgHeader.s32_len = sizeof(int);
    //         RobotMsgInit(&tAgvMsg, tAgvMsgHeader, (uint8_t *)&poAgvCtrl_->poAgvSlammer->s32SafetyLevel_);
    //         poAgvCtrl_->poAgvChassisCtrl->AsyncMsgPost(&tAgvMsg);
    //     }
    //     //发送局部子图到调试端
    //     if (poAgvCtrl_->poAgvRosSvr_->bRosConnected && s_SendPriod++ % 10 == 0)
    //     {
    //         s32Ret = poSafetyMove->ConvertCostMapToGrid(poSafetyMove->pMapDetect_, poAgvCtrl_->tMapLocal_);
    //         if (s32Ret == 0)
    //         {
    //             poAgvCtrl_->poAgvRosSvr_->SendLocalMapGrid(poAgvCtrl_->tMapLocal_);
    //         }
    //     }
    //     LinuxTimeNow(&tTimeEnd);
    //     timeDiff = ((tTimeEnd.tv_sec * 1e9 + tTimeEnd.tv_nsec * 1.0) - 
    //         (tTimeOrg.tv_sec * 1e9 + tTimeOrg.tv_nsec * 1.0)) / 1e6;

    //     if (timeDiff > poAgvCtrl_->poAgvSlammer->timeDiffMax_)
    //     {
    //         printf("INFO: SAFETY Detect Consume %f ms\n", timeDiff);
    //         poAgvCtrl_->poAgvSlammer->timeDiffMax_ = timeDiff;
    //     }
    // }

    //记录与调试
    // double poseNow;
    // g_pAgvTf_->SetTransform("world", "baselink", r3AmclPose);
    // TAgvPose2D tQRPose = ToAgvPose2D(local_slam_data_[trajectory_id]->local_pose);
    // TAgvPose2D tAmclPose = ToAgvPose2D(r3AmclPose);
    // TAgvPose2D tCartoPose = ToAgvPose2D(r3CartoPose);
    // if (bLogPose)
    // {
    //     LinuxTimeNow(&tTimeOrg);
    //     poseNow = (tTimeOrg.tv_sec * 1e9 + tTimeOrg.tv_nsec * 1.0) / 1e6;
    //     fprintf(pLogPose, "%f, QR, %f, %f, %f, CARTO, %f, %f, %f, AMCL, %f, %f, %f\n",
    //         poseNow, tQRPose.x, tQRPose.y, tQRPose.phi,
    //          tCartoPose.x, tCartoPose.y, tCartoPose.phi,
    //          tAmclPose.x, tAmclPose.y, tAmclPose.phi);
    // }
}

/**
 * @brief 开启一个新的定位或建图轨迹
 * @param expected_sensor_ids 预先设定的传感器话题集
 * @return expected_topics 传感器话题集合
 */
int AgvMapBuilder::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
    expected_sensor_ids,
    const TrajectoryOptions& trajectory_options)
{
    const int trajectory_id = map_builder_->AddTrajectoryBuilder(
        expected_sensor_ids, trajectory_options.trajectory_builder_options,
        [this](const int trajectory_id, const ::cartographer::common::Time time,
            const ::cartographer::transform::Rigid3d local_pose,
            ::cartographer::sensor::RangeData range_data_in_local,
            const std::unique_ptr<const ::cartographer::mapping::TrajectoryBuilderInterface::InsertionResult>,
            std::vector<double> eigen_value,
            std::vector<double> eigen_vector)
            {
                OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local, eigen_value, eigen_vector);
            });

    LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

// Make sure there is no trajectory with 'trajectory_id' yet.
    CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
    sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
        trajectory_options.num_subdivisions_per_laser_scan,
        trajectory_options.tracking_frame,
        node_options_.lookup_transform_timeout_sec,
        poAgvTf_,
        map_builder_->GetTrajectoryBuilder(trajectory_id));
    auto emplace_result = trajectory_options_.emplace(trajectory_id, trajectory_options);
    CHECK(emplace_result.second == true);
    
    return trajectory_id;
}

/**
 * @brief 终止轨迹
 * @param trajectory_id 
 * @return 
 */
int AgvMapBuilder::FinishTrajectory(const int trajectory_id) {
  map_builder_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);

  return 0;
}

/**
 * @brief 终止轨迹后执行优化
 * @param trajectory_id 
 * @return 
 */
int AgvMapBuilder::RunFinalOptimization() {
  map_builder_->pose_graph()->RunFinalOptimization();
  return 0;
}
/**
 * @brief 生成pgm图
 * @param  
 * @return 
 */
void AgvMapBuilder::WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer) {
  const std::string header =
      absl::StrCat("P5\n# Cartographer map; ", resolution, " m/pixel\n",
                   image.width(), " ", image.height(), "\n255\n");
  file_writer->Write(header.data(), header.size());
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      const char color = image.GetPixel(x, y)[0];
      file_writer->Write(&color, 1);
    }
  }
}
/**
 * @brief 生成yaml文件
 * @param  
 * @return 
 */
void AgvMapBuilder::WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename,
               ::cartographer::io::FileWriter* file_writer) {
  // Magic constants taken directly from ros map_saver code:
  // https://github.com/ros-planning/navigation/blob/ac41d2480c4cf1602daf39a6e9629142731d92b0/map_server/src/map_saver.cpp#L114
  const std::string output = absl::StrCat(
      "image: ", pgm_filename, "\n", "resolution: ", resolution, "\n",
      "origin: [", origin.x(), ", ", origin.y(),
      ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n");
  file_writer->Write(output.data(), output.size());
}

/**
 * @brief 保存地图
 * @param //pbstream_filename map_filestem resolution
 * @return 
 */
void AgvMapBuilder::save(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::ValueConversionTables conversion_tables;
  ::cartographer::io::DeserializeAndFillSubmapSlices(
      &deserializer, &submap_slices, &conversion_tables);

  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

  ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

  ::cartographer::io::Image image(std::move(result.surface));
  WritePgm(image, resolution, &pgm_writer);

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
  WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
}
/**
 * @brief 获取子图列表
 * @param 
 * @return 
 */
void AgvMapBuilder::getSubmapList(TSubmapList *ptSubmapList)
{
    TSubmapList submap_list;
    TAgvTimeStamp tAgvTimeStamp;
    //获取系统时间ns
    const std::chrono::nanoseconds now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            std::chrono::system_clock::now().time_since_epoch());
    int64_t s64_now =  now.count();
    tAgvTimeStamp = fromNSec(s64_now);
   
    submap_list.header.stamp = tAgvTimeStamp;
    snprintf(submap_list.header.frame_id, FRAME_IDS_MAX, "map");
    // submap_list.header.frame_id = "map";
     for (const auto& submap_id_pose : map_builder_->pose_graph()->GetAllSubmapPoses()) {
        SubmapEntry submap_entry;
        submap_entry.is_frozen = map_builder_->pose_graph()->IsTrajectoryFrozen(submap_id_pose.id.trajectory_id);
        submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
        submap_entry.submap_index = submap_id_pose.id.submap_index;
        submap_entry.submap_version = submap_id_pose.data.version;
        submap_entry.pose = ToAgvPoseMsgPose(submap_id_pose.data.pose);
        submap_list.submap.push_back(submap_entry);
  }
  *ptSubmapList = submap_list;
}
/**
 * @brief 处理子图
 * @param 
 * @return 
 */
void AgvMapBuilder::HandleSubmapQuery(const ::cartographer::mapping::SubmapId& submap_id,TSubmapQuery* ptSubmapQuery) 
{
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  const std::string error =
      map_builder_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
      printf("%s", error.c_str());
    return;
  }

  ptSubmapQuery->submap_version = response_proto.submap_version();
  for (const auto& texture_proto : response_proto.textures()) {
    ptSubmapQuery->textures.emplace_back();
    auto& texture = ptSubmapQuery->textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToAgvPoseMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
}
/**
 * @brief 获取局部相对于全局位姿
 * @param 
 * @return 
 */
Rigid3d AgvMapBuilder::getLocalToGlobal(const int trajectory_id)
{
    Rigid3d localToGlobal = map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
    // AgvTransUtil::Rigid3d localToGlobal(tmpPose.translation(), tmpPose.rotation());                                                    
    
    return localToGlobal;
}


/**
 * @name: OnGlobalSlamOptimizations
 * @des:  回环检测与全局优化的回调接口
 * @return {*}
 * @author: yang.cheng
 * @ver: 1.01
 */
void AgvMapBuilder::OnGlobalSlamOptimizations(
        const std::map<int, ::cartographer::mapping::SubmapId>& last_optimized_submap_ids,
        const std::map<int, ::cartographer::mapping::NodeId>& last_optimized_node_ids)
{
    // AmclTracker *poAmclTracker = poAgvCtrl_->poAgvSlammer->poAmclTracker_;
    // int trajectory_id;
    // Rigid3d r3M2O, r3M2B, r3O2B;
    // trajectory_id = map_builder_->num_trajectory_builders() - 1;
    // assert(trajectory_id >= 0);
    // //设置坐标变换GlobalToLocal  即 MaptoOdom
    // r3M2O = map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
    // poAgvTf_->SetTransform("map", "odom", r3M2O);

    // //获取机器人当前全局位姿和局部位姿
    // poAgvTf_->GetTransform("map", "base_link", r3M2B);
    // poAgvTf_->GetTransform("odom", "base_link", r3O2B);

    // //调试
    // TAgvPose2D tGlobalPose2D = ToAgvPose2D(r3M2B);
    // printf("WARN: AgvMapBuilder Loop Closure With Robot Global Pose X[%f]-Y[%f]-PHI[%f]\n",
    //     tGlobalPose2D.x, tGlobalPose2D.y, tGlobalPose2D.phi);
    
    //     Rigid3d r3I;
    //     if (poAmclTracker->reset_conut < 5)
    //     {
    //         poAgvTf_->SetTransform("world", "map", r3I); //调试用 记录偏移量
    //     }
    //重置AMCL辅助定位器
    // int trajectory_id;
    // Rigid3d r3M2O;
    // {
    // absl::MutexLock lock(&poAgvCtrl_->poAgvSlammer->mutex_);
    // //周期性发布全局位姿0.01s
    // trajectory_id = poAgvCtrl_->poAgvSlammer->upAgvMapBuilder_
    //                   ->map_builder_->num_trajectory_builders() - 1;
    // r3M2O = poAgvCtrl_->poAgvSlammer->upAgvMapBuilder_
    //                   ->getLocalToGlobal(trajectory_id);
    // }

    // poAgvTf_->SetTransform("map", "odom", r3M2O, isPureLoc_);

}
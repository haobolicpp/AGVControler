/*********************************************************************
 * AgvMapBuilder Class Definiation
 * 主要实现与Slam库的调用
 * Copyright: BGI
 * Author: yang.cheng
 *********************************************************************/


#ifndef AGVMAPBUILDER_H
#define AGVMAPBUILDER_H


#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"

#include "agv_type.h"
#include "AgvTf.h"
#include "SlamPara.h"
#include "SensorBrige.h"
#include "SlamUtil.h"


#include <map>
#include "Eigen/Core"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/2d/map_limits.h"

//前端SLAM结果存放的数据结构
struct LocalTrajectoryData 
{
// Contains the trajectory data received from local SLAM, after
// it had processed accumulated 'range_data_in_local' and estimated
// current 'local_pose' at 'time'.
    struct LocalSlamData 
    {
        ::cartographer::common::Time time;
        ::cartographer::transform::Rigid3d local_pose;
        ::cartographer::sensor::RangeData range_data_in_local;
    };
    std::shared_ptr<const LocalSlamData> local_slam_data;
    cartographer::transform::Rigid3d local_to_map;
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;
};


//AgvMapBuilder
class AgvMapBuilder
{
private:
    /* data */
public:
    AgvMapBuilder(
        const NodeOptions& node_options,
        bool isPureLoc,
        std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
        AgvCtrl *poAgvCtrl,
        AgvTf* poAgvTf);

    ~AgvMapBuilder();

    AgvMapBuilder(const AgvMapBuilder&) = delete;
    AgvMapBuilder& operator=(const AgvMapBuilder&) = delete;

public:
    void LoadState(const std::string& state_filename, bool load_frozen_state);

    int AddTrajectory(
        const std::set<
        ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids, const TrajectoryOptions& trajectory_options);

    void OnLocalSlamResult(const int trajectory_id,
            const ::cartographer::common::Time time,
            const ::cartographer::transform::Rigid3d local_pose,
            ::cartographer::sensor::RangeData range_data_in_local,            
            std::vector<double> eigen_value,
            std::vector<double> eigen_vector);
    LOCKS_EXCLUDED(mutex_);

    void OnGlobalSlamOptimizations(
            const std::map<int, ::cartographer::mapping::SubmapId>& last_optimized_submap_ids,
            const std::map<int, ::cartographer::mapping::NodeId>& last_optimized_node_ids);


    Rigid3d getLocalToGlobal(const int trajectory_id);
    // std::unordered_map<int, LocalTrajectoryData> GetLocalTrajectoryData()
    // LOCKS_EXCLUDED(mutex_);
    int FinishTrajectory(const int trajectory_id);
    int RunFinalOptimization();
    void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer);
    void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename,
               ::cartographer::io::FileWriter* file_writer);
    void save(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution);
    //获取子图列表
    void getSubmapList(TSubmapList *ptSubmapList);
    void HandleSubmapQuery(const ::cartographer::mapping::SubmapId& submap_id,TSubmapQuery* ptSubmapQuery);
public:
    absl::Mutex mutex_;
    const NodeOptions node_options_;
    std::unordered_map<int, std::shared_ptr<const LocalTrajectoryData::LocalSlamData>>
        local_slam_data_ GUARDED_BY(mutex_);
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;

    AgvCtrl *poAgvCtrl_;
    AgvTf *poAgvTf_;
    // These are keyed with 'trajectory_id'.
    std::unordered_map<int, TrajectoryOptions> trajectory_options_;
    std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
    std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;

    bool bGetLocalToGloable = false;
    bool isPureLoc_;

    //Debug
    FILE *pLogPose;
    bool bLogPose;
    bool bStartLog = false;
    double timePre = 0;
};


#endif // AGVMAPBUILDER_H
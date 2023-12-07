/*********************************************************************
 * SlamPara.cc
 * 根据LUA解析当前配置的方法实现
 * Copyright: BGI
 * Author: yang.cheng
 *********************************************************************/


#include "glog/logging.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

#include "SlamPara.h"

/**
 * @brief 轨迹参数合法性检验
 * @param 
 * @return
 */
void CheckTrajectoryOptions(const TrajectoryOptions& options) {
  CHECK_GE(options.num_subdivisions_per_laser_scan, 1);
  CHECK_GE(options.num_laser_scans + options.num_multi_echo_laser_scans +
               options.num_point_clouds,
           1)
      << "Configuration error: 'num_laser_scans', "
         "'num_multi_echo_laser_scans' and 'num_point_clouds' are "
         "all zero, but at least one is required.";
}


/**
 * @brief 创建轨迹参数对象
 * @param lua_parameter_dictionary LUA参数文件路径
 * @return 轨迹参数对象
 */
TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* const lua_parameter_dictionary)
{
    TrajectoryOptions options;

    options.trajectory_builder_options = ::cartographer::mapping::CreateTrajectoryBuilderOptions(
        lua_parameter_dictionary->GetDictionary("trajectory_builder").get());
    options.tracking_frame = lua_parameter_dictionary->GetString("tracking_frame");
    options.published_frame = lua_parameter_dictionary->GetString("published_frame");
    options.odom_frame = lua_parameter_dictionary->GetString("odom_frame");
    options.provide_odom_frame = lua_parameter_dictionary->GetBool("provide_odom_frame");
    options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
    options.use_nav_sat = lua_parameter_dictionary->GetBool("use_nav_sat");
    options.use_landmarks = lua_parameter_dictionary->GetBool("use_landmarks");
    options.publish_frame_projected_to_2d = lua_parameter_dictionary->GetBool("publish_frame_projected_to_2d");
    options.num_laser_scans = lua_parameter_dictionary->GetNonNegativeInt("num_laser_scans");
    options.num_multi_echo_laser_scans = lua_parameter_dictionary->GetNonNegativeInt("num_multi_echo_laser_scans");
    options.num_subdivisions_per_laser_scan = lua_parameter_dictionary->GetNonNegativeInt("num_subdivisions_per_laser_scan");
    options.num_point_clouds = lua_parameter_dictionary->GetNonNegativeInt("num_point_clouds");
    options.rangefinder_sampling_ratio = lua_parameter_dictionary->GetDouble("rangefinder_sampling_ratio");
    options.odometry_sampling_ratio = lua_parameter_dictionary->GetDouble("odometry_sampling_ratio");
    options.fixed_frame_pose_sampling_ratio = lua_parameter_dictionary->GetDouble("fixed_frame_pose_sampling_ratio");
    options.imu_sampling_ratio = lua_parameter_dictionary->GetDouble("imu_sampling_ratio");
    options.landmarks_sampling_ratio = lua_parameter_dictionary->GetDouble("landmarks_sampling_ratio");

    CheckTrajectoryOptions(options);
    return options;
}

/**
 * @brief 创建主配置参数对象
 * @param lua_parameter_dictionary LUA参数文件路径
 * @return 主配置参数对象
 */
NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const lua_parameter_dictionary)
{
    NodeOptions options;
    options.map_builder_options = ::cartographer::mapping::CreateMapBuilderOptions(
        lua_parameter_dictionary->GetDictionary("map_builder").get());
    options.map_frame = lua_parameter_dictionary->GetString("map_frame");
    options.lookup_transform_timeout_sec = lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
    options.submap_publish_period_sec = lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
    options.pose_publish_period_sec = lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
    options.trajectory_publish_period_sec = lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
    if (lua_parameter_dictionary->HasKey("publish_to_tf")) 
    {
    options.publish_to_tf = lua_parameter_dictionary->GetBool("publish_to_tf");
    }
    if (lua_parameter_dictionary->HasKey("publish_tracked_pose"))
    {
    options.publish_tracked_pose = lua_parameter_dictionary->GetBool("publish_tracked_pose");
    }
    if (lua_parameter_dictionary->HasKey("use_pose_extrapolator"))
    {
    options.use_pose_extrapolator = lua_parameter_dictionary->GetBool("use_pose_extrapolator");
    }

    return options;
}

/**
 * @brief 从用户默认路径加载配置
 * @param configuration_directory 
 * @param configuration_basename LUA参数文件路径
 * @return 主配置和轨迹配置联合
 */
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename)
{
    auto file_resolver = absl::make_unique<cartographer::common::ConfigurationFileResolver>(
        std::vector<std::string>{configuration_directory});
    const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));

    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary), 
        CreateTrajectoryOptions(&lua_parameter_dictionary));
}